#include "gnss_m8l.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define I2C_0 0
#define I2C_1 1
#define I2C_2 2

#define I2C_DEVICE I2C_2
#define I2C_M8L_DEVICE_ADDR 0x42
#define I2C_DEVICE_PATH "/dev/i2c- "
#define I2C_SEND_DELAY 20000
#define I2C_READ_DELAY 20000
#define I2C_MAX_READ_RETIES 5

#define UBX_BUF_LEN 1024
#define UBX_CRC_LEN 2
#define UBX_HEADER_LEN (4 + 2) //+2 Length
#define UBX_WAIT_FOR_MSG 2

#define BINARY_FORMAT " %c  %c  %c  %c  %c  %c  %c  %c\n"
#define BYTE_TO_BINARY(byte)                                  \
    (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'),     \
        (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

int m8l_fd = 0;
char rx_buf[UBX_BUF_LEN];
int errno;
uint8_t sensor_fusion_active = 0;
uint8_t esf_active = 0;

#ifdef DEBUG
#define debug(...)           \
    do {                     \
        printf(__VA_ARGS__); \
    } while (0)
#else
#define debug(...)
#endif

int i2c_init(uint8_t device_in);
int i2c_send_req(const char* value_in, uint8_t len_in);
int i2c_read_req(void);
int ubx_init(void);
void ubx_dedode_class_mon_msg(char* buf, uint len);
void ubx_dedode_class_cfg_msg(char* buf, uint len);
void ubx_dedode_class_nav_msg(char* buf, uint len);
void ubx_decode_rxm_rawx(char* rmx, uint len);
void ubx_decode_rxm_measx(char* rmx, uint len);
void ubx_decode_esf_meas(char* buf, uint len);
void ubx_start_esf_meas(void);
void ubx_stop_esf_meas(void);
void ubx_decode_esf_status(char* buf, uint len);
void ubx_decode_esf_raw(char* buf, uint len);
void ubx_start_esf_raw(void);
void ubx_stop_esf_raw(void);
void ubx_decode_esf_status(char* esf, uint len);
char* ubx_get_calibration_status(uint8_t calib);
char* ubx_get_sensor_type(uint8_t type);
uint ubx_parse_ubx_data(char* buf, uint len);
void ubx_clear(void);

/*********************************************************
 * 						I2C routinen
 * *******************************************************/

/* I2C init */
int i2c_init(uint8_t device_in)
{
    char i2c_dev_path[] = I2C_DEVICE_PATH;

    i2c_dev_path[strlen(i2c_dev_path) - 1] = device_in + 0x30;

    printf("%s: device: %s, addr: 0x%02x\n", __FUNCTION__, i2c_dev_path, I2C_M8L_DEVICE_ADDR);

    switch (device_in) {
    case I2C_1:
        system("config-pin P9_17 i2c");
        system("config-pin P9_18 i2c");
        break;
    case I2C_2:
        system("config-pin P9_19 i2c");
        system("config-pin P9_20 i2c");
        break;
    case I2C_0:
    default:
        printf("not supported\n");
        return -1;
    }

    if (0 > (m8l_fd = open((const char*)i2c_dev_path, O_RDWR))) {
        printf("%d: Failed to open the i2c busi: %s\n", __LINE__, strerror(errno));
        return -1;
    }

    if (0 > (ioctl(m8l_fd, I2C_SLAVE, I2C_M8L_DEVICE_ADDR))) {
        printf("%d: Failed to set the i2c addr (0x%02x): %s\n", __LINE__, I2C_M8L_DEVICE_ADDR, strerror(errno));
        return -1;
    }

    return 1;
}

/* send */
int i2c_send_req(const char* value_in, uint8_t len_in)
{
    debug("%s: len: %d\n", __FUNCTION__, len_in);

    if (len_in != write(m8l_fd, value_in, len_in)) {
        printf("write to fd failed: %s\n", strerror(errno));
        return -1;
    }

    usleep(I2C_SEND_DELAY);

    return 1;
}

/* receive */
int i2c_read_req(void)
{
    debug("%s: \n", __FUNCTION__);
    uint msg_len;
    uint8_t counter = 0;

    memset(rx_buf, 0x00, UBX_BUF_LEN);

    while (1) {
        if (UBX_HEADER_LEN == read(m8l_fd, rx_buf, UBX_HEADER_LEN)) {
            if ((rx_buf[0] == 0xB5) && (rx_buf[1] == 0x62)) {
                // header received
                break;
            }
        }

        if (I2C_MAX_READ_RETIES > counter) {
            counter++;
            usleep(I2C_READ_DELAY);
        } else {
            debug("no more data to be received.\n");
            return -1;
        }
    }

    debug("header: %02x, %02x, %02x, %02x, %02x, %02x\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]);

    /* header received. Request the message */

    msg_len = ((rx_buf[5] << 8) | rx_buf[4]) + UBX_CRC_LEN;

    if (UBX_BUF_LEN < (UBX_HEADER_LEN + msg_len)) {
        printf("****** MESSAGE BUFFER EXCEEDED *******\n");
        usleep(I2C_READ_DELAY);
        return -1;
    } else {
        //read the message
        if (msg_len == read(m8l_fd, &rx_buf[6], msg_len)) {
            debug("ubx header received (msg_len = %d)\n", msg_len - UBX_CRC_LEN);

            //for (int i = 0; i < (UBX_HEADER_LEN + len); i++)
            //for (int i = 0; i < 10; i++)
            //{
            //	printf("%02x, ", rx_buf[i]);
            //}
            //printf("\n");

            ubx_parse_ubx_data(rx_buf, UBX_HEADER_LEN + msg_len);
        } else {
            printf("failure by reading form receiver %s\n", strerror(errno));
            usleep(I2C_READ_DELAY);
            return -1;
        }
    }

    usleep(I2C_READ_DELAY);

    return 1;
}

/*********************************************************
 * 					UBX routines
 * *******************************************************/

int ubx_init(void)
{
    sensor_fusion_active = 0;
    esf_active = 0; // no ESF messages will be sent

    /* request reset of the GNSS module */
    printf(">>> send ubx_cfg_rst_req (%02x %02x)...\n", ubx_cfg_rst_req[2], ubx_cfg_rst_req[3]);
    i2c_send_req(ubx_cfg_rst_req, sizeof(ubx_cfg_rst_req));

    usleep(500000);

    /* request version from GNSS module */
    printf(">>> send ubx_mon_ver_req (%02x %02x)...\n", ubx_mon_ver_req[2], ubx_mon_ver_req[3]);
    i2c_send_req(ubx_mon_ver_req, sizeof(ubx_mon_ver_req));
    if (0 > i2c_read_req()) {
        printf("ubx_mon_ver_req failed\n");
        return -1;
    }

    /* request HW version from GNSS module */
    //printf(">>> send ubx_mon_hw_req (%02x %02x)...\n", ubx_mon_hw_req[2], ubx_mon_hw_req[3]);
    //i2c_send_req(ubx_mon_hw_req, sizeof(ubx_mon_hw_req));
    //if (0 > i2c_read_req())
    //{
    //	printf("ubx_mon_hw_req failed\n");
    //	return -1;
    //}

    /* set data format/protocol of serial port to UBX */
    printf(">>> send ubx_cfg_prt_ubx_i2c_set \"Set port to I2C\" (%02x %02x)...\n", ubx_cfg_prt_ubx_i2c_set[2], ubx_cfg_prt_ubx_i2c_set[3]);
    i2c_send_req(ubx_cfg_prt_ubx_i2c_set, sizeof(ubx_cfg_prt_ubx_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_prt_i2c_set_ubx failed\n");
        return -1;
    }

	/* request UBX port configuration */
	printf(">>> send ubx_cfg_prt_i2c_req (%02x %02x)...\n", ubx_cfg_prt_i2c_req[2], ubx_cfg_prt_i2c_req[3]);
	i2c_send_req(ubx_cfg_prt_i2c_req, sizeof(ubx_cfg_prt_i2c_req));
	if (0 > i2c_read_req())
	{
		printf("ubx_cfg_prt_i2c_req failed\n");
		return -1;
	}

#if 0 /* not necessary */
	/* request positioning */
	printf(">>> send ubx_nav_pvt_req (%02x %02x)...\n", ubx_nav_pvt_req[2], ubx_nav_pvt_req[3]);
	i2c_send_req(ubx_nav_pvt_req, sizeof(ubx_nav_pvt_req));
	if (0 > i2c_read_req())
	{
		printf("ubx_nav_pvt_req failed\n");
		return -1;
	}
	if (0 > i2c_read_req())
	{
		printf("ubx_nav_pvt_req failed\n");
		return -1;
	}
#endif

    /* set pps  */
    printf(">>> send ubx_cfg_set_pps_tp5 (%02x %02x)...\n", ubx_cfg_set_pps_tp5[2], ubx_cfg_set_pps_tp5[3]);
    i2c_send_req(ubx_cfg_set_pps_tp5, sizeof(ubx_cfg_set_pps_tp5));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_set_pps_tp5 failed\n");
        return -1;
    }

    /* Automatic INU allignment -> IMU-mount misalignment configuration */
    printf(">>> send ubx_cfg_esf_alg_set \"IMU-mount misalignment configuration\" (%02x %02x)...\n", ubx_cfg_esf_alg_set[2], ubx_cfg_esf_alg_set[3]);
    i2c_send_req(ubx_cfg_esf_alg_set, sizeof(ubx_cfg_esf_alg_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_esf_alg_set failed\n");
        return -1;
    }
    /* set measurement rate */
    printf(">>> send ubx_cfg_rate_set_1000 (%02x %02x)...\n", ubx_cfg_rate_set_1000[2], ubx_cfg_rate_set_1000[3]);
    i2c_send_req(ubx_cfg_rate_set_1000, sizeof(ubx_cfg_rate_set_1000));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_rate_set_1000 failed\n");
        return -1;
    }

    /* set periodic pvt data ind -> Navigation position velocity time solution */
    printf(">>> send ubx_cfg_prt_pvt_i2c_set \"Navigation position velocity time solution\" (%02x %02x)...\n", ubx_cfg_prt_pvt_i2c_set[2], ubx_cfg_prt_pvt_i2c_set[3]);
    i2c_send_req(ubx_cfg_prt_pvt_i2c_set, sizeof(ubx_cfg_prt_pvt_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_prt_pvt_i2c_set failed\n");
        return -1;
    }

    /* set periodic rmx_rawx data ind -> Multi-GNSS raw measurement data*/
    printf(">>> send ubx_cfg_rxm_rawx_i2c_set \"Multi-GNSS raw measurement data\" (%02x %02x)...\n", ubx_cfg_rxm_rawx_i2c_set[2], ubx_cfg_rxm_rawx_i2c_set[3]);
    i2c_send_req(ubx_cfg_rxm_rawx_i2c_set, sizeof(ubx_cfg_rxm_rawx_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_rxm_rawx_i2c_set failed\n");
        return -1;
    }

    /* set periodic rmx_measx data ind -> Satellite measurements for RRLP */
    printf(">>> send ubx_cfg_rxm_measx_i2c_set \"Satellite measurements for RRLP\" (%02x %02x)...\n", ubx_cfg_rxm_measx_i2c_set[2], ubx_cfg_rxm_measx_i2c_set[3]);
    i2c_send_req(ubx_cfg_rxm_measx_i2c_set, sizeof(ubx_cfg_rxm_measx_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_rxm_measx_i2c_set failed\n");
        return -1;
    }

#if 0 /* to be checked */
	/* set periodic ESF_RAW data ind -> Raw sensor measurements */
	printf(">>> send ubx_cfg_esf_raw_i2c_set \"Raw sensor measurements\" (%02x %02x)...\n", ubx_cfg_esf_raw_i2c_set[2], ubx_cfg_esf_raw_i2c_set[3]);
	i2c_send_req(ubx_cfg_esf_raw_i2c_set, sizeof(ubx_cfg_esf_raw_i2c_set));
	if (0 > i2c_read_req())
	{
		printf("ubx_cfg_esf_raw_i2c_set failed\n");
		return -1;
	}

	/* set periodic ESF-MEAS data ind -> External sensor fusion measurements */
	printf(">>> send ubx_cfg_esf_meas_i2c_set \"External sensor fusion measurements\" (%02x %02x)...\n", ubx_cfg_esf_meas_i2c_set[2], ubx_cfg_esf_meas_i2c_set[3]);
	i2c_send_req(ubx_cfg_esf_meas_i2c_set, sizeof(ubx_cfg_esf_meas_i2c_set));
	if (0 > i2c_read_req())
	{
		printf("ubx_cfg_esf_meas_i2c_set failed\n");
		return -1;
	}
#endif

    /* set periodic ESF-STATUS data ind -> External sensor fusion status */
    printf(">>> send ubx_cfg_esf_status_i2c_set \"External sensor fusion status\" (%02x %02x)...\n", ubx_cfg_esf_status_i2c_set[2], ubx_cfg_esf_status_i2c_set[3]);
    i2c_send_req(ubx_cfg_esf_status_i2c_set, sizeof(ubx_cfg_esf_status_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_esf_status_i2c_set failed\n");
        return -1;
    }

    printf("\n------------ init completed ------------\n\n");

    return 1;
}

void ubx_start_esf_meas(void)
{
    /* set periodic ESF-MEAS data ind -> External sensor fusion measurements */
    printf(">>> send ubx_cfg_esf_meas_i2c_set \"External sensor fusion measurements\" (%02x %02x)...\n", ubx_cfg_esf_meas_i2c_set[2], ubx_cfg_esf_meas_i2c_set[3]);
    i2c_send_req(ubx_cfg_esf_meas_i2c_set, sizeof(ubx_cfg_esf_meas_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_esf_meas_i2c_set failed\n");
    }
}

void ubx_stop_esf_meas(void)
{
    /* set periodic ESF-MEAS data ind -> External sensor fusion measurements */
    printf(">>> send ubx_cfg_esf_meas_i2c_stop \"External sensor fusion measurements\" (%02x %02x)...\n", ubx_cfg_esf_meas_i2c_stop[2], ubx_cfg_esf_meas_i2c_stop[3]);
    i2c_send_req(ubx_cfg_esf_meas_i2c_stop, sizeof(ubx_cfg_esf_meas_i2c_stop));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_esf_meas_i2c_stop failed\n");
    }
}

void ubx_start_esf_raw(void)
{
    /* set periodic ESF_RAW data ind -> Raw sensor measurements */
    printf(">>> send ubx_cfg_esf_raw_i2c_set \"Raw sensor measurements\" (%02x %02x)...\n", ubx_cfg_esf_raw_i2c_set[2], ubx_cfg_esf_raw_i2c_set[3]);
    i2c_send_req(ubx_cfg_esf_raw_i2c_set, sizeof(ubx_cfg_esf_raw_i2c_set));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_esf_raw_i2c_set failed\n");
    }
}

void ubx_stop_esf_raw(void)
{
    /* set periodic ESF_RAW data ind -> Raw sensor measurements */
    printf(">>> send ubx_cfg_esf_raw_i2c_stop \"Raw sensor measurements\" (%02x %02x)...\n", ubx_cfg_esf_raw_i2c_stop[2], ubx_cfg_esf_raw_i2c_stop[3]);
    i2c_send_req(ubx_cfg_esf_raw_i2c_stop, sizeof(ubx_cfg_esf_raw_i2c_stop));
    if (0 > i2c_read_req()) {
        printf("ubx_cfg_esf_raw_i2c_stop failed\n");
    }
}

void ubx_decode_mon_ver(char* buf, uint len)
{
    uint i = 40;

    printf("sw:%s, hw: %s\n", &buf[0], &buf[20]);

    while (i <= len) {
        printf("%s, ", &buf[i]);
        i += 30;
    }
    printf("\n");
}

void ubx_dedode_class_mon_msg(char* buf, uint len)
{
    switch (buf[3]) //ID
    {
    case UBX_MON_VER:
        debug("----Version received\n");
        ubx_decode_mon_ver(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    default:
        printf("----unexpected ID %02x received\n", buf[3]);
        break;
    }
}

void ubx_dedode_class_cfg_msg(char* buf, uint len)
{
    switch (buf[3]) //ID
    {
    case UBX_CFG_PRT:
        debug("----Config PRT received\n");
        break;
    default:
        printf("----unexpected ID %02x received\n", buf[3]);
        break;
    }
}

void ubx_dedode_class_inf_msg(char* buf, uint len)
{
    switch (buf[3]) //ID
    {
    case UBX_INF_DEBUG:
        debug("----Debug (len: %d) received: %s\n", len - UBX_HEADER_LEN - UBX_CRC_LEN, &buf[UBX_HEADER_LEN]);
        break;
    case UBX_INF_NOTICE:
        debug("----Notice (len: %d) received: %s\n", len - UBX_HEADER_LEN - UBX_CRC_LEN, &buf[UBX_HEADER_LEN]);
        break;
    default:
        printf("----unexpected ID %02x received\n", buf[3]);
        break;
    }
}

void ubx_decode_nav_pvt(char* nav, uint len)
{
    uint32_t iTow = (nav[3] << 24) | (nav[2] << 16) | (nav[1] << 8) | nav[0];
    uint16_t year = (nav[5] << 8) | nav[4];
    uint8_t month = nav[6];
    uint8_t day = nav[7];
    uint8_t hour = nav[8];
    uint8_t min = nav[9];
    uint8_t sec = nav[10];
    uint8_t valid = nav[11];
    uint8_t fixType = nav[20];
    // ...
    uint8_t numSV = nav[23];
    double lon = (((nav[27] << 24) | (nav[26] << 16) | (nav[25] << 8) | nav[24])) / 10000000.0;
    double lat = (((nav[31] << 24) | (nav[30] << 16) | (nav[29] << 8) | nav[28])) / 10000000.0;
    double high = (((nav[35] << 24) | (nav[34] << 16) | (nav[33] << 8) | nav[32])) / 1000.0;
    double gSpeed = ((nav[63] << 24) | (nav[62] << 16) | (nav[61] << 8) | nav[60]) * 0.036;
    char* fixTypeStr;

    switch (fixType) {
    case 0:
        fixTypeStr = "no fix";
        break;
    case 1:
        fixTypeStr = "dead reckoning";
        break;
    case 2:
        fixTypeStr = "2D-fix";
        break;
    case 3:
        fixTypeStr = "3D-fix";
        break;
    case 4:
        fixTypeStr = "GNSS + dead reckoning combinatio";
        break;
    case 5:
        fixTypeStr = "time only fix";
        break;
    default:
        fixTypeStr = "unexpected fix type";
        break;
    }

    char* validData = (valid & 0x01) ? "date" : "";
    char* validTime = (valid & 0x02) ? "time" : "";
    char* validFull = (valid & 0x04) ? "full" : "";
    char* validMagn = (valid & 0x08) ? "magn" : "";
    char validStr[21];
    sprintf(validStr, "%s|%s|%s|%s", validData, validTime, validFull, validMagn);

    printf("------NAV-PVT: TOW: %d [ms], %d-%d-%d %02d:%02d:%02d valid: %s (%02x), fixType: %s (%02x), numSV: %d, lon: %f, lat: %f, high: %f, speed: %f [km/h]\n",
        iTow, year, month, day, hour, min, sec, validStr, valid, fixTypeStr, fixType, numSV, lon, lat, high, gSpeed);
}

void ubx_dedode_class_nav_msg(char* buf, uint len)
{
    switch (buf[3]) //ID
    {
    case UBX_NAV_PVT:
        debug("----PVT Navigation data received\n");
        ubx_decode_nav_pvt(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    default:
        printf("----PVT unexpected ID %02x received\n", buf[3]);
        break;
    }
}

void ubx_decode_rxm_rawx(char* rxm, uint len)
{
    //uint64_t rcvTow =
    uint16_t week = (rxm[9] << 8) | rxm[8];
    uint8_t numMeas = rxm[11];
    // ...

    printf("------RXM-RAWX: week: %d, numMess %d\n", week, numMeas);

    printf("      gnssId:  ");
    for (int i = 0; i < numMeas; i++) {
        printf("%d, \t", rxm[36 + 32 * i]);
    }
    printf("\n      satId:   ");
    for (int i = 0; i < numMeas; i++) {
        printf("%d, \t", rxm[37 + 32 * i]);
    }
    printf("\n      sigId:   ");
    for (int i = 0; i < numMeas; i++) {
        printf("%d, \t", rxm[38 + 32 * i]);
    }
    printf("\n      freqId:  ");
    for (int i = 0; i < numMeas; i++) {
        printf("%d, \t", rxm[39 + 32 * i]);
    }
    printf("\n");
}

void ubx_decode_rxm_measx(char* rxm, uint len)
{
    //uint64_t rcvTow =
    uint8_t numSV = rxm[34];
    uint8_t flags = rxm[35];
    // ...

    printf("------RXM-MEASX: numSV: %d, flags: %s (%02x)\n", numSV, ((flags & 0x03) > 0) ? "towSet" : "towNotSet", flags);

    printf("      gnssId: ");
    for (int i = 0; i < numSV; i++) {
        printf("%d,\t", rxm[44 + 24 * i]);
    }
    printf("\n      satId:  ");
    for (int i = 0; i < numSV; i++) {
        printf("%d,\t", rxm[45 + 24 * i]);
    }
    printf("\n");
}

void ubx_dedode_class_rxm_msg(char* buf, uint len)
{
    switch (buf[3]) //ID
    {
    case UBX_RXM_RAWX:
        debug("----RXM-RAWX data received\n");
        ubx_decode_rxm_rawx(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    case UBX_RXM_MEASX:
        debug("----RXM-MEASX data received\n");
        ubx_decode_rxm_measx(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    default:
        printf("----RXM unexpected ID %02x received\n", buf[3]);
        break;
    }
}

void ubx_decode_esf_meas(char* esf, uint len)
{
    uint8_t numMeas = (esf[5] >> 3) & 0x1F;
    uint8_t dataType;
    double dataField;

    printf("------ESF-MEAS: id: %d, flags: %02x, numMeas: %d\n", (esf[7] << 8) | esf[6], esf[4] & 0x0F, numMeas);
    for (int i = 0; i < numMeas; i++) {
        dataType = esf[8 + 3 + 4 * i] & 0x3F;
        dataField = ((esf[8 + 4 * i] << 16) | (esf[8 + 1 + 4 * i] << 8) | esf[8 + 2 + 4 * i]) / 10000000.0;
        printf("      sensor%d: %s (%d), %f\n", i, ubx_get_sensor_type(dataType), dataType, dataField);
    }
}

char* ubx_get_sensor_type(uint8_t type)
{
    char* typeStr;
    switch (type) {
    case 0:
        typeStr = "no data";
        break;
    case 5:
        typeStr = "z-axis gyro";
        break;
    case 6:
        typeStr = "FL wheel ticks";
        break;
    case 7:
        typeStr = "FR wheel ticks";
        break;
    case 8:
        typeStr = "RL wheel ticks";
        break;
    case 9:
        typeStr = "RR wheel ticks";
        break;
    case 10:
        typeStr = "single (speed) tick";
        break;
    case 11:
        typeStr = "speed";
        break;
    case 12:
        typeStr = "gyro - termperatue";
        break;
    case 13:
        typeStr = "y-axis gyro";
        break;
    case 14:
        typeStr = "x-axis gyro";
        break;
    case 16:
        typeStr = "x-axis acc";
        break;
    case 17:
        typeStr = "y-axis acc";
        break;
    case 18:
        typeStr = "z-axis acc";
        break;
    default:
        typeStr = "unknown type";
    }

    return typeStr;
}

char* ubx_get_calibration_status(uint8_t calib)
{
    switch (calib) {
    case 0:
        return "not calibrated";
    case 1:
        return "calibrating";
    case 2:
    case 3:
    default:
        return "calibrated";
    }
}

void ubx_decode_esf_status(char* esf, uint len)
{
    uint8_t fusionMode = esf[12];
    uint8_t numSens = esf[15];
    char* mode;
    uint8_t dataType;
    char* used;
    char* ready;
    uint8_t freq;
    uint8_t fault;
    uint8_t status1;
    uint8_t status2;

    switch (fusionMode) {
    case 0:
        mode = "initialisation mode";
        sensor_fusion_active = 0;
        break;
    case 1:
        mode = "fusionmode";
        sensor_fusion_active = 1;
        break;
    case 2:
        mode = "disabled fusion mode";
        sensor_fusion_active = 0;
        break;
    default:
        mode = "unknown mode";
        break;
    }

    printf("------ESF-STATUS: fusionMode: %s (%02x), numSens: %d\n", mode, fusionMode, numSens);

    for (int i = 0; i < numSens; i++) {
        dataType = esf[16 + 4 * i] & 0x3F;
        ready = (esf[16 + 4 * i] & 0x80) ? "ready" : "notReady";
        used = (esf[16 + 4 * i] & 0x40) ? "used" : "notUsed";
        freq = esf[18 + 4 * i];
        fault = esf[19 + i * 4];
        status1 = esf[16 + i * 4];
        status2 = esf[17 + i * 4];

        //if ((esf[16 + 4 * i] & 0x80) && (esf[16 + 4 * i] & 0x40))
        //{
        //	sensor_fusion_active = 1;
        //}

        printf("      ESF-STATUS: sensor%d: sensStatus1: %s %s %s (%02x), senseStatus2: %s (%02x), freq: %d, faults: %02x\n",
            i, ubx_get_sensor_type(dataType), ready, used, status1,
            ubx_get_calibration_status(status2 & 0x03), status2, freq, fault);
    }
}

void ubx_decode_esf_raw(char* esf, uint len)
{
    int i = 0;
    uint8_t dataType;
    double dataField;

    printf("------ESF-RAW: len=%d\n", len);
    // data: bit24-31: DataType, bit 0-23: DataField

    while ((8 + 8 * i) < len) {
        dataType = esf[4 + 3 + 8 * i];
        dataField = ((esf[4 + 8 * i] << 16) | (esf[4 + 1 + 8 * i] << 8) | esf[4 + 2 + 8 * i]) / 10000000.0;
        printf("      dataType: %s (%02x), dataField: %f\n", ubx_get_sensor_type(dataType), dataType, dataField);
        i++;
    }
    //printf("\n");
}

void ubx_dedode_class_esf_msg(char* buf, uint len)
{
    switch (buf[3]) //ID
    {
    case UBX_ESF_MEAS:
        debug("----ESF-MEAS data received\n");
        ubx_decode_esf_meas(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    case UBX_ESF_RAW:
        debug("----ESF-RAW data received\n");
        ubx_decode_esf_raw(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    case UBX_ESF_STATUS:
        debug("----ESF-STATUS data received\n");
        ubx_decode_esf_status(&buf[UBX_HEADER_LEN], len - UBX_HEADER_LEN - UBX_CRC_LEN);
        break;
    default:
        printf("----ESF unexpected ID %02x received\n", buf[3]);
        break;
    }
}

uint ubx_parse_ubx_data(char* buf, uint len)
{
    if ((NULL != buf) && (0 < len)) {
        switch (buf[2]) //class
        {
        case MSG_CLASS_NAV:
            debug("--Navigation message received\n");
            ubx_dedode_class_nav_msg(buf, len);
            break;
        case MSG_CLASS_MON:
            debug("--Monitoring message received\n");
            ubx_dedode_class_mon_msg(buf, len);
            break;
        case MSG_CLASS_CFG:
            debug("--Config message received\n");
            ubx_dedode_class_cfg_msg(buf, len);
            break;
        case MSG_CLASS_INF:
            debug("--Info message received\n");
            ubx_dedode_class_inf_msg(buf, len);
            break;
        case MSG_CLASS_RXM:
            debug("--RXM message received\n");
            ubx_dedode_class_rxm_msg(buf, len);
            break;
        case MSG_CLASS_ESF:
            debug("--ESF message received\n");
            ubx_dedode_class_esf_msg(buf, len);
            break;
        case MSG_CLASS_ACK:
            debug("--ACK message %02x %02x received (%s)\n", buf[6], buf[7], (buf[3] == 1) ? "OK" : "NOK");
            break;
        default:
            printf("--unexpected message received: Class=%02x, Id=%02x\n", buf[2], buf[3]);
        }
    }

    //consume the whole buffer
    return len;
}

void ubx_clear(void)
{
    close(m8l_fd);
}

/*********************************************************
 * 					Signal Handler
 * *******************************************************/
void sigterm_handler(int signal, siginfo_t* info, void* _unused)
{
    printf("\n\nReceived signo = %d sigcode = %d  signal from process with pid = %u\n\n",
        info->si_signo, info->si_code, info->si_pid);

    ubx_clear();

    exit(0);
}

/*********************************************************
 * 							main
 * *******************************************************/
int main(int argc, char* argv[])
{
    printf("%s >>\n", __FUNCTION__);

    struct sigaction action;

    action.sa_handler = NULL;
    action.sa_sigaction = sigterm_handler;
    // action.sa_mask = 0;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO;
    action.sa_restorer = NULL;

    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGKILL, &action, NULL);
    sigaction(SIGINT, &action, NULL); // Ctrl+c

    uint8_t i2c_device = I2C_DEVICE;
    uint8_t wait_for_msg = 0;

    if (1 < argc) {
        i2c_device = atoi(argv[1]);
    }

    if (0 > (i2c_init(i2c_device))) {
        printf("%d: Failed to initialize I2C bus\n", __LINE__);
        ubx_clear();
        printf("%s <<\n", __FUNCTION__);
        return -1;
    }

    if (0 > (ubx_init())) {
        printf("%d: Failed to initialize UBX module\n", __LINE__);
        ubx_clear();
        printf("%s <<\n", __FUNCTION__);
        return -1;
    }

    while (1) {
        while (0 < i2c_read_req()) {
            //read until no more data available
            wait_for_msg = 0;
        }

        if (UBX_WAIT_FOR_MSG > wait_for_msg) {
            wait_for_msg++;
            printf("\n");
            sleep(1);
        } else {
            // communication to GNSS receiver broken
            printf("COMMUNICATION TO GNSS RECEIVER BROKEN\n");
            ubx_init();
            usleep(250000);
            wait_for_msg = 0;
        }

        /* activate/deactivate sensor notification dependent on sensor status*/
        if (sensor_fusion_active && !esf_active) {
            ubx_start_esf_meas();
            //ubx_start_esf_raw();
        } else if (!sensor_fusion_active && esf_active) {
            ubx_stop_esf_meas();
            //ubx_stop_esf_raw();
        }
    }

    ubx_clear();

    return 1;
}
