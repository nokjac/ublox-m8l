/* ubx message classes */
int ubx_init(void);
void ubx_dedode_class_mon_msg(char* buf, uint len);
void ubx_dedode_class_cfg_msg(char* buf, uint len);
void ubx_dedode_class_nav_msg(char* buf, uint len);
void ubx_dedode_class_inf_msg(char* buf, uint len);
void ubx_dedode_class_rxm_msg(char* buf, uint len);
void ubx_dedode_class_esf_msg(char* buf, uint len);

void ubx_decode_rxm_rawx(char* rmx, uint len);
void ubx_decode_rxm_measx(char* rmx, uint len);
void ubx_decode_esf_meas(char* buf, uint len);
void ubx_decode_esf_status(char* buf, uint len);
void ubx_decode_esf_raw(char* buf, uint len);
void ubx_decode_esf_status(char* esf, uint len);
void ubx_decode_esf_status(char* esf, uint len);
void ubx_decode_mon_ver(char* buf, uint len);
void ubx_decode_nav_pvt(char* nav, uint len);

void ubx_start_esf_meas(void);
void ubx_stop_esf_meas(void);
void ubx_start_esf_raw(void);
void ubx_stop_esf_raw(void);

uint ubx_parse_ubx_data(char* buf, uint len);
void ubx_clear(void);





#define MSG_CLASS_NAV 0x01 /* Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used */
#define MSG_CLASS_RXM 0x02 /* Receiver Manager Messages: Satellite Status, RTC Status */
#define MSG_CLASS_INF 0x04 /* Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice */
#define MSG_CLASS_ACK 0x05 /* Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages */
#define MSG_CLASS_CFG 0x06 /* Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc. */
#define MSG_CLASS_UPD 0x09 /* Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc. */
#define MSG_CLASS_MON 0x0A /* Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status */
#define MSG_CLASS_AID 0x0B /* AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input */
#define MSG_CLASS_TIM 0x0D /* Timing Messages: Time Pulse Output, Time Mark Results */
#define MSG_CLASS_ESF 0x10 /* External Sensor Fusion Messages: External Sensor Measurements and Status Information */
#define MSG_CLASS_MGA 0x13 /* Multiple GNSS Assistance Messages: Assistance data for various GNSS */
#define MSG_CLASS_LOG 0x21 /* Logging Messages: Log creation, deletion, info and retrieval */
#define MSG_CLASS_SEC 0x27 /* Security Feature Messages */
#define MSG_CLASS_HNR 0x28 /* High Rate Navigation Results Messages: High rate time, position, speed, heading */

/* supported nav messages */
#define UBX_NAV_PVT 0x07
#define UXB_NAV_SAT 0x35
#define UBX_NAV_ATT 0x05
#define UBX_NAV_SBAS 0x32
#define UBX_NAV_DOP 0x04

/* supported cfg messages */
#define UBX_CFG_PRT 0x00
#define UBX_CFG_RATE 0x08
#define UBX_CFG_INF 0x02
#define UBX_CFG_NAVX5 0x23

/* supported mon message */
#define UBX_MON_VER 0x04
#define UBX_MON_HW  0x09

/* supported esf messages */
#define UBX_ESF_MEAS   0x02
#define UBX_ESF_STATUS 0x10
#define UBX_ESF_RAW    0x03

/* supported inf message */
#define UBX_INF_DEBUG 0x04
#define UBX_INF_ERROR 0x00
#define UBX_INF_NOTICE 0x02
#define UBX_INF_TEST  0x03
#define UBX_INF_WARNING 0x01

/* supported RXM messages */
#define UBX_RXM_MEASX 0x14
#define UBX_RXM_RAWX  0x15

/* supported HNR message */
#define UBX_HNR_PVT  0x00

/*                                 Header   Class Id    Payloadlen  payload CRC (last 2 bytes) */
const char ubx_cfg_rst_req[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
								0x00, 0x00, 0x01, 0x00, /* payload */
								0x0F, 0x66};			/* CRC*/
const char ubx_cfg_prt_ubx_i2c_set[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
										0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
										0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
										0xA0, 0x96};
const char ubx_cfg_prt_i2c_req[] = {0xB5, 0x62, 0x06, 0x00, 0x01, 0x00,
									0x01,
									0x08, 0x22};
const char ubx_cfg_prt_pvt_i2c_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										0x01, 0x07 /*NAV-PVT*/, 0x01 /*I2c rate*/, 0x00, 0x00, 0x00, 0x00, 0x00 /*rate*/,
										0x18, 0xE2};
const char ubx_cfg_rxm_rawx_i2c_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										 0x02, 0x15 /*RMX-RAWX*/, 0x01 /*I2c rate*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										 0x27, 0x4C};
const char ubx_cfg_rxm_measx_i2c_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										  0x02, 0x14 /*RMX-MEASX*/, 0x01 /*I2c rate*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										  0x26, 0x45};
const char ubx_cfg_esf_raw_i2c_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										0x10, 0x03 /* ESF-RAW*/, 0x01 /*I2c rate * 100ms*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										0x23, 0x3e};
const char ubx_cfg_esf_raw_i2c_stop[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										 0x10, 0x03 /* ESF-RAW*/, 0x00 /*I2c rate*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										 0x22, 0x38};
const char ubx_cfg_esf_meas_i2c_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										 0x10, 0x02 /*ESF-MEAS*/, 0x05 /*I2c rate * 100ms*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										 0x26, 0x4F};
const char ubx_cfg_esf_meas_i2c_stop[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										 0x10, 0x02 /*ESF-MEAS*/, 0x00 /*I2c rate*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										 0x21, 0x31};
const char ubx_cfg_esf_status_i2c_set[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
										   0x10, 0x10 /* ESF-STATUS*/, 0x01 /*I2c rate*/, 0x00, 0x00, 0x00, 0x00, 0x00,
										   0x30, 0x99};
const char ubx_cfg_esf_alg_set[] = {0xB5, 0x62, 0x06, 0x56, 0x0C, 0x00,
									0x00, 0x01 /* IMU auto allignment*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x69, 0x1D};
const char ubx_cfg_rate_set_1000[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
									  0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
									  0x01, 0x39};
const char ubx_cfg_rate_set_2000[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
									  0xD0, 0x07, 0x01, 0x00, 0x01, 0x00,
									  0xED, 0xBD};
const char ubx_cfg_set_pps_tp5[] = {0xB5, 0x62, 0x06, 0x31, 0x20, 0x00,
									0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x80, 0x9A, 0x99, 0x99, 0x19, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00,
									0x2E, 0xF8};
const char ubx_mon_hw_req[] = {0xB5, 0x62, 0x0A, 0x09, 0x00, 0x00,
							   0x13, 0x43};
const char ubx_mon_ver_req[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00,
								0x0E, 0x34};
const char ubx_nav_pvt_req[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00,
								0x08, 0x19};