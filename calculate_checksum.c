#include <stdio.h>
#include <inttypes.h>

void main(void)
{
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	const char data[] =   { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 
                                                    0x10, 0x03, 0x00, 0x05 /*UART *100ms*/, 0x00, 0x00, 0x00, 0x00, 
                                                    0x23, 0x3d};

	for (int i = 0; i < sizeof(data) - 4; i++)
	{
		ck_a = ck_a + data[i+2];
		ck_b = ck_b + ck_a;
	}

	printf("ck_a: %02x, ck_b: %02x [%02x %02x]\n", ck_a, ck_b, ck_a, ck_b);
}