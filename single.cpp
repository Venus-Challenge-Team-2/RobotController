extern "C" {
#include <libpynq.h>
#include <iic.h>
}
#include "vl53l0x.h"
#include <cstdio>

vl53x sensor;

int init_distance(void) {
	uint8_t addr = 0x29;
	int i = tofPing(IIC0, addr);
	printf("Sensor Ping: ");
	if(i != 0)
	{
		printf("Fail\n");
		return 1;
	}
	printf("Success\n");

	i = tofInit(&sensor, IIC0, addr, 0);
	if (i != 0)
	{
		return -1;
	}

	uint8_t model, revision;
	printf("VL53L0X device successfully opened.\n");
	tofGetModel(&sensor, &model, &revision);
	printf("Model ID - %d\n", model);
	printf("Revision ID - %d\n", revision);
	fflush(nullptr);
	return 0;
} 

uint32_t read_distance(void) {
	uint32_t iDistance = tofReadDistance(&sensor);
	printf("Distance %dmm\n", iDistance);
	fflush(stdout);
	return iDistance;
}
