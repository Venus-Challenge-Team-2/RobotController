#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"
#include <stdio.h>

 /*
 *  TU/e 5EID0::LIBPYNQ Driver for VL53L0X TOF Sensor
 *  Example Code 
 * 
 *  Original: Larry Bank
 *  Adapted for PYNQ: Walthzer
 * 
 */

vl53x sensor;

int init_distance(void) {
	int i;
	uint8_t addr = 0x29;
	i = tofPing(IIC0, addr);
	printf("Sensor Ping: ");
	if(i != 0)
	{
		printf("Fail\n");
		return 1;
	}
	printf("Succes\n");

	//Initialize the sensor
	i = tofInit(&sensor, IIC0, addr, 0); // set default range mode (up to 800mm)
	if (i != 0)
	{
		return -1; // problem - quit
	}


	uint8_t model, revision;

	printf("VL53L0X device successfully opened.\n");
	tofGetModel(&sensor, &model, &revision);
	printf("Model ID - %d\n", model);
	printf("Revision ID - %d\n", revision);
	fflush(NULL); //Get some output even is distance readings hang
	return 0;
} 

uint32_t read_distance(void) {
	uint32_t iDistance = tofReadDistance(&sensor);
	printf("Distance %dmm", iDistance);
	return iDistance;
}
