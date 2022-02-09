/*
 * bh1750.c
 *
 *  Created on: Aug 31, 2021
 *      Author: Krzysztof Półchłopek
 */

#include "bh1750.h"

void bh1750_init(bh1750_t *bh) {
// power on the device and reset it
	bh1750_Write(bh, BH1750_POWER_ON);
	bh1750_Write(bh, BH1750_RESET);

// check if work_mode is set to continuous measurements
	if (bh->work_mode <= BH1750_ONE_H_RES_MODE) {
		// if yes set measurement mode
		bh1750_Write(bh, bh->work_mode);
	}
}

void bh1750_ReadMeasurement(bh1750_t *bh) {
	if (bh->work_mode == BH1750_ONE_L_RES_MODE) {
		// set one shot low res measurement and wait 30 ms
		bh1750_Write(bh, BH1750_ONE_L_RES_MODE);
		HAL_Delay(30);
	} else if (bh->work_mode == BH1750_ONE_H_RES_MODE) {
		// set one shot high res measurement and wait 180 ms
		bh1750_Write(bh, BH1750_ONE_H_RES_MODE);
		HAL_Delay(180);
	} else if (bh->work_mode == BH1750_ONE_H_RES2_MODE) {
		// set one shot high res 2 measurement and wait 180 ms
		bh1750_Write(bh, BH1750_ONE_H_RES2_MODE);
		HAL_Delay(180);
	}

	uint16_t temp = bh1750_Read(bh);
	bh->lx = (temp / 1.2f);
	return;
}


HAL_StatusTypeDef bh1750_Write(bh1750_t *bh, uint8_t data) {
	return HAL_I2C_Master_Transmit(bh->i2c, BH1750_I2C_ADDR, &data, 1, 100);
}

uint16_t bh1750_Read(bh1750_t *bh) {
	uint8_t temp[2];
	HAL_I2C_Master_Receive(bh->i2c, BH1750_I2C_ADDR, (uint8_t*)temp, 2, 100);
	return ((temp[0] << 8) | temp[1]);
}
