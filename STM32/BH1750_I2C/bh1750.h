/*
 * bh1750.h
 *
 *  Created on: Aug 31, 2021
 *      Author: Krzysztof Półchłopek
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "main.h"


/* BH1750 I2C ADDRESSES */
//#define BH1750_I2C_ADDR				0x46		// uncomment for addr 0x23 (ADDR terminal connected to GND)
#define BH1750_I2C_ADDR				0xB8		// uncomment for addr 0x5C (ADDR terminal connected to VCC)

/* BH1750 INSTRUCTION SET */
#define BH1750_POWER_DOWN				0
#define BH1750_POWER_ON					1
#define BH1750_RESET					7			// reset data register value (works only in power on state)
// Continuous measurements
#define BH1750_CONT_H_RES_MODE			16			// start measurement at 1lx resolution (typical 120ms measurement time)
#define BH1750_CONT_H_RES2_MODE			17			// start measurement at 0.5lx resolution (typical 120ms measurement time)
#define BH1750_CONT_L_RES_MODE			19			// start measurement at 4lx resolution (typical 16ms measurement time)
// One time measurements (automatic power down after measurement)
#define BH1750_ONE_H_RES_MODE			32			// start measurement at 1lx resolution (typical 120ms measurement time)
#define BH1750_ONE_H_RES2_MODE			33			// start measurement at 0.5lx resolution (typical 120ms measurement time)
#define BH1750_ONE_L_RES_MODE			35			// start measurement at 4lx resolution (typical 16ms measurement time)
// Changing measurement time
#define BH1750_MEAS_TIME_MSB			64			// bits [2:0] are for 3 msb bits for measurements time
#define BH1750_MEAS_TIME_LSB			96			// bits [4:0] are for 5 lsb bits for measurements time

typedef struct {
	I2C_HandleTypeDef *i2c;
	uint8_t work_mode;
	float lx;
}bh1750_t;

void bh1750_init(bh1750_t *bh);
void bh1750_ReadMeasurement(bh1750_t *bh);

HAL_StatusTypeDef bh1750_Write(bh1750_t *bh, uint8_t data);
uint16_t bh1750_Read(bh1750_t *bh);

#endif /* INC_BH1750_H_ */
