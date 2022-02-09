/*
 * bme280.h
 *
 *  Created on: 25.08.2021
 *      Author: Krzysztof Półchłopek
 *
 *      Temperature measurement should be enabled if you want to compensated data.
 *
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"
#include "gpio.h"

#define MODE_I2C_NSPI 					1		// Set communication mode: 1 for I2C, 0 for SPI

/* BME280 I2C ADDRESSES */
#define BME280_I2C_ADDR					0xEC	// uncomment for addr 0x76
// #define BME280_I2C_ADDR				0xEE		// uncomment for addr 0x77

/* BME280 SETTINGS */
// temperature measurements
#define BME280_TEMPERATURE_DISABLED		0		// disables temperature measurements
#define BME280_TEMPERATURE_OVER_1		1		// set temperature measurements oversampling to x1
#define BME280_TEMPERATURE_OVER_2		2		// set temperature measurements oversampling to x2
#define BME280_TEMPERATURE_OVER_4		3		// set temperature measurements oversampling to x4
#define BME280_TEMPERATURE_OVER_8		4		// set temperature measurements oversampling to x8
#define BME280_TEMPERATURE_OVER_16		5		// set temperature measurements oversampling to x16

// humidity measurements
#define BME280_HUMIDITY_DISABLED		0		// disables humidity measurements
#define BME280_HUMIDITY_OVER_1			1		// set humidity measurements oversampling to x1
#define BME280_HUMIDITY_OVER_2			2		// set humidity measurements oversampling to x2
#define BME280_HUMIDITY_OVER_4			3		// set humidity measurements oversampling to x4
#define BME280_HUMIDITY_OVER_8			4		// set humidity measurements oversampling to x8
#define BME280_HUMIDITY_OVER_16			5		// set humidity measurements oversampling to x16

// pressure measurements
#define BME280_PRESSURE_DISABLED		0		// disables pressure measurements
#define BME280_PRESSURE_OVER_1			1		// set pressure measurements oversampling to x1
#define BME280_PRESSURE_OVER_2			2		// set pressure measurements oversampling to x2
#define BME280_PRESSURE_OVER_4			3		// set pressure measurements oversampling to x4
#define BME280_PRESSURE_OVER_8			4		// set pressure measurements oversampling to x8
#define BME280_PRESSURE_OVER_16			5		// set pressure measurements oversampling to x16

// sensor working modes
#define BME280_SLEEP_MODE				0		// set sleep mode
#define BME280_FORCED_MODE				1		// set forced mode
#define BME280_NORMAL_MODE				3		// set normal mode

// standby time (in normal mode)
#define BME280_STANDBY_0_5				0		// set standby time to 0.5 ms
#define BME280_STANDBY_10				6		// set standby time to 10 ms
#define BME280_STANDBY_20				7		// set standby time to 20 ms
#define BME280_STANDBY_62_5				1		// set standby time to 62.5 ms
#define BME280_STANDBY_125				2		// set standby time to 125 ms
#define BME280_STANDBY_250				3		// set standby time to 250 ms
#define BME280_STANDBY_500				4		// set standby time to 500 ms
#define BME280_STANDBY_1000				5		// set standby time to 1000 ms

// filter settings (filter coefficients)
#define BME280_FILTER_OFF				0		// set filter off
#define BME280_FILTER_COEF_2			1		// set filter coefficients to 2
#define BME280_FILTER_COEF_4			2		// set filter coefficients to 4
#define BME280_FILTER_COEF_8			3		// set filter coefficients to 8
#define BME280_FILTER_COEF_16			4		// set filter coefficients to 16

/* BME280 INTERNAL MEMORY ADDRESSES */
// pressure raw data addresses [19:0]
#define	BME280_PRESS_MSB_ADDR			0xF7	// address for msb of pressure [19:12]
#define	BME280_PRESS_LSB_ADDR			0xF8	// address for lsb of pressure [11:4]
#define	BME280_PRESS_XLSB_ADDR			0xF9	// address for xlsb of pressure [3:0] (bits 7 to 4)

// temperature raw data addresses [19:0]
#define	BME280_TEMP_MSB_ADDR			0xFA	// address for msb of temperature [19:12]
#define	BME280_TEMP_LSB_ADDR			0xFB	// address for lsb of temperature [11:4]
#define	BME280_TEMP_XLSB_ADDR			0xFC	// address for xlsb of temperature [3:0] (bits 7 to 4)

// humidity raw data addresses [15:0]
#define	BME280_HUM_MSB_ADDR				0xFD	// address for msb of humidity [15:8]
#define	BME280_HUM_LSB_ADDR				0xFE	// address for lsb of humidity [7:0]

// other addresses
#define BME280_CHIP_ID					0xD0	// stores chip id (0x60)
#define BME280_RESET_ADDR				0xE0	// address contains the soft reset world. Write 0xB6 for reset
#define BME280_CALIB_DATA_00_25			0x88	// stores calibration data (calib00 to calib25) (0x88 to 0xA1)
#define BME280_CALIB_DATA_26_41			0xE1	// stores calibration data (calib26 to calib41)	(0xE1 to 0xF0)
#define BME280_STATUS_ADDR				0xF3	// stores status of the device
#define BME280_CTRL_HUM_ADDR			0xF2	// stores configuration of humidity part of sensor
#define BME280_CTRL_MEAS_ADDR			0xF4	// stores configuration of temperature, pressure and mode part of sensor
#define BME280_CONFIG_ADDR				0xF5	// stores configuration of standby time, filter and SPI 3-wire mode

//calibration addresses
#define BME280_DIG_T1_ADDR				0x88	// stores compensation value for temperature
#define BME280_DIG_T2_ADDR				0x8A	// stores compensation value for temperature
#define BME280_DIG_T3_ADDR				0x8C	// stores compensation value for temperature
#define BME280_DIG_P1_ADDR				0x8E	// stores compensation value for pressure
#define BME280_DIG_P2_ADDR				0x90	// stores compensation value for pressure
#define BME280_DIG_P3_ADDR				0x92	// stores compensation value for pressure
#define BME280_DIG_P4_ADDR				0x94	// stores compensation value for pressure
#define BME280_DIG_P5_ADDR				0x96	// stores compensation value for pressure
#define BME280_DIG_P6_ADDR				0x98	// stores compensation value for pressure
#define BME280_DIG_P7_ADDR				0x9A	// stores compensation value for pressure
#define BME280_DIG_P8_ADDR				0x9C	// stores compensation value for pressure
#define BME280_DIG_P9_ADDR				0x9E	// stores compensation value for pressure
#define BME280_DIG_H1_ADDR				0xA1	// stores compensation value for humidity
#define BME280_DIG_H2_ADDR				0xE1	// stores compensation value for humidity
#define BME280_DIG_H3_ADDR				0xE3	// stores compensation value for humidity
#define BME280_DIG_H4_ADDR				0xE4	// stores compensation value for humidity
#define BME280_DIG_H5_ADDR				0xE5	// stores compensation value for humidity
#define BME280_DIG_H6_ADDR				0xE7	// stores compensation value for humidity

/* BME280 MEASURMENT IN PROGRESS */
#define BME280_MEAS_MASQ				(1<<3)	// set to 1 whenever conversion is running


/* BME280 STRUCTURE */
typedef struct {
	#if MODE_I2C_NSPI == 1
		// I2C communication
		I2C_HandleTypeDef *i2c;
	#else
		// SPI communication
		SPI_HandleTypeDef *spi;
		GPIO_TypeDef GPIO_port;
		uint16_t GPIO_pin
	#endif
	uint8_t temp_oversampling;
	uint8_t press_oversampling;
	uint8_t hum_oversampling;
	uint8_t work_mode;
	uint8_t standby_time;
	uint8_t filter;

// output data
	float temp;
	float hum;
	float press;
} bme280_t;

typedef struct {
	uint16_t t1;
	int16_t t2;
	int16_t t3;

	uint16_t p1;
	int16_t p2;
	int16_t p3;
	int16_t p4;
	int16_t p5;
	int16_t p6;
	int16_t p7;
	int16_t p8;
	int16_t p9;

	uint8_t h1;
	int16_t h2;
	uint8_t h3;
	int16_t h4;
	int16_t h5;
	int8_t h6;
}bme280_compensation_t;

/* BME280 FUNCTIONS */

void BME280_Init(bme280_t *bme);
uint8_t BME280_Callibrating(bme280_t *bme);

void BME280_ReadTemp(bme280_t *bme);
void BME280_ReadPress(bme280_t *bme);
void BME280_ReadHum(bme280_t *bme);
void BME280_ReadAll(bme280_t *bme);

/* BME280 FUNCTIONS FOR READING/WRITING DATA FROM/TO SENSOR */
void BME280_WriteByte(bme280_t *bme, uint8_t addr, uint8_t data);
void BME280_ReadMeasurements(bme280_t *bme, int32_t *raw_temp, int32_t *raw_hum, int32_t *raw_press);
uint8_t BME280_ReadByte(bme280_t *bme, uint8_t addr);
uint16_t BME280_Read2Bytes(bme280_t *bme, uint8_t addr);
uint16_t BME280_Read2BytesRV(bme280_t *bme, uint8_t addr);
uint32_t BME280_Read3Bytes(bme280_t *bme, uint8_t addr);

#endif /* INC_BME280_H_ */
