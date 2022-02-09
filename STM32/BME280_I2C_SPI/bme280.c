/*
 * bme280.c
 *
 *  Created on: 25.08.2021
 *      Author: Krzysztof Półchłopek
 */
#include "bme280.h"

bme280_compensation_t comp_data;
// value for compensation
int32_t t_fine;


#if MODE_I2C_NSPI
	// I2C communication
	void BME280_WriteByte(bme280_t *bme, uint8_t addr, uint8_t data)
	{
		HAL_I2C_Mem_Write(bme->i2c, BME280_I2C_ADDR, addr, 1, &data, 1, 100);
	}

	uint8_t BME280_ReadByte(bme280_t *bme, uint8_t addr)
	{
		uint8_t tmp = 0;
		HAL_I2C_Mem_Read(bme->i2c, BME280_I2C_ADDR, addr, 1, &tmp, 1, 100);
		return tmp;
	}

	uint16_t BME280_Read2Bytes(bme280_t *bme, uint8_t addr)
	{
		uint8_t temp[2];
		HAL_I2C_Mem_Read(bme->i2c, BME280_I2C_ADDR, addr, 1, (uint8_t*)temp, 2, 100);
		return ((temp[0] << 8) | temp[1]);
	}

	uint32_t BME280_Read3Bytes(bme280_t *bme, uint8_t addr)
	{
		uint8_t temp[3];
		HAL_I2C_Mem_Read(bme->i2c, BME280_I2C_ADDR, addr, 1, (uint8_t*)temp, 3, 100);
		return ((temp[0] << 16) | (temp[1] << 8) | temp[2]);
	}
	void BME280_ReadMeasurements(bme280_t *bme, int32_t *raw_temp, int32_t *raw_hum, int32_t *raw_press)
	{
		uint8_t temp[8];
		HAL_I2C_Mem_Read(bme->i2c, BME280_I2C_ADDR, BME280_PRESS_MSB_ADDR, 1, (uint8_t*)temp, 8, 100);

		*raw_press = ((temp[0] << 16) | (temp[1] << 8) | temp[2]);
		*raw_temp = ((temp[3] << 16) | (temp[4] << 8) | temp[5]);
		*raw_hum = ((temp[6] << 8) | temp[7]);
		return;
	}
#else
	// SPI communication
	void BME_WriteByte(bme280_t *bme, uint8_t addr, uint8_t data)
	{
		uint8_t temp[2];
		temp[0] = address;
		temp[0] &= ~(1<<7); // reseting write bit
		temp[1] = data;
		HAL_GPIO_WritePin(bme->GPIO_port, bme->GPIO_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(bme->spi, temp, temp, 2, 10);
		HAL_GPIO_WritePin(bme->GPIO_port, bme->GPIO_pin, GPIO_PIN_SET);
	}

	uint8_t BME280_ReadByte(bme280_t *bme, uint8_t addr)
	{
		uint8_t temp[2];
		temp[0] = addr;
		temp[0] |= (1 << 7); // setting write bit
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(spi_h, temp, temp, 2, 10);
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
		return tmp[1];
	}

	uint16_t BME280_Read2Bytes(bme280_t *bme, uint8_t addr, uint8_t data)
	{
		uint8_t temp[3];
		temp[0] = addr;
		temp[0] |= (1 << 7); // setting write bit
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(spi_h, temp, temp, 3, 10);
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
		return ((temp[1] << 8) | temp[2]);
	}

	uint32_t BME280_Read3Bytes(bme280_t *bme, uint8_t addr)
	{
		uint8_t temp[3];
		temp[0] = addr;
		temp[0] |= (1 << 7); // setting write bit
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(spi_h, temp, temp, 3, 10);
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
		return ((temp[1] << 16) | (temp[2] << 8) | temp[3]);
	}

	void BME280_ReadMeasurements(bme280_t *bme, int32_t *raw_temp, int32_t *raw_hum, int32_t *raw_press)
	{
		uint8_t temp[9];

		temp[0] = BME280_PRESS_MSB_ADDR;
		temp[0] |= (1 << 7); // setting write bit
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(spi_h, temp, temp, 8, 10);
		HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);

		*raw_press = ((temp[1] << 16) | (temp[2] << 8) | temp[3]);
		*raw_temp = ((temp[4] << 16) | (temp[5] << 8) | temp[6]);
		*raw_hum = ((temp[7] << 8) | temp[8]);
		return;
	}
#endif

uint16_t BME280_Read2BytesRV(bme280_t *bme, uint8_t addr)
{
	uint16_t temp = BME280_Read2Bytes(bme, addr);
	return (temp >> 8 | temp << 8);
}


uint8_t BME280_Callibrating(bme280_t *bme)
{
	uint8_t status = BME280_ReadByte(bme,BME280_STATUS_ADDR);
	return ((status & 1 ) != 0) ? 1 : 0; // return 1 if sensor is calibrating else return 0
}



void BME280_Init(bme280_t *bme)
{
	// initialize values for temperature, humidity and pressure
	bme->temp = -100.0;
	bme->press = -100.0;
	bme->hum = -100.0;

	#if !MODE_I2C_NSPI
		// toggling board into SPI communication
		HAL_GPIO_WritePin(bme->GPIO_port, bme->GPIO_pin, GPIO_PIN_RESET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(bme->GPIO_port, bme->GPIO_pin, GPIO_PIN_SET);
	#endif

	// performing soft reset of the board
	BME280_WriteByte(bme, BME280_RESET_ADDR, 0xB6);
	HAL_Delay(50);

	// waiting for end of calibration
	while(BME280_Callibrating(bme))
	{
		HAL_Delay(10);
	}

	//reading calibration data from sensor
	comp_data.t1 = BME280_Read2BytesRV(bme,BME280_DIG_T1_ADDR);
	comp_data.t2 = BME280_Read2BytesRV(bme,BME280_DIG_T2_ADDR);
	comp_data.t3 = BME280_Read2BytesRV(bme,BME280_DIG_T3_ADDR);

	comp_data.p1 = BME280_Read2BytesRV(bme,BME280_DIG_P1_ADDR);
	comp_data.p2 = BME280_Read2BytesRV(bme,BME280_DIG_P2_ADDR);
	comp_data.p3 = BME280_Read2BytesRV(bme,BME280_DIG_P3_ADDR);
	comp_data.p4 = BME280_Read2BytesRV(bme,BME280_DIG_P4_ADDR);
	comp_data.p5 = BME280_Read2BytesRV(bme,BME280_DIG_P5_ADDR);
	comp_data.p6 = BME280_Read2BytesRV(bme,BME280_DIG_P6_ADDR);
	comp_data.p7 = BME280_Read2BytesRV(bme,BME280_DIG_P7_ADDR);
	comp_data.p8 = BME280_Read2BytesRV(bme,BME280_DIG_P8_ADDR);
	comp_data.p9 = BME280_Read2BytesRV(bme,BME280_DIG_P9_ADDR);

	comp_data.h1 = BME280_ReadByte(bme,BME280_DIG_H1_ADDR);
	comp_data.h2 = BME280_Read2BytesRV(bme,BME280_DIG_H2_ADDR);
	comp_data.h3 = BME280_ReadByte(bme,BME280_DIG_H3_ADDR);
	comp_data.h4 = ((BME280_ReadByte(bme,BME280_DIG_H4_ADDR) << 4) | (BME280_ReadByte(bme,BME280_DIG_H4_ADDR + 1) & 0xF));
	comp_data.h5 = ((BME280_ReadByte(bme,BME280_DIG_H5_ADDR) >> 4) | (BME280_ReadByte(bme,BME280_DIG_H5_ADDR + 1) << 4));
	comp_data.h6 = BME280_ReadByte(bme,BME280_DIG_H6_ADDR);

	uint8_t temp_reg = BME280_ReadByte(bme, BME280_CTRL_HUM_ADDR);
	temp_reg &= 0xF8;	// clear [2:0] bits
	temp_reg |= bme->hum_oversampling;
	BME280_WriteByte(bme,BME280_CTRL_HUM_ADDR,temp_reg);
	BME280_WriteByte(bme,BME280_CTRL_MEAS_ADDR,((bme->temp_oversampling << 5) | (bme->press_oversampling << 2) | bme->work_mode));

	if (bme->work_mode == BME280_NORMAL_MODE)
	{
		// set filter and standby time for normal mode operation of sensor
		BME280_WriteByte(bme,BME280_CONFIG_ADDR,(uint8_t)(((bme->standby_time & 0x7) << 5) | ((bme->filter &7) << 2)) & 0xFC );
	}
}

void BME280_ReadTemp(bme280_t *bme)
{
	// check mode of the sensor
	if (bme->work_mode == BME280_FORCED_MODE)
	{
		uint8_t curr_mode;
		// force measurement
		uint8_t control = BME280_ReadByte(bme, BME280_CTRL_MEAS_ADDR);
		control &= ~(0x03);
		control |= BME280_FORCED_MODE;
		BME280_WriteByte(bme, BME280_CTRL_MEAS_ADDR, control);

		// wait for end of measurements
		while(1)
		{
			curr_mode = BME280_ReadByte(bme, BME280_CTRL_MEAS_ADDR);
			curr_mode &= 0x03;
			// break if board finished measurements
			if (curr_mode == BME280_SLEEP_MODE)
				break;
		}
	}
	// read temperature data
	int32_t raw_temp = BME280_Read3Bytes(bme,BME280_TEMP_MSB_ADDR);
	// if measurement was skipped return -100
	if(raw_temp == 0x800000)
	{
		bme->temp = -100;
		return;
	}
	raw_temp = raw_temp >> 4;

	uint32_t var1, var2;

	var1 = ((((raw_temp >> 3) - ((int32_t)comp_data.t1 <<1))) *
			((int32_t)comp_data.t2)) >> 11;
	var2 = (((((raw_temp >> 4)- ((int32_t)comp_data.t1)) *
			((raw_temp >> 4) - ((int32_t)comp_data.t1))) >> 12) *
			((int32_t)comp_data.t3)) >> 14;
	// t_fine carries global temperature for compensation for humidity and pressure
	t_fine = var1 + var2;
	// return value in DegC
	bme->temp = ((t_fine * 5 + 128) >> 8) / 100;
	return;
}

void BME280_ReadHum(bme280_t *bme)
{

	// read temperature to get current t_fine variable for compensation
	BME280_ReadTemp(bme);

	int32_t raw_hum = BME280_Read2Bytes(bme, BME280_HUM_MSB_ADDR);
	// if measurement was skipped return -100
	if(raw_hum == 0x8000)
	{
		bme->hum = -100;
		return;
	}

	int32_t val1;

	val1 = (t_fine - ((int32_t)76800));
	val1 = (((((raw_hum << 14) - (((int32_t)comp_data.h4) << 20) -
			(((int32_t)comp_data.h5)* val1)) + ((int32_t)16384)) >> 15) *
			(((((((val1 * ((int32_t)comp_data.h6)) >> 10) *
			(((val1 * ((int32_t)comp_data.h3)) >> 11) +
			((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
			((int32_t)comp_data.h2) + 8192) >> 14));
	val1 = (val1 - (((((val1 >> 15) * (val1 >> 15))  >> 7) * ((int32_t)comp_data.h1)) >> 4));
	val1 = (val1 < 0) ? 0 : val1;
	val1 = (val1 > 419430400) ? 419430400 : val1;
	// get value in 32 bit unsigned integer in Q22.10 format (22 integer and 10 fractional bits)
	float h = (val1 >> 12);
	// convert value to final % RH float value
	bme->hum = h / 1024.0;
	return;
}

void BME280_ReadPress(bme280_t *bme)
{
	// read temperature to get current t_fine variable for compensation
	BME280_ReadTemp(bme);

	int32_t raw_press = BME280_Read3Bytes(bme, BME280_PRESS_MSB_ADDR);
	raw_press = raw_press >> 4;

	int64_t val1, val2, p;

	val1 = ((int64_t)t_fine) - 128000;
	val2 = val1 * val1 * (int64_t)comp_data.p6;
	val2 = val2 + ((val1 * (int64_t)comp_data.p5) << 17);
	val2 = val2 + (((int64_t)comp_data.p4) << 35);
	val1 = ((val1 * val1 * (int64_t)comp_data.p3) >> 8) + ((val1 *(int64_t )comp_data.p2) << 12);
	val1 = (((((int64_t)1) << 47) + val1)) * ((int64_t)comp_data.p1)  >> 33;

	if (val1 == 0)
	{
		// avoid division by 0
		bme->press = 0;
		return;
	}

	p = 1048576 - raw_press;
	p = (((p << 31) - val2) * 3125) / val1;
	val1 = (((int64_t)comp_data.p9) * (p >> 13) * (p >> 13)) >> 25;
	val2 = (((int64_t)comp_data.p8) * p) >> 19;

	p = ((p + val1 + val2) >> 8) + (((int64_t)comp_data.p7) << 4);
	// returning value in hPa
	bme->press = (int32_t)p / 256000.0;
	return;
}

void BME280_ReadAll(bme280_t *bme)
{
	// check mode of the sensor
	if (bme->work_mode == BME280_FORCED_MODE) {
		uint8_t curr_mode;
		// force measurement
		uint8_t control = BME280_ReadByte(bme, BME280_CTRL_MEAS_ADDR);
		control &= ~(0x03);
		control |= BME280_FORCED_MODE;
		BME280_WriteByte(bme, BME280_CTRL_MEAS_ADDR, control);

		// wait for end of measurements
		while (1) {
			curr_mode = BME280_ReadByte(bme, BME280_CTRL_MEAS_ADDR);
			curr_mode &= 0x03;
			// break if board finished measurements
			if (curr_mode == BME280_SLEEP_MODE)
				break;
		}
	}
	int32_t raw_press, raw_temp, raw_hum;
	// read all raw data from the sensor
	BME280_ReadMeasurements(bme, &raw_temp, &raw_hum, &raw_press);

	// temperature part
	if (raw_temp == 0x800000) {
		bme->temp = -100;
	} else {
		raw_temp = raw_temp >> 4;

		uint32_t var1, var2;

		var1 = ((((raw_temp >> 3) - ((int32_t) comp_data.t1 << 1)))
				* ((int32_t) comp_data.t2)) >> 11;
		var2 = (((((raw_temp >> 4) - ((int32_t) comp_data.t1))
				* ((raw_temp >> 4) - ((int32_t) comp_data.t1))) >> 12)
				* ((int32_t) comp_data.t3)) >> 14;
		// t_fine carries global temperature for compensation for humidity and pressure
		t_fine = var1 + var2;
		// return value in DegC
		bme->temp = ((t_fine * 5 + 128) >> 8) / 100.0;
	}

	// pressure part
	if (raw_temp == 0x800000) {
		bme->press = 0;
	} else {
		raw_press = raw_press >> 4;

		int64_t val1, val2, p;

		val1 = ((int64_t) t_fine) - 128000;
		val2 = val1 * val1 * (int64_t) comp_data.p6;
		val2 = val2 + ((val1 * (int64_t) comp_data.p5) << 17);
		val2 = val2 + (((int64_t) comp_data.p4) << 35);
		val1 = ((val1 * val1 * (int64_t) comp_data.p3) >> 8)
				+ ((val1 * (int64_t) comp_data.p2) << 12);
		val1 = (((((int64_t) 1) << 47) + val1)) * ((int64_t) comp_data.p1) >> 33;

		if (val1 == 0) {
			// avoid division by 0
			bme->press = 0;
		} else {
			p = 1048576 - raw_press;
			p = (((p << 31) - val2) * 3125) / val1;
			val1 = (((int64_t) comp_data.p9) * (p >> 13) * (p >> 13)) >> 25;
			val2 = (((int64_t) comp_data.p8) * p) >> 19;

			p = ((p + val1 + val2) >> 8) + (((int64_t) comp_data.p7) << 4);
			// returning value in hPa
			bme->press = (int32_t) p / 25600.0;
		}
	}

	// humidity part
	if (raw_hum == 0x8000) {
		bme->hum = -100;
	} else {

		int32_t val1;

		val1 = (t_fine - ((int32_t) 76800));
		val1 = (((((raw_hum << 14) - (((int32_t) comp_data.h4) << 20) -
					(((int32_t) comp_data.h5) * val1)) + ((int32_t) 16384)) >> 15) *
					(((((((val1 * ((int32_t) comp_data.h6)) >> 10) *
					(((val1 * ((int32_t) comp_data.h3)) >> 11) +
					((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
					((int32_t) comp_data.h2) + 8192) >> 14));
		val1 = (val1 - (((((val1 >> 15) * (val1 >> 15)) >> 7) *
					((int32_t) comp_data.h1)) >> 4));
		val1 = (val1 < 0) ? 0 : val1;
		val1 = (val1 > 419430400) ? 419430400 : val1;
		// get value in 32 bit unsigned integer in Q22.10 format (22 integer and 10 fractional bits)
		float h = (val1 >> 12);
		// convert value to final % RH float value
		bme->hum = h / 1024.0;
	}

	return;
}
