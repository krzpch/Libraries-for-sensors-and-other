/*
 * lcd_i2c.c
 *
 *  Created on: Mar 21, 2021
 *      Author: Krzysztof Półchłopek
 */

#include "lcd_i2c.h"
#include "stm32l4xx_hal.h"
#include "i2c.h"



void lcd_init(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;
	/* set backlight */
	if(lcd->backlight)
	{
		xpin = BL_PIN;
	}

	/* init sequence */
	HAL_Delay(40);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(5);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(1);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);

	/* set 4-bit mode */
	lcd_write(lcd->addr, INIT_4_BIT_MODE, xpin);

	/* set cursor mode */
	lcd_updateCursor(lcd);

	/* clear */
	lcd_clear(lcd);

}

void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t tx_data[4];

	/* split data */
	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	/* send data via i2c */
	HAL_I2C_Master_Transmit(&HI2C_DEF, addr, tx_data, 4, 100);
	HAL_Delay(5);
}


void lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	/* set backlight */
	if(lcd->backlight)
	{
		xpin = BL_PIN;
	}

	/* clear display */
	lcd_write(lcd->addr, CLEAR_LCD, xpin);
}

void lcd_setCursor(struct lcd_disp * lcd, uint8_t col, uint8_t row)
{
	uint8_t xpin = 0;
	if(lcd->backlight)
	{
		xpin = BL_PIN;
	}
	/* check if row is valid */
	if(row > (lcd->rows - 1))
	{
		row = lcd->rows - 1;
	}

	uint8_t command = 0x80 + col+(0x40*row);
	lcd_write(lcd->addr, command, xpin);
}

void lcd_print(struct lcd_disp * lcd,char * str)
{
	uint8_t xpin = 0;
	uint8_t str_len = 0;
		if(lcd->backlight)
			{
				xpin = BL_PIN;
			}
		while(str[str_len] != '\0')
		{
			lcd_write(lcd->addr, str[str_len], (xpin | RS_PIN));
			++str_len;
		}

}

void lcd_updateCursor(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;
	uint8_t data = 0x0C;
	/* set backlight */
	if(lcd->backlight)
	{
		xpin = BL_PIN;
	}

	if (lcd->blink)
	{
		data = data | 0x01;
	}
	else if (lcd->underline)
	{
		data = data | 0x02;
	}
	lcd_write(lcd->addr, data, xpin);
}

