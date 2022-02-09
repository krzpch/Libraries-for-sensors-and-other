/*
 * lcd_i2c.h
 *
 *  Created on: Mar 21, 2021
 *      Author: Krzysztof Półchłopek
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#include <stdbool.h>
#include <stdint.h>

/*
 *         PCF8574 <-> HD44780
 *
 * I2C I/O   P7 P6 P5 P4 P3 P2 P1 P0
 * LCD       D7 D6 D5 D4 BL EN RW RS
 *
 * */

#define HI2C_DEF hi2c1

#define RS_PIN 0x01
#define RW_PIN 0x02
#define EN_PIN 0x04
#define BL_PIN 0x08

#define INIT_8_BIT_MODE	0x30
#define INIT_4_BIT_MODE	0x02

#define CLEAR_LCD	0x01

#define FIRST_CHAR_LINE_1	0x80
#define FIRST_CHAR_LINE_2	0xC0


struct lcd_disp {
	uint8_t addr;
	uint8_t rows,cols;
	bool backlight;
	bool underline;
	bool blink;
};

void lcd_init(struct lcd_disp * lcd);
void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin);
void lcd_setCursor(struct lcd_disp * lcd, uint8_t col, uint8_t row);
void lcd_print(struct lcd_disp * lcd,char * str);
void lcd_clear(struct lcd_disp * lcd);
void lcd_updateCursor(struct lcd_disp * lcd);

#endif /* INC_LCD_I2C_H_ */
