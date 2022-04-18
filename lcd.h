/*
 * lcd.h
 *
 * Created: 2022-04-06 18:47:18
 *  Author: ch5152ce
 */ 


#ifndef LCD_H_
#define LCD_H_

#include "f_cpu.h"
#include "lcd.h"
#include <util/delay.h>
#include <avr/io.h>

#define	LCD_RS		PD4
#define	LCD_RW		PD5
#define	LCD_ENABLE	PD6

#define LCD_CTRL_PORT		PORTD
#define LCD_CTRL_DDR		DDRD
#define LCD_DATA_PORT		PORTB
#define LCD_DATA_DDR		DDRB

void lcd_init(void);
void lcd_put_char(char data);
void lcd_put_string(char *str);
void roll_display_right(void);
void second_row(void);
void lcd_commmand(uint8_t cmd);
void lcd_clear_and_home(void);
void lcd_home(void);

#endif /* LCD_H_ */