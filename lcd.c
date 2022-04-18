/*
 * lcd.c
 *
 * Created: 2022-04-06 18:47:30
 *  Author: ch5152ce
 */ 

#include "lcd.h"

void lcd_commmand(uint8_t cmd) {
	//lcd_busy_check();
	LCD_DATA_PORT = cmd;
	LCD_CTRL_PORT &= ~((1 << LCD_RS) | (1 << LCD_RW));	// RS = 0 => command reg, RW = 0 => Write
	LCD_CTRL_PORT |= (1 << LCD_ENABLE);		// |
	_delay_us(1);					// |
	LCD_CTRL_PORT &= ~(1 << LCD_ENABLE);	// V Generating enable pulse so lcd latches in the data
	_delay_ms(3);
	
	
}

void lcd_init(){
	LCD_DATA_DDR |= 0xFF;
	LCD_CTRL_DDR |= (1 << LCD_ENABLE) | (1 << LCD_RW) |(1 << LCD_RS);
	
	_delay_ms(20);	// LCD start up time > 15 ms
	
	lcd_commmand(0b00111000);		// LCD in 8 bit mode, 2 line display and  5 x 8 dots format display mode
	_delay_us(50);
	lcd_commmand(0b00001100);		// Display on. entire display, cursor on and cursor position on
	_delay_us(50);
	lcd_commmand(0b00000110);		// Increment cursor (moves to the right), address is incremented by 1
	_delay_us(50);
	lcd_commmand(0b00000001);		// Clear display
	_delay_ms(5);
	lcd_commmand(0b10000000);		// Return home
	_delay_us(50);
}

void lcd_put_char(char data) {
	LCD_DATA_PORT = data;
	LCD_CTRL_PORT |=  (1 << LCD_RS);		// RS = 0 => command reg
	LCD_CTRL_PORT &= ~(1 << LCD_RW);		// RW = 0 => Write
	LCD_CTRL_PORT |=  (1 << LCD_ENABLE);	// |
	_delay_us(1);					// |
	LCD_CTRL_PORT &= ~(1 << LCD_ENABLE);	// V Generating enable pulse so lcd latches in the data
	_delay_ms(2);
}


void lcd_put_string(char *str) {
	
	while (*str) lcd_put_char(*str++);
	
}

void roll_display_right() {
	
	lcd_commmand(0b00011100);
	_delay_ms(100);
	
}

void second_row() {
	lcd_commmand(0xc0);
}

void lcd_clear_and_home() {
	lcd_commmand(0b00000001);		// Clear display
	_delay_ms(5);
	lcd_commmand(0b10000000);		// Return home
	_delay_us(50);
}


void lcd_home() {
	lcd_commmand(0b10000000);		// Return home
	_delay_us(50);
}
