/*
 * DS18B20.c
 *
 * Created: 2022-04-16 23:28:03
 * Author : Knalle
 */ 

#include "f_cpu.h"
#include "lcd.h"
#include "ds18b20.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


void ext_int1_init();
void pin_change_init();


volatile uint8_t timer0_ovf_cnt, times_up, time;

volatile uint8_t enc_a, enc_b, enc_flag;
volatile int16_t enc_dir, enc_cnt, enc_cnt_old;
char buf[10];

uint16_t temperature;

uint8_t device_found;

ds18b20_t sensor1, sensor2;

uint8_t power_mode;


int main(void)
{
	
	pin_change_init();
	lcd_init();
	
	device_found = rst_check_dq_bus_for_device();
	
	sensor1.address.byte8 = 0xe5;
	sensor1.address.byte7 = 0x00;
	sensor1.address.byte6 = 0x00;
	sensor1.address.byte5 = 0x0d;
	sensor1.address.byte4 = 0x7b;
	sensor1.address.byte3 = 0xbf;
	sensor1.address.byte2 = 0xf7;
	sensor1.address.byte1 = 0x28;
	
	sensor2.address.byte8 = 0x8c;
	sensor2.address.byte7 = 0x00;
	sensor2.address.byte6 = 0x00;
	sensor2.address.byte5 = 0x0d;
	sensor2.address.byte4 = 0x7b;
	sensor2.address.byte3 = 0x34;
	sensor2.address.byte2 = 0x8a;
	sensor2.address.byte1 = 0x28;
	
	if(device_found) {
		lcd_home();
		lcd_put_string("Found a device");
	} else {
		lcd_home();
		lcd_put_string("No device found");
	}
	
	char addr[3];	
	
	DDRA |= (1 << DDRA0);
	
	_delay_ms(2000);
	lcd_clear_and_home();
	second_row();
	
	sei();

    while (1) {	


		lcd_home();
		match_rom_read_temperature(&sensor1);
		temperature = (sensor1.scratch_pad.temperature_msb << 4) | (sensor1.scratch_pad.temperature_lsb >> 4);
		lcd_put_string(itoa(temperature, addr, 10));


		second_row();
		match_rom_read_temperature(&sensor2);	
		temperature = (sensor2.scratch_pad.temperature_msb << 4) | (sensor2.scratch_pad.temperature_lsb >> 4);
		lcd_put_string(itoa(temperature, addr, 10));
		//_delay_ms(1);
		


		
		//if(enc_flag == 1) {
 			//enc_flag = 0;
 			//lcd_commmand(0b00000001);		// Clear display
			//_delay_ms(5);
 			//lcd_commmand(0b10000000);		// Return home
 			////_delay_us(50);
 			//itoa(enc_cnt, buf, 10);
 			//lcd_put_string(buf);	
		//}
		
    }
}



void pin_change_init(){
	DDRD &= ~(1<<PIND3);
	PCICR |= (1<<PCIE3);
	PCMSK3 |= (1<<PCINT27);
}

void ext_int1_init(){
	EICRA	|= (1 << ISC10);
	EIMSK	|= (1 << INT1);
}


ISR(PCINT3_vect) {
	enc_flag = 1;
	
	enc_a = (PIND & (1 << PIND3)) >> PIND3;
	enc_b = (PIND & (1 << PIND7)) >> PIND7;
	
	
	if (enc_a != enc_b) {
		enc_dir = -1;
	}
	
	if(enc_a == enc_b) {
		enc_dir = 1;
	}

	enc_cnt += enc_dir;
}

ISR(INT1_vect) {
	enc_flag = 1;
	
	enc_a = (PIND & (1 << PIND3)) >> PIND3;
	enc_b = (PIND & (1 << PIND7)) >> PIND7;
	
	
	if (enc_a != enc_b) {
		enc_dir = -1;
	}
	
	if(enc_a == enc_b) {
		enc_dir = 1;
	}

	enc_cnt += enc_dir;
}