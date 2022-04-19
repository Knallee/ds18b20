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

ds18b20_t sensor1;

uint8_t power_mode;

int main(void)
{
	//ext_int1_init();
	
	pin_change_init();
	lcd_init();
	
	device_found = rst_check_dq_bus_for_device();
	
	read_rom_address(&sensor1);
	
	if(device_found && (sensor1.address.byte1 == 0x28)) {
		lcd_home();
		lcd_put_string("Found a DS18B20");
	} else {
		lcd_home();
		lcd_put_string("No device found");
	}
	
	second_row();
	
	char addr[3];
	
	lcd_put_string("0x");
	lcd_put_string(itoa(sensor1.address.byte8, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte7, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte6, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte5, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte4, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte3, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte2, addr, 16));
	lcd_put_string(itoa(sensor1.address.byte1, addr, 16));
	
	sei();
    /* Replace with your application code */
    while (1) {	
		//rst_check_dq_bus_for_device();
		//_delay_us(30);

		read_temperature(&sensor1);

	
		
		//while(!read_bit_from_dq_bus());
		rst_check_dq_bus_for_device();
		write_dq_command(SKIP_ROM);
		read_scratch_pad(&sensor1);
		
		temperature = (sensor1.scratch_pad.temperature_msb << 4) | (sensor1.scratch_pad.temperature_lsb >> 4);

		
		rst_check_dq_bus_for_device();
		power_mode = check_power_mode();
		


		
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


ISR(TIMER0_OVF_vect) {
	timer0_ovf_cnt++;
	if (timer0_ovf_cnt == time) {
		times_up = 0;
	}

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