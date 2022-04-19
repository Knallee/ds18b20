/*
 * ds18b20.c
 *
 * Created: 2022-04-17 01:56:17
 *  Author: Knalle
 */ 

#include "ds18b20.h"


void pull_dq_bus_low(){
	DQ_DDR |= (1<<DQ_PIN);
	DQ_PORT &= ~(1<<DQ_PIN);
}

void release_dq_bus() {
	DQ_DDR &= ~(1<<DQ_PIN);
	//DQ_PORT |= (1<<DQ_PIN);
}

 uint8_t read_dq_bus_level(){
	 return (DQ_PIN_REG & (1 << DQ_PIN)) >> DQ_PIN;
 }
 
 uint8_t rst_check_dq_bus_for_device(){
		uint8_t dev_found;
	 	pull_dq_bus_low();
	 	_delay_us(500);
	 	release_dq_bus();
	 	_delay_us(120);
		 
		dev_found = !((DQ_PIN_REG & (1 << DQ_PIN)) >> DQ_PIN);
		
		_delay_us(400);
		
		return dev_found;
		
		
 }
 
 void write_one_to_dq_bus(){
	 pull_dq_bus_low();
	 _delay_us(10);
	 release_dq_bus();
	 _delay_us(55);
 }
 
 void write_zero_to_dq_bus(){
	 pull_dq_bus_low();
	 _delay_us(80);
	 release_dq_bus();
	 _delay_us(2);
 }
 
 uint8_t read_bit_from_dq_bus() {
	uint8_t dq_bit;
	pull_dq_bus_low();
	_delay_us(5);
	release_dq_bus();
	_delay_us(1);
	dq_bit = read_dq_bus_level();
	_delay_us(60);
	return dq_bit;
	 
 }
 
 void write_byte_to_dq_bus(uint8_t byte) {
	 uint8_t bit;
	 for(int i = 0; i < 8; i++) {
		 bit = byte & (1 << i);
		 if (bit) {
			 write_one_to_dq_bus();
		 } else {
			 write_zero_to_dq_bus();
		 }
	 }
	 
 }
 
 uint8_t read_byte_from_dq_bus(){
	 uint8_t byte= 0;
	 uint8_t bit;
	 for(int i = 0; i < 8; i++) {
		 bit = read_bit_from_dq_bus();
		 if (bit) {
			 byte |= (1<<i);
		 } else {
			 byte &= ~(1<<i);
		 }
	 }
	 return byte;
 }
 
 void write_dq_command(uint8_t cmd) {
	 
	 write_byte_to_dq_bus(cmd);
	 
 }
 
void read_scratch_pad(ds18b20_t *dev) {
	write_dq_command(READ_SCRATCHPAD);
	dev->scratch_pad.temperature_lsb			= read_byte_from_dq_bus();
	dev->scratch_pad.temperature_msb			= read_byte_from_dq_bus();
	dev->scratch_pad.t_high_user_byte			= read_byte_from_dq_bus();
	dev->scratch_pad.conf_reg.conf_reg_uint8	= read_byte_from_dq_bus();
	dev->scratch_pad.reserved1					= read_byte_from_dq_bus();
	dev->scratch_pad.reserved2					= read_byte_from_dq_bus();
	dev->scratch_pad.reserved3					= read_byte_from_dq_bus();
	dev->scratch_pad.crc_reg					= read_byte_from_dq_bus();
	
}

void read_rom_address(ds18b20_t *dev){
	write_dq_command(READ_ROM);
	dev->address.byte1 = read_byte_from_dq_bus();
	dev->address.byte2 = read_byte_from_dq_bus();
	dev->address.byte3 = read_byte_from_dq_bus();
	dev->address.byte4 = read_byte_from_dq_bus();
	dev->address.byte5 = read_byte_from_dq_bus();
	dev->address.byte6 = read_byte_from_dq_bus();
	dev->address.byte7 = read_byte_from_dq_bus();
	dev->address.byte8 = read_byte_from_dq_bus();
	
}

void write_address(ds18b20_t *dev) {
	write_byte_to_dq_bus(dev->address.byte1);
	write_byte_to_dq_bus(dev->address.byte2);
	write_byte_to_dq_bus(dev->address.byte3);
	write_byte_to_dq_bus(dev->address.byte4);
	write_byte_to_dq_bus(dev->address.byte5);
	write_byte_to_dq_bus(dev->address.byte6);
	write_byte_to_dq_bus(dev->address.byte7);
	write_byte_to_dq_bus(dev->address.byte8);
}

void read_temperature(ds18b20_t *dev) {
	
	rst_check_dq_bus_for_device();
	write_dq_command(SKIP_ROM);
	write_dq_command(CONVERT_TEMP);

	if(check_power_mode() == PARASITIC_PWR) {
		DQ_DDR |= (1<<DQ_PIN);
		DQ_PORT |= (1<<DQ_PIN);
		_delay_ms(750);
		DQ_DDR &= ~(1<<DQ_PIN);
		} else {
		while(!read_bit_from_dq_bus());
	}
	
	
	rst_check_dq_bus_for_device();
	write_dq_command(SKIP_ROM);

	read_scratch_pad(&dev);
}

uint8_t check_power_mode() {
	rst_check_dq_bus_for_device();
	write_dq_command(SKIP_ROM);
	write_dq_command(READ_PWR_SUPPLY);
	return read_bit_from_dq_bus();
}