/*
 * ds18b20.h
 *
 * Created: 2022-04-17 01:55:41
 *  Author: Knalle
 */ 


#ifndef DS18B20_H_
#define DS18B20_H_


#include "f_cpu.h"
#include <avr/io.h>
#include <util/delay.h>


#define DQ_PORT		PORTC
#define DQ_DDR		DDRC
#define DQ_PIN_REG	PINC
#define DQ_PIN		(6)


#define PARASITIC_PWR	0

                                                                                                                                                                        
/** ______ ________  ___                           _
	| ___ |  _  |  \/  |                          | |
	| |_/ | | | | .  . |   ___ _ __ ___  _ __   __| |___
	|    /| | | | |\/| |  / __| '_ ` _ \| '_ \ / _` / __|
	| |\ \\ \_/ | |  | | | (__| | | | | | | | | (_| \__ \
	\_| \_|\___/\_|  |_/  \___|_| |_| |_|_| |_|\__,_|___/
*/
#define SEARCH_ROM		(0xF0)		/**< Commands the all slaves tell its unique 64-bit address.*/
#define READ_ROM		(0x33)		/**< Use this command to read a slave rom. Use this command only
										if there is one slave on the bus.*/
#define MATCH_ROM		(0x55)		/**< Address a specific slave device on a multidrop or single-drop bus.*/
#define SKIP_ROM		(0xCC)		/**< Use this command to address all devices on the bus simultaneously or 
										if there is only one device on the bus.*/
#define ALARM_SEARCH	(0xEC)		/**< This command is identical to the operation of the Search ROM command 
										except that only slaves with a set alarm flag will respond.*/
/** 
	______                _   _                                        _
	|  ___|              | | (_)                                      | |
	| |_ _   _ _ __   ___| |_ _  ___  _ __     ___ _ __ ___  _ __   __| |___
	|  _| | | | '_ \ / __| __| |/ _ \| '_ \   / __| '_ ` _ \| '_ \ / _` / __|
	| | | |_| | | | | (__| |_| | (_) | | | | | (__| | | | | | | | | (_| \__ \
	\_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|  \___|_| |_| |_|_| |_|\__,_|___/
*/
#define CONVERT_TEMP		(0x44)		/**< This command initiates a single temperature conversion. The DS18B20 will
											respond by transmitting a 0 while the temperature conversion is in progress 
											and a 1 when the conversion is done.*/
#define WRITE_SCRATCH_PAD	(0x4E)		/**< Write data to t_high_user_byte, t_low_user_byte and config_register.*/
#define READ_SCRATCHPAD		(0xBE)		/**< Read the contents of the scratchpad starting with byte o and lsb first.*/
#define COPY_SCRATCHPAD		(0x48)		/**< This command copies the contents of the scratchpad TH, TL and 
											configuration registers to EEPROM.*/
#define RECALL_E			(0cB8)		/**< This command copies the contents of the scratchpad TH, TL and 
											configuration registers from the EEPROM.*/
#define READ_PWR_SUPPLY		(0xB4)		/**< Determines if any DS18B20s on the bus are using parasite power.*/


#define RES_9_BIT		(0)				/**< 9-bit resolution. Conversion time is 93.75 ms.*/
#define RES_10_BIT		(1)				/**< 10-bit resolution. Conversion time is 187.5 ms.*/
#define RES_11_BIT		(2)				/**< 11-bit resolution. Conversion time is 375 ms.*/
#define RES_12_BIT		(3)				/**< 12-bit resolution. Conversion time is 750 ms.*/

typedef struct {
	uint8_t byte8;
	uint8_t byte7;
	uint8_t byte6;
	uint8_t byte5;
	uint8_t byte4;
	uint8_t byte3;
	uint8_t byte2;
	uint8_t byte1;
} slave_address_t;

struct config_reg {
	uint8_t reserved6	: 1;
	uint8_t resolution	: 2;
	uint8_t reserved5	: 1;
	uint8_t reserved4	: 1;
	uint8_t reserved3	: 1;
	uint8_t reserved2	: 1;
	uint8_t reserved1	: 1;
};

typedef struct {
	uint8_t temperature_lsb;
	uint8_t temperature_msb;
	uint8_t t_high_user_byte;
	uint8_t t_low_user_byte;
	union {
		struct config_reg config_register;
		uint8_t conf_reg_uint8;
	}conf_reg;
	
	uint8_t reserved3;
	uint8_t reserved2;
	uint8_t reserved1;
	uint8_t crc_reg;
}scratch_pad_t;

typedef struct{
	slave_address_t address;
	scratch_pad_t scratch_pad;
}ds18b20_t;

void start_timer(void);
void stop_timer(void);
void pull_dq_bus_low(void);
void release_dq_bus(void);
uint8_t read_dq_bus_level(void);
uint8_t rst_check_dq_bus_for_device(void);
void write_one_to_dq_bus(void);
void write_zero_to_dq_bus(void);
void write_byte_to_dq_bus(uint8_t byte);
uint8_t read_byte_from_dq_bus(void);
void write_dq_command(uint8_t);
void read_scratch_pad(ds18b20_t *);
void read_rom_address(ds18b20_t *);
void skip_rom_read_temperature(ds18b20_t *);
uint8_t check_power_mode();
void match_rom_read_temperature(ds18b20_t *dev);

#endif /* DS18B20_H_ */