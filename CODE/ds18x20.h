
#ifndef DS18x20_H
#define DS18x20_H
#include "onewire.h"

#define DS18x20_FLOAT	1

/* list of these commands translated into C defines:*/
#define THERM_CMD_CONVERTTEMP 0x44
#define THERM_CMD_RSCRATCHPAD 0xBE
#define THERM_CMD_WSCRATCHPAD 0x4E
#define THERM_CMD_CPYSCRATCHPAD 0x48
#define THERM_CMD_RECEEPROM 0xB8
#define THERM_CMD_RPWRSUPPLY 0xB4

#define DS18B20_9_BIT       0
#define DS18B20_10_BIT      1
#define DS18B20_11_BIT      2
#define DS18B20_12_BIT      3

#define THERM_CMD_ALARMSEARCH 0xEC
/* constants */
#define THERM_DECIMAL_STEPS_12BIT 625 //.0625

#define DS18X20_CHECK_CRC

uint8_t DS18x20_Init( uint8_t *rom, int8_t temp_low, int8_t temp_high, uint8_t resolution);
uint8_t DS18x20_StartMeasure(uint8_t* rom);	//if rom==0 then skip rom
uint8_t DS18x20_ReadData(uint8_t *rom); //if rom==0 then skip rom

#if DS18x20_FLOAT
	#define DS18x20_RET_TYPE float
#else
	#define DS18x20_RET_TYPE int16_t
#endif

DS18x20_RET_TYPE DS18x20_ConvertThemperature();

#endif
