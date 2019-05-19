#include "ds18x20.h"
#include <util/delay.h>

volatile struct
{
	uint8_t TemperatureLSB;
	uint8_t TemperatureMSB;
	uint8_t THRegister;
	uint8_t TLRegister;
	uint8_t ConfigRegister;
	uint8_t Reserved1;
	uint8_t Reserved2;
	uint8_t Reserved3;
	uint8_t CRC;
} ds18b20_scratch_pad;

int16_t bit_mask[4] = { 0xFFF8, 0xFFFC, 0xFFFE, 0xFFFF };

uint8_t DS18x20_StartMeasure(uint8_t* rom)
{
	//Reset, skip ROM and start temperature conversion
	if (!OW_Reset()) return 0;
	
	if (rom) OW_MatchROM(rom);
	else OW_WriteByte(OW_CMD_SKIPROM);
	
	OW_WriteByte(THERM_CMD_CONVERTTEMP);
	return 1;
}

#ifdef DS18X20_CHECK_CRC
#define CRC8INIT	0x00
#define CRC8POLY	0x18              //0x18 = X^8+X^5+X^4+X^0

uint8_t crc8(uint8_t *data_in, unsigned int number_of_bytes_to_read )
{
	uint8_t	crc;
	unsigned int	loop_count;
	uint8_t	bit_counter;
	uint8_t	data;
	uint8_t	feedback_bit;

	crc = CRC8INIT;
	
	for (loop_count = 0; loop_count != number_of_bytes_to_read; loop_count++)
	{ 
		data = data_in[loop_count];

		bit_counter = 8;
		do { 
			feedback_bit = (crc ^ data) & 0x01;
			if (feedback_bit==0x01) crc = crc ^ CRC8POLY;

			crc = (crc >> 1) & 0x7F;
			if (feedback_bit==0x01) crc = crc | 0x80;

			data = data >> 1;
			bit_counter--;
		}
		while (bit_counter > 0);
	}
	return crc;
}
#endif 

unsigned char DS18x20_Init( uint8_t *rom, int8_t temp_low, int8_t temp_high, uint8_t resolution)
{
	if (!OW_Reset()) return 0;

	if (rom) OW_MatchROM(rom);

	resolution = ( resolution << 5 ) | 0x1f;

	OW_WriteByte( THERM_CMD_WSCRATCHPAD );
	OW_WriteByte( temp_high );
	OW_WriteByte( temp_low );
	OW_WriteByte( resolution );

	if (rom) OW_MatchROM(rom);
	OW_WriteByte(THERM_CMD_CPYSCRATCHPAD);

	_delay_ms(15);

	return OW_Reset();
}

uint8_t DS18x20_ReadData(uint8_t *rom )
{
	uint8_t *lPoint;
	lPoint = &ds18b20_scratch_pad;
	//Reset, skip ROM and send command to read Scratchpad
	if (!OW_Reset()) return 0;
	
	if (rom) OW_MatchROM(rom);
	else OW_WriteByte(OW_CMD_SKIPROM);
	
	OW_WriteByte(THERM_CMD_RSCRATCHPAD);
	
#ifdef DS18X20_CHECK_CRC
	for (uint8_t i = 0; i < 9; i++ ) *( lPoint + i ) = OW_ReadByte();
	if (crc8( &ds18b20_scratch_pad, 9 ) ) return 0;	// если контрольная сумма не совпала, возвращаем ошибку
#else
	//Read Scratchpad (only 2 first bytes)
	buffer[0] = OW_ReadByte(); // Read TL
	buffer[1] = OW_ReadByte(); // Read TH
#endif

	return 1;
}

/*
#ifdef DS18x20_FLOAT
	float DS18x20_ConvertThemperature()
	{
		uint8_t resolution;
		resolution = ( ds18b20_scratch_pad.ConfigRegister >> 5 ) & 3;
		return ( *( ( int16_t *)&ds18b20_scratch_pad.TemperatureLSB ) & ( bit_mask[resolution] ) ) * 0.0625;
	};
#else
	int DS18x20_ConvertThemperature()
	{
		uint8_t resolution;
		resolution = ( ds18b20_scratch_pad.ConfigRegister >> 5 ) & 3;
		return ( *( ( int16_t *)&ds18b20_scratch_pad.TemperatureLSB ) & ( bit_mask[resolution] ) ) * 5 / 8;
	}
#endif
*/

#ifdef DS18x20_FLOAT
	#define COEF 0.0625
#else
	#define COEF 5/8
#endif

DS18x20_RET_TYPE DS18x20_ConvertThemperature()
{
	uint8_t resolution;
	resolution = ( ds18b20_scratch_pad.ConfigRegister >> 5 ) & 3;
	return ( *( ( int16_t *)&ds18b20_scratch_pad.TemperatureLSB ) &  ( resolution[bit_mask] ) ) * COEF;
}

