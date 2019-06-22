/*
 * main.c
 *
 *  Created on: 17 сент. 2018 г.
 *      Author: gavrilov.iv
 */

#include "include.h"

#define FW_VERSION	11
#define BUT_ENTER	BUT_1_ID
#define BUT_ENTER_PIN	7
#define BUT_ENTER_DDR	DDRD
#define BUT_ENTER_PORT	PORTD
#define SOFT_START		1

#ifdef DEBUG
	#define BUT_UP		BUT_1_ID
	#define BUT_DOWN	BUT_3_ID
#endif

// PH sensor definitions
#define ISOPC_PH			6.7f
#define	ISOPC_E				0.018f
#define TEMP_D_VOLTAGE		0.053f
#define TEMP_U_VOLTAGE		0.057f
#define TEMP_D				0.0f
#define TEMP_U				20.0f

#define INTERNAL_ADC	1
#define ADC_VOLTAGE		4.970F

#if INTERNAL_ADC
#define ELECTRODE_INPUT		0
#define	AMPL_OFFSET_INPUT	1
#endif

// Motor driver definitions
#define M_EN_PIN 		4
#define M_EN_DDR 		DDRD
#define M_EN_PORT 		PORTD
#define M_INA_PIN 		5
#define M_INA_DDR 		DDRD
#define M_INA_PORT 		PORTD
#define M_INB_PIN 		6
#define M_INB_DDR 		DDRD
#define M_INB_PORT 		PORTD
#define M_PWM_PIN 		3
#define M_PWM_DDR 		DDRD
#define M_PWM_PORT 		PORTD
// Beeper definitions
#define BEEP_PIN 		2
#define BEEP_DDR 		DDRC
#define BEEP_PORT 		PORTC

#define FORWARD		0
#define BACKWARD	1

#define ON			1
#define OFF			0

#define HIGH		1
#define LOW			0

#define MAX_CYCLE_NUMBERS	10
#define PROG_NUMBERS		2

#define SHOW_POWER_CYCLES	1000
#define EN_CHANGE_POWER		200

#define ACC_SIZE			8
#define AccSizeMacro(val)	( 1 << val )

enum Mode {
	StartModeList,
	ProgrammMode,
	ManualMode,
	PHMeterMode,
	EndModeList
};

enum PHModeDisplayData {
	PHDisplay,
	TemperatureDisplay,
	VoltageDisplay,
	RefVoltageDisplay,
	EndPHModeDispDataEnum
};

enum State{
	Stop,
	Pause,
	Work
};

enum ProgName {
	Gauda,
	TestProg
};


typedef struct ProgCycle{
	uint8_t State;
	uint16_t CycleTime;
	uint16_t OWRTime;
	uint8_t PBRevers;
	uint16_t FullCycleTime;
	uint8_t Power;
} ProgCycle;

typedef struct Programm{
	uint8_t NuberOfCycles;
	ProgCycle ProgCycleArray[MAX_CYCLE_NUMBERS];
} Programm;

Programm ProgArray[PROG_NUMBERS];

// Global variables
uint16_t GTime = 7200, RTime = 15, PTime = 5;
int8_t PowerInManualMode = 50;
#if SOFT_START
int8_t SoftPower = 0;
#endif
uint8_t WorkMode = ProgrammMode;
uint32_t ProgTime = 0;
uint8_t ButtonCode, ButtonEvent, CurrentCycle = 0;
int8_t CurrentProgNumber = Gauda;
uint16_t BeepTime = 0, BeepCycle;
int16_t ShowPower = 0;
uint8_t Version = 0;
uint16_t ADCData[2] = { 0 };
uint16_t ADCDataOffsetVal;
uint16_t ADCDataArray[AccSizeMacro(ACC_SIZE)] = { 0 };
uint8_t OneWireDevices[MAXDEVICES][OW_ROMCODE_SIZE];

struct Flag {
	uint8_t ProgIsStarted :1;
	uint8_t ProgIsPaused :1;
	uint8_t InitLocalTimerVar :1;
	uint8_t SecondDot :1;
	uint8_t BeepOnStop :1;
	uint8_t EnChangePower :1;
	uint8_t SaveEEPROM :1;
	uint8_t Mute : 1;
	uint8_t ReadTemperature : 1;
	uint8_t SoftStart :1;
	uint8_t IsStarted :1;
} Flag;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float get_pH_slope(float temperature) {
	return mapf(temperature, TEMP_D, TEMP_U, TEMP_D_VOLTAGE, TEMP_U_VOLTAGE);
}

uint16_t float_window(uint16_t Value) {
	static uint16_t Index = 0;
	static uint64_t Summ = 0;

	Summ += Value;

	Summ -= ADCDataArray[Index];

	ADCDataArray[Index++] = Value;

	if (Index == AccSizeMacro(ACC_SIZE)) {
		Index = 0;
	}

	return (Summ >> ACC_SIZE);
}

unsigned char search_ow_devices(void) {
	unsigned char	i;
   	unsigned char	id[OW_ROMCODE_SIZE];
   	unsigned char	diff, sensors_count;

	sensors_count = 0;
	cli();
	for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE && sensors_count < MAXDEVICES ; ) {
		OW_FindROM( &diff, &id[0] );

      	if( diff == OW_PRESENCE_ERR ) break;

      	if( diff == OW_DATA_ERR )	break;

      	for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      		OneWireDevices[sensors_count][i] = id[i];

		sensors_count++;
    }
	sei();
	return sensors_count;
}

void save_eeprom() {
	cli();
	eeprom_write_byte(0, Version);
	eeprom_update_block( (uint8_t*)&ProgArray, (uint8_t*)2, sizeof( ProgArray ) );
	sei();
}

void get_but() {
	ButtonCode = BUT_GetBut();
	if(ButtonCode) {
		ButtonEvent = BUT_GetBut();
	}
}

uint32_t cycle_time(uint8_t prog, uint8_t cycle) {
	uint32_t result = 0;
	result = ProgArray[prog].ProgCycleArray[cycle].CycleTime;

	if(ProgArray[prog].ProgCycleArray[cycle].OWRTime != 0) {
		result += ProgArray[prog].ProgCycleArray[cycle].CycleTime /
				ProgArray[prog].ProgCycleArray[cycle].OWRTime *
				ProgArray[prog].ProgCycleArray[cycle].PBRevers;
	}

	return result;
}

uint32_t prog_time(uint8_t progNumber) {
	uint32_t result = 0;
	for(uint8_t i = 0; i < ProgArray[progNumber].NuberOfCycles; i++) {
		result += ProgArray[progNumber].ProgCycleArray[i].FullCycleTime;
	}
	return result;
}

void prog_init() {

	uint8_t* Pointer = (uint8_t*)&ProgArray;
	for(int i = 0; i < sizeof(ProgArray); i++) {
		*Pointer = 0x00;
		Pointer++;
	}

	/* Gauda ******************************************/
	ProgArray[Gauda].NuberOfCycles = 5;
	// Gauda 1 cycle
	ProgArray[Gauda].ProgCycleArray[0].State = Work;
	ProgArray[Gauda].ProgCycleArray[0].CycleTime = 300;
	ProgArray[Gauda].ProgCycleArray[0].OWRTime = 30;
	ProgArray[Gauda].ProgCycleArray[0].PBRevers = 2;
	ProgArray[Gauda].ProgCycleArray[0].FullCycleTime = cycle_time(Gauda, 0);
	ProgArray[Gauda].ProgCycleArray[0].Power = 75;
	// Gauda 2 cycle
	ProgArray[Gauda].ProgCycleArray[1].State = Pause;
	ProgArray[Gauda].ProgCycleArray[1].CycleTime = 300;
	ProgArray[Gauda].ProgCycleArray[1].OWRTime = 0;
	ProgArray[Gauda].ProgCycleArray[1].PBRevers = 0;
	ProgArray[Gauda].ProgCycleArray[1].FullCycleTime = cycle_time(Gauda, 1);
	ProgArray[Gauda].ProgCycleArray[1].Power = 0;
	// Gauda 3 cycle
	ProgArray[Gauda].ProgCycleArray[2].State = Work;
	ProgArray[Gauda].ProgCycleArray[2].CycleTime = 600;
	ProgArray[Gauda].ProgCycleArray[2].OWRTime = 30;
	ProgArray[Gauda].ProgCycleArray[2].PBRevers = 2;
	ProgArray[Gauda].ProgCycleArray[2].FullCycleTime = cycle_time(Gauda, 2);
	ProgArray[Gauda].ProgCycleArray[2].Power = 75;
	// Gauda 4 cycle
	ProgArray[Gauda].ProgCycleArray[3].State = Pause;
	ProgArray[Gauda].ProgCycleArray[3].CycleTime = 600;
	ProgArray[Gauda].ProgCycleArray[3].OWRTime = 0;
	ProgArray[Gauda].ProgCycleArray[3].PBRevers = 0;
	ProgArray[Gauda].ProgCycleArray[3].FullCycleTime = cycle_time(Gauda, 3);
	ProgArray[Gauda].ProgCycleArray[3].Power = 0;
	// Gauda 5 cycle
	ProgArray[Gauda].ProgCycleArray[4].State = Work;
	ProgArray[Gauda].ProgCycleArray[4].CycleTime = 300;
	ProgArray[Gauda].ProgCycleArray[4].OWRTime = 30;
	ProgArray[Gauda].ProgCycleArray[4].PBRevers = 2;
	ProgArray[Gauda].ProgCycleArray[4].FullCycleTime = cycle_time(Gauda, 4);
	ProgArray[Gauda].ProgCycleArray[4].Power = 75;
	/****************************************** Gauda */

	/* Test prog **************************************/
	ProgArray[TestProg].NuberOfCycles = 4;
	// TestProg 1 cycle
	ProgArray[TestProg].ProgCycleArray[0].State = Work;
	ProgArray[TestProg].ProgCycleArray[0].CycleTime = 20;
	ProgArray[TestProg].ProgCycleArray[0].OWRTime = 5;
	ProgArray[TestProg].ProgCycleArray[0].PBRevers = 2;
	ProgArray[TestProg].ProgCycleArray[0].FullCycleTime = cycle_time(TestProg, 0);
	ProgArray[TestProg].ProgCycleArray[0].Power = 20;
	// TestProg 2 cycle
	ProgArray[TestProg].ProgCycleArray[1].State = Pause;
	ProgArray[TestProg].ProgCycleArray[1].CycleTime = 10;
	ProgArray[TestProg].ProgCycleArray[1].OWRTime = 0;
	ProgArray[TestProg].ProgCycleArray[1].PBRevers = 0;
	ProgArray[TestProg].ProgCycleArray[1].FullCycleTime = cycle_time(TestProg, 1);
	ProgArray[TestProg].ProgCycleArray[1].Power = 0;
	// TestProg 3 cycle
	ProgArray[TestProg].ProgCycleArray[2].State = Work;
	ProgArray[TestProg].ProgCycleArray[2].CycleTime = 20;
	ProgArray[TestProg].ProgCycleArray[2].OWRTime = 5;
	ProgArray[TestProg].ProgCycleArray[2].PBRevers = 2;
	ProgArray[TestProg].ProgCycleArray[2].FullCycleTime = cycle_time(TestProg, 2);
	ProgArray[TestProg].ProgCycleArray[2].Power = 80;
	// TestProg 4 cycle
	ProgArray[TestProg].ProgCycleArray[3].State = Pause;
	ProgArray[TestProg].ProgCycleArray[3].CycleTime = 10;
	ProgArray[TestProg].ProgCycleArray[3].OWRTime = 0;
	ProgArray[TestProg].ProgCycleArray[3].PBRevers = 0;
	ProgArray[TestProg].ProgCycleArray[3].FullCycleTime = cycle_time(TestProg, 3);
	ProgArray[TestProg].ProgCycleArray[3].Power = 0;
	/************************************** Test prog */
}
void all_init() {
	Version = 0xFF;
	uint8_t* Pointer = (uint8_t*)&ProgArray;
	for(int i = 0; i < sizeof(ProgArray); i++) {
		*Pointer = 0xFF;
		Pointer++;
	}
}

void motor_ctrl(uint8_t enable, uint8_t direct) {
	if(enable == ON) {
		SetBitVal(M_EN_PORT, M_EN_PIN, HIGH);
		if(direct == FORWARD) {
			SetBitVal(M_INA_PORT, M_INA_PIN, HIGH);
			SetBitVal(M_INB_PORT, M_INB_PIN, LOW);
		}
		if(direct == BACKWARD) {
			SetBitVal(M_INA_PORT, M_INA_PIN, LOW);
			SetBitVal(M_INB_PORT, M_INB_PIN, HIGH);
		}
	}
	else {
		SetBitVal(M_EN_PORT, M_EN_PIN, LOW);
		SetBitVal(M_INA_PORT, M_INA_PIN, LOW);
		SetBitVal(M_INB_PORT, M_INB_PIN, LOW);
	}
}

void beep(uint16_t time, uint8_t cycle) {
	if(!Flag.Mute) {
		BeepTime = time;
		BeepCycle = cycle;
	}
}

void paused_timer() {
	TCCR1B = (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
}

void continued_timer() {
	TCCR1B = (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10);
}

void start_timer() {
	if((GTime > 0) || (WorkMode == ProgrammMode)) {
		TCNT1 = 0;
		TCCR1A = 0x00;
		TCCR1B = (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10);
		TCCR1C = 0x00;
#ifdef DEBUG
		OCR1A = 0x0C34;
#else
		OCR1A = 0x7A11;
#endif
		Flag.SecondDot = 0;

		beep(200, 1);
	}
}

void stop_timer() {
	TCNT1 = 0;
	TCCR1A = 0x00;
	TCCR1B = (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
	TCCR1C = 0x00;
	motor_ctrl(OFF, FORWARD);
	CurrentCycle = 0;
	Flag.ProgIsStarted = 0;
	ProgTime = prog_time(CurrentProgNumber);
	Flag.SecondDot = 0;
}

void adc_init() {
	ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (0 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void timer0_init() {
	TCCR0A = 0x00;
	TCCR0B = (1 << CS02) | (0 << CS01) | (0 << CS00);
	TCNT0 = 0x00;
	OCR0A = 0x31;
}

int8_t set_power(uint8_t val) {
	if(val > 100) return 0;
	else {
		if(val == 0) {
			TCCR2A = (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
			OCR2B = 0x00;
			Flag.IsStarted = 0;
			Flag.SoftStart = 0;
			SoftPower = 0;
		}
		else {
			TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (1 << WGM20);
#if SOFT_START == 0
			OCR2B = map(val, 0, 100, 0, 255);
#else
			if(!Flag.IsStarted) Flag.SoftStart = 1;
			else OCR2B = map(val, 0, 100, 0, 255);
#endif
		}
	}
	return val;
}

void timer2_init() {
	//TCCR2A = (1 << COM2B1) | (0 << COM2B0) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (0 << WGM22) | (0 << CS22) | (0 << CS21) | (1 << CS20);
	OCR2A = 0;
	OCR2B = 0;
}

#define ACC_VAL	1000

#if INTERNAL_ADC
ISR(ADC_vect) {
	static uint8_t Tact = 0;
	static uint64_t ADCAcc[2] = { 0 };
	static uint16_t AccCnt = 0;
	// uint8_t Input = 0;

	/*if(Tact == 0) {
		ADCData = float_window(ADCW);
		Input = AMPL_OFFSET_INPUT;
	}
	if(Tact == 1) {
		if(AccCnt < 100) {
			ADCAcc += ADCW;
		}
		if(++AccCnt == 100) {
			ADCDataOffsetVal = ADCAcc / 100;
			AccCnt = 0;
			ADCAcc = 0;
		}
		Input = ELECTRODE_INPUT;
	}*/

	ADCAcc[Tact] += ADCW;

	if(++Tact == 2) {
		Tact = 0;
		if(++AccCnt == ACC_VAL) {
			AccCnt = 0;

			ADCData[0] = ADCAcc[0] / (ACC_VAL * 2);
			ADCAcc[0] = 0;

			ADCData[1] = ADCAcc[1] / ACC_VAL;
			ADCAcc[1] = 0;
		}
	}

	ADMUX = 0x40 | Tact;
	ADCSRA |= (1 << ADSC);
}
#endif

ISR(TIMER0_COMPA_vect) {
	TCNT0 = 0x00;
	static uint16_t lCyclePause = 0;
	static uint16_t lBeepTime = 0;
	static uint16_t lReadTempCnt = 0;

	BUT_Poll();
	ENC_PollEncoder();

	if(++lReadTempCnt == 625) {
		lReadTempCnt = 0;
		Flag.ReadTemperature = 1;
	}

#if SOFT_START
	static uint8_t DivCnt = 0;
	if(Flag.SoftStart) {
		OCR2B = map(SoftPower, 0, 100, 0, 255);
		if(++DivCnt == 20) {
			DivCnt = 0;
			if(++SoftPower == PowerInManualMode) {
				SoftPower = 0;
				Flag.SoftStart = 0;
				Flag.IsStarted = 1;
			}
		}
	}
#endif

	if(ShowPower > 0) {
		if(--ShowPower == (SHOW_POWER_CYCLES - EN_CHANGE_POWER)) Flag.EnChangePower = 1;
	}

	if(BeepCycle > 0) {
		if((++lBeepTime < BeepTime) && (lCyclePause == 0)) {
			SetBitVal(BEEP_PORT, BEEP_PIN, HIGH);
		}
		else {
			SetBitVal(BEEP_PORT, BEEP_PIN, LOW);
			if(++lCyclePause >= BeepTime) {
				lCyclePause = 0;
				lBeepTime = 0;
				BeepCycle--;
			}
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	// Variable for manual mode
	static uint16_t lCycleTime = 0, lPTime = 0, lRTime = 0, lDirection = FORWARD, lState = Work;

	// Variable for programm mode
	static uint8_t lCycleCount = 0;

	TCNT1 = 0;

	if(Flag.InitLocalTimerVar) {
		Flag.InitLocalTimerVar = 0;
		lPTime = 0;
		lRTime = 0;
		lDirection = FORWARD;
		lState = Work;
		lCycleCount = 0;
	}

	if(WorkMode == ManualMode) {
		if(GTime > 0) {
			GTime--;
			if(lState == Work) {
				set_power(PowerInManualMode);
				motor_ctrl(ON, lDirection);
				if(++lRTime == RTime) {
					lRTime = 0;
					lState = Pause;
				}
			}
			else {
				set_power(0);
				motor_ctrl(OFF, lDirection);
				if(RTime > 0) {
					lDirection = !lDirection;
					beep(30, 1);
				}
				else {
					lDirection = FORWARD;
				}
				if(++lPTime == PTime) {
					lState = Work;
					lPTime = 0;
				}
			}
		}
		if (GTime == 0) {
			lPTime = 0;
			lRTime = 0;
			lDirection = FORWARD;
			lState = Work;
			stop_timer();
		}
	}

	if(WorkMode == ProgrammMode) {
		Flag.SecondDot = !Flag.SecondDot;
		if(lCycleCount < ProgArray[CurrentProgNumber].NuberOfCycles) {
			if(ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].State == Work) {

				if(lState == Work) {
					PowerInManualMode = set_power(ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].Power);
					motor_ctrl(ON, lDirection);
					if(++lRTime == ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].OWRTime) {
						lRTime = 0;
						lState = Pause;
					}
				}
				else {
					set_power(0);
					motor_ctrl(OFF, FORWARD);
					if(++lPTime == ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].PBRevers) {
						lPTime = 0;
						if(ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].OWRTime > 0) {
							lDirection = !lDirection;
						}
						else {
							lDirection = FORWARD;
						}
						lState = Work;
					}
				}
			}
			else {
				set_power(ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].Power);
				motor_ctrl(OFF, FORWARD);
			}

			if(++lCycleTime == ProgArray[CurrentProgNumber].ProgCycleArray[lCycleCount].FullCycleTime) {
				lCycleTime = 0;
				lCycleCount++;
				CurrentCycle = lCycleCount;
				beep(30, 1);
			}
			ProgTime--;
		}
		if(lCycleCount == ProgArray[CurrentProgNumber].NuberOfCycles) {
			lPTime = 0;
			lRTime = 0;
			lDirection = FORWARD;
			lState = Work;
			lCycleCount = 0;
			stop_timer();
			Flag.BeepOnStop = 1;
		}
	}
}

int main() {
	uint8_t EncoderState = 0;
	uint8_t Temp = 0;
	int8_t PHModeDispDataCnt = PHDisplay;

	float phBalance = 0.0;
	double OffsetVoltage = 0.0;
	double ElectrodeVoltage;
	float Temperature = 0.0;
	float pHSlope;

	MAX72xx_Init(7);
	BUT_Init();
	ENC_InitEncoder();

	if (BitIsClear(BUT_ENTER_PORT, BUT_ENTER_PIN)) {
		MAX72xx_OutSym("--Init--", 8);
		while(BitIsClear(BUT_ENTER_PORT, BUT_ENTER_PIN));
		_delay_ms(1000);
		MAX72xx_Clear(0);
		all_init();
		save_eeprom();
	}

	Version = eeprom_read_byte(0);
	eeprom_read_block((uint8_t*)&ProgArray, (uint8_t*)2, sizeof(ProgArray));

	if (Version == 0xFF) {
		MAX72xx_OutSym("ProGInit", 8);
		_delay_ms(1000);
		MAX72xx_Clear(0);
		Version = FW_VERSION;
		prog_init();
		save_eeprom();
	}

	MAX72xx_OutSym("--    --", 8);
	MAX72xx_OutIntFormat(Version, 4, 5, 5);

	OW_Reset();
	uint8_t nDevices = search_ow_devices();

	_delay_ms(500);
	MAX72xx_Clear(0);

	if (nDevices > 0) {
		MAX72xx_OutSym("Find    ", 8);
		MAX72xx_OutIntFormat(nDevices, 1, 1, 0);
		_delay_ms(500);
		MAX72xx_OutSym("dS 18b20", 8);
		_delay_ms(500);
		MAX72xx_OutSym("  SEnS  ", 8);

		DS18x20_Init( (uint8_t*)OneWireDevices[0], 25, 25, DS18B20_12_BIT );
		DS18x20_StartMeasure((uint8_t*)OneWireDevices[0]);
		_delay_ms(1000);
		DS18x20_ReadData((uint8_t*)OneWireDevices[0]);
		Temperature = DS18x20_ConvertThemperature();
		MAX72xx_Clear(0);
	}
	else {
		MAX72xx_OutSym("dS 18b20", 8);
		_delay_ms(500);
		MAX72xx_OutSym("IS FAuLt", 8);
		_delay_ms(500);
		MAX72xx_Clear(0);
		Temperature = 20.0;
	}

	SetBit(M_INA_DDR, M_INA_PIN);
	SetBit(M_INB_DDR, M_INB_PIN);
	SetBit(M_EN_DDR, M_EN_PIN);
	SetBit(M_PWM_DDR, M_PWM_PIN);
	SetBit(BEEP_DDR, BEEP_PIN);

	timer0_init();

	timer2_init();

	adc_init();

	TIMSK0 = (1 << OCIE0A);

	TIMSK1 = (1 << OCIE1A);

	WorkMode = PHMeterMode;

	Flag.ProgIsStarted = 0;
	Flag.BeepOnStop = 0;

	sei();

	set_power(0);
	motor_ctrl(OFF, FORWARD);

	Flag.Mute = 1;

	beep(300, 1);

	ProgTime = prog_time(CurrentProgNumber);

	if(ProgTime < 3600) Flag.SecondDot = 0;
	else Flag.SecondDot = 1;

	while(1) {
		get_but();
		EncoderState = ENC_GetStateEncoder();

		if(Flag.BeepOnStop == 1) {
			Flag.BeepOnStop = 0;
			MAX72xx_OutSym("--StoP--", 8);
			beep(200, 3);
			_delay_ms(1000);
			if(Flag.SaveEEPROM) {
				Flag.SaveEEPROM = 0;
				save_eeprom();
			}
			MAX72xx_Clear(0);
			ProgTime = prog_time(CurrentProgNumber);
			if(ProgTime < 3600) Flag.SecondDot = 0;
			else Flag.SecondDot = 1;
		}

		if((ButtonCode == BUT_ENTER) && (ButtonEvent == BUT_RELEASED_LONG_CODE) && (Flag.ProgIsStarted == 0)) {
			if(++WorkMode == EndModeList) {
				WorkMode = StartModeList + 1;
			}
			MAX72xx_Clear(0);
		}

		switch (WorkMode) {
			case ProgrammMode: {
				MAX72xx_OutSym("P", 8);
				MAX72xx_OutIntFormat(CurrentProgNumber + 1, 7, 7, (Flag.ProgIsStarted)?(7):(0));

				if(Flag.ProgIsStarted == 0) {
					MAX72xx_OutSym(" ", 6);
				}
				else {
					MAX72xx_OutIntFormat(CurrentCycle + 1, 6, 6, 0);
				}

				if(ShowPower == 0) {
					if(ProgTime < 3600) {
						MAX72xx_OutIntFormat(ProgTime / 60, 3, 4, 3);
						MAX72xx_OutIntFormat(ProgTime % 60, 1, 2, 3);
					}
					else {
						MAX72xx_OutIntFormat(ProgTime / 3600, 3, 4, 3);
						MAX72xx_OutIntFormat((ProgTime / 60) - 60, 1, 2, 3);
						MAX72xx_SetComma(1, Flag.SecondDot);
					}
					Flag.EnChangePower = 0;
				}
				else {
					MAX72xx_OutIntFormat(ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power, 1, 4, 0);
				}

				if((ButtonCode == BUT_ENTER) && (ButtonEvent == BUT_DOUBLE_CLICK_CODE)) {
					if(Flag.ProgIsStarted == 0) {
						Flag.ProgIsPaused = 0;
						Flag.ProgIsStarted = 1;
						start_timer();
					}
					else {
						if(Flag.ProgIsPaused == 0) {
							Flag.ProgIsPaused = 1;
							paused_timer();
						}
						else {
							Flag.ProgIsPaused = 0;
							continued_timer();
						}
					}
				}

				if((ButtonCode == BUT_ENTER) && (ButtonEvent == BUT_RELEASED_LONG_CODE) && (Flag.ProgIsStarted == 1)) {
					Flag.ProgIsPaused = 0;
					Flag.ProgIsStarted = 0;
					stop_timer();
					set_power(0);
					Flag.InitLocalTimerVar = 1;
					MAX72xx_OutSym("--StoP--", 8);
					beep(200, 3);
					_delay_ms(1000);
					if(Flag.SaveEEPROM) {
						Flag.SaveEEPROM = 0;
						save_eeprom();
					}
					MAX72xx_Clear(0);
					ProgTime = prog_time(CurrentProgNumber);
					if(ProgTime < 3600) Flag.SecondDot = 0;
					else Flag.SecondDot = 1;
				}

				if(Flag.ProgIsStarted == 0) {
					if(EncoderState == RIGHT_SPIN) {
						if(++CurrentProgNumber == PROG_NUMBERS) CurrentProgNumber = 0;
						ProgTime = prog_time(CurrentProgNumber);
						if(ProgTime < 3600) Flag.SecondDot = 0;
						else Flag.SecondDot = 1;
					}

					if(EncoderState == LEFT_SPIN) {
						if(--CurrentProgNumber < 0) CurrentProgNumber = PROG_NUMBERS - 1;
						ProgTime = prog_time(CurrentProgNumber);
						if(ProgTime < 3600) Flag.SecondDot = 0;
						else Flag.SecondDot = 1;
					}
				}
				else {
					if(EncoderState == RIGHT_SPIN) {
						if(ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].State == Work) {
							Temp = ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power;
							ShowPower = SHOW_POWER_CYCLES;
							if((ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power < 100) && (Flag.EnChangePower == 1)) {
								ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power++;
								set_power(ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power);
							}
							if(Temp != ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power) Flag.SaveEEPROM = 1;
						}
					}

					if(EncoderState == LEFT_SPIN) {
						if(ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].State == Work) {
							Temp = ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power;
							ShowPower = SHOW_POWER_CYCLES;
							if((ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power > 0) && (Flag.EnChangePower == 1)) {
								ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power--;
								set_power(ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power);
							}
							if(Temp != ProgArray[CurrentProgNumber].ProgCycleArray[CurrentCycle].Power) Flag.SaveEEPROM = 1;
						}
					}
				}
			} break;
			case ManualMode: {
				MAX72xx_OutSym("H", 8);
				MAX72xx_OutIntFormat(PowerInManualMode, 1, 3, 0);
				if(EncoderState == RIGHT_SPIN) {
					if(PowerInManualMode < 100) {
						PowerInManualMode++;
						if(Flag.ProgIsStarted) set_power(PowerInManualMode);
					}
				}
				if(EncoderState == LEFT_SPIN) {
					if(PowerInManualMode > 0) {
						PowerInManualMode--;
						if(Flag.ProgIsStarted) set_power(PowerInManualMode);
					}
				}

				if(Flag.ProgIsStarted == 0) {
					if((BitIsClear(BUT_2_PINX, BUT_2_PIN)) && (BitIsSet(BUT_3_PINX, BUT_3_PIN))) {
						MAX72xx_OutSym("-F-", 7);
						set_power(PowerInManualMode);
						//start_timer();
						motor_ctrl(ON, FORWARD);
					} else if((BitIsClear(BUT_3_PINX, BUT_3_PIN)) && (BitIsSet(BUT_2_PINX, BUT_2_PIN))) {
						MAX72xx_OutSym("-r-", 7);
						set_power(PowerInManualMode);
						//start_timer();
						motor_ctrl(ON, BACKWARD);
					} else {
						MAX72xx_OutSym("   ", 7);
						set_power(0);
						//stop_timer();
						motor_ctrl(OFF, BACKWARD);
					}
				}

				if((ButtonCode == BUT_ENTER) && (ButtonEvent == BUT_DOUBLE_CLICK_CODE) && (Flag.ProgIsStarted == 0)) {
					GTime = 7200;
					RTime = 30;
					MAX72xx_OutSym("En", 6);
					start_timer();
					Flag.ProgIsStarted = 1;
				}

				if((ButtonCode == BUT_ENTER) && (ButtonEvent == BUT_RELEASED_LONG_CODE) && (Flag.ProgIsStarted == 1)) {
					Flag.ProgIsStarted = 0;
					set_power(0);
					stop_timer();
					Flag.InitLocalTimerVar = 1;
					MAX72xx_OutSym("--StoP--", 8);
					beep(200, 3);
					_delay_ms(1000);
					MAX72xx_Clear(0);
				}


			} break;
			case PHMeterMode: {
				ElectrodeVoltage = (ADC_VOLTAGE / (double)1024.0 * (double)ADCData[0]);					// Напряжение электрода
				OffsetVoltage = (ADC_VOLTAGE / (double)1024.0 * (double)ADCData[1]);					// Напряжение смещения

				if((Flag.ReadTemperature) && (nDevices)){
					Flag.ReadTemperature = 0;
					cli();
					if (DS18x20_ReadData((uint8_t*)OneWireDevices[0]) == 0) {
						nDevices = 0;
						Temperature = 20.0;
					}
					else Temperature = DS18x20_ConvertThemperature();
					if (DS18x20_StartMeasure((uint8_t*)OneWireDevices[0]) == 0) {
						nDevices = 0;
						Temperature = 20.0;
					}
					sei();
				}

				if(EncoderState == RIGHT_SPIN) {
					if(++PHModeDispDataCnt >= EndPHModeDispDataEnum) {
						PHModeDispDataCnt = PHDisplay;
					}
				}
				if(EncoderState == LEFT_SPIN) {
					if(--PHModeDispDataCnt < PHDisplay) {
						PHModeDispDataCnt = EndPHModeDispDataEnum - 1;
					}
				}

				switch (PHModeDispDataCnt) {
					case PHDisplay: {
						MAX72xx_OutSym("PH", 8);
						pHSlope = get_pH_slope(Temperature);
						float Offset = ISOPC_PH + (1.0 / pHSlope * ISOPC_E);

						phBalance = Offset + mapf(ElectrodeVoltage - OffsetVoltage, pHSlope, 0.0, 1.0, 0.0);

						//phBalance = -((Voltage - OffsetVoltage) / pHSlope) + Offset;

						MAX72xx_OutIntFormat(phBalance * 100, 1, 5, 3);
					} break;
					case TemperatureDisplay: {
						MAX72xx_OutSym("t", 8);
						if(nDevices) MAX72xx_OutSym(" ", 7);
						else MAX72xx_OutSym("c", 7);
						MAX72xx_OutSym("*", 1);
						MAX72xx_OutIntFormat(Temperature * 10.0, 2, 5, 3);
					} break;
					case VoltageDisplay: {
						MAX72xx_OutSym("UE", 8);
						MAX72xx_OutIntFormat(ElectrodeVoltage * 1000.0, 1, 5, 4);
					} break;
					case RefVoltageDisplay: {
						MAX72xx_OutSym("Uo", 8);
						MAX72xx_OutIntFormat(OffsetVoltage * 1000.0, 1, 5, 4);
					} break;
					default: break;
				}

			} break;
			default: break;
		}
	}
}

