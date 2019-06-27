/*
 * main.c
 *
 *  Created on: 17 сент. 2018 г.
 *      Author: gavrilov.iv
 */

#include "include.h"

#define FW_VERSION		12
#define BUT_ENTER		BUT_1_ID
#define BUT_ENTER_PINNUM	7
#define BUT_ENTER_PIN	PIND
#define BUT_ENTER_DDR	DDRD
#define BUT_ENTER_PORT	PORTD
#define SOFT_START		1

#ifdef DEBUG
	#define BUT_UP		BUT_1_ID
	#define BUT_DOWN	BUT_3_ID
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
#define AccSizeMacro(val)	(1 << val)

enum Mode {
	StartModeList,
	ProgrammMode,
	ManualMode,
	EndModeList
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

	MAX72xx_Init(7);

	ENC_InitEncoder();
	BUT_Init();

	if (BitIsClear(BUT_ENTER_PIN, BUT_ENTER_PINNUM)) {
		MAX72xx_OutSym("--Init--", 8);
		while(BitIsClear(BUT_ENTER_PIN, BUT_ENTER_PINNUM));
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

	_delay_ms(1000);
	MAX72xx_Clear(0);

	SetBit(M_INA_DDR, M_INA_PIN);
	SetBit(M_INB_DDR, M_INB_PIN);
	SetBit(M_EN_DDR, M_EN_PIN);
	SetBit(M_PWM_DDR, M_PWM_PIN);
	SetBit(BEEP_DDR, BEEP_PIN);

	timer0_init();

	timer2_init();

	TIMSK0 = (1 << OCIE0A);

	TIMSK1 = (1 << OCIE1A);

	WorkMode = ProgrammMode;

	Flag.ProgIsStarted = 0;
	Flag.BeepOnStop = 0;

	sei();

	set_power(0);
	motor_ctrl(OFF, FORWARD);

	Flag.Mute = 0;

	beep(200, 1);

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
						MAX72xx_OutIntFormat(ProgTime / 60, 3, 5, 3);
						MAX72xx_OutIntFormat(ProgTime % 60, 1, 2, 3);
					}
					else {
						MAX72xx_OutIntFormat(ProgTime / 3600, 3, 5, 3);
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
			default: break;
		}
	}
}

