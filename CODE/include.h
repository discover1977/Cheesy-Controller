/*
 * include.h
 *
 *  Created on: 19 сент. 2018 г.
 *      Author: gavrilov.iv
 */

#ifndef INCLUDE_H_
#define INCLUDE_H_

//#define DEBUG

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <math.h>

#include "bits_macros.h"
#include "MAX72xx.h"
#include "buttons.h"
#include "onewire.h"
#include "ds18x20.h"

#ifndef DEBUG
	#include "encoder.h"
#endif

#endif /* INCLUDE_H_ */
