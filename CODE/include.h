/*
 * include.h
 *
 *  Created on: 19 ����. 2018 �.
 *      Author: gavrilov.iv
 */

#ifndef INCLUDE_H_
#define INCLUDE_H_

//#define DEBUG

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "bits_macros.h"
#include "MAX72xx.h"

#include "buttons.h"

#ifndef DEBUG
	#include "encoder.h"
#endif

#endif /* INCLUDE_H_ */
