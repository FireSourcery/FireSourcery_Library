/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
	@file  	Blinky.h
	@author FireSourcery
	@brief 	Pin Indicator

	@version V0
 */
/******************************************************************************/
#ifndef BLINKY_H
#define BLINKY_H

#include <stdint.h>
#include <stdbool.h>

#include "Peripheral/Pin/Pin.h"
#include "Utility/Timer/Timer.h"


typedef struct
{
	void (*Pattern)(void * p_context);
	void * p_Context;
} Blinky_Pattern_T;

typedef struct
{
	const Pin_T PIN;
	Timer_T Timer;

	volatile bool IsOn;

//	Blinky_Pattern_T * p_PatternActive;

	uint32_t Index;
	uint32_t Max;

//	uint32_t OnTime;
//	uint32_t OffTime;
//	uint32_t OnTimeActive;
}
Blinky_T;

#define BLINKY_CONFIG(p_Pin_Hal, Pin_Id, p_TimerBase, TimerBaseFreq) 	\
{															\
	.PIN 	= PIN_CONFIG(p_Pin_Hal, Pin_Id),				\
	.Timer 	= TIMER_CONFIG(p_TimerBase, TimerBaseFreq)		\
}

extern void Blinky_Init(Blinky_T * p_blinky);
extern void Blinky_Proc(Blinky_T * p_blinky);
extern void Blinky_On(Blinky_T * p_blinky);
extern void Blinky_Off(Blinky_T * p_blinky);
extern void Blinky_Toggle(Blinky_T * p_blinky);
extern void Blinky_Blink(Blinky_T * p_blinky, uint32_t duration);

#endif
