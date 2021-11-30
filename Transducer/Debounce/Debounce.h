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
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef PIN_DEBOUNCE_H
#define PIN_DEBOUNCE_H

#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	const Pin_T PIN;
	const volatile uint32_t * P_TIMER;
} Debounce_Config_T;

typedef struct
{
	Debounce_Config_T CONFIG;

	uint16_t DebounceTime;

	uint16_t TimePrev;
	bool DebouncedState;
	bool DebouncedStatePrev;
	bool RawStatePrev;
} Debounce_T;

#define DEBOUNCE_CONFIG(p_Pin_Hal, Pin_Id, p_Timer) \
{													\
	.CONFIG = 										\
	{												\
		.PIN = PIN_CONFIG(p_Pin_Hal, Pin_Id),		\
		.P_TIMER = p_Timer,							\
	},												\
}


/*
 * return true if state changed
 */
static inline bool Debounce_PollCaptureState(Debounce_T * p_debounce)
{
    bool pinState = Pin_Input_Read(&p_debounce->CONFIG.PIN);

    /*
     * check if state is the same for specified duration,
     * vs preemptive lock out - noise may lock out real input
     */
	if (pinState != p_debounce->RawStatePrev)
	{
		p_debounce->TimePrev = *p_debounce->CONFIG.P_TIMER;
		p_debounce->RawStatePrev = pinState;  //RawStatePrev ^= pinState;
	}
	else
	{
		if (*p_debounce->CONFIG.P_TIMER - p_debounce->TimePrev > p_debounce->DebounceTime)
		{
			p_debounce->TimePrev = UINT32_MAX - p_debounce->DebounceTime; //disable until next change in pin
			p_debounce->DebouncedStatePrev = p_debounce->DebouncedState;
			p_debounce->DebouncedState = pinState;
		}
	}

	return (p_debounce->DebouncedStatePrev ^ p_debounce->DebouncedState);
}

static inline bool Debounce_GetRawState(Debounce_T *p_debounce)
{
	return p_debounce->RawStatePrev;
}

static inline bool Debounce_GetState(Debounce_T *p_debounce)
{
	return p_debounce->DebouncedState;
}

static inline bool Debounce_GetFallingEdge(Debounce_T * p_debounce)
{
	bool isEdge = ((p_debounce->DebouncedState == false) && (p_debounce->DebouncedStatePrev == true)) ? true : false;
	if (isEdge == true) {p_debounce->DebouncedStatePrev = p_debounce->DebouncedState;}
	return isEdge;
}

static inline bool Debounce_GetRisingEdge(Debounce_T * p_debounce)
{
	bool isEdge = ((p_debounce->DebouncedState == true) && (p_debounce->DebouncedStatePrev == false)) ? true : false;
	if (isEdge == true) {p_debounce->DebouncedStatePrev = p_debounce->DebouncedState;}
	return isEdge;
}

static inline void Debounce_SetTime(Debounce_T * p_debounce, uint16_t millis)
{
	p_debounce->DebounceTime = millis;
}

extern void Debounce_Init(Debounce_T * p_debounce, uint16_t debounceTime);


//static inline unsigned Debounce_GetStateTime()   { }
//static inline bool Debounce_GetLongPress(Debounce_T * p_debounce){}
//todo capturetime for long and short press
//static inline bool Debounce_GetShortPress(Debounce_T * p_debounce){}
//static inline bool Pin_PremptiveDebounce_GetState(Debounce_T * p_debounce)
//{
//    //lock out and set blank time
//}

#endif /* PIN_DEBOUNCE_H */
