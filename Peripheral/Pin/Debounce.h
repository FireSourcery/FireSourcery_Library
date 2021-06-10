/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#ifndef PIN_DEBOUNCE_H
#define PIN_DEBOUNCE_H

#include "HAL_Pin.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	const volatile HAL_Pin_T * p_HAL_Pin;

	const volatile uint32_t * p_Timer;
	uint16_t DebounceTime;

	volatile uint16_t TimePrev;
	volatile bool DebouncedState;
	volatile bool DebouncedStatePrev;
	volatile bool RawStatePrev;

} Debounce_T;



static inline bool Debounce_CaptureState_IO(Debounce_T * p_pin) //run prior to get
{
    bool pinState = HAL_Pin_ReadState(p_pin->p_HAL_Pin);

    //check if state is the same for specified duration, vs lock out - noise may lock out real input
	if (pinState != p_pin->RawStatePrev)
	{
		p_pin->TimePrev = *p_pin->p_Timer;
		p_pin->RawStatePrev = pinState;  //RawStatePrev ^= pinState;
	}
	else
	{
		if (*p_pin->p_Timer > p_pin->TimePrev + p_pin->DebounceTime)
		{
//			p_pin->TimePrev = UINT32_MAX - p_pin->DebounceTime  ; //disable until timer loops?
			p_pin->DebouncedStatePrev = p_pin->DebouncedState;
			p_pin->DebouncedState = pinState;
		}
	}

	//todo capturetime for long and short press

	//if updated
	return (p_pin->DebouncedStatePrev ^ p_pin->DebouncedState);
}

static inline bool Debounce_GetRawState(Debounce_T *p_pin)
{
	return p_pin->RawStatePrev;
}

static inline bool Debounce_GetState(Debounce_T *p_pin)
{
	return p_pin->DebouncedState;
}

static inline bool Debounce_GetFallingEdge(Debounce_T *p_pin)
{
	return ((p_pin->DebouncedState == false) && (p_pin->DebouncedStatePrev == true)) ? true : false;
}

static inline bool Debounce_GetRisingEdge(Debounce_T *p_pin)
{
	return ((p_pin->DebouncedState == true) && (p_pin->DebouncedStatePrev == false)) ? true : false;

//	must clear rising edge if read is separate from Capture
}


static inline void Debounce_SetTime(Debounce_T * p_pin, uint16_t millis)
{
	p_pin->DebounceTime = millis;
}


extern void Debounce_Init(Debounce_T *p_pin, const volatile HAL_Pin_T *p_hal_pin, const volatile uint32_t *p_Timer, uint16_t debounceTime);



//static inline unsigned Debounce_GetStateTime()   { }
//static inline bool Debounce_GetLongPress(Debounce_T * p_pin){}
//static inline bool Debounce_GetShortPress(Debounce_T * p_pin){}



//static inline bool Pin_PremptiveDebounce_GetState(Debounce_T * p_pin)
//{
//
//
//    //lock out and set blank time
//
//}

#endif /* PIN_DEBOUNCE_H */
