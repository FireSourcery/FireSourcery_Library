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
    @file 	Pin.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#ifndef PIN_H
#define PIN_H

#include "HAL_Pin.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	volatile const HAL_Pin_T * p_HAL_Pin;

	volatile const uint32_t * p_Timer;
	uint16_t DebounceTime;

	volatile uint16_t TimePrev;
	volatile bool DebouncedState;
	volatile bool DebouncedStatePrev;
	volatile bool RawStatePrev;

}Pin_T;

//typedef struct
//{
//	volatile const HAL_Pin_T * p_HAL_Pin;
//
//	volatile const uint32_t * p_Timer;
//	uint16_t DebounceTime;
//
//	volatile uint16_t TimePrev;
//	volatile bool DebouncedState;
//	volatile bool DebouncedStatePrev;
//	volatile bool RawStatePrev;
//}Pin_Bounce_T;


static inline bool Pin_ReadState(Pin_T * p_pin)
{
	HAL_Pin_ReadState(p_pin->p_HAL_Pin);
}

static inline void Pin_WriteState(Pin_T *p_pin, bool isOn)
{
	HAL_Pin_WriteState(p_pin->p_HAL_Pin, isOn);
}

static inline void Pin_Init(Pin_T *p_pin, HAL_Pin_T *p_hal_pin, uint32_t *p_Timer, uint16_t debounceTime)
{
	p_pin->p_HAL_Pin = p_hal_pin;

	p_pin->p_Timer = p_Timer;
	p_pin->DebounceTime = debounceTime;

	p_pin->DebouncedState = HAL_Pin_ReadState(p_pin->p_HAL_Pin);
	p_pin->DebouncedStatePrev = p_pin->DebouncedState;
	p_pin->RawStatePrev = p_pin->DebouncedState;
}

static inline bool Pin_Debounce_CaptureState_IO(Pin_T * p_pin) //run prior to get
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
//				p_pin->TimePrev = UINT32_MAX - p_pin->DebounceTime  ; //disable until timer loops?
				p_pin->DebouncedStatePrev = p_pin->DebouncedState;
				p_pin->DebouncedState  = pinState;

		}
	}

	//capturetime for long and short press
	return (p_pin->DebouncedStatePrev ^ p_pin->DebouncedState);
}


static inline bool Pin_Debounce_GetState(Pin_T *p_pin)
{
	return p_pin->DebouncedState;
}

static inline bool Pin_Debounce_GetFallingEdge(Pin_T *p_pin)
{
	return ((p_pin->DebouncedState == false) && (p_pin->DebouncedStatePrev == true)) ? true : false;
}

static inline bool Pin_Debounce_GetRisingEdge(Pin_T *p_pin)
{
	return ((p_pin->DebouncedState == true) && (p_pin->DebouncedStatePrev == false)) ? true : false;

//	must clear rising edge if read is separate from Capture
}


static inline void Pin_Debounce_SetTime(Pin_T * p_pin, uint16_t millis)
{
	p_pin->DebounceTime = millis;
}



//static inline unsigned Pin_Debounce_GetStateTime()   { }
//static inline bool Pin_Debounce_GetLongPress(Pin_T * p_pin){}
//static inline bool Pin_Debounce_GetShortPress(Pin_T * p_pin){}



//static inline bool Pin_PremptiveDebounce_GetState(Pin_T * p_pin)
//{
//
//
//    //lock out and set blank time
//
//}

//typedef struct
//{
//	HAL_Pin_T * p_HAL_Pin;
//
//	uint32_t PinsMask;
//
//	uint16_t DebounceTime_Ms;
//
//	uint8_t * p_VirtualPinMap;
//
//}Pins_T;
//
//void Pins_Register_GetPinState(Pins_T * p_pins, uint8_t virtualPinIndex)
//{
//	HAL_Pin_ReadRegisterState(p_pins->p_HAL_Pin, p_pins->p_VirtualPinMap[virtualPinIndex]);
//}
#endif
