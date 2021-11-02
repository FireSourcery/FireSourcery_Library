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
	const volatile HAL_Pin_T * p_HAL_Pin;

//	const volatile uint32_t * p_Timer;
//	uint16_t DebounceTime;
//
//	volatile uint16_t TimePrev;
//	volatile bool DebouncedState;
//	volatile bool DebouncedStatePrev;
//	volatile bool RawStatePrev;

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
	return HAL_Pin_ReadState(p_pin->p_HAL_Pin);
}

static inline void Pin_WriteState(Pin_T *p_pin, bool isOn)
{
	return HAL_Pin_WriteState(p_pin->p_HAL_Pin, isOn);
}



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
