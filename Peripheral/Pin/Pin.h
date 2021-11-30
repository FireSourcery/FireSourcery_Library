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
    @file 	Pin.h
    @author FireSoucery
    @brief	Uniform Wrapper for HAL_Pin
    @version V0
*/
/******************************************************************************/
#ifndef PIN_H
#define PIN_H

#include "Config.h"
#include "HAL_Pin.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	HAL_Pin_T * const P_HAL_PIN;
	const uint32_t ID;
	const uint32_t MASK;
}
Pin_T;

#define PIN_CONFIG(p_Hal, Id)	\
{								\
	.P_HAL_PIN = p_Hal,			\
	.ID = Id,					\
	.MASK = (1UL << Id)			\
}

#ifdef CONFIG_PIN_HAL_USE_MASK
	#define PIN_ARG_ID p_pin->MASK
#elif defined(CONFIG_PIN_HAL_USE_ID)
	#define PIN_ARG_ID p_pin->ID
#endif

static inline void Pin_Input_Init(const Pin_T * p_pin) {HAL_Pin_InitInput(p_pin->P_HAL_PIN, PIN_ARG_ID);}
static inline bool Pin_Input_Read(const Pin_T * p_pin) {return HAL_Pin_ReadInput(p_pin->P_HAL_PIN, PIN_ARG_ID);}

static inline void Pin_Output_Init(const Pin_T * p_pin) 				{HAL_Pin_InitOutput(p_pin->P_HAL_PIN, PIN_ARG_ID); HAL_Pin_WriteOutputOff(p_pin->P_HAL_PIN, PIN_ARG_ID);}
static inline void Pin_Output_Write(const Pin_T * p_pin, bool isOn) 	{return HAL_Pin_WriteOutput(p_pin->P_HAL_PIN, PIN_ARG_ID, isOn);}
static inline void Pin_Output_Off(const Pin_T * p_pin) 					{return HAL_Pin_WriteOutputOff(p_pin->P_HAL_PIN, PIN_ARG_ID);}
static inline void Pin_Output_On(const Pin_T * p_pin)					{return HAL_Pin_WriteOutputOn(p_pin->P_HAL_PIN, PIN_ARG_ID);}

static inline void Pin_Deinit(const Pin_T * p_pin) {HAL_Pin_Deinit(p_pin->P_HAL_PIN, PIN_ARG_ID);}

#endif
