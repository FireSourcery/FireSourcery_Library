/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
	@file  	HAL.h
	@author FireSourcery
	@brief 	Encoder module import functions.
			User must provide HW functions, or configure HAL
	@version V0
 */
/******************************************************************************/
#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

#if defined(CONFIG_ENCODER_HAL_S32K)
	#include "HAL/Platform/S32K/HAL_Encoder.h"
#elif defined(CONFIG_ENCODER_HAL_XYZ)

#elif defined(CONFIG_ENCODER_HAL_USER_DEFINED)

typedef struct
{
	void * p_TimerCounterBase;
	uint8_t TimerCounterChannel;
} HAL_Encoder_T;

extern inline uint32_t HAL_Encoder_ReadTimerCounter(const HAL_Encoder_T * p_encoder);
extern inline bool HAL_Encoder_ReadDirection(const HAL_Encoder_T * p_encoder);
extern inline bool HAL_Encoder_ReadPhaseA(const HAL_Encoder_T * p_encoder);
extern inline bool HAL_Encoder_ReadPhaseB(const HAL_Encoder_T * p_encoder);
extern inline bool HAL_Encoder_EnableInterrupt(const HAL_Encoder_T * p_encoder);
//Timer_ConfigPeriodicIRQ
//Timer_ConfigCaptureIRQ
#endif



#endif

