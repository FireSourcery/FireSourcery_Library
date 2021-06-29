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

#if defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include "HAL/HAL.h"
#elif defined(CONFIG_HAL_LIBRARY_USER_DEFINED)
	#include "HAL.h"
#elif defined(CONFIG_HAL_ENCODER_USER_DEFINED)
	#include "HAL_Encoder.h"
#endif

//#if defined(CONFIG_ENCODER_HAL_S32K)
//	#include "HAL/Platform/S32K/HAL_Encoder.h"
//#elif defined(CONFIG_ENCODER_HAL_KLS_S32K)
//	#include "HAL/Board/KLS_S32K/HAL_Encoder.h"
//#elif defined(CONFIG_ENCODER_HAL_XYZ)
//
//#elif defined(CONFIG_ENCODER_HAL_USER_DEFINED)
//	#include "HAL_Encoder.h"
///*
//	typedef struct
//	{
//		HAL_TimerCounter_T * p_TimerCounter;
//		uint8_t TimerCounterChannel;
//
//		HAL_Pin_T * p_PinPhaseA;
//		uint32_t PinMaskPhaseA;
//
//		HAL_Pin_T * p_PinPhaseB;
//		uint32_t PinMaskPhaseB;
//	} HAL_Encoder_T;
//
//	static inline uint32_t HAL_Encoder_ReadTimerCounter(const HAL_Encoder_T * p_encoder){ }
//	static inline bool HAL_Encoder_ReadDirection(const HAL_Encoder_T * p_encoder){ }
//	static inline bool HAL_Encoder_ReadPhaseA(const HAL_Encoder_T * p_encoder){ }
//	static inline bool HAL_Encoder_ReadPhaseB(const HAL_Encoder_T * p_encoder){ }
//	static inline bool HAL_Encoder_EnableInterrupt(const HAL_Encoder_T * p_encoder){ }
//*/
////Timer_ConfigPeriodicIRQ
////Timer_ConfigCaptureIRQ
//#endif

#endif
