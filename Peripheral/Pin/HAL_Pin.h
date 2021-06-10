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
    @file 	HAL.h
    @author FireSoucery
    @brief  Motor module HAL
    @version V0
*/
/**************************************************************************/
#ifndef HAL_PIN_H
#define HAL_PIN_H

#include "Config.h"

#if defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include "Peripheral/HAL/HAL.h"
#elif defined(CONFIG_HAL_USER_DEFINED)
	#include "HAL/HAL.h"
#elif defined(CONFIG_HAL_PIN_USER_DEFINED)
	#include "HAL/HAL_Pin.h"
#endif

//#ifdef CONFIG_PIN_HAL_S32K
//	#include "Peripheral/HAL/Platform/S32K/HAL_Pin.h"
//#elif defined(CONFIG_PIN_HAL_USER_DEFINED)
//
//typedef const struct
//{
//	void * p_PinBase;
//	uint32_t PinMask;
//} HAL_Pin_T;
//
//extern inline void HAL_Pin_WriteState(const HAL_Pin_T * p_pin, bool isOn);
//extern inline bool HAL_Pin_ReadState(const HAL_Pin_T * p_pin);
//
//#endif

#endif
