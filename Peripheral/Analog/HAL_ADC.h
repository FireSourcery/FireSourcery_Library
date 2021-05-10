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
	@file 	HAL.h
	@author FireSoucery
	@brief 	Analog module imports the following ADC functions.
			User must provide HW functions, or configure peripheral HAL
	@version V0
*/
/******************************************************************************/
#ifndef HAL_ANALOG_H
#define HAL_ANALOG_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

//#if defined(CONFIG_ANALOG_HAL_S32K)
//	#include "Peripheral/HAL/Platform/S32K/HAL_ADC.h"
//#elif defined(CONFIG_ANALOG_HAL_USER_DEFINED)
//	#include "HAL/HAL_ADC.h"
//#endif

#if defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include "Peripheral/HAL/HAL.h"
#elif defined(CONFIG_HAL_USER_DEFINED)
	#include "HAL/HAL.h"
#elif defined(CONFIG_HAL_ADC_USER_DEFINED)
	#include "HAL/HAL_ADC.h"
#endif


#endif
