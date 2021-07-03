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
	@file 	HAL_Serial.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_SERIAL_H
#define HAL_SERIAL_H

#include "HAL/HAL.h"

#if 	defined(CONFIG_HAL_SERIAL_USER_DEFINED) || defined(CONFIG_HAL_USER_DEFINED)
	#include HAL_PATH_USER(HAL_Serial.h)
#elif 	defined(CONFIG_HAL_SERIAL_LIBRARY_DEFINED) || defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include HAL_PATH_PLATFORM(Peripheral/HAL, HAL_Serial.h)
#endif

#endif
