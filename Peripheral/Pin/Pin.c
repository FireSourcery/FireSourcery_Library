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
    @file 	Pin.c
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Pin.h"

bool Pin_Input_Read(const Pin_T * p_pin) {return HAL_Pin_ReadInput(p_pin->P_HAL_PIN, GetPinHalArg(p_pin));}
void Pin_Input_Init(const Pin_T * p_pin) {HAL_Pin_InitInput(p_pin->P_HAL_PIN, GetPinHalArg(p_pin));}
void Pin_Deinit(const Pin_T * p_pin) {HAL_Pin_Deinit(p_pin->P_HAL_PIN, GetPinHalArg(p_pin));}
