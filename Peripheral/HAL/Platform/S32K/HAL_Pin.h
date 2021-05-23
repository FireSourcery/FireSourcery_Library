/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/
#ifndef HAL_PIN_PLATFORM_H
#define HAL_PIN_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	GPIO_Type * p_GpioBase;
	uint32_t GpioPinMask;
} HAL_Pin_T;

static inline void HAL_Pin_WriteState(const HAL_Pin_T * p_pin, bool isOn)
{
	isOn ? (p_pin->p_GpioBase->PDOR |= p_pin->GpioPinMask) : (p_pin->p_GpioBase->PDOR &= ~(p_pin->GpioPinMask));
}

static inline bool HAL_Pin_ReadState(const HAL_Pin_T * p_pin)
{
	return ((p_pin->p_GpioBase->PDIR | p_pin->GpioPinMask) != 0U) ? true : false;
}

#endif
