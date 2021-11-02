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
	GPIO_Type * P_GPIO_BASE;
	uint32_t GPIO_PIN_MASK;
} HAL_Pin_T;

static inline void HAL_Pin_WriteState(const HAL_Pin_T * p_pin, bool isOn)
{
//	isOn ? (p_pin->P_GPIO_BASE->PDOR |= p_pin->GPIO_PIN_MASK) : (p_pin->P_GPIO_BASE->PDOR &= ~(p_pin->GPIO_PIN_MASK));
	isOn ? (p_pin->P_GPIO_BASE->PSOR |= p_pin->GPIO_PIN_MASK) : (p_pin->P_GPIO_BASE->PCOR |= p_pin->GPIO_PIN_MASK);
}

static inline bool HAL_Pin_ReadState(const HAL_Pin_T * p_pin)
{
	return ((p_pin->P_GPIO_BASE->PDIR & p_pin->GPIO_PIN_MASK) == p_pin->GPIO_PIN_MASK) ? true : false;
}

static inline void HAL_Pin_InitInput(const HAL_Pin_T * p_pin)
{
	p_pin->P_GPIO_BASE->PDDR &= ~(p_pin->GPIO_PIN_MASK);
	p_pin->P_GPIO_BASE->PIDR &= ~(p_pin->GPIO_PIN_MASK);
}

static inline void HAL_Pin_InitOutput(const HAL_Pin_T * p_pin)
{
	p_pin->P_GPIO_BASE->PDDR |= (p_pin->GPIO_PIN_MASK);
	p_pin->P_GPIO_BASE->PIDR |= (p_pin->GPIO_PIN_MASK);
}

static inline void HAL_Pin_Deinit(const HAL_Pin_T * p_pin)
{
	p_pin->P_GPIO_BASE->PDDR &= ~(p_pin->GPIO_PIN_MASK);
	p_pin->P_GPIO_BASE->PIDR |= (p_pin->GPIO_PIN_MASK);
}

#endif
