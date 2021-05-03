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
	@file 	HAL_Phase.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_PHASE_PLATFORM_H
#define HAL_PHASE_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	FTM_Type * p_FtmA;
	uint8_t FtmChannelA;

	GPIO_Type * p_GpioA;
	uint32_t GpioPinMaskA;
	//	FTM_Type * p_FtmA;
	//	uint8_t FtmChannelA;
	//
	//	GPIO_Type * p_GpioA;
	//	uint32_t GpioPinMaskA;
	//	FTM_Type * p_FtmA;
	//	uint8_t FtmChannelA;
	//
	//	GPIO_Type * p_GpioA;
	//	uint32_t GpioPinMaskA;
} HAL_Phase_T;

/*
	true is inverted, false is noninverted
 */
static inline void HAL_Phase_WriteInvertPolarityA(const HAL_Phase_T * p_phase, bool isInvertedPolarity)
{
//	isInvertedPolarity ? (p_phase->p_FtmBase->POL |= (1U << p_phase->FtmChannel)) : (p_phase->p_FtmBase->POL &= ~(1U << p_phase->FtmChannel));
}

/*
	KLS board uses additional hw enable/disable pin
 */
static inline void HAL_Phase_WriteStateA(const HAL_Phase_T * p_phase, bool isOn)
{
//	isOn ? (p_phase->p_GpioBase->PDOR |= p_phase->GpioPinMask) : (p_phase->p_GpioBase->PDOR &= ~(p_phase->GpioPinMask));

}

static inline void HAL_Phase_WriteState(const HAL_Phase_T * p_phase, bool isOnPwmA, bool isOnPwmB, bool isOnPwmC)
{
	isOnPwmA ? (p_phase->p_FtmA->OUTMASK |= ((uint32_t)1U << p_phase->FtmChannelA)) : (p_phase->p_FtmA->OUTMASK &= ~((uint32_t)1U << p_phase->FtmChannelA));
}

#endif
