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
#ifndef HAL_PHASE_BOARD_H
#define HAL_PHASE_BOARD_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
//	uint32_t InstanceID;
} HAL_Phase_T;

/*
 * Use compile time constant for improved performance.
 */
//static inline void WritePeriodA(uint32_t pwmPeroid){FTM0->CONTROLS[5].CnV = pwmPeroid;}
//static inline void WritePeriodB(uint32_t pwmPeroid){FTM0->CONTROLS[6].CnV = pwmPeroid;}
//static inline void WritePeriodC(uint32_t pwmPeroid){FTM0->CONTROLS[7].CnV = pwmPeroid;}

static inline void HAL_Phase_WritePeriod(const HAL_Phase_T * p_phase, uint32_t pwmPeroidA, uint32_t pwmPeroidB, uint32_t pwmPeroidC)
{
	(void)p_phase;

	FTM0->CONTROLS[5].CnV = pwmPeroidA;
	FTM0->CONTROLS[6].CnV = pwmPeroidB;
	FTM0->CONTROLS[7].CnV = pwmPeroidC;
}

//static inline void WriteStateA(bool isOn);

/*
	KLS board uses additional hw enable/disable pin
 */
static inline void HAL_Phase_WriteState(const HAL_Phase_T * p_phase, bool isOnPwmA, bool isOnPwmB, bool isOnPwmC)
{
	(void)p_phase;
	//Optionally also mask pwm module output?
	isOnPwmA ? (PTE->PDOR |= ((uint32_t)1U << 0U)) : (PTE->PDOR &= ~((uint32_t)1U << 0U));
	isOnPwmB ? (PTE->PDOR |= ((uint32_t)1U << 1U)) : (PTE->PDOR &= ~((uint32_t)1U << 1U));
	isOnPwmC ? (PTE->PDOR |= ((uint32_t)1U << 2U)) : (PTE->PDOR &= ~((uint32_t)1U << 2U));
}

/*
	true is inverted, false is noninverted
 */
static inline void HAL_Phase_WriteInvertPolarity(const HAL_Phase_T * p_phase, bool isInvPwmA, bool isInvPwmB, bool isInvPwmC)
{
	(void)p_phase;

	isInvPwmA ? (FTM0->POL |= FTM_POL_POL5_MASK) : (FTM0->POL &= ~FTM_POL_POL5_MASK);
	isInvPwmB ? (FTM0->POL |= FTM_POL_POL6_MASK) : (FTM0->POL &= ~FTM_POL_POL6_MASK);
	isInvPwmC ? (FTM0->POL |= FTM_POL_POL7_MASK) : (FTM0->POL &= ~FTM_POL_POL7_MASK);
}

#endif
