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
	@file 	HAL_PWM.h
	@author FireSoucery
	@brief 	PWM HAL for S32K
	@version V0
*/
/******************************************************************************/
#ifndef HAL_PWM_H
#define HAL_PWM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	FTM_Type * p_FtmBase;
	uint8_t FtmChannel;
} HAL_PWM_T;

static inline void HAL_PWM_WritePeriod(const HAL_PWM_T * p_pwm, uint32_t pwmPeroid)
{
	p_pwm->p_FtmBase->CONTROLS[p_pwm->FtmChannel].CnV = pwmPeroid;
}

/*
	true is inverted, false is noninverted
 */
static inline void HAL_PWM_WriteInvertPolarity(const HAL_PWM_T * p_pwm, bool isInvertedPolarity)
{
	isInvertedPolarity ? (p_pwm->p_FtmBase->POL |= (1U << p_pwm->FtmChannel)) : (p_pwm->p_FtmBase->POL &= ~(1U << p_pwm->FtmChannel));
}

/*
 * Mask to disable output
 */
static inline void HAL_PWM_WriteState(const HAL_PWM_T * p_pwm, bool isOn)
{
	(!isOn) ? (p_pwm->p_FtmBase->OUTMASK |= (1U << p_pwm->FtmChannel)) : (p_pwm->p_FtmBase->OUTMASK &= ~(1U << p_pwm->FtmChannel));
}



#endif
