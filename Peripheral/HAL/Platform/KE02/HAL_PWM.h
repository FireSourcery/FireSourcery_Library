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
#ifndef HAL_PWM_PLATFORM_H
#define HAL_PWM_PLATFORM_H

//#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>


typedef FTM_Type HAL_PWM_T;

static inline void HAL_PWM_ClearInterrupt(HAL_PWM_T * p_hal)
{

}

/*
 * Common Sync, may split for polarity and CV
 */
static inline void HAL_PWM_Sync(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_WriteDuty(HAL_PWM_T * p_hal, uint32_t channel, uint32_t pwm)
{

}

/*
 * Mask to disable output
 */
static inline void HAL_PWM_EnableOutput(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_DisableOutput(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_EnableInvertPolarity(HAL_PWM_T * p_hal, uint32_t channel)
{

}
static inline void HAL_PWM_DisableInvertPolarity(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_EnableSoftwareControl(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_DisableSoftwareControl(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_WriteHigh(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_WriteLow(HAL_PWM_T * p_hal, uint32_t channel)
{

}

static inline void HAL_PWM_InitModule(HAL_PWM_T * p_hal)
{

}

static inline void HAL_PWM_InitChannel(HAL_PWM_T * p_hal, uint32_t channel)
{

}

#endif
