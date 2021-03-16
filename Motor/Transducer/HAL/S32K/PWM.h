/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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
#ifndef PWM_HAL_H
#define PWM_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	FTM_Type * p_FtmBase;
	uint8_t FtmChannel;
} HAL_PWM_T;

static inline void HAL_PWM_WritePeriod(const HAL_PWM_T * p_pwm, uint32_t pwmPeroid)
{
	p_pwm->p_FtmBase->CONTROLS[p_pwm->FtmChannel].CnV = pwmPeroid;

//	if (softwareTrigger)
//	{
//		ftmBase->SYNC |= FTM_SYNC_SWSYNC_MASK;
//	}
}

/*
 true is invert, false is noninvert
 */
static inline void HAL_PWM_WriteInvertPolarity(const HAL_PWM_T * p_pwm, bool isInvertedPolarity)
{
//    p_pwm->p_FtmBase->In[p_pwm->FtmChannel].CnV = isInvertedPolarity;
//	PWM_1ABC_PIN_SET_POL((uint8_t)invA<<2|(uint8_t)invB<<1|(uint8_t)invC);
}

//static inline void HAL_Pin_WriteState(HAL_Pin_T * p_pwmPin, bool state)
static inline void HAL_PWM_WriteState(const HAL_PWM_T * p_pwmPin, bool state)
{
	//reg enable disable
//	p_pwmPin->p_FtmBase->In[p_pwm->FtmChannel].CnV = state;
}


#endif
