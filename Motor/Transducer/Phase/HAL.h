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
	@brief 	Phase HAL import functions
	@version V0
*/
/******************************************************************************/
#ifndef HAL_PHASE_H
#define HAL_PHASE_H

#include "Config.h"

#if defined(CONFIG_PHASE_HAL_PLATFORM_S32K)
	#include "Peripheral/HAL/Platform/S32K/HAL_Pin.h"
	#include "Peripheral/HAL/Platform/S32K/HAL_PWM.h"
#elif defined(CONFIG_PHASE_HAL_KLS_S32K)
	#include "Motor/Transducer/HAL/Board/KLS_S32K/HAL_Phase.h"
#elif defined(CONFIG_PHASE_HAL_USER_DEFINED_PWM)

	typedef struct
	{
		PWMTimer_T * p_PwmTimer;
		uint8_t PwmTimerChannel;
	} HAL_PWM_T;

	extern inline void HAL_PWM_WritePeriod(const HAL_PWM_T * p_pwm, uint32_t pwmPeroid);

	/*	true is inverted, false is noninverted	 */
	extern inline void HAL_PWM_WriteInvertPolarity(const HAL_PWM_T * p_pwm, bool isInvertedPolarity);
	extern inline void HAL_PWM_WriteState(const HAL_PWM_T * p_pwm, bool isOn);

#elif defined(CONFIG_PHASE_HAL_USER_DEFINED_PHASE)

	typedef struct
	{

	} HAL_Phase_T;

#endif


#endif
