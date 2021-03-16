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
	@file 	HAL.h
	@author FireSoucery
	@brief 	Phase HAL import functions
	@version V0
*/
/******************************************************************************/
#ifndef HAL_PHASE_H
#define HAL_PHASE_H

#include "Motor/Config.h" /* module shared default config */

#if defined(CONFIG_PHASE_HAL_PWM_FUNCTIONS_S32K)
	#include "Motor/Transducer/HAL/S32K/PWM.h"
#elif defined(CONFIG_PHASE_HAL_PWM_FUNCTIONS_KLS_S32K)
	#include "Motor/Transducer/HAL/KLS_S32K/PWM.h"
#elif defined(CONFIG_PHASE_HAL_PWM_FUNCTIONS_USER_DEFINED)

	typedef struct
	{
		PWMTimer_T * p_PwmTimer;
		uint8_t PwmTimerChannel;
	} HAL_PWM_T;

	extern inline void HAL_PWM_WritePeriod(const HAL_PWM_T * p_pwm, uint32_t pwmPeroid);

	/*	true is inverted, false is noninverted	 */
	extern inline void HAL_PWM_WriteInvertPolarity(const HAL_PWM_T * p_pwm, bool isInvertedPolarity);
	extern inline void HAL_PWM_WriteState(const HAL_PWM_T * p_pwm, bool isOn);

#endif


#endif
