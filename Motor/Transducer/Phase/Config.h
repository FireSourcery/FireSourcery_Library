/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Config.h
    @author FireSoucery
    @brief
    @version V0
*/
/**************************************************************************/
#ifndef CONFIG_PHASE_H
#define CONFIG_PHASE_H

//#if defined(CONFIG_PHASE_HAL_PWM_S32K)
//	#define CONFIG_PHASE_HAL_PWM
//#elif defined(CONFIG_PHASE_HAL_PHASE_S32K)
//	#define CONFIG_PHASE_HAL_PHASE
//#elif defined(CONFIG_PHASE_HAL_KLS_S32K)
//	#define CONFIG_PHASE_HAL_PHASE
//#elif defined(CONFIG_PHASE_HAL_PWM_USER_DEFINED)
//	#define CONFIG_PHASE_HAL_PWM
//#elif defined(CONFIG_PHASE_HAL_PHASE_USER_DEFINED)
//	#define CONFIG_PHASE_HAL_PHASE
//#else
//	#define CONFIG_PHASE_HAL_PHASE_USER_DEFINED
//	#define CONFIG_PHASE_HAL_PHASE
//#endif

#if  	defined(CONFIG_PHASE_HAL_PWM)

#elif 	defined(CONFIG_PHASE_HAL_PHASE)

#else
	#define CONFIG_PHASE_HAL_PHASE
#endif


//todo pwm period as compile time cost
//#if  	defined(CONFIG_PHASE_HAL_PWM_PERIOD)
//
//#endif
//

#endif

