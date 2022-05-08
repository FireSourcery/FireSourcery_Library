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
	@file 	Config.h
	@author FireSoucery
	@brief  Motor module preprocessor configuration options and defaults
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_MOTOR_H
#define CONFIG_MOTOR_H

#if 	defined(CONFIG_MOTOR_ALIGN_VOLTAGE_MAX)
#else
	#define CONFIG_MOTOR_ALIGN_VOLTAGE_MAX (65536U /10U)
#endif

#if 	defined(CONFIG_MOTOR_I_SENSORS_NONINVERT)
#elif 	defined(CONFIG_MOTOR_I_SENSORS_INVERT) 
#else
	#define	CONFIG_MOTOR_I_SENSORS_INVERT
#endif

#if 	defined(CONFIG_MOTOR_I_SENSORS_AB)
#elif 	defined(CONFIG_MOTOR_I_SENSORS_ABC) 
#else
	#define	CONFIG_MOTOR_I_SENSORS_ABC
#endif

#if 	defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) 
#elif 	defined(CONFIG_MOTOR_V_SENSORS_ADC) 
#else
	#define	CONFIG_MOTOR_V_SENSORS_ADC
#endif

#if   defined(ADC_BITS) 
#else  
	#define ADC_BITS 		12U
#endif

#endif



