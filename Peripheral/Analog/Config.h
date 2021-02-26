/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
	@file 	Config.h
	@author FireSoucery
	@brief 	Analog module preprocessor configuration options and defaults.
	@version V0
*/
/**************************************************************************/
#ifndef CONFIG_ANALOG_H
#define CONFIG_ANALOG_H


#ifdef CONFIG_ANALOG_MULTITHREADED_OS_HAL

#elif defined(CONFIG_ANALOG_MULTITHREADED_USER_DEFINED)

#elif defined(CONFIG_ANALOG_MULTITHREADED_DISABLE)

#else
	#define CONFIG_ANALOG_MULTITHREADED_DISABLE
#endif

/*
 *
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_REGISTER_8BIT

#elif defined(CONFIG_ANALOG_ADC_RESULT_REGISTER_16BIT)
/* Default case */
#else
	#define CONFIG_ANALOG_ADC_RESULT_REGISTER_16BIT
#endif

/*
 *
 */
#ifdef CONFIG_ANALOG_ADC_RESULT_DATA_8BIT

#elif defined(CONFIG_ANALOG_ADC_RESULT_DATA_16BIT)
/* Default case */
#else
	#define CONFIG_ANALOG_ADC_RESULT_DATA_16BIT
#endif


#ifdef CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT8

#elif defined(CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT16)

#elif defined(CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT32)

#else
	#define CONFIG_ANALOG_ADC_PIN_CHANNEL_UINT32
#endif

/*
 *
 */
#ifdef CONFIG_ANALOG_VIRUTAL_CHANNEL_ENUM_USER_DEFINED
/*
 * User must provide enum typedef
 */
#elif defined(CONFIG_ANALOG_VIRUTAL_CHANNEL_UINT8)
/* Default case */
#else
	#define CONFIG_ANALOG_VIRUTAL_CHANNEL_UINT8
#endif


#ifdef CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER

#elif defined(CONFIG_ANALOG_ADC_HW_1_ADC_M_BUFFER)

#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_1_BUFFER)

#elif defined(CONFIG_ANALOG_ADC_HW_N_ADC_M_BUFFER)

#else
	#define CONFIG_ANALOG_ADC_HW_1_ADC_1_BUFFER
	#define ANALOG_N_ADC_DISABLED
#endif

#ifdef CONFIG_ANALOG_ADC_FUNCTIONS_HAL_DEFINED

#elif defined(CONFIG_ANALOG_ADC_FUNCTIONS_USER_DEFINED)

#else
	#error "Config Analog: USER MUST DEFINE ADC FUNCTIONS"
#endif

#endif
