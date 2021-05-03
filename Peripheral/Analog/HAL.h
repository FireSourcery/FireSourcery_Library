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
	@brief 	Analog module imports the following ADC functions.
			User must provide HW functions, or configure peripheral HAL
	@version V0
*/
/******************************************************************************/
#ifndef HAL_ANALOG_H
#define HAL_ANALOG_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_ANALOG_HAL_S32K
	#include "Peripheral/HAL/Platform/S32K/HAL_ADC.h"
#elif defined(CONFIG_ANALOG_HAL_USER_DEFINED)

	typedef volatile void HAL_ADC_T;

	/*
	 * Must Implement
	 */
	extern inline void 		HAL_ADC_Activate(HAL_ADC_T * p_adcRegBase, uint32_t pinChannel);
	extern inline uint32_t 	HAL_ADC_ReadResult(const HAL_ADC_T * p_adcRegBase);
	extern inline void 		HAL_ADC_WritePinSelect(HAL_ADC_T * p_adcRegBase, uint32_t pinChannel);

	extern inline bool HAL_ADC_ReadActiveCompleteFlag(const HAL_ADC_T * p_adcMap);
	extern inline bool HAL_ADC_ReadConversionCompleteFlag(const HAL_ADC_T * p_adcMap);
	extern inline void HAL_ADC_DisableInterrupt(HAL_ADC_T * p_adcMap);

	extern inline void HAL_ADC_WriteLast(HAL_ADC_T * p_adcRegBase, uint32_t pinChannel);

	/*
	 * Optionally Implement
	 */
	extern inline void 		HAL_ADC_Dectivate(HAL_ADC_T * p_adcMap);
	extern inline uint32_t 	HAL_ADC_ReadRequest(const HAL_ADC_T * p_adcMap, Analog_Request_T req);
	extern inline void 		HAL_ADC_WriteConfig(HAL_ADC_T * p_adcMap, Analog_Config_T config);

	/*
	 * User implements operations using HW Queue/FIFO/Buffer.
	 */
	//	extern void HAL_ADC_Activate(void * p_adcRegBase, const uint8_t * p_virtualChannels, const uint32_t * p_channelMap, uint8_t channelCount, Analog_Config_T config)
	//	extern void HAL_ADC_ReadResult(void * p_adcRegBase, analog_t * p_resultsBuffer, const uint32_t * p_channelMap, const uint8_t * p_virtualChannels, uint8_t channelCount);
#endif


#endif
