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
	@file 	Import.h
	@author FireSoucery
	@brief 	Analog module imports the following ADC functions.
			User must provide HW functions, or configure HAL
	@version V0
*/
/**************************************************************************/
#ifndef IMPORT_ANALOG_H
#define IMPORT_ANALOG_H

#include "Analog.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Must Implement
 */
/* Activate conversion - Set pins and configure. Implementation may specify which config options are supported. */
extern inline void ADC_Activate(volatile void * p_adcRegBase, uint32_t pinChannel, uint8_t hwBufferVirtualIndex, Analog_Config_T config);
extern inline uint32_t ADC_ReadResult(const volatile void * p_adcRegBase, uint8_t hwBufferVirtualIndex);
extern inline void ADC_WritePinSelect(volatile void * p_adcRegBase, uint32_t pinChannel, uint8_t hwBufferVirtualIndex);

extern inline bool ADC_ReadActiveCompleteFlag(const volatile void * p_adcMap);
extern inline bool ADC_ReadConversionCompleteFlag(const volatile void * p_adcMap);
extern inline void ADC_DisableInterrupt(volatile void * p_adcMap);

/*
 * Optionally Implement
 */
extern inline void 		ADC_Dectivate(volatile void * p_adcMap);
extern inline uint32_t 	ADC_ReadRequest	(volatile const void 	* p_adcMap, Analog_Request_T req);
extern inline void 		ADC_WriteConfig	(volatile 		void 	* p_adcMap, Analog_Config_T config);

/*
 * User implements operations using HW Queue/FIFO/Buffer.
 */
//	extern void ADC_Activate(void * p_adcRegBase, const uint8_t * p_virtualChannels, const uint32_t * p_channelMap, uint8_t channelCount, Analog_Config_T config)
//	extern void ADC_ReadResult(void * p_adcRegBase, analog_t * p_resultsBuffer, const uint32_t * p_channelMap, const uint8_t * p_virtualChannels, uint8_t channelCount);


#endif
