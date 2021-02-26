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
	@file 	ADC.h
	@author FireSoucery
	@brief 	S32K ADC functions. Used by HAL
	@version V0
*/
/**************************************************************************/
#ifndef PLATFORM_S32K_ADC_H
#define PLATFORM_S32K_ADC_H

#include "Peripheral/Analog/Analog.h"

#include "SDK/platform/devices/S32K142/include/S32K142.h"
#include "SDK/platform/drivers/inc/adc_driver.h"
#include "SDK/platform/drivers/src/adc/adc_hw_access.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Dependent vs independent from peripheral abstraction
 * If HAL "driver" does not include peripheral abstraction e.g. Analog.h. it can be used as independent hw driver in addition to HAL
 * Some modules may need to include peripheral abstraction for passing large data struct, or optional type checking of user typedefs.
 * HW access function usually provided by vendor. Implement this module as HAL for generalized peripheral module
 */

/*
	ADC TOTAL CONVERSION TIME = Sample Phase Time (set by SMPLTS + 1) + Hold
	Phase (1 ADC Cycle) + Compare Phase Time (8-bit Mode = 20 ADC Cycles, 10-bit
	Mode = 24 ADC Cycles, 12-bit Mode = 28 ADC Cycles) + Single or First continuous
	time adder (5 ADC cycles + 5 bus clock cycles)
*/

static inline void ADC_Activate(volatile void * p_adcRegBase, uint32_t pinChannel, uint8_t hwBufferVirtualIndex, Analog_Config_T config)
{
	(void)hwBufferVirtualIndex;

	((ADC_Type *) p_adcRegBase)->SC2 = (((ADC_Type *) p_adcRegBase)->SC2 & ~(ADC_SC2_ADTRG_MASK)) | ADC_SC2_ADTRG((uint32_t )config.UseHwTrigger);
	((ADC_Type *) p_adcRegBase)->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(pinChannel);
}

static inline uint32_t ADC_ReadResult(const volatile void * p_adcRegBase, uint8_t hwBufferVirtualIndex)
{
	(void)hwBufferVirtualIndex;

	return (uint32_t) ((ADC_Type *) p_adcRegBase)->R[0];
}

/*
 * cannot support cases where interrupt must be set along with pinChannel
 */
static inline void ADC_WritePinSelect(volatile void * p_adcRegBase, uint32_t pinChannel, uint8_t hwBufferVirtualIndex)
{
	(void)hwBufferVirtualIndex;
	((ADC_Type *) p_adcRegBase)->SC1[0] = ADC_SC1_ADCH((uint32_t)pinChannel);
}

static inline void ADC_DisableInterrupt(volatile void * p_adcRegBase)
{
	/* this will start conversion on channel 0 (without interrupt)? */
	((ADC_Type *) p_adcRegBase)->SC1[0] = ((ADC_Type *) p_adcRegBase)->SC1[0] & ~ADC_SC1_AIEN_MASK;
}

static inline void ADC_EnableInterrupt(volatile void * p_adcRegBase)
{
	((ADC_Type *) p_adcRegBase)->SC1[0] = ((ADC_Type *) p_adcRegBase)->SC1[0] | ADC_SC1_AIEN_MASK;
}

static inline void ADC_Dectivate(volatile void * p_adcRegBase)
{
	/*
	 *  111111b - Module is disabled
	 *
	 * 	The successive approximation converter subsystem is turned off when the pinChannel bits are all set (i.e.
	 *	ADCH set to all 1s). This feature allows explicit disabling of the ADC and isolation of the input channel
	 *	from all sources. Terminating continuous conversions this way prevents an additional single conversion
	 *	from being performed. It is not necessary to set ADCH to all 1s to place the ADC in a low-power state
	 *	when continuous conversions are not enabled because the module automatically enters a low-power
	 *	state when a conversion completes.
	 */
	((ADC_Type *) p_adcRegBase)->SC1[0] = 0x3FU;
}

static inline bool ADC_ReadConversionActiveFlag(const volatile void * p_adcRegBase)
{
	uint32_t tmp = (uint32_t) ((ADC_Type *) p_adcRegBase)->SC2;
	tmp = (tmp & ADC_SC2_ADACT_MASK) >> ADC_SC2_ADACT_SHIFT;
	return (tmp != 0u) ? true : false;
}

static inline bool ADC_ReadConversionCompleteFlag(const volatile void * p_adcRegBase)
{
	uint32_t tmp = (uint32_t) ((ADC_Type *) p_adcRegBase)->SC1[0];
	tmp = (tmp & ADC_SC1_COCO_MASK) >> ADC_SC1_COCO_SHIFT;
	return (tmp != 0u) ? true : false;
}

//static inline uint32_t ADC_ReadRequest(const volatile void * p_adcRegBase, Analog_Request_T request)
//{
//	uint32_t response;
//
//	switch (request)
//	{
//	case 1:
//		response = (uint32_t) ADC_ReadConversionCompleteFlag(p_adcRegBase);
//		break;
//	default:
//		response = 0;
//		break;
//	}
//
//	return response;
//}
//
static inline void ADC_WriteConfig(volatile void * p_adcRegBase, Analog_Config_T config)
{
	if (config.UseHwTrigger)
	{
		((ADC_Type *) p_adcRegBase)->SC2 = ((ADC_Type *) p_adcRegBase)->SC2 | ADC_SC2_ADTRG((uint32_t )config.UseHwTrigger);
	}
	else
	{
		((ADC_Type *) p_adcRegBase)->SC2 = (((ADC_Type *) p_adcRegBase)->SC2 & ~(ADC_SC2_ADTRG_MASK));
	}

	if (config.UseInterrrupt)
	{
		ADC_EnableInterrupt(p_adcRegBase);
	}
	else
	{
		ADC_DisableInterrupt(p_adcRegBase);
	}
}

static inline void ADC_Init(const uint32_t instance, const adc_converter_config_t * const config)
{
	ADC_DRV_ConfigConverter(instance, config);
}

#endif
