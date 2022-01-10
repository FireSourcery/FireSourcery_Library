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
	@file 	HAL_ADC.h
	@author FireSoucery
	@brief 	S32K Analog import functions. Used by HAL.
			Runtime configure options only. Runtime constant settings delegated to MCU init.
	@version V0
*/
/******************************************************************************/
#ifndef HAL_ADC_PLATFORM_H
#define HAL_ADC_PLATFORM_H

#include "External/S32K142/include/S32K142.h" /* use drivers or direct register access */

#include <stdint.h>
#include <stdbool.h>

typedef ADC_Type HAL_ADC_T;

/*
	ADC TOTAL CONVERSION TIME = Sample Phase Time (set by SMPLTS + 1) + Hold
	Phase (1 ADC Cycle) + Compare Phase Time (8-bit Mode = 20 ADC Cycles, 10-bit
	Mode = 24 ADC Cycles, 12-bit Mode = 28 ADC Cycles) + Single or First continuous
	time adder (5 ADC cycles + 5 bus clock cycles)
*/

static inline void HAL_ADC_Activate(HAL_ADC_T * p_adc, uint32_t pinChannel)
{
	p_adc->SC1[0U] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(pinChannel);
}


static inline uint32_t HAL_ADC_ReadResult(const HAL_ADC_T * p_adc, uint32_t pinChannel)
{
	(void)pinChannel;
	return (uint32_t)p_adc->R[0U];
}

/*
 * cannot support cases where interrupt must be set along with pinChannel
 */
//static inline void HAL_ADC_WritePinSelect(HAL_ADC_T * p_adc, uint32_t pinChannel)
//{
//	p_adc->SC1[0U] = ADC_SC1_ADCH((uint32_t )pinChannel);
//}
//
//static inline void HAL_ADC_WriteLast(HAL_ADC_T * p_adc, uint32_t pinChannel)
//{
//	p_adc->SC1[0U] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(pinChannel);
//}

static inline void HAL_ADC_EnableHwTrigger(HAL_ADC_T * p_adc)
{
	p_adc->SC2 |= ADC_SC2_ADTRG_MASK;
}

static inline void HAL_ADC_DisableHwTrigger(HAL_ADC_T * p_adc)
{
	p_adc->SC2 &= ~(ADC_SC2_ADTRG_MASK);
}



static inline void HAL_ADC_DisableInterrupt(HAL_ADC_T * p_adc)
{
//	p_adc->SC1[0U] &= ~ADC_SC1_AIEN_MASK;
	//may need to switch to nvic interrupt for critical section, with interfering with conversion
	switch ((uint32_t)p_adc)
	{
		case (uint32_t)ADC0: S32_NVIC->ICER[ADC0_IRQn >> 5U] = (1UL << (ADC0_IRQn & 0x1FU)); break;
		case (uint32_t)ADC1: S32_NVIC->ICER[ADC1_IRQn >> 5U] = (1UL << (ADC1_IRQn & 0x1FU)); break;
		default: break;
	}
}

static inline void HAL_ADC_EnableInterrupt(HAL_ADC_T * p_adc)
{
//	p_adc->SC1[0U] |= ADC_SC1_AIEN_MASK;

	//may need to switch to nvic interrupt for critical section, with interfering with conversion
	switch ((uint32_t)p_adc)
	{
		case (uint32_t)ADC0: S32_NVIC->ISER[ADC0_IRQn >> 5U] = (1UL << (ADC0_IRQn & 0x1FU)); break;
		case (uint32_t)ADC1: S32_NVIC->ISER[ADC1_IRQn >> 5U] = (1UL << (ADC1_IRQn & 0x1FU)); break;
		default: break;
	}
}

static inline void HAL_ADC_DisableContinuousConversion(HAL_ADC_T * p_adc)
{
	p_adc->SC3 &= ~ADC_SC3_ADCO_MASK;
}

static inline void HAL_ADC_EnableContinuousConversion(HAL_ADC_T * p_adc)
{
	p_adc->SC3 |= ADC_SC3_ADCO_MASK;
}

static inline void HAL_ADC_Deactivate(HAL_ADC_T * p_adc)
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

	p_adc->SC1[0U] = 0x3FU;
}

static inline bool HAL_ADC_ReadConversionActiveFlag(const HAL_ADC_T * p_adc)
{
	return ((p_adc->SC2 & (uint32_t)ADC_SC2_ADACT_MASK) != 0U) ? true : false;
}

static inline bool HAL_ADC_ReadConversionCompleteFlag(const HAL_ADC_T * p_adc)
{
	return ((p_adc->SC1[0U] & (uint32_t)ADC_SC1_COCO_MASK) != 0U) ? true : false;
}

//clears interrupt
static inline void HAL_ADC_ClearConversionCompleteFlag(const HAL_ADC_T * p_adc)
{
	(void)p_adc;
	//automatic after read on S32k
}


static inline void HAL_ADC_Init(const HAL_ADC_T * p_adc)
{
	(void)p_adc;
}

#endif
