/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
	@brief 	S32K Analog import functions. Used by HAL.
			Runtime configure options only. Runtime constant settings delegated to MCU init.
	@version V0
*/
/******************************************************************************/
/*
	ADC TOTAL CONVERSION TIME = Sample Phase Time (set by SMPLTS + 1) + Hold
	Phase (1 ADC Cycle) + Compare Phase Time (8-bit Mode = 20 ADC Cycles, 10-bit
	Mode = 24 ADC Cycles, 12-bit Mode = 28 ADC Cycles) + Single or First continuous
	time adder (5 ADC cycles + 5 bus clock cycles)
*/
#ifndef HAL_ANALOG_PLATFORM_H
#define HAL_ANALOG_PLATFORM_H

#include "External/S32K142/include/S32K142.h" /* use drivers or direct register access */
#include "External/CMSIS/Core/Include/cmsis_compiler.h"

#include <stdint.h>
#include <stdbool.h>

typedef ADC_Type HAL_Analog_T;

static inline void HAL_Analog_Activate(HAL_Analog_T * p_hal, uint32_t pinChannel) { p_hal->SC1[0U] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(pinChannel); }
static inline uint32_t HAL_Analog_ReadResult(const HAL_Analog_T * p_hal, uint32_t pinChannel) { (void)pinChannel; return (uint32_t)p_hal->R[0U]; }

static inline void HAL_Analog_WriteFifoCount(HAL_Analog_T * p_hal, uint32_t count) { (void)p_hal; (void)count; }
static inline void HAL_Analog_WriteFifoPin(HAL_Analog_T * p_hal, uint32_t pinChannel) { (void)p_hal;(void)pinChannel; }
static inline void HAL_Analog_ActivateFifo(HAL_Analog_T * p_hal, uint32_t pinChannel) { (void)p_hal;(void)pinChannel; }

/**
  \brief   Enable Interrupt
  \details Enables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __COMPILER_BARRIER();
    S32_NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __COMPILER_BARRIER();
  }
}

/**
  \brief   Disable Interrupt
  \details Disables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    S32_NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}

/*
	Use NVIC interrupt for local critical section
	p_hal->SC1[0U] &= ~ADC_SC1_AIEN_MASK; aborts conversion
*/
static inline void HAL_Analog_DisableInterrupt(HAL_Analog_T * p_hal)
{
	switch((uint32_t)p_hal)
	{
		case (uint32_t)ADC0: __NVIC_DisableIRQ(ADC0_IRQn); break;
		case (uint32_t)ADC1: __NVIC_DisableIRQ(ADC1_IRQn); break;
		default: break;
	}
}

/*
	Use NVIC interrupt for local critical section
	p_hal->SC1[0U] |= ADC_SC1_AIEN_MASK;
*/
static inline void HAL_Analog_EnableInterrupt(HAL_Analog_T * p_hal)
{
	switch((uint32_t)p_hal)
	{
		case (uint32_t)ADC0: __NVIC_EnableIRQ(ADC0_IRQn); break;
		case (uint32_t)ADC1: __NVIC_EnableIRQ(ADC1_IRQn); break;
		default: break;
	}
}

/*
	Clear interrupt - automatic after read on S32k
*/
static inline void HAL_Analog_ClearConversionCompleteFlag(const HAL_Analog_T * p_hal) { (void)p_hal; }
static inline bool HAL_Analog_ReadConversionCompleteFlag(const HAL_Analog_T * p_hal) { return ((p_hal->SC1[0U] & (uint32_t)ADC_SC1_COCO_MASK) != 0U); }
static inline bool HAL_Analog_ReadConversionActiveFlag(const HAL_Analog_T * p_hal) { return ((p_hal->SC2 & (uint32_t)ADC_SC2_ADACT_MASK) != 0U); }

static inline void HAL_Analog_AbortConversion(HAL_Analog_T * p_hal) { p_hal->SC1[0U] |= ADC_SC1_ADCH_MASK; }
/*
	111111b - Module is disabled

	The successive approximation converter subsystem is turned off when the pinChannel bits are all set (i.e.
	ADCH set to all 1s). This feature allows explicit disabling of the ADC and isolation of the input channel
	from all sources. Terminating continuous conversions this way prevents an additional single conversion
	from being performed. It is not necessary to set ADCH to all 1s to place the ADC in a low-power state
	when continuous conversions are not enabled because the module automatically enters a low-power
	state when a conversion completes.
*/
static inline void HAL_Analog_Deactivate(HAL_Analog_T * p_hal) { p_hal->SC1[0U] |= ADC_SC1_ADCH_MASK; }

static inline void HAL_Analog_EnableHwTrigger(HAL_Analog_T * p_hal){	p_hal->SC2 |= ADC_SC2_ADTRG_MASK;}
static inline void HAL_Analog_DisableHwTrigger(HAL_Analog_T * p_hal){	p_hal->SC2 &= ~(ADC_SC2_ADTRG_MASK);}
static inline void HAL_Analog_DisableContinuousConversion(HAL_Analog_T * p_hal){	p_hal->SC3 &= ~ADC_SC3_ADCO_MASK;}
static inline void HAL_Analog_EnableContinuousConversion(HAL_Analog_T * p_hal){	p_hal->SC3 |= ADC_SC3_ADCO_MASK;}

/*
	In Board_Init
*/
static inline void HAL_Analog_Init(const HAL_Analog_T * p_hal)
{
	(void)p_hal;
}

#endif
