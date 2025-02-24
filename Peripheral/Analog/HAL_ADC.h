/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   HAL_ADC.h
    @author FireSourcery
    @brief     Analog module import functions.
            User must provide HW functions, or configure peripheral HAL
    @version V0
*/
/******************************************************************************/
#ifndef HAL_ADC_H
#define HAL_ADC_H

#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_ADC.h)

#include <stdint.h>
#include <assert.h>

#ifndef HAL_ADC_VALUE_T
#define HAL_ADC_VALUE_T uint16_t
#endif

#ifndef HAL_ADC_PIN_T
#define HAL_ADC_PIN_T uint8_t
#endif

typedef HAL_ADC_VALUE_T adc_t; //adcu_t
typedef HAL_ADC_PIN_T adc_pin_t;



/*
typedef void HAL_ADC_T;

static inline void HAL_ADC_Activate(HAL_ADC_T * p_hal, uint32_t pinChannel) {}
static inline uint32_t HAL_ADC_ReadResult(const HAL_ADC_T * p_hal, uint32_t pinChannel) {}

static inline void HAL_ADC_WriteFifoCount(HAL_ADC_T * p_hal, uint32_t count) { (void)p_hal; (void)count; }
static inline void HAL_ADC_WriteFifoPin(HAL_ADC_T * p_hal, uint32_t pinChannel) { (void)p_hal;(void)pinChannel; }
static inline void HAL_ADC_ActivateFifo(HAL_ADC_T * p_hal, uint32_t pinChannel) { (void)p_hal;(void)pinChannel; }

static inline void HAL_ADC_DisableInterrupt(HAL_ADC_T * p_hal) {}
static inline void HAL_ADC_EnableInterrupt(HAL_ADC_T * p_hal) {}

static inline void HAL_ADC_ClearConversionCompleteFlag(const HAL_ADC_T * p_hal) { (void)p_hal; }
static inline bool HAL_ADC_ReadConversionCompleteFlag(const HAL_ADC_T * p_hal) {}
static inline bool HAL_ADC_ReadConversionActiveFlag(const HAL_ADC_T * p_hal) {}

static inline void HAL_ADC_AbortConversion(HAL_ADC_T * p_hal) {}
static inline void HAL_ADC_Deactivate(HAL_ADC_T * p_hal) {}

static inline void HAL_ADC_EnableHwTrigger(HAL_ADC_T * p_hal) {}
static inline void HAL_ADC_DisableHwTrigger(HAL_ADC_T * p_hal) {}
static inline void HAL_ADC_DisableContinuousConversion(HAL_ADC_T * p_hal) {}
static inline void HAL_ADC_EnableContinuousConversion(HAL_ADC_T * p_hal) {}

static inline void HAL_ADC_Init(const HAL_ADC_T * p_hal) { (void)p_hal; }
*/


static inline void HAL_ADC_WriteFifo(HAL_ADC_T * p_hal, adc_pin_t * p_pins, uint8_t count)
{
    assert(count <= HAL_ADC_FIFO_LENGTH_MAX);
    HAL_ADC_WriteFifoCount(p_hal, count);
    for (uint8_t iPin = 0U; iPin < count; iPin++) { HAL_ADC_WriteFifoPin(p_hal, p_pins[iPin]); }
    // HAL_ADC_ActivateFifo(p_hal);
}

static inline void HAL_ADC_WriteFifo_ActivateOnLast(HAL_ADC_T * p_hal, adc_pin_t * p_pins, uint8_t count)
{
    assert(count <= HAL_ADC_FIFO_LENGTH_MAX);
    HAL_ADC_WriteFifoCount(p_hal, count);
    for (uint8_t iPin = 0U; iPin < count - 1U; iPin++) { HAL_ADC_WriteFifoPin(p_hal, p_pins[iPin]); }
    HAL_ADC_ActivateFifo(p_hal, p_pins[count - 1U]);
}

#endif
