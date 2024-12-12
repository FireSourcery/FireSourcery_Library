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
    @file   HAL_Analog.h
    @author FireSourcery
    @brief     Analog module import functions.
            User must provide HW functions, or configure peripheral HAL
    @version V0
*/
/******************************************************************************/
#ifndef HAL_ANALOG_H
#define HAL_ANALOG_H

#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_Analog.h)

#include <stdint.h>
#include <assert.h>

/*
typedef void HAL_Analog_T;

static inline void HAL_Analog_Activate(HAL_Analog_T * p_hal, uint32_t pinChannel) {}
static inline uint32_t HAL_Analog_ReadResult(const HAL_Analog_T * p_hal, uint32_t pinChannel) {}

static inline void HAL_Analog_WriteFifoCount(HAL_Analog_T * p_hal, uint32_t count) { (void)p_hal; (void)count; }
static inline void HAL_Analog_WriteFifoPin(HAL_Analog_T * p_hal, uint32_t pinChannel) { (void)p_hal;(void)pinChannel; }
static inline void HAL_Analog_ActivateFifo(HAL_Analog_T * p_hal, uint32_t pinChannel) { (void)p_hal;(void)pinChannel; }

static inline void HAL_Analog_DisableInterrupt(HAL_Analog_T * p_hal) {}
static inline void HAL_Analog_EnableInterrupt(HAL_Analog_T * p_hal) {}

static inline void HAL_Analog_ClearConversionCompleteFlag(const HAL_Analog_T * p_hal) { (void)p_hal; }
static inline bool HAL_Analog_ReadConversionCompleteFlag(const HAL_Analog_T * p_hal) {}
static inline bool HAL_Analog_ReadConversionActiveFlag(const HAL_Analog_T * p_hal) {}

static inline void HAL_Analog_AbortConversion(HAL_Analog_T * p_hal) {}
static inline void HAL_Analog_Deactivate(HAL_Analog_T * p_hal) {}

static inline void HAL_Analog_EnableHwTrigger(HAL_Analog_T * p_hal) {}
static inline void HAL_Analog_DisableHwTrigger(HAL_Analog_T * p_hal) {}
static inline void HAL_Analog_DisableContinuousConversion(HAL_Analog_T * p_hal) {}
static inline void HAL_Analog_EnableContinuousConversion(HAL_Analog_T * p_hal) {}

static inline void HAL_Analog_Init(const HAL_Analog_T * p_hal) { (void)p_hal; }
*/

/*

*/

static inline void HAL_Analog_WriteFifo(HAL_Analog_T * p_hal, uint32_t * p_pins, uint8_t count)
{
    assert(count <= HAL_ADC_FIFO_LENGTH_MAX);
    HAL_Analog_WriteFifoCount(p_hal, count);
    for (uint8_t i = 0U; i < count; i++) { HAL_Analog_WriteFifoPin(p_hal, p_pins[i]); }
    // HAL_Analog_ActivateFifo(p_hal);
}

static inline void HAL_Analog_WriteFifo_ActivateOnLast(HAL_Analog_T * p_hal, uint32_t * p_pins, uint8_t count)
{
    assert(count <= HAL_ADC_FIFO_LENGTH_MAX);
    HAL_Analog_WriteFifoCount(p_hal, count);
    for (uint8_t i = 0U; i < count - 1U; i++) { HAL_Analog_WriteFifoPin(p_hal, p_pins[i]); }
    HAL_Analog_ActivateFifo(p_hal, p_pins[count - 1U]);
}

#endif
