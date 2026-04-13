#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   HAL_ClockTimer.h
    @author FireSourcery
    @brief  Hw Timer derived directly from CPU Clock
*/
/******************************************************************************/
#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_CLOCK_TIMER_SOURCE_FREQ
#define HAL_CLOCK_TIMER_SOURCE_FREQ CPU_FREQ
#endif

typedef FTM_Type HAL_ClockTimer_T;

/******************************************************************************/
/*!
    @section HAL_ClockTimer_T
*/
/******************************************************************************/
/* Clear interrupt. Read-after-write sequence to guarantee required serialization of memory operations */
static inline void HAL_ClockTimer_ClearOverflow(HAL_ClockTimer_T * p_timer)          { p_timer->SC &= ~FTM_SC_TOF_MASK; (void)p_timer->SC; }
static inline bool HAL_ClockTimer_ReadOverflow(const HAL_ClockTimer_T * p_timer)     { return p_timer->SC & FTM_SC_TOF_MASK; }
static inline uint32_t HAL_ClockTimer_Read(const HAL_ClockTimer_T * p_timer)         { return p_timer->CNT; }
static inline void HAL_ClockTimer_Write(HAL_ClockTimer_T * p_timer, uint32_t count)  { p_timer->CNT = FTM_CNT_COUNT(count); }

static inline void HAL_ClockTimer_Enable(HAL_ClockTimer_T * p_timer)                 { p_timer->SC |= FTM_SC_CLKS(0b01U); }

/*!
    @return freq set

    Prescale Factor Selection
    000 Divide by 1
    001 Divide by 2
    010 Divide by 4
    011 Divide by 8
    100 Divide by 16
    101 Divide by 32
    110 Divide by 64
    111 Divide by 128
*/
static inline uint32_t HAL_ClockTimer_InitFreq(HAL_ClockTimer_T * p_timer, uint32_t freq)
{
    uint32_t preScalerValue = HAL_CLOCK_TIMER_SOURCE_FREQ / freq;
    uint8_t preScaler = 0U;
    while ((preScalerValue >> preScaler) > 1U) { preScaler++; } /* log2 */

    p_timer->SC &= ~FTM_SC_CLKS_MASK;
    p_timer->SC = (p_timer->SC & ~FTM_SC_PS_MASK) | FTM_SC_PS(preScaler);
    p_timer->SC |= FTM_SC_CLKS(0b01U);
    return HAL_CLOCK_TIMER_SOURCE_FREQ / ((uint32_t)1UL << preScaler);
}

static inline void HAL_ClockTimer_Init(HAL_ClockTimer_T * p_timer)
{
    p_timer->MOD = FTM_MOD_MOD(0xFFFFU);
    p_timer->SC |= FTM_SC_CLKS(0b01U);
    // HAL_ClockTimer_InitTimerFreq(p_timer, HAL_CLOCK_TIMER_FREQ);
}


