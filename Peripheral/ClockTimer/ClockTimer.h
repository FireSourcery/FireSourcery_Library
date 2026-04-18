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
    @file   ClockTimer.h
    @author FireSourcery
    @brief  Thin clock timer/timer HAL wrapper.
*/
/******************************************************************************/
#include "HAL_ClockTimer.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef CLOCK_TIMER_MAX
#define CLOCK_TIMER_MAX (0xFFFFU)
#endif

/******************************************************************************/
/*
    ClockTimer_T
*/
/******************************************************************************/
typedef const struct ClockTimer
{
    HAL_ClockTimer_T * P_HAL_TIMER;      /* DeltaT Timer */
    uint32_t TIMER_FREQ;
    uint32_t SAMPLE_FREQ;       /* Speed sample freq (e.g. 1kHz) */
    uint32_t SAMPLE_TIME;       /* TIMER_FREQ / SAMPLE_FREQ */
}
ClockTimer_T;

#define CLOCK_TIMER_INIT(p_TimerHal, TimerFreq, SampleFreq, p_State) (ClockTimer_T) \
    { .P_HAL_TIMER = (p_TimerHal), .TIMER_FREQ = (TimerFreq), .SAMPLE_FREQ = (SampleFreq), .SAMPLE_TIME = (TimerFreq) / (SampleFreq), }

/******************************************************************************/
/*!
    @brief  Capture Sample Time - Clock Edge Polling/ISR
*/
/******************************************************************************/
/*
    Capture DeltaT - Clock Edge Polling/ISR
    [0:CLOCK_TIMER_MAX] => (0xFFFF / TIMER_FREQ) [seconds]
*/
static inline uint32_t ClockTimer_CaptureDeltaT(const ClockTimer_T * p_timer) { return HAL_ClockTimer_CapturePeriod(p_timer->P_HAL_TIMER); }

/*
    Per SAMPLE_TIME
*/
static inline uint32_t ClockTimer_CaptureSampleTh(const ClockTimer_T * p_timer, uint32_t prevSampleTh)
{
    return HAL_ClockTimer_ReadOverflow(p_timer->P_HAL_TIMER) ? (prevSampleTh + p_timer->SAMPLE_TIME) : HAL_ClockTimer_Read(p_timer->P_HAL_TIMER);
}


/******************************************************************************/
/*!
    @brief Initialize the clock timer.
*/
/******************************************************************************/
static inline void ClockTimer_Init(const ClockTimer_T * p_timer)
{
    HAL_ClockTimer_Init(p_timer->P_HAL_TIMER);
    HAL_ClockTimer_InitFreq(p_timer->P_HAL_TIMER, p_timer->TIMER_FREQ);
}

static inline void ClockTimer_SetInitial(const ClockTimer_T * p_timer)
{
    HAL_ClockTimer_ClearOverflow(p_timer->P_HAL_TIMER);
    HAL_ClockTimer_Write(p_timer->P_HAL_TIMER, 0U);
}

/*
    Capture with overflow
*/
// static inline void ClockTimer_Overflow_ISR(const ClockTimer_T * p_timer)
// {
//     p_timer->P_STATE->OverflowCount++;
// }

// static inline void _DeltaT_CaptureExtended(const ClockTimer_T * p_timer)
// {
//     uint32_t periodT = (p_timer->P_STATE->OverflowCount << 16) | HAL_Encoder_ReadTimer(p_timer->P_HAL_ENCODER_TIMER);
//     HAL_Encoder_WriteTimer(p_timer->P_HAL_ENCODER_TIMER, 0U);
//     HAL_Encoder_ClearTimerOverflow(p_timer->P_HAL_ENCODER_TIMER);
//     p_timer->P_STATE->OverflowCount = 0U;
//     p_timer->P_STATE->DeltaT = periodT;
// }

// // Stop detection — single threshold in base timer ticks
// static inline bool _DeltaT_IsExtendedStop(const ClockTimer_T * p_timer)
// {
//     uint32_t elapsed = (p_timer->P_STATE->OverflowCount << 16) | HAL_Encoder_ReadTimer(p_timer->P_HAL_ENCODER_TIMER);
//     return (elapsed > p_timer->P_STATE->Config.DeltaTStop);
// }




