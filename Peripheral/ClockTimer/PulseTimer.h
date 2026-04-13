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
    @file   PulseTimer.h
    @author FireSourcery
    @brief  Thin pulse timer/timer HAL wrapper.
*/
/******************************************************************************/
#include "HAL_ClockTimer.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef PULSE_TIMER_MAX
#define PULSE_TIMER_MAX (0xFFFFU)
#endif

/*
    Square wave counter
*/
/******************************************************************************/
/*
    State - Timer-specific only
    Counter/FreqD state lives in AngleCounter_T (math layer)
*/
/******************************************************************************/
typedef struct PulseTimer_State
{
    /* Timer Capture */
    uint32_t DeltaT;        /* Timer counts between 2 pulse edges. Units in raw timer ticks */
    uint32_t SampleTh;       /* Ts timer value at last sample */
    uint32_t SampleT;

    /* Extended Timer */
    uint32_t ExtendedTimer;
    uint32_t ExtendedTimerConversion;   /* Extended timer ticks to base timer ticks */
    uint16_t ExtendedTimerStop;        /* Extended timer ticks to determine stopped */
    /* Optional Ref */
}
PulseTimer_State_T;

/******************************************************************************/
/*
    Const Instance
*/
/******************************************************************************/
typedef const struct PulseTimer
{
    HAL_ClockTimer_T * P_HAL_TIMER;      /* DeltaT Timer */
    uint32_t TIMER_FREQ;

    uint32_t SAMPLE_FREQ;       /* Speed sample freq (e.g. 1kHz) */
    uint32_t SAMPLE_TIME;       /* TIMER_FREQ / SAMPLE_FREQ */

    PulseTimer_State_T * P_STATE;

    const volatile uint32_t * P_EXTENDED_TIMER; /* 32-bit extension for low-speed stop detection */
    uint32_t EXTENDED_TIMER_FREQ;
    uint32_t EXTENDED_TIMER_CONVERSION;
}
PulseTimer_T;

#define PULSE_TIMER_INIT(p_TimerHal, TimerFreq, SampleFreq, p_State) (PulseTimer_T) \
{                                                           \
    .P_HAL_TIMER        = (p_TimerHal),                     \
    .TIMER_FREQ         = (TimerFreq),                      \
    .SAMPLE_FREQ        = (SampleFreq),                     \
    .SAMPLE_TIME        = (TimerFreq) / (SampleFreq),       \
    .P_STATE            = (p_State),                        \
}

#define PULSE_TIMER_INIT_EXTENDED(p_TimerHal, TimerFreq, p_ExtTimer, ExtTimerFreq, SampleFreq, p_State) (PulseTimer_T) \
{                                                           \
    .P_HAL_TIMER        = (p_TimerHal),                     \
    .TIMER_FREQ         = (TimerFreq),                      \
    .P_EXTENDED_TIMER   = (p_ExtTimer),                     \
    .EXTENDED_TIMER_FREQ = (ExtTimerFreq),                  \
    .SAMPLE_FREQ        = (SampleFreq),                     \
    .SAMPLE_TIME        = (TimerFreq) / (SampleFreq),       \
    .P_STATE            = (p_State),                        \
}

#define PULSE_TIMER_STATE_ALLOC() (&(PulseTimer_State_T){0})


/******************************************************************************/
/*

*/
/******************************************************************************/
// static inline uint32_t _PulseTimer_BaseFreq(const PulseTimer_T * p_timer)
// {
// #ifdef PULSE_DYNAMIC_TIMER
//     return p_timer->P_STATE->UnitTime_Freq;
// #else
//     return p_timer->TIMER_FREQ;
// #endif
// }



/******************************************************************************/
/*!
    @brief  Capture Sample Time - Pulse Edge Polling/ISR
*/
/******************************************************************************/

/*
    Capture DeltaT - Pulse Edge Polling/ISR
    [0:PULSE_TIMER_MAX] => (0xFFFF / TIMER_FREQ) [seconds]
*/
static inline uint32_t PulseTimer_CaptureDeltaT(const PulseTimer_T * p_timer)
{
    PulseTimer_State_T * p_state = p_timer->P_STATE;
    if (!HAL_ClockTimer_ReadOverflow(p_timer->P_HAL_TIMER))
    {
        p_state->DeltaT = HAL_ClockTimer_Read(p_timer->P_HAL_TIMER);
    }
    else
    {
        p_state->DeltaT = PULSE_TIMER_MAX;
        HAL_ClockTimer_ClearOverflow(p_timer->P_HAL_TIMER);
    }
    HAL_ClockTimer_Write(p_timer->P_HAL_TIMER, 0U);
    return p_state->DeltaT;
}

/*
    Per SAMPLE_TIME
*/
static inline uint32_t PulseTimer_CaptureSampleTh(const PulseTimer_T * p_timer)
{
    PulseTimer_State_T * p_state = p_timer->P_STATE;
    p_state->SampleTh = HAL_ClockTimer_ReadOverflow(p_timer->P_HAL_TIMER) ? (p_state->SampleTh + p_timer->SAMPLE_TIME) : HAL_ClockTimer_Read(p_timer->P_HAL_TIMER);
    return p_state->SampleTh;
}

static inline uint32_t _PulseTimer_CaptureSampleTh(const PulseTimer_T * p_timer)
{
    PulseTimer_State_T * p_state = p_timer->P_STATE;
    p_state->SampleTh = HAL_ClockTimer_Read(p_timer->P_HAL_TIMER);
    return p_state->SampleTh;
}

static inline uint32_t PulseTimer_CaptureSampleTk(const PulseTimer_T * p_timer)
{
    PulseTimer_State_T * p_state = p_timer->P_STATE;

    if (HAL_ClockTimer_ReadOverflow(p_timer->P_HAL_TIMER))
    {
        p_state->SampleTh = (p_state->SampleTh + p_timer->SAMPLE_TIME); /* Accumulate DeltaTh on overflow */
        // p_state->SampleT = 0;
        return 0;
    }
    else
    {
        return p_timer->SAMPLE_TIME + p_state->SampleTh - _PulseTimer_CaptureSampleTh(p_timer);
        // p_state->SampleT = p_timer->SAMPLE_TIME + p_state->SampleTh - _PulseTimer_CaptureSampleTh(p_timer);
    }

    /* updates SampleTh and returns 0 on overflow */
    // return p_timer->SAMPLE_TIME + p_state->SampleTh - PulseTimer_CaptureSampleTh(p_timer);
}

static inline uint32_t PulseTimer_CaptureSampleTk_Freq(const PulseTimer_T * p_timer)
{
    uint32_t sampleTk = PulseTimer_CaptureSampleTk(p_timer);
    return (sampleTk > p_timer->SAMPLE_TIME / 2) ? (p_timer->TIMER_FREQ / sampleTk) : 0;
}

static inline bool PulseTimer_IsStop(const PulseTimer_T * p_timer) { return (p_timer->P_STATE->DeltaT >= PULSE_TIMER_MAX); }

/******************************************************************************/
/*!
    @brief Extend base timer, 16bit timer cases to 32bit.
    Use Extended Timer to extend Low RPM range.
    EXTENDED_TIMER_FREQ = 1000Hz, PULSE_TIMER_MAX = 0xFFFF
        209ms to 13743S for TimerFreq = 312500Hz, 3.2us period
        104ms to 6871S for TimerFreq = 625000Hz
        1.6ms to 107S for TimerFreq = 40Mhz
*/
/******************************************************************************/
static inline uint32_t _PulseTimer_GetExtendedDelta(const PulseTimer_T * p_timer) { return *(p_timer->P_EXTENDED_TIMER) - p_timer->P_STATE->ExtendedTimer; }

/*
    Call on Encoder Edge.
    Capture as overwrite
    32 bit DeltaT overflow should be caught by CheckExtendedStop, which is a shorter period
*/
static inline uint32_t PulseTimer_CaptureExtendedDeltaT(const PulseTimer_T * p_timer)
{
    if (PulseTimer_CaptureDeltaT(p_timer) == PULSE_TIMER_MAX) { p_timer->P_STATE->DeltaT = _PulseTimer_GetExtendedDelta(p_timer) * p_timer->P_STATE->ExtendedTimerConversion; }
    p_timer->P_STATE->ExtendedTimer = *(p_timer->P_EXTENDED_TIMER);
    return p_timer->P_STATE->DeltaT;
}

/*
    Use Extended Timer to check 0 speed.
*/
static inline bool PulseTimer_IsExtendedStop(const PulseTimer_T * p_timer) { return (_PulseTimer_GetExtendedDelta(p_timer) > p_timer->P_STATE->ExtendedTimerStop); }

/*
    Set stop detection threshold from milliseconds.
    @param[in] millis  Timeout in milliseconds — no edges within this period => stopped
*/
static inline void PulseTimer_SetExtendedWatchStop_Millis(const PulseTimer_T * p_timer, uint32_t millis)
{
    p_timer->P_STATE->ExtendedTimerStop = (uint16_t)(p_timer->EXTENDED_TIMER_FREQ * millis / 1000U);
}



/* Alias for Hall edge capture — captures DeltaT and resets timer */
static inline uint32_t PulseTimer_CaptureEdge(const PulseTimer_T * p_timer) { return PulseTimer_CaptureExtendedDeltaT(p_timer); }


/******************************************************************************/
/*
    Capture Period < 1S
    DeltaT / TimerFreq = DeltaT [Seconds]
*/
/******************************************************************************/
static inline uint32_t PulseTimer_DeltaT_Freq(const PulseTimer_T * p_timer) { return p_timer->TIMER_FREQ / p_timer->P_STATE->DeltaT; }
static inline uint32_t PulseTimer_DeltaT_Ms(const PulseTimer_T * p_timer) { return p_timer->P_STATE->DeltaT * 1000U / p_timer->TIMER_FREQ; }



/******************************************************************************/
/*!
    @brief Initialize the pulse timer.
*/
/******************************************************************************/
static inline void PulseTimer_Init(const PulseTimer_T * p_timer)
{
    HAL_ClockTimer_Init(p_timer->P_HAL_TIMER);
    HAL_ClockTimer_InitFreq(p_timer->P_HAL_TIMER, p_timer->TIMER_FREQ);
    p_timer->P_STATE->ExtendedTimerConversion = p_timer->TIMER_FREQ / p_timer->EXTENDED_TIMER_FREQ;
}

static inline void PulseTimer_SetInitial(const PulseTimer_T * p_timer)
{
    HAL_ClockTimer_ClearOverflow(p_timer->P_HAL_TIMER);
    HAL_ClockTimer_Write(p_timer->P_HAL_TIMER, 0U);
    p_timer->P_STATE->ExtendedTimer = *(p_timer->P_EXTENDED_TIMER);
}


/*
    Capture with overflow
*/
// static inline void PulseTimer_Overflow_ISR(const PulseTimer_T * p_timer)
// {
//     p_timer->P_STATE->OverflowCount++;
// }

// static inline void Encoder_DeltaT_CaptureExtended(const PulseTimer_T * p_timer)
// {
//     uint32_t periodT = (p_timer->P_STATE->OverflowCount << 16) | HAL_Encoder_ReadTimer(p_timer->P_HAL_ENCODER_TIMER);
//     HAL_Encoder_WriteTimer(p_timer->P_HAL_ENCODER_TIMER, 0U);
//     HAL_Encoder_ClearTimerOverflow(p_timer->P_HAL_ENCODER_TIMER);
//     p_timer->P_STATE->OverflowCount = 0U;
//     p_timer->P_STATE->DeltaT = periodT;
// }

// // Stop detection — single threshold in base timer ticks
// static inline bool Encoder_DeltaT_IsExtendedStop(const PulseTimer_T * p_timer)
// {
//     uint32_t elapsed = (p_timer->P_STATE->OverflowCount << 16) | HAL_Encoder_ReadTimer(p_timer->P_HAL_ENCODER_TIMER);
//     return (elapsed > p_timer->P_STATE->Config.DeltaTStop);
// }




