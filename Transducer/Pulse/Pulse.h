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
    @file   Pulse.h
    @author FireSourcery
    @brief  Thin pulse/timer HAL wrapper. Timer capture, extended timer,
            and stop detection. Counter math delegated to Angle_Counter_T.
*/
/******************************************************************************/
#include "HAL_Encoder.h"
#include "Math/Angle/Angle_Counter.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Encoder_Base / TimerCounter / Sqaure wave counter
*/
/******************************************************************************/
/*
    State - Timer-specific only
    Counter/FreqD state lives in Angle_Counter_T (math layer)
*/
/******************************************************************************/
typedef struct Pulse_State
{
    /* Timer Capture */
    uint32_t DeltaT;        /* Timer ticks between edges */

    /* Extended Timer */
    uint32_t ExtendedTimerPrev;
    uint32_t ExtendedTimerConversion;   /* Extended timer ticks to base timer ticks */

    /* Direction */
    int32_t DirectionComp;  /* Direction sign for unsigned capture modes */

    /* Stop detection threshold */
    uint16_t ExtendedDeltaTStop;        /* Extended timer ticks to determine stopped */

    Angle_Counter_T Counter; /* Embedded math state for count accumulation and speed estimation */
}
Pulse_State_T;

/******************************************************************************/
/*
    Const Instance
*/
/******************************************************************************/
typedef const struct Pulse
{
    HAL_Encoder_Timer_T * P_HAL_TIMER;      /* DeltaT Timer */
    uint32_t TIMER_FREQ;

    const volatile uint32_t * P_EXTENDED_TIMER; /* 32-bit extension for low-speed stop detection */
    uint32_t EXTENDED_TIMER_FREQ;

    uint32_t SAMPLE_FREQ;       /* Speed sample freq (e.g. 1kHz) */
    uint32_t SAMPLE_TIME;       /* TIMER_FREQ / SAMPLE_FREQ */

    Pulse_State_T * P_STATE;
}
Pulse_T;

#define PULSE_INIT(p_TimerHal, TimerFreq, p_ExtTimer, ExtTimerFreq, SampleFreq, p_State, p_Counter) \
{                                                           \
    .P_HAL_TIMER        = (p_TimerHal),                     \
    .TIMER_FREQ         = (TimerFreq),                      \
    .P_EXTENDED_TIMER   = (p_ExtTimer),                     \
    .EXTENDED_TIMER_FREQ = (ExtTimerFreq),                  \
    .SAMPLE_FREQ        = (SampleFreq),                     \
    .SAMPLE_TIME        = (TimerFreq) / (SampleFreq),       \
    .P_STATE            = (p_State),                        \
    .P_COUNTER          = (p_Counter),                      \
}

#define PULSE_STATE_ALLOC() (&(Pulse_State_T){0})
#define PULSE_COUNTER_ALLOC() (&(Angle_Counter_T){0})

/******************************************************************************/
/*
    Timer Capture - Private
*/
/******************************************************************************/
static inline bool _Pulse_CaptureTimer(const Pulse_T * p_pulse)
{
    bool isValid = !HAL_Encoder_ReadTimerOverflow(p_pulse->P_HAL_TIMER);
    if (isValid == true) { p_pulse->P_STATE->DeltaT = HAL_Encoder_ReadTimer(p_pulse->P_HAL_TIMER); }
    else { HAL_Encoder_ClearTimerOverflow(p_pulse->P_HAL_TIMER); }
    HAL_Encoder_WriteTimer(p_pulse->P_HAL_TIMER, 0U);
    return isValid;
}

static inline uint32_t _Pulse_GetExtendedTimerDelta(const Pulse_T * p_pulse)
{
    return *(p_pulse->P_EXTENDED_TIMER) - p_pulse->P_STATE->ExtendedTimerPrev;
}

static inline void _Pulse_CaptureTimerExtended(const Pulse_T * p_pulse)
{
    if (_Pulse_CaptureTimer(p_pulse) == false)
    {
        p_pulse->P_STATE->DeltaT = _Pulse_GetExtendedTimerDelta(p_pulse) * p_pulse->P_STATE->ExtendedTimerConversion;
    }
    p_pulse->P_STATE->ExtendedTimerPrev = *(p_pulse->P_EXTENDED_TIMER);
}

static inline bool Pulse_IsStop(const Pulse_T * p_pulse)
{
    return (_Pulse_GetExtendedTimerDelta(p_pulse) > p_pulse->P_STATE->ExtendedDeltaTStop);
}

/******************************************************************************/
/*
    Timer Reading - For Angle_Counter math layer
    Read HAL timer and provide values for pure math functions.
*/
/******************************************************************************/
/*
    Read timer value for ModeDT DeltaTh.
    Returns 0 on overflow (convention for Angle_Counter_CaptureFreqD).
*/
static inline uint32_t Pulse_ReadTimerDeltaTh(const Pulse_T * p_pulse)
{
    return HAL_Encoder_ReadTimerOverflow(p_pulse->P_HAL_TIMER) ? 0U : HAL_Encoder_ReadTimer(p_pulse->P_HAL_TIMER);
}

/******************************************************************************/
/*
    Count Accumulation - Delegates to Angle_Counter_T
*/
/******************************************************************************/
/*
    On pulse edge. Accumulates signed count and captures DeltaT.
    @param[in] sign { -1, 0, +1 } direction of this edge
*/
static inline void Pulse_CaptureCount(const Pulse_T * p_pulse, int sign)
{
    Angle_Counter_CaptureCount(p_pulse->P_COUNTER, sign);
    _Pulse_CaptureTimerExtended(p_pulse);
}

/******************************************************************************/
/*
    ModeDT Speed Estimation - Bridges HAL to Angle_Counter math
    Call at SAMPLE_FREQ (~1kHz)
*/
/******************************************************************************/
static inline void _Pulse_CaptureFreqD(const Pulse_T * p_pulse, const Angle_CounterRef_T * p_ref)
{
    Angle_Counter_CaptureDeltaD(p_pulse->P_COUNTER);
    Angle_Counter_CaptureFreqD(p_pulse->P_COUNTER, p_ref, Pulse_ReadTimerDeltaTh(p_pulse));
}

/* Speed => 0 when extended timer exceeds stop threshold */
static inline void Pulse_CaptureFreqD(const Pulse_T * p_pulse, const Angle_CounterRef_T * p_ref)
{
    if (Pulse_IsStop(p_pulse) == false) { _Pulse_CaptureFreqD(p_pulse, p_ref); }
    else { p_pulse->P_COUNTER->FreqD = 0; }
}

/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline int32_t Pulse_GetFreqD(const Pulse_T * p_pulse) { return p_pulse->P_COUNTER->FreqD; }
static inline int32_t Pulse_GetDeltaD(const Pulse_T * p_pulse) { return p_pulse->P_COUNTER->DeltaD; }

/* Signed velocity with direction compensation */
static inline int32_t Pulse_GetSignedFreqD(const Pulse_T * p_pulse) { return p_pulse->P_STATE->DirectionComp * p_pulse->P_COUNTER->FreqD; }

static inline int32_t Pulse_GetDirectionComp(const Pulse_T * p_pulse) { return p_pulse->P_STATE->DirectionComp; }

/* Set direction for unsigned capture modes (single-phase) */
static inline void Pulse_SetDirection(const Pulse_T * p_pulse, int8_t direction) { p_pulse->P_STATE->DirectionComp = direction; }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Pulse_Init(const Pulse_T * p_pulse);
extern void Pulse_SetInitial(const Pulse_T * p_pulse);
extern void Pulse_SetExtendedWatchStop_Millis(const Pulse_T * p_pulse, uint16_t stopTime_Millis);
