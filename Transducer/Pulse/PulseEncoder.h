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
    @file   PulseEncoder.h
    @author FireSourcery
    @brief  Pulse-driven angle/speed engine.
            Composite wiring PulseTimer (edge timing) + AngleCounter (count + angle math).
            Single edge/sample pipeline for rotational pulse-edge sensors:
            Hall, soft-decode encoder, sensorless BEMF zero-cross, VR wheel, tachometer.
*/
/******************************************************************************/
#include "PulseTimer.h"
#include "Math/Angle/AngleCounter.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Instance
    Const outer (TIMER config + state pointer) over bundled mutable state.
    Single allocation for counter + timer state — contiguous, one pointer hand-off.
*/
/******************************************************************************/
typedef struct PulseEncoder_State
{
    AngleCounter_T Counter;     /* Count accumulation + FreqD + Angle_T bridge */
    PulseTimer_State_T Timer;   /* DeltaT + sample + extended-timer state */
}
PulseEncoder_State_T;

/*
    Caller Maps TIMER.P_STATE
*/
typedef const struct PulseEncoder
{
    const PulseTimer_T TIMER;         /* PulseTimer const; its P_STATE points into P_STATE->Timer */
    PulseEncoder_State_T * P_STATE;
    // const uint32_t POLLING_FREQ;   /* Control loop frequency [Hz] for angle delta conversion */
}
PulseEncoder_T;

// #define PULSE_ENCODER_INIT(Timer, p_State) (PulseEncoder_T) { .TIMER = (Timer), .P_STATE = (p_State), }

/*
    Assemble from primitives
*/
#define PULSE_ENCODER_INIT_FROM(p_TimerHal, TimerFreq, SampleFreq, p_State) \
    PULSE_ENCODER_INIT(PULSE_TIMER_INIT(p_TimerHal, TimerFreq, SampleFreq, &((p_State)->Timer)), p_State)

// #define PULSE_ENCODER_INIT_FROM(p_TimerHal, TimerFreq, SampleFreq, p_State, extended, extendedFreq) \
// {
//     .TIMER = PULSE_TIMER_INIT_EXTENDED
//     (
//         p_TimerHal, TimerFreq, SampleFreq,
//         &p_State->Timer,
//         extended, extendedFreq
//     ),
//     .P_STATE = p_State
// };


/******************************************************************************/
/*
    Edge ISR — call on every pulse edge
    Captures DeltaT on the timer, accumulates the counter.
    @param[in] sign  { -1, 0, +1 } — direction of this edge (resolved by caller)
*/
/******************************************************************************/
/* Count only. Angle supplied externally (sector LUT, Hall snap, etc.) */
static inline void PulseEncoder_CaptureEdge(const PulseEncoder_T * p_encoder, int sign)
{
    PulseTimer_CaptureEdge(&p_encoder->TIMER);
    AngleCounter_CaptureCount(&p_encoder->P_STATE->Counter, sign);
}

/* Count + wrap angle. Angle derived from counter (encoder-style) */
static inline void PulseEncoder_CaptureEdgeWrap(const PulseEncoder_T * p_encoder, int sign)
{
    PulseTimer_CaptureEdge(&p_encoder->TIMER);
    AngleCounter_CaptureCountAngle(&p_encoder->P_STATE->Counter, sign);
}

/******************************************************************************/
/*
    Sample Loop — call at sample rate (~1kHz)
    Derives FreqD from PulseTimer sample period and the accumulated DeltaD.
*/
/******************************************************************************/
static inline void PulseEncoder_CaptureFreq(const PulseEncoder_T * p_encoder)
{
    AngleCounter_CaptureFreq(&p_encoder->P_STATE->Counter, PulseTimer_CaptureSampleTk_Freq(&p_encoder->TIMER));
}

static inline Angle_T * PulseEncoder_Angle(const PulseEncoder_T * p_encoder) { return AngleCounter_Angle(&p_encoder->P_STATE->Counter); }
static inline AngleCounter_T * PulseEncoder_Counter(const PulseEncoder_T * p_encoder) { return &p_encoder->P_STATE->Counter; }
static inline PulseTimer_T * PulseEncoder_Timer(const PulseEncoder_T * p_encoder) { return &p_encoder->TIMER; }



static inline void PulseEncoder_InitFrom(const PulseEncoder_T * p_encoder, const AngleCounter_Config_T * p_config)
{
    PulseTimer_Init(&p_encoder->TIMER);
    AngleCounter_InitFrom(&p_encoder->P_STATE->Counter, p_config);
}

static inline void PulseEncoder_SetInitial(const PulseEncoder_T * p_encoder)
{
    PulseTimer_SetInitial(&p_encoder->TIMER);
}




