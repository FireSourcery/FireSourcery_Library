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
    const PulseTimer_T TIMER;           /* PulseTimer const; its P_STATE points into P_STATE->Timer */
    PulseEncoder_State_T * P_STATE;
    // const uint32_t POLLING_FREQ;        /* Control loop frequency [Hz] for angle delta conversion */
}
PulseEncoder_T;

// #define PULSE_ENCODER_INIT(Timer, p_State) (PulseEncoder_T) { .TIMER = (Timer), .P_STATE = (p_State), }

/*
    Assemble from primitives
*/
#define PULSE_ENCODER_INIT_FROM(p_TimerHal, TimerFreq, SampleFreq, p_State) \
    PULSE_ENCODER_INIT(PULSE_TIMER_INIT(p_TimerHal, TimerFreq, SampleFreq, &((p_State)->Timer)), p_State)


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
    AngleCounter_CaptureCountWrap(&p_encoder->P_STATE->Counter, sign);
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

/*
    Bridge FreqD → Angle_T.Delta in shifted Q16.16 for interpolation
    Call after CaptureFreq, before the control-loop interpolation step.
*/
static inline angle16_t PulseEncoder_ResolveAngleDelta(const PulseEncoder_T * p_encoder)
{
    return AngleCounter_ResolveAngleDelta(&p_encoder->P_STATE->Counter);
}

/******************************************************************************/
/*
    Control Loop — call per control cycle for inter-edge angle estimate
*/
/******************************************************************************/
static inline angle16_t PulseEncoder_Interpolate(const PulseEncoder_T * p_encoder)
{
    return AngleCounter_Interpolate(&p_encoder->P_STATE->Counter);
}

/******************************************************************************/
/*
    Angle snap — caller-driven angle capture (Hall sector boundary, index pulse)
*/
/******************************************************************************/
static inline void PulseEncoder_SetAngle(const PulseEncoder_T * p_encoder, angle16_t angle)    { AngleCounter_SetAngle(&p_encoder->P_STATE->Counter, angle); }
static inline void PulseEncoder_ZeroAngle(const PulseEncoder_T * p_encoder)                    { AngleCounter_ZeroAngle(&p_encoder->P_STATE->Counter); }
static inline void PulseEncoder_SetLimitWindow(const PulseEncoder_T * p_encoder, uangle16_t w) { AngleCounter_SetLimitWindow(&p_encoder->P_STATE->Counter, w); }

/******************************************************************************/
/*
    Stop Detection — delegate to PulseTimer
*/
/******************************************************************************/
static inline bool PulseEncoder_IsStop(const PulseEncoder_T * p_encoder)         { return PulseTimer_IsStop(&p_encoder->TIMER); }
static inline bool PulseEncoder_IsExtendedStop(const PulseEncoder_T * p_encoder) { return PulseTimer_IsExtendedStop(&p_encoder->TIMER); }

/******************************************************************************/
/*
    Query — delegate to AngleCounter
*/
/******************************************************************************/
static inline int32_t PulseEncoder_GetFreqD(const PulseEncoder_T * p_encoder)         { return AngleCounter_GetFreqD(&p_encoder->P_STATE->Counter); }
static inline int32_t PulseEncoder_GetRpm(const PulseEncoder_T * p_encoder)           { return AngleCounter_GetRpm(&p_encoder->P_STATE->Counter); }
static inline int32_t PulseEncoder_GetCps(const PulseEncoder_T * p_encoder)           { return AngleCounter_GetCps(&p_encoder->P_STATE->Counter); }
static inline int32_t PulseEncoder_GetSpeed_Fract16(const PulseEncoder_T * p_encoder) { return AngleCounter_GetSpeed_Fract16(&p_encoder->P_STATE->Counter); }

static inline const Angle_T * PulseEncoder_GetAngleState(const PulseEncoder_T * p_encoder) { return AngleCounter_Base(&p_encoder->P_STATE->Counter); }

/******************************************************************************/
/*
    Init
*/
/******************************************************************************/
static inline void PulseEncoder_Init(const PulseEncoder_T * p_encoder)
{
    PulseTimer_Init(&p_encoder->TIMER);
}

static inline void PulseEncoder_InitFrom(const PulseEncoder_T * p_encoder, const AngleCounter_Config_T * p_config)
{
    PulseTimer_Init(&p_encoder->TIMER);
    AngleCounter_InitFrom(&p_encoder->P_STATE->Counter, p_config);
}

static inline void PulseEncoder_SetInitial(const PulseEncoder_T * p_encoder)
{
    PulseTimer_SetInitial(&p_encoder->TIMER);
}
