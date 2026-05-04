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
    @file   Ramp.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Accumulator/Accumulator.h"
#include "../math_general.h"

#include <stdint.h>
#include <stdbool.h>


#define RAMP_SHIFT 14

/* storage in range, ticks */
#define RAMP_TICKS_OF_MILLIS(updateFreq_Hz, duration_Ms) (updateFreq_Hz * duration_Ms / 1000U)
#define RAMP_TICKS_OF_RATE(UpdateFreq_Hz, Range, UnitsPerSecond) ((UpdateFreq_Hz) * (Range) / (UnitsPerSecond))

#define RAMP_COEF_OF_SLOPE(UpdateFreq_Hz, SlopePerSecond) (((int32_t)(SlopePerSecond) << RAMP_SHIFT) / (UpdateFreq_Hz))
#define RAMP_COEF_OF_DURATION_TICKS(Range, DurationTicks) (((int32_t)(Range) << RAMP_SHIFT) / (DurationTicks))
#define RAMP_COEF_OF_DURATION_MS(UpdateFreq_Hz, Range, DurationMs) (((int32_t)(Range) << RAMP_SHIFT) * 1000U / ((UpdateFreq_Hz) * (DurationMs)))
// #define RAMP_COEF_OF_DURATION_MS(UpdateFreq_Hz, Range, DurationMs) (RAMP_COEF_OF_DURATION_TICKS(Range, RAMP_TICKS_OF_MILLIS((float)UpdateFreq_Hz, DurationMs)) )


/*
    Compile time full init
*/
#define RAMP_INIT(Coefficient, LowerLimit, UpperLimit, Initial) (Ramp_T) \
{                                                           \
    .Accumulator =                                          \
    {                                                       \
        .Shift       = RAMP_SHIFT,                          \
        .Coefficient = (Coefficient),                       \
        .LimitLower  = (int32_t)(LowerLimit) << RAMP_SHIFT, \
        .LimitUpper  = (int32_t)(UpperLimit) << RAMP_SHIFT, \
        .Accumulator = (int32_t)(Initial)    << RAMP_SHIFT, \
    },                                                      \
}

/* convenience: full-fract16 saturating ramp from a duration */
#define RAMP_INIT_FRACT16_MS(UpdateFreq_Hz, DurationMs) RAMP_INIT(RAMP_COEF_OF_DURATION_MS((UpdateFreq_Hz), 65535, (DurationMs)), INT16_MIN, INT16_MAX, 0)


/******************************************************************************/
/*
    Implementation by shifted accumulator.
        output state can be set directly.
        Index based implementation need inverse function
    complete setpoint object: { shape, bound, output, target }
*/
/******************************************************************************/
typedef struct Ramp
{
    Accumulator_T Accumulator;
    int32_t Target; /* including target collapses procNext paths, handle on set instaed */
    // int32_t TargetLimitUpper; /* optionally 2 layer limit, with Accumulator.Limit as hw saturation */
    // int32_t TargetLimitLower;
}
Ramp_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
/*  */
static inline int32_t Ramp_GetOutput(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Accumulator >> RAMP_SHIFT); }
static inline void Ramp_SetOutputState(Ramp_T * p_ramp, int32_t match) { p_ramp->Accumulator.Accumulator = math_clamp((match << RAMP_SHIFT), p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper); }

static inline int32_t Ramp_GetTarget(const Ramp_T * p_ramp) { return p_ramp->Target >> RAMP_SHIFT; }
static inline void  Ramp_SetTarget(Ramp_T * p_ramp, int32_t target) { p_ramp->Target = math_clamp((int32_t)target << RAMP_SHIFT, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper); }

static inline int32_t Ramp_GetLimitLower(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.LimitLower >> RAMP_SHIFT); }
static inline int32_t Ramp_GetLimitUpper(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.LimitUpper >> RAMP_SHIFT); }

static inline void Ramp_SetOutputLimit(Ramp_T * p_ramp, int32_t lower, int32_t upper)
{
    p_ramp->Accumulator.LimitLower = (int32_t)lower << RAMP_SHIFT;
    p_ramp->Accumulator.LimitUpper = (int32_t)upper << RAMP_SHIFT;
    /* Limit setter re-clamps the stored target alongside the accumulator. */
    p_ramp->Target = math_clamp(p_ramp->Target, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper);
}

/* Snap the output */
static inline void Ramp_SetOutputLimit_Snap(Ramp_T * p_ramp, int32_t lower, int32_t upper)
{
    Ramp_SetOutputLimit(p_ramp, lower, upper);
    p_ramp->Accumulator.Accumulator = math_clamp(p_ramp->Accumulator.Accumulator, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper);

}

/* single step proc only */
static inline bool _Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Coefficient == (UINT16_MAX << RAMP_SHIFT)); }
static inline bool _Ramp_IsEnabled(const Ramp_T * p_ramp) { return !_Ramp_IsDisabled(p_ramp); }
static inline void _Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Accumulator.Coefficient = (UINT16_MAX << RAMP_SHIFT); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern int32_t Ramp_ProcNextOf(Ramp_T * p_ramp, int16_t target);

extern int32_t _Ramp_ProcNextOnInputOf(Ramp_T * p_ramp, int16_t target);
extern int32_t Ramp_ProcNextOnInputOf(Ramp_T * p_ramp, int16_t target);

extern int32_t Ramp_ProcNext(Ramp_T * p_ramp);

extern void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range);
extern void Ramp_SetCoefficient(Ramp_T * p_ramp, uint32_t rate_accum32);
extern void Ramp_SetSlope(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range);

extern void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range);

