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


/* storage in range, ticks */
#define RAMP_TICKS_OF_MILLIS(updateFreq_Hz, duration_Ms) (updateFreq_Hz * duration_Ms / 1000U)
#define RAMP_TICKS_OF_RATE(UpdateFreq_Hz, Range, UnitsPerSecond) ((UpdateFreq_Hz) * (Range) / (UnitsPerSecond))

/* SLOPE_OF_S */
#define RAMP_COEF_OF_SLOPE(UpdateFreq_Hz, SlopePerSecond) (((int32_t)(SlopePerSecond) << ACCUMULATOR_SHIFT) / (UpdateFreq_Hz))
#define RAMP_COEF_OF_DURATION_TICKS(Range, DurationTicks) (((int32_t)(Range) << ACCUMULATOR_SHIFT) / (DurationTicks))
#define RAMP_COEF_OF_DURATION_MS(UpdateFreq_Hz, Range, DurationMs) (((int64_t)(Range) << ACCUMULATOR_SHIFT) * 1000U / ((UpdateFreq_Hz) * (DurationMs)))
// #define RAMP_COEF_OF_DURATION_MS(UpdateFreq_Hz, Range, DurationMs) (RAMP_COEF_OF_DURATION_TICKS(Range, RAMP_TICKS_OF_MILLIS((float)UpdateFreq_Hz, DurationMs)) )

/*
    Compile time full init
*/
#define RAMP_INIT(Coefficient, LowerLimit, UpperLimit, Initial) (Ramp_T) \
{                                                           \
    .Accumulator =                                          \
    {                                                       \
        .Coefficient = (Coefficient),                       \
        .LimitLower  = (int32_t)(LowerLimit) << ACCUMULATOR_SHIFT, \
        .LimitUpper  = (int32_t)(UpperLimit) << ACCUMULATOR_SHIFT, \
        .Accumulator = (int32_t)(Initial)    << ACCUMULATOR_SHIFT, \
    },                                                      \
}

/* convenience: full-fract16 saturating ramp from a duration */
#define RAMP_INIT_FRACT16_MS(UpdateFreq_Hz, DurationMs) RAMP_INIT(RAMP_COEF_OF_DURATION_MS((UpdateFreq_Hz), INT16_MAX, (DurationMs)), INT16_MIN, INT16_MAX, 0)


/******************************************************************************/
/*
    complete setpoint object: { shape, bound, output, target }
    Implementation by shifted accumulator.
        output state can be set directly.
        Index based implementation need inverse function
*/
/******************************************************************************/
// int32_t TargetLimitUpper; /* optionally 2 layer limit, with Accumulator.Limit as hw saturation */
// int32_t TargetLimitLower;
typedef struct Ramp
{
    Accumulator_T Accumulator;
    accumulator_raw_t Target; /* including target collapses procNext paths, handle on set instaed */
}
Ramp_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline accumulator_t Ramp_GetOutput(const Ramp_T * p_ramp) { return Accumulator_Output(&p_ramp->Accumulator); }
static inline void Ramp_SetOutputState(Ramp_T * p_ramp, accumulator_t match) { Accumulator_SetOutput(&p_ramp->Accumulator, match); }

static inline accumulator_t Ramp_GetTarget(const Ramp_T * p_ramp) { return _ACCUM_FROM_RAW(p_ramp->Target); }
static inline void Ramp_SetTarget(Ramp_T * p_ramp, accumulator_t target) { p_ramp->Target = math_clamp(_ACCUM_TO_RAW(target), p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper); }

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline accumulator_t Ramp_GetLimitLower(const Ramp_T * p_ramp){ return Accumulator_LimitLower(&p_ramp->Accumulator); }
static inline accumulator_t Ramp_GetLimitUpper(const Ramp_T * p_ramp){ return Accumulator_LimitUpper(&p_ramp->Accumulator); }

/* OnInput Limits */
/* Set Target Window */
static inline void Ramp_SetLimits(Ramp_T * p_ramp, accumulator_t lower, accumulator_t upper)
{
    Accumulator_SetLimits(&p_ramp->Accumulator, lower, upper);
    p_ramp->Target = math_clamp(p_ramp->Target, p_ramp->Accumulator.LimitLower, p_ramp->Accumulator.LimitUpper); /* Limit setter re-clamps the stored target alongside the accumulator. */
}

/* Clamp Output immediately */
static inline void Ramp_SetOutputLimits(Ramp_T * p_ramp, accumulator_t lower, accumulator_t upper)
{
    Ramp_SetLimits(p_ramp, lower, upper);
    Ramp_SetOutputState(p_ramp, Ramp_GetOutput(p_ramp));  /* re-clamps */
}


/* single step proc only */
static inline bool _Ramp_IsDisabled(const Ramp_T * p_ramp) { return (p_ramp->Accumulator.Coefficient == (INT16_MAX << ACCUMULATOR_SHIFT)); }
static inline bool _Ramp_IsEnabled(const Ramp_T * p_ramp) { return !_Ramp_IsDisabled(p_ramp); }
static inline void _Ramp_Disable(Ramp_T * p_ramp) { p_ramp->Accumulator.Coefficient = (INT16_MAX << ACCUMULATOR_SHIFT); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern int32_t _Ramp_ProcNextOf(Ramp_T * p_ramp, int16_t target);
extern int32_t Ramp_ProcNextOf(Ramp_T * p_ramp, int16_t target);
extern int32_t Ramp_ProcNext(Ramp_T * p_ramp);

extern void Ramp_Init(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range);
extern void Ramp_SetCoefficient(Ramp_T * p_ramp, uint32_t rate_accum32);
extern void Ramp_SetSlope_Ticks(Ramp_T * p_ramp, uint32_t duration_Ticks, uint16_t range);
extern void Ramp_SetSlope_Millis(Ramp_T * p_ramp, uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range);

