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
    @file   Accumulator.h
    @author FireSourcery
    @brief  [Brief description of the file]
    @note   Stores internal state as shifted for precision, provides unshifted interface
*/
/******************************************************************************/
#include "../math_general.h"
#include <stdint.h>
#include <stdbool.h>

/*
    Integrator with Saturation forward rectangular approximation.

    Internal state is kept shifted
    Accumulator += input * Coefficient lands directly in shifted state.
*/
typedef struct Accumulator
{
    /* Accumulator Max [INT32_MAX/2] => Accumulator + Input_Shifted < INT32_MAX */
    /*
        value range [-UINT16_MAX:UINT16_MAX] => 2 * range << 14
        value range [-INT16_MAX:INT16_MAX]  => 2 * range << 15
    */
    int32_t Accumulator;    /* Shifted state; user-visible output = Accumulator >> Shift */
    int32_t Coefficient;    /* Shifted gain K; Accumulator += input * Coefficient */
    int8_t Shift;           /* Q-factor — bits of fractional precision in Accumulator */

    int32_t LimitUpper;     /* Shifted upper saturation bound */
    int32_t LimitLower;     /* Shifted lower saturation bound */

    uint16_t Index;         /* Sample counter (used by Avg) */
}
Accumulator_T;


/* y(n) = clamp(y(n-1) + K·u(n), LimitLower, LimitUpper)  */
static inline int32_t accumulator(int32_t rate, int32_t min, int32_t max, int32_t state, int32_t input)
{
    return math_clamp(state + (rate * input), min, max);
}

static inline int32_t accumulator_target(int32_t rate, int32_t min, int32_t max, int32_t state, int32_t target32)
{
    return accumulator(rate, min, max, state, math_sign(target32 - state));
}

/******************************************************************************/
/*
    Discrete-Time Integrator
*/
/******************************************************************************/
static inline int32_t Accumulator_Output(Accumulator_T * p_accum) { return (p_accum->Accumulator >> p_accum->Shift); }

/*
    Unsaturated accumulation.
*/
static inline int32_t Accumulator_Add(Accumulator_T * p_accum, int16_t input)
{
    p_accum->Accumulator += ((int32_t)input * p_accum->Coefficient);
    return Accumulator_Output(p_accum);
}

/*
    Forward Euler with saturation.
*/
static inline int32_t Accumulator_Step(Accumulator_T * p_accum, int16_t input)
{
    p_accum->Accumulator = accumulator(p_accum->Coefficient, p_accum->LimitLower, p_accum->LimitUpper, p_accum->Accumulator, input);
    return Accumulator_Output(p_accum);
}

/*
    Ramp output toward target at ±Coefficient per step, saturated.
    Target is unshifted (user-scale). Output settles on target when within one step.
*/
// static inline int32_t Accumulator_Ramp_Signed(Accumulator_T * p_accum, int32_t target)
// {
//     int32_t error = (target << p_accum->Shift) - p_accum->Accumulator;
//     int32_t step = math_clamp((int32_t)math_sign(error) * p_accum->Coefficient, -math_abs(error), math_abs(error));
//     p_accum->Accumulator = math_clamp(p_accum->Accumulator + step, p_accum->LimitLower, p_accum->LimitUpper);
// }

static inline int32_t Accumulator_Ramp(Accumulator_T * p_accum, int32_t target)
{
    int32_t targetAccum = target << p_accum->Shift;
    int32_t error = targetAccum - p_accum->Accumulator;
    p_accum->Accumulator = (math_abs(error) <= (uint32_t)p_accum->Coefficient) ? targetAccum : (p_accum->Accumulator + math_sign(error) * (int32_t)p_accum->Coefficient);
    return Accumulator_Output(p_accum);
}

/******************************************************************************/
/*
    Accessors
*/
/******************************************************************************/
static inline void Accumulator_SetOutput(Accumulator_T * p_accum, int32_t value)
{
    p_accum->Accumulator = math_clamp((int32_t)value << p_accum->Shift, p_accum->LimitLower, p_accum->LimitUpper);
}

static inline int32_t Accumulator_Coefficient(const Accumulator_T * p_accum) { return p_accum->Coefficient >> p_accum->Shift; }
static inline void Accumulator_SetCoefficient(Accumulator_T * p_accum, int32_t coefficient) { p_accum->Coefficient = coefficient; }

static inline int32_t Accumulator_LimitUpper(const Accumulator_T * p_accum) { return p_accum->LimitUpper >> p_accum->Shift; }
static inline int32_t Accumulator_LimitLower(const Accumulator_T * p_accum) { return p_accum->LimitLower >> p_accum->Shift; }

static inline void Accumulator_SetLimits(Accumulator_T * p_accum, int32_t lower, int32_t upper)
{
    p_accum->LimitLower = ((int32_t)lower << p_accum->Shift);
    p_accum->LimitUpper = ((int32_t)upper << p_accum->Shift);
}

/******************************************************************************/
/*
    Status
*/
/******************************************************************************/
static inline bool Accumulator_IsSaturatedHigh(const Accumulator_T * p_accum) { return (p_accum->Accumulator >= p_accum->LimitUpper); }
static inline bool Accumulator_IsSaturatedLow(const Accumulator_T * p_accum) { return (p_accum->Accumulator <= p_accum->LimitLower); }
static inline bool Accumulator_IsSaturated(const Accumulator_T * p_accum) { return Accumulator_IsSaturatedHigh(p_accum) || Accumulator_IsSaturatedLow(p_accum); }

/******************************************************************************/
/*
    Reset
*/
/******************************************************************************/
static inline void Accumulator_Reset(Accumulator_T * p_accum, int32_t value) { Accumulator_SetOutput(p_accum, value); }
static inline void Accumulator_Clear(Accumulator_T * p_accum) { p_accum->Accumulator = 0; }


/******************************************************************************/
/*
    Extended Functions
*/
/******************************************************************************/
/* external track index */
static inline int32_t _Accumulator_Avg(Accumulator_T * p_accum, int32_t index, int32_t in) { return Accumulator_Add(p_accum, in) / index; }

static inline int32_t Accumulator_Avg(Accumulator_T * p_accum, int32_t in)
{
    p_accum->Index++;
    return _Accumulator_Avg(p_accum, p_accum->Index, in);
}


extern void Accumulator_Init(Accumulator_T * p_accum);
extern void Accumulator_Init_Sat(Accumulator_T * p_accum, int8_t shift, int32_t coefficient, int32_t lower, int32_t upper, int32_t initial);
extern void Accumulator_Init_AsFract16(Accumulator_T * p_accum, int16_t coefficient_param);


