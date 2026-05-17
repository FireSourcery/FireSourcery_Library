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
    @brief  Discrete-time integrator with output saturation.

    Forward-Euler form: y[n] = clamp(y[n-1] + K * u[n], LimitLower, LimitUpper)

*/
/******************************************************************************/
#include "../math_general.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Mode selection: fixed-point (default) or float
*/
/******************************************************************************/
#ifndef ACCUMULATOR_FLOAT
/* Accumulator Max [INT32_MAX/2] => Accumulator + Input < INT32_MAX */
/*
    value range [-UINT16_MAX:UINT16_MAX] => 2 * range << 14
    value range [-INT16_MAX:INT16_MAX]  => 2 * range << 15
*/
typedef int16_t accumulator_t;      /* user-scale value */
typedef int32_t accumulator_raw_t;  /* internal Q-format state */
#define ACCUMULATOR_SHIFT 15
#define ACCUMULATOR_SCALE (1L << ACCUMULATOR_SHIFT)
#define _ACCUM_TO_RAW(x)        ((accumulator_raw_t)((x) * ACCUMULATOR_SCALE))
#define _ACCUM_FROM_RAW(x)      ((accumulator_t)((x) / ACCUMULATOR_SCALE))
#else
#include <math.h>
typedef float accumulator_t;
typedef float accumulator_raw_t;
#define _ACCUM_TO_RAW(x)        ((accumulator_raw_t)(x))
#define _ACCUM_FROM_RAW(x)      ((accumulator_t)(x))
// #define math_clamp(v, lo, hi) fmaxf((lo), fminf((hi), (v))) //todo
#endif

/******************************************************************************/
/*
    State
*/
/******************************************************************************/
typedef struct Accumulator
{
    accumulator_raw_t Accumulator;  /* integrator output, internal scale */
    accumulator_raw_t Coefficient;  /* gain K, internal scale */
    accumulator_raw_t LimitUpper;   /* saturation bound, internal scale */
    accumulator_raw_t LimitLower;
}
Accumulator_T;

/* y(n) = clamp(y(n-1) + K·u(n), LimitLower, LimitUpper)  */
static inline accumulator_raw_t accumulator(accumulator_raw_t rate, accumulator_raw_t min, accumulator_raw_t max, accumulator_raw_t state, accumulator_raw_t input)
{
    return math_clamp(state + (rate * input), min, max);
}




/******************************************************************************/
/*
    Field Accessors
*/
/******************************************************************************/
/******************************************************************************/
/*
    Output / state access (user-scale)
*/
/******************************************************************************/
static inline accumulator_t Accumulator_Output(const Accumulator_T * p_accum) { return _ACCUM_FROM_RAW(p_accum->Accumulator); }
static inline void Accumulator_SetOutput(Accumulator_T * p_accum, accumulator_t value) { p_accum->Accumulator = math_clamp(_ACCUM_TO_RAW(value), p_accum->LimitLower, p_accum->LimitUpper); }

static inline void Accumulator_Reset(Accumulator_T * p_accum) { p_accum->Accumulator = 0; }

static inline accumulator_t Accumulator_LimitUpper(const Accumulator_T * p_accum) { return _ACCUM_FROM_RAW(p_accum->LimitUpper); }
static inline accumulator_t Accumulator_LimitLower(const Accumulator_T * p_accum) { return _ACCUM_FROM_RAW(p_accum->LimitLower); }

static inline void Accumulator_SetLimits(Accumulator_T * p_accum, accumulator_t lower, accumulator_t upper)
{
    p_accum->LimitLower = _ACCUM_TO_RAW(lower);
    p_accum->LimitUpper = _ACCUM_TO_RAW(upper);
}

/******************************************************************************/
/*
    Discrete-time integrator
*/
/******************************************************************************/
/*! Unsaturated accumulation; output may exceed configured limits. */
static inline accumulator_t _Accumulator_Add(Accumulator_T * p_accum, accumulator_t input)
{
    p_accum->Accumulator += p_accum->Coefficient * input;
    return Accumulator_Output(p_accum);
}

/*! y[n] = clamp(y[n-1] + K * u[n], LimitLower, LimitUpper) — clamp at output. */
static inline accumulator_t Accumulator_Step(Accumulator_T * p_accum, accumulator_t input)
{
    p_accum->Accumulator = accumulator(p_accum->Coefficient, p_accum->LimitLower, p_accum->LimitUpper, p_accum->Accumulator, input);
    return Accumulator_Output(p_accum);
}

/*! y[n] = y[n-1] + clamp(K * u[n], LimitLower - y[n-1], LimitUpper - y[n-1]) — clamp at input. */
// static inline accumulator_t Accumulator_AddSat(Accumulator_T * p_accum, accumulator_t input)
// {
//     p_accum->Accumulator += math_clamp(p_accum->Coefficient * input, p_accum->LimitLower - p_accum->Accumulator, p_accum->LimitUpper - p_accum->Accumulator);
//     return Accumulator_Output(p_accum);
// }

/******************************************************************************/
/*
    Configuration (user-scale)
*/
/******************************************************************************/
static inline accumulator_t Accumulator_Coefficient(const Accumulator_T * p_accum) { return _ACCUM_FROM_RAW(p_accum->Coefficient); }
static inline void Accumulator_SetCoefficient(Accumulator_T * p_accum, accumulator_t coefficient) { p_accum->Coefficient = _ACCUM_TO_RAW(coefficient); }

/******************************************************************************/
/*
    Saturation status
*/
/******************************************************************************/
static inline bool Accumulator_IsSaturatedHigh(const Accumulator_T * p_accum) { return p_accum->Accumulator >= p_accum->LimitUpper; }
static inline bool Accumulator_IsSaturatedLow(const Accumulator_T * p_accum) { return p_accum->Accumulator <= p_accum->LimitLower; }
static inline bool Accumulator_IsSaturated(const Accumulator_T * p_accum) { return Accumulator_IsSaturatedHigh(p_accum) || Accumulator_IsSaturatedLow(p_accum); }
static inline sign_t Accumulator_SaturationSign(const Accumulator_T * p_accum) { return (sign_t)(Accumulator_IsSaturatedHigh(p_accum) ? 1 : (Accumulator_IsSaturatedLow(p_accum) ? -1 : 0)); }


/******************************************************************************/
/*
    Init
*/
/******************************************************************************/
extern void Accumulator_Init(Accumulator_T * p_accum);
extern void Accumulator_InitSat(Accumulator_T * p_accum, accumulator_t coefficient, accumulator_t lower, accumulator_t upper);

