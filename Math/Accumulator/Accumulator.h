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
typedef int16_t accumulator_io_t;       /* user-scale value */
typedef int32_t accumulator_state_t;      /* internal state. include additional scaling for fixed-point representation */
/*
    Accumulator Max [INT32_MAX/2] => Accumulator + Input < INT32_MAX
    value range [-INT16_MAX:INT16_MAX]  => 2 * range << 15
    value range [-UINT16_MAX:UINT16_MAX] => 2 * range << 14
*/
#define ACCUMULATOR_SHIFT 15
#define ACCUMULATOR_SCALE (1L << ACCUMULATOR_SHIFT)
#define _ACCUM_STATE(io)    ((accumulator_state_t)((io) * ACCUMULATOR_SCALE))
#define _ACCUM_IO(state)    ((accumulator_io_t)((state) / ACCUMULATOR_SCALE))
#else
#include <math.h>
typedef float accumulator_io_t;
typedef float accumulator_state_t;
#define _ACCUM_STATE(x)        ((accumulator_state_t)(x))
#define _ACCUM_IO(x)      ((accumulator_io_t)(x))
// #define math_clamp(v, lo, hi) fmaxf((lo), fminf((hi), (v))) //todo
#endif


/******************************************************************************/
/*
    State
*/
/******************************************************************************/
typedef struct Accumulator
{
    accumulator_state_t Accumulator;  /* integrator output, internal scale */
    accumulator_state_t Coefficient;  /* gain K, internal scale */
    accumulator_state_t LimitUpper;   /* saturation bound, internal scale */
    accumulator_state_t LimitLower;
}
Accumulator_T;

/* y(n) = clamp(y(n-1) + K·u(n), LimitLower, LimitUpper)  */
static inline accumulator_state_t accumulator(accumulator_state_t rate, accumulator_state_t min, accumulator_state_t max, accumulator_state_t state, accumulator_state_t input)
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
static inline accumulator_io_t Accumulator_Output(const Accumulator_T * p_accum) { return _ACCUM_IO(p_accum->Accumulator); }
static inline void Accumulator_SetOutput(Accumulator_T * p_accum, accumulator_io_t value) { p_accum->Accumulator = math_clamp(_ACCUM_STATE(value), p_accum->LimitLower, p_accum->LimitUpper); }

static inline void Accumulator_Reset(Accumulator_T * p_accum) { p_accum->Accumulator = 0; }

static inline accumulator_io_t Accumulator_LimitUpper(const Accumulator_T * p_accum) { return _ACCUM_IO(p_accum->LimitUpper); }
static inline accumulator_io_t Accumulator_LimitLower(const Accumulator_T * p_accum) { return _ACCUM_IO(p_accum->LimitLower); }

static inline void Accumulator_SetLimits(Accumulator_T * p_accum, accumulator_io_t lower, accumulator_io_t upper)
{
    p_accum->LimitLower = _ACCUM_STATE(lower);
    p_accum->LimitUpper = _ACCUM_STATE(upper);
}

/******************************************************************************/
/*
    Discrete-time integrator
*/
/******************************************************************************/
/*! Unsaturated accumulation; output may exceed configured limits. */
static inline accumulator_io_t _Accumulator_Add(Accumulator_T * p_accum, accumulator_io_t input)
{
    p_accum->Accumulator += p_accum->Coefficient * input;
    return Accumulator_Output(p_accum);
}

/*! y[n] = clamp(y[n-1] + K * u[n], LimitLower, LimitUpper) — clamp at output. */
static inline accumulator_io_t Accumulator_Step(Accumulator_T * p_accum, accumulator_io_t input)
{
    p_accum->Accumulator = accumulator(p_accum->Coefficient, p_accum->LimitLower, p_accum->LimitUpper, p_accum->Accumulator, input);
    return Accumulator_Output(p_accum);
}

/*! y[n] = y[n-1] + clamp(K * u[n], LimitLower - y[n-1], LimitUpper - y[n-1]) — clamp at input. */
// static inline accumulator_io_t Accumulator_AddSat(Accumulator_T * p_accum, accumulator_io_t input)
// {
//     p_accum->Accumulator += math_clamp(p_accum->Coefficient * input, p_accum->LimitLower - p_accum->Accumulator, p_accum->LimitUpper - p_accum->Accumulator);
//     return Accumulator_Output(p_accum);
// }

/******************************************************************************/
/*
    Configuration (user-scale)
*/
/******************************************************************************/
static inline accumulator_io_t Accumulator_Coefficient(const Accumulator_T * p_accum) { return _ACCUM_IO(p_accum->Coefficient); }
static inline void Accumulator_SetCoefficient(Accumulator_T * p_accum, accumulator_io_t coefficient) { p_accum->Coefficient = _ACCUM_STATE(coefficient); }

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
extern void Accumulator_InitSat(Accumulator_T * p_accum, accumulator_io_t coefficient, accumulator_io_t lower, accumulator_io_t upper);

