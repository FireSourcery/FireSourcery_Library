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
#include <stdint.h>
#include <stdbool.h>

/*
    Accumulator32 Store as shifted
*/
typedef struct Accumulator
{
    /* [2 * [INT16_MIN:INT16_MAX] << 15] */
    int32_t Accumulator; /* Output shifted */

    int32_t Coefficient; /* shifted */
    int8_t Shift;

    int16_t LimitUpper;
    int16_t LimitLower;

    // uint32_t SampleFreq;
    // uint16_t Index;
    // uint16_t PrevInput;
    // int32_t CoefficientRef;
}
Accumulator_T;



static inline int32_t _Accumulator_Add(Accumulator_T * p_accum, int16_t input)
{
    return p_accum->Accumulator + ((int32_t)input * p_accum->Coefficient);
}

/* Simple accumulation with coefficient scaling */
static inline int32_t Accumulator_Add(Accumulator_T * p_accum, int16_t input)
{
    p_accum->Accumulator += ((int32_t)input * p_accum->Coefficient);
    return (p_accum->Accumulator >> p_accum->Shift);
}



/* Accessors */
static inline int32_t Accumulator_GetOutput(const Accumulator_T * p_accum) { return (p_accum->Accumulator >> p_accum->Shift); }
static inline void Accumulator_SetOutput(Accumulator_T * p_accum, int32_t value) { p_accum->Accumulator = (value << p_accum->Shift); }

static inline int32_t Accumulator_GetCoefficient(const Accumulator_T * p_accum) { return p_accum->Coefficient >> p_accum->Shift; }
// static inline int8_t Accumulator_GetShift(const Accumulator_T * p_accum) { return p_accum->Shift; }

static inline int32_t Accumulator_GetLimitUpper(const Accumulator_T * p_accum) { return p_accum->LimitUpper; }
static inline int32_t Accumulator_GetLimitLower(const Accumulator_T * p_accum) { return p_accum->LimitLower; }

static inline void Accumulator_SetCoefficient(Accumulator_T * p_accum, int32_t coefficient) { p_accum->Coefficient = coefficient; }
static inline void Accumulator_SetLimits(Accumulator_T * p_accum, int16_t lower, int16_t upper) { p_accum->LimitLower = lower; p_accum->LimitUpper = upper; }

/* Status Functions */
static inline bool Accumulator_IsSaturatedHigh(const Accumulator_T * p_accum) { return (Accumulator_GetOutput(p_accum) >= p_accum->LimitUpper); }
static inline bool Accumulator_IsSaturatedLow(const Accumulator_T * p_accum) { return (Accumulator_GetOutput(p_accum) <= p_accum->LimitLower); }
static inline bool Accumulator_IsSaturated(const Accumulator_T * p_accum) { return Accumulator_IsSaturatedHigh(p_accum) || Accumulator_IsSaturatedLow(p_accum); }

static inline void Accumulator_Reset(Accumulator_T * p_accum, int32_t value) { p_accum->Accumulator = value << p_accum->Shift; }
static inline void Accumulator_Clear(Accumulator_T * p_accum) { p_accum->Accumulator = 0; }


/******************************************************************************/
/*
    Mathematical Utility Functions
*/
/******************************************************************************/

// /*!
//     @brief  Calculate coefficient from time constant
//     @return Coefficient value
//     @note   For exponential response: coeff = (1 << shift) / time_constant
// */
// static inline int32_t Accumulator_CoefficientFromTimeConstant(int32_t time_constant, int8_t shift)
// {
//     return (time_constant > 0) ? ((1 << shift) / time_constant) : (1 << shift);
// }

// /*!
//     @brief  Calculate coefficient from alpha (0.0 to 1.0 range)
//     @param  alpha_q16  Alpha value in Q0.16 format (0-65535)
//     @return Coefficient scaled for specified shift
// */
// static inline int32_t Accumulator_CoefficientFromAlpha(int32_t alpha_q16, int8_t shift)
// {
//     return (alpha_q16 * (1 << shift)) >> 16;
// }

// /*!
//     @brief  Calculate time constant from coefficient
//     @return Time constant in samples
// */
// static inline int32_t Accumulator_TimeConstantFromCoefficient(int32_t coefficient, int8_t shift)
// {
//     return (coefficient > 0) ? ((1 << shift) / coefficient) : INT32_MAX;
// }

// /*!
//     @brief  Scale coefficient for different shift value
//     @return Scaled coefficient
// */
// static inline int32_t Accumulator_ScaleCoefficient(int32_t coefficient, int8_t old_shift, int8_t new_shift)
// {
//     if (new_shift > old_shift)
//     {
//         return coefficient << (new_shift - old_shift);
//     }
//     else
//     {
//         return coefficient >> (old_shift - new_shift);
//     }
// }


