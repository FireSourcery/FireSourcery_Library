
/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Accumulator.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Accumulator.h"



/******************************************************************************/
/*
    Init
*/
/******************************************************************************/
void Accumulator_Init(Accumulator_T * p_accum)
{
    p_accum->LimitLower = INT16_MIN;
    p_accum->LimitUpper = INT16_MAX;
    p_accum->Accumulator = 0;
    p_accum->Index = 0;
    p_accum->Shift = 0;
    p_accum->Coefficient = 1;
}


void _Accumulator_Init_Sat(Accumulator_T * p_accum, int8_t shift, int32_t coefficient, int32_t lower, int32_t upper, int32_t initial)
{
    p_accum->Shift = shift;
    p_accum->Coefficient = coefficient;
    p_accum->LimitLower = lower << shift;
    p_accum->LimitUpper = upper << shift;
    p_accum->Accumulator = math_clamp(initial << shift, p_accum->LimitLower, p_accum->LimitUpper);
    p_accum->Index = 0;
}

// void Accumulator_Init_Fract16(Accumulator_T * p_accum, int16_t coefficient)
// {
//     p_accum->Shift = 15U;
//     p_accum->Coefficient = (coefficient << p_accum->Shift);
// }

// void Accumulator_Init_Scaled(Accumulator_T * p_accum, int32_t coefficient)
// {
//     p_accum->Shift = math_max(15 - fixed_bit_width(coefficient), 0); /* left shift only */
//     p_accum->Coefficient = (coefficient << p_accum->Shift);
// }

// void Accumulator_Init_Integrator(Accumulator_T * p_accum, int32_t sampleFreq, int32_t coefficient)
// {
//     p_accum->Shift = math_limit_lower(15 - fixed_bit_width(coefficient / sampleFreq), 0); /* left shift only */
//     p_accum->Coefficient = (coefficient << p_accum->Shift) / sampleFreq;
// }


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


