#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   scalar.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
// #include "math_general.h"
#include "Math/Fixed/fract16.h"

/*
    HAL for math
    data abstraction layer for math value type
    wrap semantics over storage.
    uniform handling in float <=> selective in fixed-point
*/
#if !defined(FLOATING_POINT) && !defined(FIXED_POINT_MUL_WIDE) && !defined(FIXED_POINT_MUL_EXP)
// #if (__ARM_ARCH_ISA_THUMB >= 2)
#define FIXED_POINT_MUL_WIDE       /* SMULL: M3/M4/M7 */
// #else
// #define FIXED_POINT_MUL_EXP      /* ARMv6-M: 32-bit product only */
// #endif
#endif

/*
    fract16_mul (32-bit)	coeff16_mul	accum32_mul (int64)
    M4	2 (mul, asrs)	2 + loading its shift	3 (smull, lsrs, orr)
    M0+	2 (muls, asrs)	2 + loading its shift	9 + a call to __aeabi_lmul (39 instructions, 6 muls)
*/

#if defined(FLOATING_POINT)
typedef float fract_t;  /* < 1.0 */
typedef float accum_t;  /* > 1.0 */
typedef float coeff_t;  /* > 1.0, fast loop */
#else
typedef fract16_t fract_t;
typedef accum32_t accum_t;
// #if defined(FIXED_POINT_MUL_WIDE)
// typedef accum32_t frexp_t;
// #else
// typedef fract16e_t frexp_t;
// #endif

#if defined(FIXED_POINT_MUL_WIDE)
typedef accum32_t coeff_t;
#else
typedef fract16e_t coeff_t;
// typedef fract16e_t factor_t;
#endif

#endif

static inline float _float_mul(float a, float b) { return a * b; }

// preprocessor macro swaps to semantics type first.
#define coeff_mul(a, b) _Generic((a), \
    accum32_t: accum_mul, \
    fract16e_t: fract16e_mul, \
    float : _float_mul \
)(a, b)


/******************************************************************************/
/*
    Number formats

    [Fract16]           [-1:1) <=> [-32768:32767] in Q1.15
    [UFract16]          [0:2) <=> [0:65535] in Q1.15
    [Accum32]           [-2:2] <=> [-65536:65536] in Q17.15     Max [INT32_MIN:INT32_MAX]
    [UQ16]              [0:1) <=> [0:65535] in Q0.16
    [Fixed16]           [-1:1] <=> [-256:256] in Q8.8           Max [-32768:32767]
    [Fixed32]           [-1:1] <=> [-65536:65536] in Q16.16     Max [INT32_MIN:INT32_MAX]
*/
/******************************************************************************/