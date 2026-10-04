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
    @file   real_fixed.h
    @author FireSourcery
    @brief  [real.h] fixed-point implementation. Binds the HAL vocabulary to [fract16.h].
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*
    Types
*/
/******************************************************************************/
typedef fract16_t fract_t;      /*!< [-1, 1) */
typedef ufract16_t ufract_t;    /*!< [0, 2) */
typedef accum32_t accum_t;      /*!< [-65536, 65536) */
typedef struct fract16_xy fract_xy_t;

/*
    Coefficient multiply
                fract16_mul (32-bit)    fract16e_mul            accum32_mul (int64)
        M4      2 (mul, asrs)           2 + loading its shift   3 (smull, lsrs, orr)
        M0+     2 (muls, asrs)          2 + loading its shift   9 + a call to __aeabi_lmul (39 instructions, 6 muls)
*/
#if !defined(FIXED_POINT_MUL_WIDE) && !defined(FIXED_POINT_MUL_EXP)
// #if (__ARM_ARCH_ISA_THUMB >= 2)
#define FIXED_POINT_MUL_WIDE       /* SMULL: M3/M4/M7 */
// #else
// #define FIXED_POINT_MUL_EXP      /* ARMv6-M: 32-bit product only */
// #endif
#endif

// #if defined(FIXED_POINT_MUL_WIDE)
// typedef accum32_t frexp_t;
// #else
// typedef fract16e_t frexp_t;
// #endif

#if defined(FIXED_POINT_MUL_WIDE)
typedef accum32_t coeff_t;
#define coeff_mul accum32_mul
static inline coeff_t coeff_of_accum(accum32_t k) { return k; }
#else
typedef fract16e_t coeff_t;
// typedef fract16e_t factor_t;
#define coeff_mul fract16e_mul
#define coeff_of_accum fract16e
#endif

/******************************************************************************/
/*
    Literals and constants
*/
/******************************************************************************/
#define FRACT(x) FRACT16(x)
#define ACCUM(x) ACCUM32(x)

#define FRACT_MAX           FRACT16_MAX
#define FRACT_1_DIV_2       FRACT16_1_DIV_2
#define FRACT_1_DIV_3       FRACT16_1_DIV_3
#define FRACT_2_DIV_3       FRACT16_2_DIV_3
#define FRACT_1_DIV_SQRT3   FRACT16_1_DIV_SQRT3
#define FRACT_SQRT3_DIV_2   FRACT16_SQRT3_DIV_2
#define FRACT_COS_120       FRACT16_COS_120
#define FRACT_SIN_120       FRACT16_SIN_120

#define ACCUM_ONE           FRACT16_1_OVERSAT
#define ACCUM_SQRT2         FRACT16_SQRT2
#define ACCUM_SQRT3         FRACT16_SQRT3
#define ACCUM_PI            FRACT16_PI

/******************************************************************************/
/*
    Storage boundary - wire, NVM, ADC
*/
/******************************************************************************/
static inline fract_t fract_of_fract16(fract16_t x) { return x; }
static inline fract16_t fract16_of_fract(fract_t x) { return x; }
static inline ufract_t ufract_of_ufract16(ufract16_t x) { return x; }
static inline ufract16_t ufract16_of_ufract(ufract_t x) { return x; }
static inline accum_t accum_of_accum32(accum32_t x) { return x; }
static inline accum32_t accum32_of_accum(accum_t x) { return x; }

/******************************************************************************/
/*
    Arithmetic
*/
/******************************************************************************/
#define fract_mul           fract16_mul
#define accum_mul           accum32_mul
#define accum_div           fract16_div         /* fract16_div is the unsaturated quotient */
#define fract_div           fract16_div_sat
#define fract_sat           fract16_sat
#define fract_sat_positive  fract16_sat_positive
#define fract_abs           fract16_abs
#define fract_abs_sat       fract16_abs_sat
#define fract_sqrt          fract16_sqrt
#define fract_normalize_sat fract16_normalize_sat

/******************************************************************************/
/*
    Trigonometry and vector
*/
/******************************************************************************/
#define fract_sin               fract16_sin
#define fract_cos               fract16_cos
#define fract_atan2             fract16_atan2
#define fract_vector            fract16_vector
#define fract_vector_magnitude  fract16_vector_magnitude
#define fract_vector_component  fract16_vector_component
