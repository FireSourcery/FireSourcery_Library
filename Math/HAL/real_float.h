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
    @file   real_float.h
    @author FireSourcery
    @brief  [real.h] floating-point implementation.
            Angles stay [angle16_t]. Storage stays [fract16_t] [ufract16_t] [accum32_t].
*/
/******************************************************************************/
#include "math_general_float.h"
#include "Math/Fixed/fract16.h"
#include <math.h>

/******************************************************************************/
/*
    Types
*/
/******************************************************************************/
typedef float fract_t;      /*!< [-1, 1] */
typedef float ufract_t;     /*!< [0, 2) */
typedef float accum_t;
typedef struct { fract_t x; fract_t y; } fract_xy_t;

typedef float coeff_t;
static inline accum_t coeff_mul(coeff_t k, accum_t x) { return k * x; }
static inline coeff_t coeff_of_accum(accum_t k) { return k; }

/******************************************************************************/
/*
    Literals and constants
*/
/******************************************************************************/
/* Saturates as [FRACT16] does. Constant expression for initializers. */
#define FRACT(x) ((fract_t)(((x) < 1.0F) ? (((x) >= -1.0F) ? (x) : -1.0F) : 1.0F))
#define ACCUM(x) ((accum_t)(x))

static const fract_t FRACT_MAX          = 1.0F;
static const fract_t FRACT_1_DIV_2      = 0.5F;
static const fract_t FRACT_1_DIV_3      = 0.333333333F;
static const fract_t FRACT_2_DIV_3      = 0.666666667F;
static const fract_t FRACT_1_DIV_SQRT3  = 0.577350269F;
static const fract_t FRACT_SQRT3_DIV_2  = 0.866025404F;
static const fract_t FRACT_COS_120      = -0.5F;
static const fract_t FRACT_SIN_120      = 0.866025404F;

static const accum_t ACCUM_ONE          = 1.0F;
static const accum_t ACCUM_SQRT2        = 1.414213562F;
static const accum_t ACCUM_SQRT3        = 1.732050808F;
static const accum_t ACCUM_PI           = 3.141592654F;

/******************************************************************************/
/*
    Storage boundary - wire, NVM, ADC
*/
/******************************************************************************/
static inline fract_t fract_of_fract16(fract16_t x) { return (fract_t)x / FRACT16_SCALE; }
static inline fract16_t fract16_of_fract(fract_t x) { return FRACT16(x); }
static inline ufract_t ufract_of_ufract16(ufract16_t x) { return (ufract_t)x / FRACT16_SCALE; }
static inline ufract16_t ufract16_of_ufract(ufract_t x) { return (ufract16_t)(math_clampf(x, 0.0F, (float)UINT16_MAX / FRACT16_SCALE) * FRACT16_SCALE); }
static inline accum_t accum_of_accum32(accum32_t x) { return (accum_t)x / FRACT16_SCALE; }
static inline accum32_t accum32_of_accum(accum_t x) { return ACCUM32(x); }

/******************************************************************************/
/*
    Arithmetic
*/
/******************************************************************************/
static inline accum_t fract_mul(accum_t a, accum_t b) { return a * b; }
static inline accum_t accum_mul(accum_t a, accum_t b) { return a * b; }
static inline accum_t accum_div(accum_t num, accum_t den) { return num / den; }
static inline fract_t fract_sat(accum_t x) { return math_clampf(x, -FRACT_MAX, FRACT_MAX); }
static inline ufract_t fract_sat_positive(accum_t x) { return math_clampf(x, 0.0F, FRACT_MAX); }
static inline fract_t fract_div(accum_t num, accum_t den) { return fract_sat(num / den); }
static inline ufract_t fract_abs(fract_t x) { return math_absf(x); }
static inline fract_t fract_abs_sat(fract_t x) { return math_minf(math_absf(x), FRACT_MAX); }
/* Negative input clamps to 0 rather than NaN, NaN does not recover in a feedback loop */
static inline fract_t fract_sqrt(fract_t x) { return sqrtf(math_maxf(x, 0.0F)); }
static inline ufract_t fract_normalize_sat(accum_t lo, accum_t hi, accum_t x) { return math_clampf((x - lo) / (hi - lo), 0.0F, FRACT_MAX); }

/******************************************************************************/
/*
    Trigonometry and vector
*/
/******************************************************************************/
/* Same table as the fixed build */
static inline fract_t fract_sin(angle16_t theta) { return fract_of_fract16(fract16_sin(theta)); }
static inline fract_t fract_cos(angle16_t theta) { return fract_of_fract16(fract16_cos(theta)); }
/* via int32_t: +pi maps to 32768, which wraps to -32768 */
static inline angle16_t fract_atan2(fract_t y, fract_t x) { return (angle16_t)(int32_t)(atan2f(y, x) * (ANGLE16_PER_REVOLUTION / (2.0F * PI_FLOAT))); }

static inline fract_xy_t fract_vector(angle16_t theta) { return (fract_xy_t) { .x = fract_cos(theta), .y = fract_sin(theta) }; }
static inline ufract_t fract_vector_magnitude(fract_t x, fract_t y) { return sqrtf(x * x + y * y); }
static inline ufract_t fract_vector_component(fract_t x, ufract_t mag_limit) { return sqrtf(math_maxf(mag_limit * mag_limit - x * x, 0.0F)); }
