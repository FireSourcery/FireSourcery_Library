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
    @file   Fract16.h
    @author FireSourcery

    @brief  Math with 16 bit fractions in Q1.15 format
*/
/******************************************************************************/
#ifndef FRACT16_H
#define FRACT16_H

#include "fixed.h"
#include "angle16.h"
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>
// #include <stdfix.h>
#include <assert.h>

/*
    Convention by stdfix.h
*/

#define FRACT16_N_BITS (15)   /*!< Q1.15, 15 fractional bits. Resolution 1/(2^15) == .000030517578125 */
#define FRACT16_M_BITS (1)

#define FRACT16_SCALE (32768) /* 2^15 */
#define FRACT32_SCALE (2147483648)

#define ACCUM32_SAT   (0x3FFFFFFFUL) /* Soft max for ACCUM32_SAT * 2 without overflow */
#define ACCUM32_SCALE ((int32_t)FRACT16_SCALE * FRACT16_SCALE)

typedef int16_t fract16_t;      /*!< Q1.15 [-1, 1) */
typedef uint16_t ufract16_t;    /*!< Q1.15 [0, 2), √2 vector magnitude. */

typedef int32_t accum32_t;      /*!< Q17.15 [-65536.0, 65535.0] extended integer bits. */ /*!< Q2.30 [-2, 2) scaled fract16_t, [-1, 1) for fast saturated add. */
typedef uint32_t uaccum32_t;    /*!< Q2.30 [0, 4) extended unsigned fraction bits. Typemarker for hal. */

typedef int32_t fract32_t;      /*!< Q1.31 [-1, 1) extended fraction bits. */
typedef uint16_t uq16_t;        /*!< Q0.16 [0, 1) */ // percent16

static const fract16_t FRACT16_MAX = INT16_MAX; /*!< (32767) */
static const fract16_t FRACT16_MIN = INT16_MIN; /*!< (-32768) */

static const fract16_t FRACT16_1_DIV_4 = 0x2000;
static const fract16_t FRACT16_1_DIV_2 = 0x4000; /*!< 16384 */
static const fract16_t FRACT16_3_DIV_4 = 0x6000;
static const fract16_t FRACT16_1_DIV_3 = 0x2AAA;
static const fract16_t FRACT16_2_DIV_3 = 0x5555;
static const fract16_t FRACT16_1_DIV_SQRT3 = 0x49E7; /*!< 0.57735026919f */
static const fract16_t FRACT16_1_DIV_SQRT2 = 0x5A82; /*!< 0.70710678118f */
static const fract16_t FRACT16_SQRT3_DIV_2 = 0x6EDA;
static const fract16_t FRACT16_SQRT3_DIV_4 = 0x376D;
static const fract16_t FRACT16_SQRT2_DIV_2 = 0x5A82;
static const fract16_t FRACT16_PI_DIV_4 = 0x6487;
static const fract16_t FRACT16_1_DIV_PI = 0x28BE; /*!< 0.31830988618f */

static const fract16_t FRACT16_COS_120 = -FRACT16_1_DIV_2;      /* cos(120°) = -0.5 */
static const fract16_t FRACT16_SIN_120 = FRACT16_SQRT3_DIV_2;   /* sin(120°) = sqrt(3)/2 */
static const fract16_t FRACT16_COS_240 = -FRACT16_1_DIV_2;      /* cos(240°) = -0.5 */
static const fract16_t FRACT16_SIN_240 = -FRACT16_SQRT3_DIV_2;  /* sin(240°) = -sqrt(3)/2 */

/* Calculation with over saturation */
static const accum32_t FRACT16_1_OVERSAT      = 0x00008000; /* 32768 */
static const accum32_t FRACT16_2_DIV_SQRT3    = 0x000093CD; /* 1.15470053838f */
static const accum32_t FRACT16_SQRT2          = 0x0000B505; /* 1.41421508789f */
static const accum32_t FRACT16_SQRT3          = 0x0000DDB3; /* 1.73202514648f */
static const accum32_t FRACT16_PI             = 0x0001921F;
static const accum32_t FRACT16_3PI_DIV_4      = 0x00012D97;

#define FRACT16_FLOAT_MAX (0.999969482421875F)
#define FRACT16_FLOAT_MIN (-1.0F)
#define FRACT16_EPSILON   (0.000030517578125F)

#define FRACT16(x) ((fract16_t)(((x) < FRACT16_FLOAT_MAX) ? (((x) >= FRACT16_FLOAT_MIN) ? ((x)*32768.0F) : INT16_MIN) : INT16_MAX))

#define ACCUM32(x) ((accum32_t)((x)*32768.0F))

// static inline fract16_t fract16(int16_t value, int32_t max) { return (fract16_t)(((int32_t)value << FRACT16_N_BITS) / max); }
static inline fract16_t fract16_sat(accum32_t value) { return math_clamp(value, -FRACT16_MAX, FRACT16_MAX); }
static inline ufract16_t fract16_sat_positive(accum32_t value) { return math_clamp(value, 0, FRACT16_MAX); }

/*!
    @brief Unsaturated Multiply

    One parameter may be 2x. eliminates saturation at boundaries.
    input max factor1 * factor2 < INT32_MAX

    @param[in] factor [-65536:65535] <=> [-2:2]
    @param[in] frac   [-32768:32767] <=> [-1:1]

    @return int32_t [-65536:65535] <=> [-2:2)
*/
static inline accum32_t fract16_mul(accum32_t factor, accum32_t frac) { return ((factor * frac) >> FRACT16_N_BITS); }

/*!
    Saturate to FRACT16_MIN, FRACT16_MAX

    Parameters as accum32_t, saturation by upper and lower bounds

    case of fract16_mul(int16_t, int16_t) still needs to check for 32768
    fract16_mul(-32768, -32768) => 32768 == 0x8000, over sat 1
    (int16_t)32768 => -32768
    (product == +32768) ? 32767 : product;

    @return int16_t [-32767:32767] <=> (-1:1)
*/
static inline fract16_t fract16_mul_sat(accum32_t factor, accum32_t frac) { return fract16_sat(fract16_mul(factor, frac)); }

/*!
    @brief Unsaturated Divide

    @param[in] dividend [-65536:65535] <=> [-2:2)

    @return accum32_t [-1073741824:1073709056] <=> (-32768.0, 32767.0), dividend >= divisor
    @return fract16_t [-32767:32767] <=> (-1:1), dividend < divisor
*/
static inline accum32_t fract16_div(accum32_t dividend, accum32_t divisor)
{
    assert(dividend < (FRACT16_SCALE * 2)); /* caller must handle divide by zero */
    assert(divisor != 0); /* caller must handle divide by zero */
    return ((dividend << FRACT16_N_BITS) / divisor);
}

/*!
    @return int16_t [-32767:32767]
*/
static inline fract16_t fract16_div_sat(fract16_t dividend, accum32_t divisor) { return fract16_sat(fract16_div(dividend, divisor)); }

/* cast overflow as ufract */
static inline ufract16_t fract16_abs(fract16_t x) { return abs(x); }

static inline fract16_t fract16_abs_sat(fract16_t x) { return (x == INT16_MIN) ? INT16_MAX : abs(x); }

static inline fract16_t fract16_sqrt(fract16_t x) { return fixed_sqrt((int32_t)x << FRACT16_N_BITS); }

/* using intermediate 64-bit multiplication */
/* fract16_mul_wide */
static inline accum32_t accum32_mul(accum32_t a, accum32_t b) { return ((int64_t)a * b) >> FRACT16_N_BITS; }

static inline accum32_t accum32_div(accum32_t a, accum32_t b) { return ((int64_t)a << FRACT16_N_BITS) / b; }



/******************************************************************************/
/*!
    Linear interpolation / remap family

    Canonical forms:
        lerp(a, b, t)              = a + t*(b - a),           t in [0, 1]
        inv_lerp(lo, hi, x)        = (x - lo) / (hi - lo)     (a.k.a. normalize)
        remap(iLo,iHi, oLo,oHi, x) = lerp(oLo, oHi, inv_lerp(iLo, iHi, x))
*/
/******************************************************************************/
/* lerp: a + t*(b - a), t in [0, FRACT16_MAX] representing [0, 1] */
/* [0:FRACT16_MAX] => [a:b] */
static inline int32_t fract16_lerp(int32_t a, int32_t b, ufract16_t t) { return a + ((b - a) * t >> FRACT16_N_BITS); }

/* inv_lerp: (x - lo) / (hi - lo) scaled to [0, FRACT16_MAX] */
/* [lo:hi] => [0:FRACT16_MAX] */
static inline ufract16_t fract16_inv_lerp(int32_t lo, int32_t hi, int32_t x) { return (FRACT16_MAX * (x - lo) / (hi - lo)); }

static inline ufract16_t fract16_normalize(int32_t lo, int32_t hi, int32_t x) { return fract16_inv_lerp(lo, hi, x); }

/* Saturated normalize: input clamped to [lo, hi] -> output in [0, FRACT16_MAX] */
static inline ufract16_t fract16_normalize_sat(int32_t lo, int32_t hi, int32_t x)
{
    if (x >= hi) { return FRACT16_MAX; }
    if (x <= lo) { return 0; }
    return fract16_normalize(lo, hi, x);
}

/******************************************************************************/
/*!
    Interval arithmetic by a scalar. A negative scalar swaps the bounds.
*/
/******************************************************************************/
/* [low:high]·k */
static inline interval_t fract16_interval_scale(interval_t i, accum32_t k) { return interval_of_bounds(fract16_mul(i.low, k), fract16_mul(i.high, k)); }

/*
    [low:high] / d, the x where d·x ∈ [low:high]. The bounds open toward d = 0, saturating at ±FRACT16_MAX.
    Bounds within [fract16_div] range.
*/
static inline interval_t fract16_interval_div(interval_t i, fract16_t d)
{
    accum32_t divisor = (d != 0) ? d : 1;
    return interval_of_bounds(fract16_sat(fract16_div(i.low, divisor)), fract16_sat(fract16_div(i.high, divisor)));
}

/* accum32_mul(54612, 2^30)  */
// static inline interval_t fract16_interval_div_(interval_t i, fract16_t d)
// {
//     accum32_t inv = fract16_div(FRACT16_SCALE, (d != 0) ? d : 1);
//     return interval_of_bounds(fract16_sat(accum32_mul(i.low, inv)), fract16_sat(accum32_mul(i.high, inv)));
// }

/******************************************************************************/
/*!
    fractional functions stay with hal layer.
*/
/******************************************************************************/
/* runtime 32 bit */
/* rad of turn */
static inline accum32_t fract16_arc_unit(angle16_t theta) { return theta * (fract16_t)((int32_t)2 * FRACT16_PI / 8) / ((int32_t)ANGLE16_PER_REVOLUTION / 8); }
/* turns of arc */
/* input up to π radians */
static inline angle16_t fract16_inv_arc(accum32_t arc_radians) { return fract16_mul(arc_radians, ANGLE16_PER_RADIAN); }

/*  */
static inline accum32_t fract16_arc_length(angle16_t theta, fract16_t r) { return fract16_arc_unit(fract16_mul(theta, r)); }


/*  */
extern fract16_t fract16_sin(angle16_t theta);
extern fract16_t fract16_cos(angle16_t theta);
extern angle16_t fract16_atan2(fract16_t y, fract16_t x);

/******************************************************************************/
/*!
    vector xy
*/
/******************************************************************************/
struct fract16_xy { fract16_t x; fract16_t y; };
// struct phasor { fract16_t re; fract16_t im; };
// struct fract16_vec2 { fract16_t vec[2]; } fract16_vec2_t;
extern struct fract16_xy fract16_vector(angle16_t theta);

extern ufract16_t fract16_vector_magnitude(fract16_t x, fract16_t y);
extern ufract16_t fract16_vector_component(fract16_t x, ufract16_t mag_limit);
extern ufract16_t fract16_vector_norm(fract16_t x, fract16_t y, ufract16_t mag_limit);
extern void fract16_vector_limit(fract16_t * p_x, fract16_t * p_y, ufract16_t mag_limit);

#endif


// extern uint16_t fract16_vector_scalar_fast(fract16_t x, fract16_t y, fract16_t mag_limit);
// extern uint16_t fract16_vector_limit_fast(fract16_t * p_x, fract16_t * p_y, fract16_t limit);
// extern int32_t fast_inv_sqrt(int32_t x);
