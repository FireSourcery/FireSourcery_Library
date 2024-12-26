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
    @version V0
    @brief  Math with 16 bit fractions in Q1.15 format
*/
/******************************************************************************/
#ifndef FRACT16_H
#define FRACT16_H

#include "fixed.h"
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>
// #include <stdfix.h>

#define FRACT16_N_BITS (15)   /*!< Q1.15, 15 fractional bits. Scalar 32768. Resolution 1/(2^15) == .000030517578125 */

typedef int16_t fract16_t;      /*!< Q1.15 [-1.0, 1) */
typedef uint16_t ufract16_t;    /*!< Q1.15 [0, 2) */
typedef int32_t accum32_t;      /*!< Q17.15 */

static const fract16_t FRACT16_MAX = INT16_MAX; /*!< (32767) */
static const fract16_t FRACT16_MIN = INT16_MIN; /*!< (-32768) */

static const fract16_t FRACT16_1_DIV_2 = 0x4000; /*!< 16384 */
static const fract16_t FRACT16_1_DIV_4 = 0x2000;
static const fract16_t FRACT16_3_DIV_4 = 0x6000;
static const fract16_t FRACT16_1_DIV_3 = 0x2AAA;
static const fract16_t FRACT16_2_DIV_3 = 0x5555;
static const fract16_t FRACT16_1_DIV_SQRT2 = 0x5A82; /*!< 0.70710678118f */
static const fract16_t FRACT16_1_DIV_SQRT3 = 0x49E7; /*!< 0.57735026919f */
static const fract16_t FRACT16_SQRT3_DIV_2 = 0x6EDA;
static const fract16_t FRACT16_SQRT3_DIV_4 = 0x376D;
static const fract16_t FRACT16_SQRT2_DIV_2 = 0x5A82;
static const fract16_t FRACT16_PI_DIV_4 = 0x6487;

/* Calculation with over saturation */
static const accum32_t FRACT16_1_OVERSAT      = 0x00008000; /*!< (32768) */
static const accum32_t FRACT16_PI             = 0x0001921F; /* Over saturated */
static const accum32_t FRACT16_3PI_DIV_4      = 0x00012D97; /* Over saturated */

#define FRACT16_FLOAT_MAX (0.999969482421875F)
#define FRACT16_FLOAT_MIN (-1.0F)
#define FRACT16_FLOAT(x) ((fract16_t)(((x) < FRACT16_FLOAT_MAX) ? (((x) >= FRACT16_FLOAT_MIN) ? ((x)*32768.0F) : INT16_MIN) : INT16_MAX))

static inline fract16_t fract16(int16_t numerator, int32_t denominator) { return (fract16_t)(((int32_t)numerator << FRACT16_N_BITS) / denominator); }
static inline fract16_t fract16_sat(accum32_t value)    { return math_clamp(value, -FRACT16_MAX, FRACT16_MAX); }
static inline ufract16_t ufract16_sat(accum32_t value)  { return math_clamp(value, 0, FRACT16_MAX); } /* Although range of [0:2). Refer to 1 as saturated */

/*!
    @brief Unsaturated Multiply

    input max factor1 * factor2 < INT32_MAX
    overflow
        e.g. (65,536, 32,768)

    fract16_mul(+/-32768, +/-32768) returns 32768 [0x8000]
        (int32_t)32768 -> positive 32768, over saturated 1
        (int16_t)32768 -> -32768, -1

    @param[in]
    @return int32_t[-65536:65535] <=> [-2:2)
*/
static inline accum32_t fract16_mul(accum32_t factor, accum32_t frac)
{
    return ((factor * frac) >> FRACT16_N_BITS);
}

/*!
    Saturate to FRACT16_MIN, FRACT16_MAX

    fract16_mul(int16_t factor, int16_t frac) must still check for 32768 case
        (product == +32768) ? 32767 : product;

    @return int16_t[-32767, 32767] <=> (-1:1)
*/
static inline fract16_t fract16_mul_sat(accum32_t factor, accum32_t frac)
{
    return fract16_sat(fract16_mul(factor, frac));
}

/*!
    @brief Unsaturated Divide

    dividend >= divisor returns [-1073741824:1073709056] <=> (-32768, 32767)
        over saturated fract16_t, 32768 [0x8000] -> over saturated 1
    dividend < divisor returns [-32767:32767] <=> (-1:1)
        within fract16_t range

    @param[in] dividend [-65536:65535] <=> [-2:2)
    @return int32_t[-1073741824:1073709056], [0XC0000000, 0X3FFF8000]
*/
static inline accum32_t fract16_div(accum32_t dividend, accum32_t divisor)
{
    return ((dividend << FRACT16_N_BITS) / divisor);
}

/*!
    @return int16_t[-32767, 32767]
*/
static inline fract16_t fract16_div_sat(accum32_t dividend, accum32_t divisor)
{
    return fract16_sat(fract16_div(dividend, divisor));
}

/* cast overflow as ufract */
static inline ufract16_t fract16_abs(fract16_t x)
{
    int32_t mask = (x >> FRACT16_N_BITS); // Create a mask based on the sign bit
    return (x + mask) ^ mask; // Apply the mask to get the absolute value
}

static inline fract16_t fract16_abs_sat(fract16_t x)
{
    return (x < 0) ? ((x == -32768) ? 32767 : 0 - x) : x;
}

static inline fract16_t fract16_sqrt(fract16_t x)
{
    return fixed_sqrt((int32_t)x << FRACT16_N_BITS);
}


/******************************************************************************/
/*!
    angle16
*/
/******************************************************************************/
typedef int16_t angle16_t;     /*!< [-pi, pi) signed or [0, 2pi) unsigned, angle loops. */

#define FRACT16_SINE_90_TABLE_LENGTH    (256U)
#define FRACT16_SINE_90_TABLE_LSB       (6U)    /*!< Least significant bits, shifted away */

static const angle16_t ANGLE16_0 = 0U;         /*! 0 */
static const angle16_t ANGLE16_30 = 0x1555U;   /*! 5461 */
static const angle16_t ANGLE16_60 = 0x2AAAU;   /*! 10922 */
static const angle16_t ANGLE16_90 = 0x4000U;   /*! 16384 */
static const angle16_t ANGLE16_120 = 0x5555U;  /*! 21845 */
static const angle16_t ANGLE16_150 = 0x6AAAU;  /*! 27306 */
static const angle16_t ANGLE16_180 = 0x8000U;  /*! 32768, -32768, 180 == -180 */
static const angle16_t ANGLE16_210 = 0x9555U;  /*! 38229 */
static const angle16_t ANGLE16_240 = 0xAAAAU;  /*! 43690, -21845 */
static const angle16_t ANGLE16_270 = 0xC000U;  /*! 49152, -16384, 270 == -90 */

#define ANGLE16_QUADRANT_MASK (0xC000U)

typedef enum angle16_quadrant
{
    ANGLE16_QUADRANT_I = 0U,           /* 0_90 */
    ANGLE16_QUADRANT_II = 0x4000U,     /* 90_180 */
    ANGLE16_QUADRANT_III = 0x8000U,    /* 180_270 */
    ANGLE16_QUADRANT_IV = 0xC000U,     /* 270_360 */
}
angle16_quadrant_t;

/* e.g. Mechanical Angle */
static inline angle16_quadrant_t angle16_quadrant(angle16_t theta)
{
    return (angle16_quadrant_t)((uint16_t)theta & ANGLE16_QUADRANT_MASK);
}

/* polling freq must be sufficient */
static inline bool angle16_cycle2(angle16_t theta0, angle16_t theta1)
{
    return ((theta0 ^ theta1) < 0); /* return (((theta0 ^ theta1) & 0x8000U) != (uint16_t)0U); */
}

static inline bool angle16_cycle4(angle16_t theta0, angle16_t theta1)
{
    return (((theta0 ^ theta1) & ANGLE16_QUADRANT_MASK) != (uint16_t)0U);
}

extern fract16_t fract16_sin(angle16_t theta);
extern fract16_t fract16_cos(angle16_t theta);
extern angle16_t fract16_atan2(fract16_t y, fract16_t x);

/******************************************************************************/
/*!
    vector16
*/
/******************************************************************************/
// typedef struct vector32 { fract16_t x; fract16_t y; } vector32_t;
extern void fract16_vector(fract16_t * p_x, fract16_t * p_y, angle16_t theta);
extern uint16_t fract16_vector_magnitude(fract16_t x, fract16_t y);
extern uint16_t fract16_vector_scalar(fract16_t x, fract16_t y, fract16_t mag_limit);
extern uint16_t fract16_vector_limit(fract16_t * p_x, fract16_t * p_y, fract16_t magnitudeMax);

// extern uint16_t fract16_vector_scalar_fast(fract16_t x, fract16_t y, fract16_t mag_limit);
// extern uint16_t fract16_vector_limit_fast(fract16_t * p_x, fract16_t * p_y, fract16_t limit);
// extern int32_t fast_inv_sqrt(int32_t x);

#endif
