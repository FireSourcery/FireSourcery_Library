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
    @file   Q.h
    @author FireSourcery
    @brief     Math with 16 bit fractions in Q1.15 format
    @version V0
*/
/******************************************************************************/
#ifndef QFRAC16_H
#define QFRAC16_H

#include "Q.h"
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>
// #include <stdfix.h>

#define QFRAC16_N_BITS (15)   /*!< Q1.15, 15 fractional bits. Scalar 32768. Resolution 1/(2^15) == .000030517578125 */

typedef int16_t qfrac16_t;      /*!< Q1.15 [-1.0, 1) */
typedef uint16_t uqfrac16_t;    /*!< Q1.15 [0, 2) */
typedef int32_t qaccum32_t;     /*!< Q17.15 */

static const qfrac16_t QFRAC16_MAX = INT16_MAX; /*!< (32767) */
static const qfrac16_t QFRAC16_MIN = INT16_MIN; /*!< (-32768) */

static const qfrac16_t QFRAC16_1_DIV_2 = 0x4000; /*!< 16384 */
static const qfrac16_t QFRAC16_1_DIV_4 = 0x2000;
static const qfrac16_t QFRAC16_3_DIV_4 = 0x6000;
static const qfrac16_t QFRAC16_1_DIV_3 = 0x2AAA;
static const qfrac16_t QFRAC16_2_DIV_3 = 0x5555;
static const qfrac16_t QFRAC16_1_DIV_SQRT2 = 0x5A82; /*!< 0.70710678118f */
static const qfrac16_t QFRAC16_1_DIV_SQRT3 = 0x49E7; /*!< 0.57735026919f */
static const qfrac16_t QFRAC16_SQRT3_DIV_2 = 0x6EDA;
static const qfrac16_t QFRAC16_SQRT3_DIV_4 = 0x376D;
static const qfrac16_t QFRAC16_SQRT2_DIV_2 = 0x5A82;
static const qfrac16_t QFRAC16_PI_DIV_4 = 0x6487;

/* Calculation with over saturation */
static const qaccum32_t QFRAC16_1_OVERSAT      = 0x00008000; /*!< (32768) */
static const qaccum32_t QFRAC16_PI             = 0x0001921F; /* Over saturated */
static const qaccum32_t QFRAC16_3PI_DIV_4      = 0x00012D97; /* Over saturated */

#define QFRAC16_FLOAT_MAX (0.999969482421875F)
#define QFRAC16_FLOAT_MIN (-1.0F)
#define QFRAC16_FLOAT(x) ((qfrac16_t)(((x) < QFRAC16_FLOAT_MAX) ? (((x) >= QFRAC16_FLOAT_MIN) ? ((x)*32768.0F) : INT16_MIN) : INT16_MAX))

static inline qfrac16_t qfrac16(int16_t num, int32_t max) { return (qfrac16_t)(((int32_t)num << QFRAC16_N_BITS) / max); }
static inline qfrac16_t qfrac16_sat(int32_t qfrac) { return math_clamp(qfrac, -QFRAC16_MAX, QFRAC16_MAX); }

/*!
    @brief Unsaturated Multiply

    input max factor1 * factor2 < INT32_MAX
    overflow
        e.g. (65,536, 32,768)

    qfrac16_mul(+/-32768, +/-32768) returns 32768 [0x8000]
        (int32_t)32768 -> positive 32768, over saturated 1
        (int16_t)32768 -> -32768, -1
    Call qfrac16_sat

    @param[in]
    @return int32_t[-65536:65535] <=> [-2:2)
*/
static inline qaccum32_t qfrac16_mul(qaccum32_t factor, qaccum32_t frac)
{
    return ((factor * frac) >> QFRAC16_N_BITS);
}

/*!
    Saturate to QFRAC16_MIN, QFRAC16_MAX

    qfrac16_mul(int16_t factor, int16_t frac) must still check for 32768 case
        (product == +32768) ? 32767 : product;

    @return int16_t[-32767, 32767] <=> (-1:1)
*/
static inline qfrac16_t qfrac16_mul_sat(qaccum32_t factor, qaccum32_t frac)
{
    return qfrac16_sat(qfrac16_mul(factor, frac));
}

/*!
    @brief Unsaturated Divide

    dividend >= divisor returns [-1073741824:1073709056] <=> (-32768, 32767)
        over saturated qfrac16_t, 32768 [0x8000] -> over saturated 1
    dividend < divisor returns [-32767:32767] <=> (-1:1)
        within qfrac16_t range

    @param[in] dividend [-65536:65535] <=> [-2:2)
    @return int32_t[-1073741824:1073709056], [0XC0000000, 0X3FFF8000]
*/
static inline int32_t qfrac16_div(qaccum32_t dividend, qaccum32_t divisor)
{
    return ((dividend << QFRAC16_N_BITS) / divisor);
}

/*!
    @return int16_t[-32767, 32767]
*/
static inline qfrac16_t qfrac16_div_sat(qaccum32_t dividend, qaccum32_t divisor)
{
    return qfrac16_sat(qfrac16_div(dividend, divisor));
}

static inline qfrac16_t qfrac16_abs(qfrac16_t x)
{
    return (x < 0) ? ((x == -32768) ? 32767 : 0 - x) : x;
}

static inline qfrac16_t qfrac16_sqrt(qfrac16_t x)
{
    return q_sqrt((int32_t)x << QFRAC16_N_BITS);
}

/******************************************************************************/
/*!
    qangle16
*/
/******************************************************************************/
typedef int16_t qangle16_t;     /*!< [-pi, pi) signed or [0, 2pi) unsigned, angle loops. */

#define QFRAC16_SINE_90_TABLE_LENGTH    (256U)
#define QFRAC16_SINE_90_TABLE_LSB       (6U)    /*!< Least significant bits, shifted away */

static const qangle16_t QANGLE16_0 = 0U;         /*! 0 */
static const qangle16_t QANGLE16_30 = 0x1555U;   /*! 5461 */
static const qangle16_t QANGLE16_60 = 0x2AAAU;   /*! 10922 */
static const qangle16_t QANGLE16_90 = 0x4000U;   /*! 16384 */
static const qangle16_t QANGLE16_120 = 0x5555U;  /*! 21845 */
static const qangle16_t QANGLE16_150 = 0x6AAAU;  /*! 27306 */
static const qangle16_t QANGLE16_180 = 0x8000U;  /*! 32768, -32768, 180 == -180 */
static const qangle16_t QANGLE16_210 = 0x9555U;  /*! 38229 */
static const qangle16_t QANGLE16_240 = 0xAAAAU;  /*! 43690, -21845 */
static const qangle16_t QANGLE16_270 = 0xC000U;  /*! 49152, -16384, 270 == -90 */

#define QANGLE16_QUADRANT_MASK (0xC000U)

typedef enum qangle16_quadrant
{
    QANGLE16_QUADRANT_I,        /* 0_90 */
    QANGLE16_QUADRANT_II,       /* 90_180 */
    QANGLE16_QUADRANT_III,      /* 180_270 */
    QANGLE16_QUADRANT_IV,       /* 270_360 */
}
qangle16_quadrant_t;

/* e.g. Mechanical Angle */
static inline qangle16_quadrant_t qangle16_quadrant(qangle16_t theta)
{
    qangle16_quadrant_t quadrant;
    switch((uint16_t)theta & QANGLE16_QUADRANT_MASK)
    {
        case (const uint16_t)QANGLE16_0:      quadrant = QANGLE16_QUADRANT_I;     break;
        case (const uint16_t)QANGLE16_90:     quadrant = QANGLE16_QUADRANT_II;    break;
        case (const uint16_t)QANGLE16_180:    quadrant = QANGLE16_QUADRANT_III;   break;
        case (const uint16_t)QANGLE16_270:    quadrant = QANGLE16_QUADRANT_IV;    break;
        default: quadrant = -1; break; /* Should not occur */
    }
    return quadrant;
}

// polling freq must be sufficent
static inline bool qangle16_cycle2(qangle16_t theta0, qangle16_t theta1)
{
    // return (((theta0 ^ theta1) & 0x8000U) != (uint16_t)0U);
    return ((theta0 ^ theta1) < 0);
}

static inline bool qangle16_cycle4(qangle16_t theta0, qangle16_t theta1)
{
    return (((theta0 ^ theta1) & QANGLE16_QUADRANT_MASK) != (uint16_t)0U);
}

extern qfrac16_t qfrac16_sin(qangle16_t theta);
extern qfrac16_t qfrac16_cos(qangle16_t theta);
extern void qfrac16_vector(qfrac16_t * p_cos, qfrac16_t * p_sin, qangle16_t theta);
extern uint16_t qfrac16_vector_magnitude(qfrac16_t x, qfrac16_t y);
extern uint16_t qfrac16_vector_limit(qfrac16_t * p_x, qfrac16_t * p_y, qfrac16_t magnitudeMax);
extern qangle16_t qfrac16_atan2(qfrac16_t y, qfrac16_t x);

#endif
