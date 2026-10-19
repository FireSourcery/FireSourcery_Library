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
    @file   foc_dq0_math.h
    @author FireSourcery
    @brief  Direct dq0 transform.
*/
/******************************************************************************/
#include "foc_math.h"


/******************************************************************************/
/*!
    @brief Reference model of the known form;
    foc_clarke_park / foc_inv_clarke_park are the lower-cost factorization of the same map.
*/
/******************************************************************************/
struct foc_dq0 { int32_t d, q, zero; };
// struct foc_sincos120 { fract16_t cos, sin, cos120, sin_120; };

/*!
    Direct DQ0 Transform - converts 3-phase (a, b, c) directly to rotor reference frame (d, q, 0)
    Combines Clarke and Park transforms in one operation

    @param[in] a, b, c - Three phase values
    @param[in] sin, cos - Rotor angle sin/cos values
*/
/*
    DQ0 transformation matrix multiplication:
    [d]         [cos(θ)     cos(θ-2π/3)  cos(θ+2π/3)] [a]
    [q] = 2/3 * [-sin(θ)   -sin(θ-2π/3) -sin(θ+2π/3)] [b]
    [0]         [1/2       1/2          1/2         ] [c]
*/
static inline struct foc_dq0 _foc_dq0_transform(fract16_t a, fract16_t b, fract16_t c, fract16_t cos, fract16_t sin, accum32_t cos120, accum32_t sin120, accum32_t cos240, accum32_t sin240)
{
    int32_t d = fract16_mul(a, cos) + fract16_mul(b, cos120) + fract16_mul(c, cos240);     /* Calculate d-axis component */
    int32_t q = -fract16_mul(a, sin) - fract16_mul(b, sin120) - fract16_mul(c, sin240);    /* Calculate q-axis component (negative sin for proper orientation) */
    accum32_t zero = (accum32_t)a + (accum32_t)b + (accum32_t)c;     /* Calculate zero sequence component */

    return (struct foc_dq0)
    {
        .d = fract16_mul(d, FRACT16_2_DIV_3),
        .q = fract16_mul(q, FRACT16_2_DIV_3),
        .zero = fract16_mul(zero, FRACT16_1_DIV_3)
    };
}

/*
    θ+120° by cos(θ) + cos(θ-120°) + cos(θ+120°) = 0, likewise sin.
    Rotated values stay widened, rounding reaches ±(1.0 + 1 LSB).
*/
static inline struct foc_dq0 foc_dq0_transform(fract16_t a, fract16_t b, fract16_t c, fract16_t sin, fract16_t cos)
{
    accum32_t cos120 = fract16_mul(cos, FRACT16_COS_120) + fract16_mul(sin, FRACT16_SIN_120);  /* cos(θ-120°) */
    accum32_t sin120 = fract16_mul(sin, FRACT16_COS_120) - fract16_mul(cos, FRACT16_SIN_120);  /* sin(θ-120°) */
    accum32_t cos240 = -cos - cos120;  /* cos(θ+120°) */
    accum32_t sin240 = -sin - sin120;  /* sin(θ+120°) */

    return _foc_dq0_transform(a, b, c, cos, sin, cos120, sin120, cos240, sin240);
}


/*!
    Inverse DQ0 Transform - converts rotor reference frame (d, q, 0) directly to 3-phase (a, b, c)
    Combines inverse Park and inverse Clarke transforms in one operation

    @param[in] d, q - Direct and quadrature axis components
    @param[in] zero - Zero sequence component (typically 0 for balanced systems)
    @param[in] sin, cos - Rotor angle sin/cos values
*/
/*
    Inverse DQ0 transformation matrix multiplication:
    [a]   [cos(θ)      -sin(θ)      1] [d]
    [b] = [cos(θ-2π/3) -sin(θ-2π/3) 1] [q]
    [c]   [cos(θ+2π/3) -sin(θ+2π/3) 1] [0]
*/
static inline struct foc_abc _foc_inv_dq0_transform(fract16_t d, fract16_t q, fract16_t zero, fract16_t cos, fract16_t sin, accum32_t cos120, accum32_t sin120, accum32_t cos240, accum32_t sin240)
{
    return (struct foc_abc)
    {
        .a = fract16_mul(d, cos) - fract16_mul(q, sin) + zero,
        .b = fract16_mul(d, cos120) - fract16_mul(q, sin120) + zero,
        .c = fract16_mul(d, cos240) - fract16_mul(q, sin240) + zero,
    };
}

static inline struct foc_abc foc_inv_dq0_transform(fract16_t d, fract16_t q, fract16_t zero, fract16_t sin, fract16_t cos)
{
    accum32_t cos120 = fract16_mul(cos, FRACT16_COS_120) + fract16_mul(sin, FRACT16_SIN_120);  /* cos(θ-120°) */
    accum32_t sin120 = fract16_mul(sin, FRACT16_COS_120) - fract16_mul(cos, FRACT16_SIN_120);  /* sin(θ-120°) */
    accum32_t cos240 = -cos - cos120;  /* cos(θ+120°) */
    accum32_t sin240 = -sin - sin120;  /* sin(θ+120°) */

    return _foc_inv_dq0_transform(d, q, zero, cos, sin, cos120, sin120, cos240, sin240);
}
