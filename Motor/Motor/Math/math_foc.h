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
    @file   math_foc.h
    @author FireSourcery
    @brief  FOC pure math functions.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*!
    @brief
    Aligned in CCW order: a, b, c; alpha, beta; d, q
    d-axis aligned to 0° / a-phase
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief  Clarke
    Transform 3-phase (120 degree) stationary reference frame quantities: Ia, Ib, Ic
    into 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta

    Ialpha = (2*Ia - Ib - Ic)/3
    Ibeta = sqrt3/3*(Ib - Ic) = (Ib - Ic)/sqrt3

    @param[out] p_alpha, p_beta
    @param[in]  a, b, c
 */
 /******************************************************************************/
// static inline void foc_clarke(accum32_t * p_alpha, accum32_t * p_beta, fract16_t a, fract16_t b, fract16_t c)
static inline void foc_clarke(fract16_t * p_alpha, fract16_t * p_beta, fract16_t a, fract16_t b, fract16_t c)
{
    int32_t alpha = fract16_mul((int32_t)a * 2 - (int32_t)b - (int32_t)c, FRACT16_1_DIV_3);
    int32_t beta = fract16_mul((int32_t)b - (int32_t)c, FRACT16_1_DIV_SQRT3);

    *p_alpha = fract16_sat(alpha);
    *p_beta = fract16_sat(beta);
}

/*
    Ialpha = Ia
    Ibeta = (Ib - Ic)/sqrt(3)
*/
static inline void foc_clarke_align(fract16_t * p_alpha, fract16_t * p_beta, fract16_t a, fract16_t b, fract16_t c)
{
    int32_t beta = fract16_mul((int32_t)b - (int32_t)c, FRACT16_1_DIV_SQRT3);

    *p_alpha = a;
    *p_beta = fract16_sat(beta);
}

/*!
    @brief  2-Phase Version

    Ialpha = Ia
    Ibeta = (Ia + 2*Ib)/sqrt3
*/
static inline void foc_clarke_ab(fract16_t * p_alpha, fract16_t * p_beta, fract16_t a, fract16_t b)
{
    int32_t beta = fract16_mul((int32_t)a + (int32_t)b * 2, FRACT16_1_DIV_SQRT3);

    *p_alpha = a;
    *p_beta = fract16_sat(beta);
}

/******************************************************************************/
/*!
    @brief  Inverse Clarke

    a = alpha
    b = (-alpha + sqrt3*beta)/2
    c = (-alpha - sqrt3*beta)/2
*/
/******************************************************************************/
static inline void foc_inv_clarke(fract16_t * p_a, fract16_t * p_b, fract16_t * p_c, fract16_t alpha, fract16_t beta)
{
    int32_t alpha_div2 = fract16_mul(alpha, FRACT16_1_DIV_2);
    int32_t beta_sqrt3_div2 = fract16_mul(beta, FRACT16_SQRT3_DIV_2);

    int32_t b = -alpha_div2 + beta_sqrt3_div2;
    int32_t c = -alpha_div2 - beta_sqrt3_div2;

    *p_a = fract16_sat(alpha);
    *p_b = fract16_sat(b);
    *p_c = fract16_sat(c);
}

/******************************************************************************/
/*!
    @brief  Park
    Transforms 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta
    into 2-axis orthogonal rotor synchronous reference frame quantities: Id and Iq.

    Id = alpha*cos(theta) + beta*sin(theta)
    Iq = -alpha*sin(theta) + beta*cos(theta)

    @param[out] p_d, p_q
    @param[in]  Ialpha, Ibeta
    @param[in]  theta - rotating frame angle in q1.15 format
*/
/******************************************************************************/
static inline void foc_park_vector(fract16_t * p_d, fract16_t * p_q, fract16_t alpha, fract16_t beta, fract16_t sin, fract16_t cos)
{
    int32_t d = fract16_mul(alpha, cos) + fract16_mul(beta, sin);
    int32_t q = -fract16_mul(alpha, sin) + fract16_mul(beta, cos);

    *p_d = fract16_sat(d);
    *p_q = fract16_sat(q);
}

static inline void foc_park(fract16_t * p_d, fract16_t * p_q, fract16_t alpha, fract16_t beta, angle16_t theta)
{
    fract16_t cos, sin;

    fract16_vector(&cos, &sin, theta);
    foc_park_vector(p_d, p_q, alpha, beta, sin, cos);
}

/******************************************************************************/
/*!
    @brief  Inverse Park

    alpha = d*cos(theta) - q*sin(theta)
    beta = d*sin(theta) + q*cos(theta)
*/
/******************************************************************************/
static inline void foc_inv_park_vector(fract16_t * p_alpha, fract16_t * p_beta, fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t beta = fract16_mul(d, sin) + fract16_mul(q, cos);

    *p_alpha = fract16_sat(alpha);
    *p_beta = fract16_sat(beta);
}

static inline void foc_inv_park(fract16_t * p_alpha, fract16_t * p_beta, fract16_t d, fract16_t q, angle16_t theta)
{
    fract16_t cos, sin;

    fract16_vector(&cos, &sin, theta);
    foc_inv_park_vector(p_alpha, p_beta, d, q, sin, cos);
}

/******************************************************************************/
/*!
    @brief  Clarke-Park Transformation
    inlined for less redundant saturation
*/
/******************************************************************************/
static inline void foc_clarke_park(fract16_t * p_d, fract16_t * p_q, fract16_t a, fract16_t b, fract16_t c, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul((int32_t)a * 2 - (int32_t)b - (int32_t)c, FRACT16_1_DIV_3);
    int32_t beta = fract16_mul((int32_t)b - (int32_t)c, FRACT16_1_DIV_SQRT3);

    int32_t d = fract16_mul(alpha, cos) + fract16_mul(beta, sin);
    int32_t q = -fract16_mul(alpha, sin) + fract16_mul(beta, cos);

    *p_d = fract16_sat(d);
    *p_q = fract16_sat(q);
}

static inline void foc_inv_clarke_park(fract16_t * p_a, fract16_t * p_b, fract16_t * p_c, fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t beta = fract16_mul(d, sin) + fract16_mul(q, cos);

    int32_t alpha_div2 = fract16_mul(alpha, FRACT16_1_DIV_2);
    int32_t beta_sqrt3_div2 = fract16_mul(beta, FRACT16_SQRT3_DIV_2);

    int32_t b = -alpha_div2 + beta_sqrt3_div2;
    int32_t c = -alpha_div2 - beta_sqrt3_div2;

    *p_a = fract16_sat(alpha);
    *p_b = fract16_sat(b);
    *p_c = fract16_sat(c);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    limit around d
    mag_limit^2 - d^2 = q^2
*/
/* vector_limit_clamp */
/* applies vd limit on magnitude over limit only */
static inline bool foc_circle_limit(fract16_t * p_d, fract16_t * p_q, ufract16_t magnitude_limit, ufract16_t d_limit)
{
    uint32_t mag_limit_squared = (int32_t)magnitude_limit * magnitude_limit;
    uint32_t d_squared = (int32_t)(*p_d) * (*p_d);
    uint32_t q_squared = (int32_t)(*p_q) * (*p_q);
    uint32_t d_limit_squared;
    uint16_t q_limit;
    bool is_limited = false;

    if (d_squared + q_squared > mag_limit_squared)  /* |Vdq| > magnitude_limit */
    {
        /* Apply d limit */
        d_limit_squared = (uint32_t)d_limit * d_limit;
        /* abs(d) > d_limit */
        if (d_squared > d_limit_squared)
        {
            *p_d = (*p_d < 0) ? (0 - d_limit) : d_limit;
            d_squared = d_limit_squared;
        }

        /* Apply q limit */
        q_limit = fixed_sqrt(mag_limit_squared - d_squared);
        *p_q = (*p_q < 0) ? (0 - q_limit) : q_limit;
        is_limited = true;
    }

    return is_limited;
}

/* optionally caller handle d */
// static inline bool foc_circle_limit_d(fract16_t * p_d, ufract16_t d_limit)
// {
//     bool is_limited = false;

//     if (abs(*p_d) > d_limit)
//     {
//         *p_d = (*p_d < 0) ? (0 - d_limit) : d_limit;
//         is_limited = true;
//     }

//     return is_limited;
// }

/*  */
static inline bool foc_circle_limit_q(fract16_t * p_d, fract16_t * p_q, ufract16_t magnitude_limit)
{
    uint32_t mag_limit_squared = (int32_t)magnitude_limit * magnitude_limit;
    uint32_t d_squared = (int32_t)(*p_d) * (*p_d);
    uint32_t q_squared = (int32_t)(*p_q) * (*p_q);
    uint16_t q_limit;
    bool is_limited = false;

    if (d_squared + q_squared > mag_limit_squared)  /* |Vdq| > magnitude_limit */
    {
        q_limit = fixed_sqrt(mag_limit_squared - d_squared);
        *p_q = (*p_q < 0) ? (0 - q_limit) : q_limit;
        is_limited = true;
    }

    return is_limited;
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/*!
    Direct DQ0 Transform - converts 3-phase (a, b, c) directly to rotor reference frame (d, q, 0)
    Combines Clarke and Park transforms in one operation

    @param[out] p_d - Direct axis component
    @param[out] p_q - Quadrature axis component
    @param[out] p_0 - Zero sequence component (optional, can be NULL)
    @param[in] a, b, c - Three phase values
    @param[in] sine, cosine - Rotor angle sin/cos values
*/
/*
    DQ0 transformation matrix multiplication:
    [d]         [cos(θ)     cos(θ-2π/3)  cos(θ+2π/3)] [a]
    [q] = 2/3 * [-sin(θ)   -sin(θ-2π/3) -sin(θ+2π/3)] [b]
    [0]         [1/2       1/2          1/2         ] [c]
*/
static inline void foc_abc_dq_transform
(
    fract16_t * p_d, fract16_t * p_q,
    fract16_t a, fract16_t b, fract16_t c,
    fract16_t cos, fract16_t sin,
    fract16_t cos_120, fract16_t sin_120,
    fract16_t cos_240, fract16_t sin_240
)
{
    int32_t d = fract16_mul(a, cos) + fract16_mul(b, cos_120) + fract16_mul(c, cos_240);
    int32_t q = -fract16_mul(a, sin) - fract16_mul(b, sin_120) - fract16_mul(c, sin_240);

    *p_d = fract16_sat(fract16_mul(d, FRACT16_2_DIV_3));
    *p_q = fract16_sat(fract16_mul(q, FRACT16_2_DIV_3));
}

static inline void foc_abc_dq0_transform(fract16_t * p_d, fract16_t * p_q, fract16_t * p_0, fract16_t a, fract16_t b, fract16_t c, fract16_t sine, fract16_t cosine)
{
    /* Pre-calculate angle shifts for 120° and 240° */
    fract16_t cos120 = fract16_mul(cosine, FRACT16_COS_120) - fract16_mul(sine, FRACT16_SIN_120);  /* cos(θ-120°) */
    fract16_t sin120 = fract16_mul(sine, FRACT16_COS_120) + fract16_mul(cosine, FRACT16_SIN_120);  /* sin(θ-120°) */
    fract16_t cos240 = fract16_mul(cosine, FRACT16_COS_240) - fract16_mul(sine, FRACT16_SIN_240);  /* cos(θ+120°) */
    fract16_t sin240 = fract16_mul(sine, FRACT16_COS_240) + fract16_mul(cosine, FRACT16_SIN_240);  /* sin(θ+120°) */

    /* Calculate d-axis component */
    /* Calculate q-axis component (negative sine for proper orientation) */
    accum32_t d = fract16_mul(a, cosine) + fract16_mul(b, cos120) + fract16_mul(c, cos240);
    accum32_t q = -fract16_mul(a, sine) - fract16_mul(b, sin120) - fract16_mul(c, sin240);

    *p_d = fract16_mul(d, FRACT16_2_DIV_3);
    *p_q = fract16_mul(q, FRACT16_2_DIV_3);

    /* Calculate zero sequence component if requested */
    if (p_0 != NULL)
    {
        accum32_t zero = (accum32_t)a + (accum32_t)b + (accum32_t)c;
        *p_0 = fract16_mul(zero, FRACT16_1_DIV_3);
    }
}



/*!
    Inverse DQ0 Transform - converts rotor reference frame (d, q, 0) directly to 3-phase (a, b, c)
    Combines inverse Park and inverse Clarke transforms in one operation

    @param[out] p_a, p_b, p_c - Three phase output values
    @param[in] d, q - Direct and quadrature axis components
    @param[in] zero - Zero sequence component (typically 0 for balanced systems)
    @param[in] sine, cosine - Rotor angle sin/cos values
*/
/*
    Inverse DQ0 transformation matrix multiplication:
    [a]   [cos(θ)      -sin(θ)      1] [d]
    [b] = [cos(θ-2π/3) -sin(θ-2π/3) 1] [q]
    [c]   [cos(θ+2π/3) -sin(θ+2π/3) 1] [0]
*/
static inline void foc_inv_abc_dq_transform
(
    fract16_t * p_a, fract16_t * p_b, fract16_t * p_c,
    fract16_t d, fract16_t q,
    fract16_t cos, fract16_t sin,
    fract16_t cos_120, fract16_t sin_120,
    fract16_t cos_240, fract16_t sin_240
)
{
    int32_t a = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t b = fract16_mul(d, cos_120) - fract16_mul(q, sin_120);
    int32_t c = fract16_mul(d, cos_240) - fract16_mul(q, sin_240);

    *p_a = fract16_sat(a);
    *p_b = fract16_sat(b);
    *p_c = fract16_sat(c);
}

static inline void foc_inv_abc_dq0_transform(fract16_t * p_a, fract16_t * p_b, fract16_t * p_c, fract16_t d, fract16_t q, fract16_t zero, fract16_t sine, fract16_t cosine)
{
    /* Pre-calculate angle shifts for 120° and 240° */
    fract16_t cos120 = fract16_mul(cosine, FRACT16_COS_120) - fract16_mul(sine, FRACT16_SIN_120);  /* cos(θ-120°) */
    fract16_t sin120 = fract16_mul(sine, FRACT16_COS_120) + fract16_mul(cosine, FRACT16_SIN_120);  /* sin(θ-120°) */
    fract16_t cos240 = fract16_mul(cosine, FRACT16_COS_240) - fract16_mul(sine, FRACT16_SIN_240);  /* cos(θ+120°) */
    fract16_t sin240 = fract16_mul(sine, FRACT16_COS_240) + fract16_mul(cosine, FRACT16_SIN_240);  /* sin(θ+120°) */

    /* Calculate three-phase components */
    *p_a = fract16_mul(d, cosine) - fract16_mul(q, sine) + zero;
    *p_b = fract16_mul(d, cos120) - fract16_mul(q, sin120) + zero;
    *p_c = fract16_mul(d, cos240) - fract16_mul(q, sin240) + zero;
}

// static inline fract16_t foc_d_of_abc(fract16_t a, fract16_t b, fract16_t c, fract16_t cos, fract16_t cos_120, fract16_t cos_240)
// {
//     return fract16_mul(a, cos) + fract16_mul(b, cos_120) + fract16_mul(c, cos_240);
// }

// static inline fract16_t foc_q_of_abc(fract16_t a, fract16_t b, fract16_t c, fract16_t sin, fract16_t sin_120, fract16_t sin_240)
// {
//     return -fract16_mul(a, sin) - fract16_mul(b, sin_120) - fract16_mul(c, sin_240);
// }


/******************************************************************************/
/*!
    optimized versions with precomputed sin/cos for 120 and 240 degree shifts
*/
/******************************************************************************/
static inline void foc_abc_dq_angle
(
    fract16_t * p_cos, fract16_t * p_sin,
    fract16_t * p_cos_120, fract16_t * p_sin_120, /* (θ-120°) */
    fract16_t * p_cos_240, fract16_t * p_sin_240, /* (θ+120°) */
    angle16_t theta
)
{
    *p_cos = fract16_cos(theta);
    *p_sin = fract16_sin(theta);
    *p_cos_120 = fract16_mul(*p_cos, FRACT16_COS_120) - fract16_mul(*p_sin, FRACT16_SIN_120);  /* cos(θ-120°) */
    *p_sin_120 = fract16_mul(*p_sin, FRACT16_COS_120) + fract16_mul(*p_cos, FRACT16_SIN_120);  /* sin(θ-120°) */
    *p_cos_240 = *p_cos_120;
    *p_sin_240 = -*p_sin_120;
}


