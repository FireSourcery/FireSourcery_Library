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
    @file   math_foc.h
    @author FireSourcery
    @brief  FOC pure math functions.
            Aligned in CCW order: a, b, c; alpha, beta; d, q
    @version V0
*/
/******************************************************************************/
#ifndef MATH_FOC_H
#define MATH_FOC_H

#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*!
    @brief    Clarke
            Transform 3-phase (120 degree) stationary reference frame quantities: Ia, Ib, Ic
            into 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta

    Ialpha = (2*Ia - Ib - Ic)/3
    Ibeta = sqrt3/3*(Ib - Ic) = (Ib - Ic)/sqrt3

    Alternatively, Simplified:
    Ialpha = Ia
    Ibeta = (Ib - Ic)/sqrt(3)
    alpha = a;
    beta =  fract16_mul(b, FRACT16_1_DIV_SQRT3) -  fract16_mul(c, FRACT16_1_DIV_SQRT3);

    @param[out] p_alpha
    @param[out] p_beta
    @param[in] a
    @param[in] b
    @param[in] c
    @return  void
 */
 /******************************************************************************/
static inline void foc_clarke(fract16_t * p_alpha, fract16_t * p_beta, fract16_t a, fract16_t b, fract16_t c)
{
    int32_t alpha = fract16_mul((int32_t)a * 2 - (int32_t)b - (int32_t)c, FRACT16_1_DIV_3);
    int32_t beta = fract16_mul((int32_t)b - (int32_t)c, FRACT16_1_DIV_SQRT3);

    *p_alpha = fract16_sat(alpha);
    *p_beta = fract16_sat(beta);
}

/******************************************************************************/
/*!
    @brief    2-Phase Version

    Ialpha = Ia
    Ibeta = (Ia + 2*Ib)/sqrt3

    @param[out] p_alpha
    @param[out] p_beta
    @param[in] Ia
    @param[in] Ib
    @return  void
*/
/******************************************************************************/
static inline void foc_clarke_ab(fract16_t * p_alpha, fract16_t * p_beta, fract16_t a, fract16_t b)
{
    int32_t beta = fract16_mul((int32_t)a + (int32_t)b * 2, FRACT16_1_DIV_SQRT3);

    *p_alpha = a;
    *p_beta = fract16_sat(beta);
}

/******************************************************************************/
/*!
    @brief    Inverse Clarke

    A = alpha;
    B = (-alpha + sqrt3*beta)/2
    C = (-alpha - sqrt3*beta)/2
*/
/******************************************************************************/
static inline void foc_invclarke(fract16_t * p_a, fract16_t * p_b, fract16_t * p_c, fract16_t alpha, fract16_t beta)
{
    int32_t alphaDiv2 = 0 - fract16_mul(alpha, FRACT16_1_DIV_2);
    int32_t betaSqrt3Div2 = fract16_mul(beta, FRACT16_SQRT3_DIV_2);

    int32_t b = alphaDiv2 + betaSqrt3Div2;
    int32_t c = alphaDiv2 - betaSqrt3Div2;

    *p_a = alpha;
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

    @param[out] p_d
    @param[out] p_q
    @param[in] Ialpha
    @param[in] Ibeta
    @param[in] theta - rotating frame angle in q1.15 format
    @return void
*/
/******************************************************************************/
/* shared sin cos calc */
static inline void foc_park_vector(fract16_t * p_d, fract16_t * p_q, fract16_t alpha, fract16_t beta, fract16_t sin, fract16_t cos)
{
    int32_t d = fract16_mul(alpha, cos) + fract16_mul(beta, sin);
    int32_t q = fract16_mul(beta, cos) - fract16_mul(alpha, sin);

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

    alpha = -q*sin(theta) + d*cos(theta)
    beta = q*cos(theta) + d*sin(theta)
*/
/******************************************************************************/
static inline void foc_invpark_vector(fract16_t * p_alpha, fract16_t * p_beta, fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t beta = fract16_mul(d, sin) + fract16_mul(q, cos);

    *p_alpha = fract16_sat(alpha);
    *p_beta = fract16_sat(beta);
}

static inline void foc_invpark(fract16_t * p_alpha, fract16_t * p_beta, fract16_t d, fract16_t q, angle16_t theta)
{
    fract16_t cos, sin;

    fract16_vector(&cos, &sin, theta);
    foc_invpark_vector(p_alpha, p_beta, d, q, sin, cos);
}

/*
    limit around d
    mag_limit^2 - d^2 = q^2
*/
static inline void foc_circle_limit(fract16_t * p_d, fract16_t * p_q, fract16_t magnitude_limit, fract16_t d_limit)
{
    uint32_t mag_limit_squared = (int32_t)magnitude_limit * magnitude_limit;
    uint32_t d_squared = (int32_t)(*p_d) * (*p_d);
    uint32_t q_squared = (int32_t)(*p_q) * (*p_q);
    uint32_t d_limit_squared;
    uint32_t q_limit_squared;
    uint16_t q_limit;

    if (d_squared + q_squared > mag_limit_squared)  /* (|Vdq| > magnitude_limit) */
    {
        d_limit_squared = (int32_t)d_limit * d_limit;
        if (d_squared > d_limit_squared) /* (fract16_abs(d) > d_limit) */
        {
            d_squared = d_limit_squared;
            *p_d = (*p_d < 0) ? (0 - d_limit) : d_limit;
        }
        q_limit_squared = mag_limit_squared - d_squared;
        q_limit = q_sqrt(q_limit_squared);
        *p_q = (*p_q < 0) ? (0 - q_limit) : q_limit;
    }
}


#endif
