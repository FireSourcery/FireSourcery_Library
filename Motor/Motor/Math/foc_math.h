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
    @brief  Clarke-Park Transformation
    inlined for less redundant saturation
*/
/******************************************************************************/
/* module returns */
struct foc_abc { fract16_t a, b, c; };
struct foc_dq { fract16_t d, q; };
struct foc_alphabeta { fract16_t alpha, beta; };

static inline struct foc_dq foc_clarke_park(fract16_t a, fract16_t b, fract16_t c, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul((int32_t)a * 2 - b - c, FRACT16_1_DIV_3);
    int32_t beta = fract16_mul((int32_t)b - c, FRACT16_1_DIV_SQRT3);

    int32_t d = fract16_mul(alpha, cos) + fract16_mul(beta, sin);
    int32_t q = -fract16_mul(alpha, sin) + fract16_mul(beta, cos);

    return (struct foc_dq) { .d = fract16_sat(d), .q = fract16_sat(q) };
}

static inline struct foc_abc foc_inv_clarke_park(fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t beta = fract16_mul(d, sin) + fract16_mul(q, cos);

    int32_t alpha_div2 = fract16_mul(alpha, FRACT16_1_DIV_2);
    int32_t beta_sqrt3_div2 = fract16_mul(beta, FRACT16_SQRT3_DIV_2);

    int32_t b = -alpha_div2 + beta_sqrt3_div2;
    int32_t c = -alpha_div2 - beta_sqrt3_div2;

    return (struct foc_abc) { .a = fract16_sat(alpha), .b = fract16_sat(b), .c = fract16_sat(c) };
}

/******************************************************************************/
/*!
    @brief  Cross-coupling decoupling feedforward for PMSM.

    Cancels the speed-induced cross terms in the dq voltage equations:
        vd = R·id + Ld·d(id)/dt − ω·Lq·iq
        vq = R·iq + Lq·d(iq)/dt + ω·Ld·id + ω·ψf

    Feedforward to add to the PI output (before circle limit):
        Vd_ff = −ω·Lq·iq          → vd_PI = R·id + Ld·d(id)/dt
        Vq_ff = +ω·Ld·id + ω·ψf   → vq_PI = R·iq + Lq·d(iq)/dt

    Caller precomputes the products as fract16 in the same per-unit basis as Vd/Vq:
        omega_Ld  = omega_e * Ld
        omega_Lq  = omega_e * Lq
        omega_psi = omega_e * psi_f
    Sign of omega_* follows electrical rotation (CCW positive).
*/
/******************************************************************************/
static inline accum32_t foc_vd_ff(accum32_t omega_Lq, fract16_t iq) { return -fract16_mul(omega_Lq, iq); }
static inline accum32_t foc_vq_ff(accum32_t omega_Ld, accum32_t omega_psi, fract16_t id) { return fract16_mul(omega_Ld, id) + omega_psi; }

static inline accum32_t foc_vd_ff_wide(accum32_t omega_Lq, fract16_t iq) { return -accum32_mul(omega_Lq, iq); }
static inline accum32_t foc_vq_ff_wide(accum32_t omega_Ld, accum32_t omega_psi, fract16_t id) { return accum32_mul(omega_Ld, id) + omega_psi; }

// static inline accum32_t _foc_vd_ff_direct(accum32_t L_pu, fract16_t omega, fract16_t iq) { return -((int64_t)omega * L_pu * iq) / ((int64_t)FRACT16_SCALE * FRACT16_SCALE); }
// static inline accum32_t _foc_vq_ff_direct(accum32_t Ld_pu, accum32_t psi_pu, fract16_t omega, fract16_t id) { return ((int64_t)omega * Ld_pu * id) / ((int64_t)FRACT16_SCALE * FRACT16_SCALE) + ((int64_t)omega * psi_pu) / FRACT16_SCALE; }


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    limit around d
    mag_limit^2 - d^2 = q^2
*/
// struct foc_dq_limit { fract16_t d, q; bool is_limited; };
static inline struct foc_dq foc_circle_limit(fract16_t d, fract16_t q, ufract16_t magnitude_limit)
{
    uint32_t mag_limit_squared = (int32_t)magnitude_limit * magnitude_limit;
    uint32_t d_squared = (int32_t)d * d;
    uint32_t q_squared = (int32_t)q * q;

    /* abs(d) > d_limit */
    if (d_squared > mag_limit_squared) { return (struct foc_dq) { .d = math_sign(d) * magnitude_limit, .q = 0 }; }
    /* |Vdq| > magnitude_limit */
    if (d_squared + q_squared > mag_limit_squared) { return (struct foc_dq) { .d = d, .q = math_sign(q) * fixed_sqrt(mag_limit_squared - d_squared) }; }
    return (struct foc_dq) { .d = d, .q = q };
}

static inline struct foc_dq foc_circle_limit_ff(int32_t d, int32_t q, ufract16_t magnitude_limit)
{
    if (d > (int32_t)magnitude_limit) return (struct foc_dq) { .d = (fract16_t)magnitude_limit, .q = 0 };
    if (d < -(int32_t)magnitude_limit) return (struct foc_dq) { .d = -(fract16_t)magnitude_limit, .q = 0 };

    /* d now fits int16 and |d| ≤ magnitude_limit, so d*d ≤ magnitude_limit² fits uint32. */
    uint32_t q_max = fixed_sqrt((uint32_t)magnitude_limit * magnitude_limit - (d * d));

    if (q > (int32_t)q_max) return (struct foc_dq) { .d = (fract16_t)d, .q = (fract16_t)q_max };
    if (q < -(int32_t)q_max) return (struct foc_dq) { .d = (fract16_t)d, .q = -(fract16_t)q_max };

    return (struct foc_dq) { .d = (fract16_t)d, .q = (fract16_t)q };
}


static inline ufract16_t foc_vq_circle_limit(ufract16_t magnitude_limit, fract16_t d)
{
    assert(abs(d) <= magnitude_limit); /* set by feedback output */
    return fixed_sqrt((uint32_t)magnitude_limit * magnitude_limit - (int32_t)d * d);
}

/******************************************************************************/
/*!
    @brief  Clarke
    Transform 3-phase (120 degree) stationary reference frame quantities: Ia, Ib, Ic
    into 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta

    Ialpha = (2*Ia - Ib - Ic)/3
    Ibeta = sqrt3/3*(Ib - Ic) = (Ib - Ic)/sqrt3

    @param[in]  a, b, c
 */
 /******************************************************************************/
static inline struct foc_alphabeta foc_clarke(fract16_t a, fract16_t b, fract16_t c)
{
    int32_t alpha = fract16_mul((int32_t)a * 2 - b - c, FRACT16_1_DIV_3);
    int32_t beta = fract16_mul((int32_t)b - c, FRACT16_1_DIV_SQRT3);

    return (struct foc_alphabeta) { .alpha = fract16_sat(alpha), .beta = fract16_sat(beta) };
}

/*
    Ialpha = Ia
    Ibeta = (Ib - Ic)/sqrt(3)
*/
static inline struct foc_alphabeta foc_clarke_align(fract16_t a, fract16_t b, fract16_t c)
{
    int32_t beta = fract16_mul((int32_t)b - c, FRACT16_1_DIV_SQRT3);

    return (struct foc_alphabeta) { .alpha = a, .beta = fract16_sat(beta) };
}

/*!
    @brief  2-Phase Version

    Ialpha = Ia
    Ibeta = (Ia + 2*Ib)/sqrt3
*/
static inline struct foc_alphabeta foc_clarke_ab(fract16_t a, fract16_t b)
{
    int32_t beta = fract16_mul((int32_t)a + b * 2, FRACT16_1_DIV_SQRT3);

    return (struct foc_alphabeta) { .alpha = a, .beta = fract16_sat(beta) };
}

/******************************************************************************/
/*!
    @brief  Inverse Clarke

    a = alpha
    b = (-alpha + sqrt3*beta)/2
    c = (-alpha - sqrt3*beta)/2
*/
/******************************************************************************/
static inline struct foc_abc foc_inv_clarke(fract16_t alpha, fract16_t beta)
{
    int32_t alpha_div2 = fract16_mul(alpha, FRACT16_1_DIV_2);
    int32_t beta_sqrt3_div2 = fract16_mul(beta, FRACT16_SQRT3_DIV_2);

    int32_t b = -alpha_div2 + beta_sqrt3_div2;
    int32_t c = -alpha_div2 - beta_sqrt3_div2;

    return (struct foc_abc) { .a = fract16_sat(alpha), .b = fract16_sat(b), .c = fract16_sat(c) };
}

/******************************************************************************/
/*!
    @brief  Park
    Transforms 2-axis orthogonal stationary reference frame quantities: Ialpha and Ibeta
    into 2-axis orthogonal rotor synchronous reference frame quantities: Id and Iq.

    Id = alpha*cos(theta) + beta*sin(theta)
    Iq = -alpha*sin(theta) + beta*cos(theta)

    @param[in]  Ialpha, Ibeta
    @param[in]  theta - rotating frame angle in q1.15 format
*/
/******************************************************************************/
static inline fract16_t foc_park_axis_d(fract16_t alpha, fract16_t beta, fract16_t sin, fract16_t cos)
{
    return fract16_sat(fract16_mul(alpha, cos) + fract16_mul(beta, sin));
}

static inline struct foc_dq foc_park_vector(fract16_t alpha, fract16_t beta, fract16_t sin, fract16_t cos)
{
    int32_t d = fract16_mul(alpha, cos) + fract16_mul(beta, sin);
    int32_t q = -fract16_mul(alpha, sin) + fract16_mul(beta, cos);

    return (struct foc_dq) { .d = fract16_sat(d), .q = fract16_sat(q) };
}

/******************************************************************************/
/*!
    @brief  Inverse Park

    alpha = d*cos(theta) - q*sin(theta)
    beta = d*sin(theta) + q*cos(theta)
*/
/******************************************************************************/
static inline struct foc_alphabeta foc_inv_park_vector(fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    int32_t alpha = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t beta = fract16_mul(d, sin) + fract16_mul(q, cos);

    return (struct foc_alphabeta) { .alpha = fract16_sat(alpha), .beta = fract16_sat(beta) };
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
struct foc_dq0 { fract16_t d, q, zero; };
struct foc_sincos6 { fract16_t cos, sin, cos_120, sin_120, cos_240, sin_240; };

// static inline fract16_t foc_d_of_abc(fract16_t a, fract16_t b, fract16_t c, fract16_t cos, fract16_t cos_120, fract16_t cos_240)
// {
//     return fract16_mul(a, cos) + fract16_mul(b, cos_120) + fract16_mul(c, cos_240);
// }

// static inline fract16_t foc_q_of_abc(fract16_t a, fract16_t b, fract16_t c, fract16_t sin, fract16_t sin_120, fract16_t sin_240)
// {
//     return -fract16_mul(a, sin) - fract16_mul(b, sin_120) - fract16_mul(c, sin_240);
// }

/*!
    Direct DQ0 Transform - converts 3-phase (a, b, c) directly to rotor reference frame (d, q, 0)
    Combines Clarke and Park transforms in one operation

    @param[in] a, b, c - Three phase values
    @param[in] sine, cosine - Rotor angle sin/cos values
*/
/*
    DQ0 transformation matrix multiplication:
    [d]         [cos(θ)     cos(θ-2π/3)  cos(θ+2π/3)] [a]
    [q] = 2/3 * [-sin(θ)   -sin(θ-2π/3) -sin(θ+2π/3)] [b]
    [0]         [1/2       1/2          1/2         ] [c]
*/
/* park_dq */
static inline struct foc_dq foc_abc_dq_transform
(
    fract16_t a, fract16_t b, fract16_t c,
    fract16_t cos, fract16_t sin,
    fract16_t cos_120, fract16_t sin_120,
    fract16_t cos_240, fract16_t sin_240
)
{
    int32_t d = fract16_mul(a, cos) + fract16_mul(b, cos_120) + fract16_mul(c, cos_240);
    int32_t q = -fract16_mul(a, sin) - fract16_mul(b, sin_120) - fract16_mul(c, sin_240);

    return (struct foc_dq) { .d = fract16_sat(fract16_mul(d, FRACT16_2_DIV_3)), .q = fract16_sat(fract16_mul(q, FRACT16_2_DIV_3)) };
}


// static inline struct foc_dq0 foc_abc_dq0_transform(fract16_t a, fract16_t b, fract16_t c, fract16_t sine, fract16_t cosine)
// {
//     /* Pre-calculate angle shifts for 120° and 240° */
//     fract16_t cos120 = fract16_mul(cosine, FRACT16_COS_120) - fract16_mul(sine, FRACT16_SIN_120);  /* cos(θ-120°) */
//     fract16_t sin120 = fract16_mul(sine, FRACT16_COS_120) + fract16_mul(cosine, FRACT16_SIN_120);  /* sin(θ-120°) */
//     fract16_t cos240 = fract16_mul(cosine, FRACT16_COS_240) - fract16_mul(sine, FRACT16_SIN_240);  /* cos(θ+120°) */
//     fract16_t sin240 = fract16_mul(sine, FRACT16_COS_240) + fract16_mul(cosine, FRACT16_SIN_240);  /* sin(θ+120°) */

//     /* Calculate d-axis component */
//     /* Calculate q-axis component (negative sine for proper orientation) */
//     accum32_t d = fract16_mul(a, cosine) + fract16_mul(b, cos120) + fract16_mul(c, cos240);
//     accum32_t q = -fract16_mul(a, sine) - fract16_mul(b, sin120) - fract16_mul(c, sin240);

//     /* Calculate zero sequence component */
//     accum32_t zero = (accum32_t)a + (accum32_t)b + (accum32_t)c;

//     return (struct foc_dq0)
//     {
//         .d = fract16_mul(d, FRACT16_2_DIV_3),
//         .q = fract16_mul(q, FRACT16_2_DIV_3),
//         .zero = fract16_mul(zero, FRACT16_1_DIV_3)
//     };
// }



/*!
    Inverse DQ0 Transform - converts rotor reference frame (d, q, 0) directly to 3-phase (a, b, c)
    Combines inverse Park and inverse Clarke transforms in one operation

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
static inline struct foc_abc foc_inv_abc_dq_transform
(
    fract16_t d, fract16_t q,
    fract16_t cos, fract16_t sin,
    fract16_t cos_120, fract16_t sin_120,
    fract16_t cos_240, fract16_t sin_240
)
{
    int32_t a = fract16_mul(d, cos) - fract16_mul(q, sin);
    int32_t b = fract16_mul(d, cos_120) - fract16_mul(q, sin_120);
    int32_t c = fract16_mul(d, cos_240) - fract16_mul(q, sin_240);

    return (struct foc_abc) { .a = fract16_sat(a), .b = fract16_sat(b), .c = fract16_sat(c) };
}

// static inline struct foc_abc foc_inv_abc_dq0_transform(fract16_t d, fract16_t q, fract16_t zero, fract16_t sine, fract16_t cosine)
// {
//     /* Pre-calculate angle shifts for 120° and 240° */
//     fract16_t cos120 = fract16_mul(cosine, FRACT16_COS_120) - fract16_mul(sine, FRACT16_SIN_120);  /* cos(θ-120°) */
//     fract16_t sin120 = fract16_mul(sine, FRACT16_COS_120) + fract16_mul(cosine, FRACT16_SIN_120);  /* sin(θ-120°) */
//     fract16_t cos240 = fract16_mul(cosine, FRACT16_COS_240) - fract16_mul(sine, FRACT16_SIN_240);  /* cos(θ+120°) */
//     fract16_t sin240 = fract16_mul(sine, FRACT16_COS_240) + fract16_mul(cosine, FRACT16_SIN_240);  /* sin(θ+120°) */

//     /* Calculate three-phase components */
//     return (struct foc_abc)
//     {
//         .a = fract16_mul(d, cosine) - fract16_mul(q, sine) + zero,
//         .b = fract16_mul(d, cos120) - fract16_mul(q, sin120) + zero,
//         .c = fract16_mul(d, cos240) - fract16_mul(q, sin240) + zero
//     };
// }


/******************************************************************************/
/*!
    optimized versions with precomputed sin/cos for 120 and 240 degree shifts
*/
/******************************************************************************/
static inline struct foc_sincos6 foc_abc_dq_angle(angle16_t theta)
{
    fract16_t cos = fract16_cos(theta);
    fract16_t sin = fract16_sin(theta);
    fract16_t cos_120 = fract16_mul(cos, FRACT16_COS_120) - fract16_mul(sin, FRACT16_SIN_120);  /* cos(θ-120°) */
    fract16_t sin_120 = fract16_mul(sin, FRACT16_COS_120) + fract16_mul(cos, FRACT16_SIN_120);  /* sin(θ-120°) */

    return (struct foc_sincos6) { .cos = cos, .sin = sin, .cos_120 = cos_120, .sin_120 = sin_120, .cos_240 = cos_120, .sin_240 = -sin_120 };
}


