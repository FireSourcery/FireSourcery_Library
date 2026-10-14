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

// static inline accum32_t foc_vd_ff_frexp(fract16e_t omega_Lq, fract16_t iq)
// // static inline accum32_t foc_vq_ff_frexp(fract16e_t omega_Ld, fract16e_t omega_psi, fract16_t id)

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


/*
    q budget remaining after d has claimed its share of the circle.
    Saturates to 0 rather than underflowing the unsigned subtraction when d has already
    consumed the whole radius — the d-axis limit is not guaranteed to match this radius.
*/
static inline ufract16_t foc_q_circle_limit(ufract16_t magnitude_limit, fract16_t d)
{
    uint32_t limit_squared = (uint32_t)magnitude_limit * magnitude_limit;
    uint32_t d_squared = (uint32_t)((int32_t)d * d);
    return (d_squared < limit_squared) ? fixed_sqrt(limit_squared - d_squared) : 0U;
}


/******************************************************************************/
/*!
    Power Balance
    Amplitude-invariant dq, ideal inverter:
        V_dc·i_dc = 3/2·(v_d·i_d + v_q·i_q)
    Powers are in v·i units, the dq products without the 3/2.
*/
/******************************************************************************/
/*
    i_dc = 3/2·(v_d·i_d + v_q·i_q) / V_dc, by the reciprocal 1 / V_dc.
    |v_dq| < 2·V_dc keeps power·v_dc_inv within 32 bits. The _ form omits the 3/2, keeping the sign within fract16 range.
*/
static inline accum32_t _foc_i_dc(accum32_t power, accum32_t v_dc_inv) { return fract16_mul(power, v_dc_inv); }
static inline accum32_t foc_i_dc(accum32_t power, accum32_t v_dc_inv) { return _foc_i_dc(power, v_dc_inv) * 3 / 2; }

/* 2/3·V_dc, the power per unit i_dc */
static inline accum32_t foc_power_per_i_dc(ufract16_t v_dc) { return fract16_mul(v_dc, FRACT16_2_DIV_3); }
/* 2/3·V_dc·i_dc, the power that carries i_dc */
static inline accum32_t foc_power_of_i_dc(ufract16_t i_dc, ufract16_t v_dc) { return fract16_mul(i_dc, foc_power_per_i_dc(v_dc)); }

/*
    The phase current that bounds i_dc at every operating point, linear SVPWM |v_dq| <= V_dc/√3:
        i_dc = 3/2·(v_d·i_d + v_q·i_q) / V_dc <= √3/2·|i_dq|   =>   |i_dq| <= 2/√3·i_dc
    Exact at full modulation and unity power factor, conservative below.
*/
static inline ufract16_t foc_i_phase_of_i_dc(ufract16_t i_dc) { return math_min(fract16_mul(i_dc, FRACT16_2_DIV_SQRT3), FRACT16_MAX); }

/*

*/
static inline accum32_t foc_power_d(fract16_t vd, fract16_t id) { return fract16_mul(vd, id); }
static inline accum32_t foc_power_q(fract16_t vq, fract16_t iq) { return fract16_mul(vq, iq); }
static inline accum32_t foc_active_power(fract16_t vd, fract16_t id, fract16_t vq, fract16_t iq) { return foc_power_d(vd, id) + foc_power_q(vq, iq); }
static inline accum32_t foc_reactive_power(fract16_t vd, fract16_t id, fract16_t vq, fract16_t iq) { return fract16_mul(vq, id) - fract16_mul(vd, iq); }


/* |i_q| = power / |v_q|. None once the power is spent, open toward v_q = 0 */
static inline ufract16_t foc_iq_of_power(accum32_t power, fract16_t vq) { return math_min(fract16_div(math_max(power, 0), math_max(math_abs(vq), 1)), FRACT16_MAX); }

// /* |i_q| = power / |v_q|. None once the power is spent. v_q = 0 carries no q-axis power, any i_q holds */
// static inline ufract16_t foc_iq_of_power(accum32_t power, fract16_t vq) { return (vq == 0) ? FRACT16_MAX : math_min(fract16_div(math_max(power, 0), math_abs(vq)), FRACT16_MAX); }



/*
    Inverse at the operating point, i_d held. The |i_q| at which the inverter
        draws i_dis from the supply,    i_q along v_q:    ((2/3)·V_dc·i_dis − v_d·i_d) / |v_q|
        returns i_chg to the supply,    i_q against v_q:  ((2/3)·V_dc·i_chg + v_d·i_d) / |v_q|

    i_q ≤ i_dc_lim · V_dc / ((3/2)·v_q)
*/
static inline ufract16_t foc_iq_discharge_limit(ufract16_t i_dis, ufract16_t v_dc, accum32_t power_d, fract16_t vq) { return foc_iq_of_power(foc_power_of_i_dc(i_dis, v_dc) - power_d, vq); }
static inline ufract16_t foc_iq_charge_limit(ufract16_t i_chg, ufract16_t v_dc, accum32_t power_d, fract16_t vq) { return foc_iq_of_power(foc_power_of_i_dc(i_chg, v_dc) + power_d, vq); }

/*
    Supply window i_dc = [−I_chg : I_dis], drawing positive, as limits on i_q. [Cw:Ccw]
*/
/*
    Power balance at the operating point, i_d held.
        v_q·i_q ∈ P_q = 2/3·V_dc·i_dc − v_d·i_d   ⟹   i_q ∈ P_q / v_q
    i_q along v_q draws from the supply, against v_q returns to it. v_q = 0 moves no q-axis power, both sides open.
    P_q holds 0: a d-axis that spends a side's budget leaves that side no i_q, rather than a reversed one.
*/
static inline interval_t foc_iq_limits(interval_t i_dc, ufract16_t v_dc, accum32_t power_d, fract16_t vq)
{
    interval_t power_q = interval_offset(fract16_interval_scale(i_dc, foc_power_per_i_dc(v_dc)), -power_d);
    return fract16_interval_div(interval_hull(power_q, (interval_t){ .low = 0, .high = 0 }), vq);
}

// static inline interval_t foc_iq_limits(interval_t i_dc, ufract16_t v_dc, accum32_t power_d, fract16_t vq)
// {
//     return fract16_interval_div(interval_shift(fract16_interval_scale(i_dc, foc_power_of_i_dc(...)), -power_d), vq);
// }

/*
    Phase-current bound, keyed by the motion (the back-EMF sign) rather than v_q. [foc_i_phase_of_i_dc] bounds |i_dc| both ways.
        i_q along the motion only draws:            I_dis
        i_q against the motion returns below |ω·ψ|/R and draws beyond it (plug braking):  min(I_dis, I_chg)
        At standstill no side returns, both draw:   I_dis
    The share of the circle left to i_q after i_d.
*/
static inline interval_t foc_iq_phase_limits(interval_t i_dc, sign_t motion, fract16_t id)
{
    ufract16_t along = foc_q_circle_limit(foc_i_phase_of_i_dc(i_dc.high), id);
    ufract16_t against = foc_q_circle_limit(foc_i_phase_of_i_dc(math_min(i_dc.high, -i_dc.low)), id);
    return (motion != SIGN_ZERO) ? interval_of_sign_pair(motion, along, against) : interval_symmetric(0, along);
}

/******************************************************************************/
/*!
    State helpers
*/
/******************************************************************************/
/*
    Regeneration: the back-EMF e_q drives i_q against v_q. Needs v_q along e_q, beyond the window around standstill
    where the resistive drop sets v_q and every current draws. Inside it, return power is at most window²/(4·R).
*/
static inline bool foc_is_regen(fract16_t vq, accum32_t e_q, ufract16_t window) { return (math_abs(e_q) > window) && (math_sign(vq) == math_sign(e_q)); }

/*
    Dynamic limits
    dynamic overlay to intersect with base
*/
/* the controller to pick up the requested direction through zero */
// static inline sign_t _foc_vmotionsignof(accum32_t omega_psi, int32_t iqReq)
// {
//     sign_t motionSign = math_sign(omega_psi);
//     return (motionSign != SIGN_ZERO) ? motionSign : math_sign(iqReq);
// }

// static inline interval_t _foc_vbrakeoverlay(accum32_t omega_psi, int32_t iqReq, int32_t vqLimit)
// {
//     sign_t motionSign = _foc_vmotionsignof(omega_psi, iqReq);
//     /* Only narrow the band when the request is braking/generating. */
//     if (motionSign * iqReq < 0) { return interval_half_plane(motionSign, vqLimit); }
//     return interval_symmetric(0, vqLimit);
// }

// /* symmetric by default. dynamic half-plane anti-plugging derived per cycle */
// static inline interval_t _foc_vregenonly(int32_t omega_psi, int32_t iqReq, int32_t vqLimit)
// {
//     if (omega_psi * iqReq <= 0) { return interval_half_plane(omega_psi * iqReq, vqLimit); }
//     return interval_symmetric(0, vqLimit);
// }

// static inline interval_t _FOC_VRegenOnly(int32_t omega_psi, int32_t iqReq, int32_t vq) { return interval_half_plane(_FOC_VMotionSignOf(omega_psi, iqReq), vq); }
// /* Applied Vq is forced opposite to EEMF for explicit plugging behavior. */
// static inline interval_t _foc_vpluggingonly(int32_t omega_psi, int32_t iqReq, int32_t vq) { return interval_half_plane((sign_t)(0 - _foc_vmotionsignof(omega_psi, iqReq)), vq); }

// static inline interval_t _foc_vdynamicband(const FOC_T * p_foc, accum32_t omega_psi, int32_t iqReq, ufract16_t vqCircleLimit)
// {
//     interval_t band = interval_symmetric(0, vqCircleLimit);

//     band = interval_intersect(band, _foc_vregenonly(omega_psi, iqReq, vqCircleLimit));

//     if (p_foc->VWindow > 0)
//     {
//         band = interval_intersect(band, _foc_vbemfwindow(vqCircleLimit, omega_psi, p_foc->VWindow));
//     }

//     return band;
// }

// /*
//     VqCircleLimit = 50
//     omega_psi = 80
//     window = 10
//     desired window = [70, 90]
//     feasible slice = [-50, 50]
//     intersection = [70, 50], invalid
// */
// static inline interval_t _foc_vqbandof(ufract16_t vqCircleLimit, accum32_t omega_psi, int32_t window)
// {
//     interval_t vqBand = interval_intersect(interval_symmetric(0, vqCircleLimit), interval_symmetric(omega_psi, window));

//     if (vqBand.high < vqBand.low)
//     {
//         int32_t vqEdge = math_clamp(omega_psi, -(int32_t)vqCircleLimit, (int32_t)vqCircleLimit);
//         return (interval_t) { .low = vqEdge, .high = vqEdge };
//     }
//     return vqBand;
//     // low = math_clamp(omega_psi - window, -(int32_t)vqCircleLimit, (int32_t)vqCircleLimit);
//     // high = math_clamp(omega_psi + window, -(int32_t)vqCircleLimit, (int32_t)vqCircleLimit);

//     // center = clamp(omega_psi, -VqCircleLimit, +VqCircleLimit)
//     // band = intersect([-VqCircleLimit, +VqCircleLimit], [center - window, center + window])
// }

/******************************************************************************/
/* Secondary Section */
/******************************************************************************/
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
// static inline fract16_t foc_park_axis_d(fract16_t alpha, fract16_t beta, fract16_t sin, fract16_t cos)
// {
//     return fract16_sat(fract16_mul(alpha, cos) + fract16_mul(beta, sin));
// }

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
/* Experimental */
/******************************************************************************/
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


/******************************************************************************/
/*!

*/
/******************************************************************************/
/* The supply window as q-axis power, holding 0. Shared by both inversions. */
// static inline interval_t foc_power_q_window(interval_t i_dc, ufract16_t v_dc, accum32_t power_d)
//     { return interval_hull(interval_offset(fract16_interval_scale(i_dc, foc_power_per_i_dc(v_dc)), -power_d), (interval_t){ 0 }); }

// /* R·i², the power a current spends in the winding */
// static inline accum32_t foc_power_r(accum32_t r, fract16_t i) { return fract16_mul(fract16_mul(r, i), i); }

// /* h + √(h² + R·p) */
// static inline accum32_t _foc_emf_root(accum32_t h, accum32_t r, accum32_t p) { return h + fixed_sqrt((uint32_t)(h * h + r * p)); }

// /*
//     R·i_q² + E·i_q ∈ [low : high], E held at the operating point. h = |E|/2.
//         along E, draws:                          high / (h + √(h² + R·high))
//         against E, returns below |E|/R:          |low| / (h + √(h² − R·|low|))   while h² > R·|low|, else none
//         against E, draws beyond (plug braking):  (h + √(h² + R·high)) / R
//     R = 0 is [foc_iq_limits] with v_q = E. E = 0 gives √(high / R) on both sides.
// */
// static inline interval_t foc_iq_limits_emf(interval_t power, accum32_t e, accum32_t r)
// {
//     accum32_t h = math_abs(e) / 2;
//     accum32_t root = _foc_emf_root(h, r, power.high);
//     ufract16_t along = fract16_sat(fract16_div(power.high, math_max(root, 1)));
//     ufract16_t plug = fract16_sat(fract16_div(root, math_max(r, 1)));
//     ufract16_t regen = (h * h > r * -power.low) ? fract16_sat(fract16_div(-power.low, _foc_emf_root(h, r, power.low))) : FRACT16_MAX;
//     return interval_of_sign_pair((e < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE, along, math_min(regen, plug));
// }