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
#include "motor_electrical_math.h"
#include "Math/Fixed/fract16.h"
#include "Math/Fixed/fract16e.h"

/******************************************************************************/
/*!
    @brief
    Aligned in CCW order: a, b, c; alpha, beta; d, q
    d-axis aligned to 0° / a-phase
*/
/******************************************************************************/
/*
    Data passing interface. 32-bit. inline optimizes away.
    Stages return unsaturated and take widened inputs, the sink saturates once.
    Park preserves |αβ| = |dq|, at most 4/3 from fract16 abc and √2 from fract16 dq, in units of 2^15. A Q15 product fits 32 bits.
*/
struct foc_abc { int32_t a, b, c; };
struct foc_dq { int32_t d, q; };
struct foc_alphabeta { int32_t alpha, beta; };


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
    return (struct foc_alphabeta)
    {
        .alpha = fract16_mul((int32_t)a * 2 - b - c, FRACT16_1_DIV_3),
        .beta = fract16_mul((int32_t)b - c, FRACT16_1_DIV_SQRT3),
    };
}

/*
    Ialpha = Ia
    Ibeta = (Ib - Ic)/sqrt(3)
*/
static inline struct foc_alphabeta foc_clarke_align(fract16_t a, fract16_t b, fract16_t c)
{
    return (struct foc_alphabeta) { .alpha = a, .beta = fract16_mul((int32_t)b - c, FRACT16_1_DIV_SQRT3) };
}

/*!
    Ialpha = Ia
    Ibeta = (Ia + 2*Ib)/sqrt3
*/
static inline struct foc_alphabeta foc_clarke_ab(fract16_t a, fract16_t b)
{
    return (struct foc_alphabeta) { .alpha = a, .beta = fract16_mul((int32_t)a + b * 2, FRACT16_1_DIV_SQRT3) };
}

/******************************************************************************/
/*!
    @brief  Inverse Clarke

    a = alpha
    b = (-alpha + sqrt3*beta)/2
    c = (-alpha - sqrt3*beta)/2
*/
/******************************************************************************/
static inline struct foc_abc foc_inv_clarke(accum32_t alpha, accum32_t beta)
{
    int32_t alpha_div2 = fract16_mul(alpha, FRACT16_1_DIV_2);
    int32_t beta_sqrt3_div2 = fract16_mul(beta, FRACT16_SQRT3_DIV_2);

    return (struct foc_abc)
    {
        .a = alpha,
        .b = -alpha_div2 + beta_sqrt3_div2,
        .c = -alpha_div2 - beta_sqrt3_div2
    };
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
static inline struct foc_dq foc_park(accum32_t alpha, accum32_t beta, fract16_t sin, fract16_t cos)
{
    return (struct foc_dq)
    {
        .d = fract16_mul(alpha, cos) + fract16_mul(beta, sin),
        .q = -fract16_mul(alpha, sin) + fract16_mul(beta, cos),
    };
}

/******************************************************************************/
/*!
    @brief  Inverse Park

    alpha = d*cos(theta) - q*sin(theta)
    beta = d*sin(theta) + q*cos(theta)
*/
/******************************************************************************/
static inline struct foc_alphabeta foc_inv_park(fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    return (struct foc_alphabeta)
    {
        .alpha = fract16_mul(d, cos) - fract16_mul(q, sin),
        .beta = fract16_mul(d, sin) + fract16_mul(q, cos),
    };
}

/******************************************************************************/
/*!
    @brief  Clarke-Park Transformation
    inlined for less redundant saturation
*/
/******************************************************************************/
static inline struct foc_dq foc_clarke_park(fract16_t a, fract16_t b, fract16_t c, fract16_t sin, fract16_t cos)
{
    struct foc_alphabeta ab = foc_clarke(a, b, c);
    return foc_park(ab.alpha, ab.beta, sin, cos);
}


static inline struct foc_abc foc_inv_clarke_park(fract16_t d, fract16_t q, fract16_t sin, fract16_t cos)
{
    struct foc_alphabeta ab = foc_inv_park(d, q, sin, cos);
    return foc_inv_clarke(ab.alpha, ab.beta);
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
static inline accum32_t foc_vd_ff(accum32_t omega_Lq, accum32_t iq) { return -accum32_mul(omega_Lq, iq); }
static inline accum32_t foc_vq_ff(accum32_t omega_Ld, accum32_t omega_psi, accum32_t id) { return accum32_mul(omega_Ld, id) + omega_psi; }

/*
    fract16k_t for either fract16e_t or accum32_t
*/
static inline accum32_t foc_vd_ff_k(fract16k_t omega_Lq, fract16_t iq) { return -fract16k_mul(omega_Lq, iq); }
static inline accum32_t foc_vq_ff_k(fract16k_t omega_Ld, accum32_t omega_psi, fract16_t id) { return fract16k_mul(omega_Ld, id) + omega_psi; }

/* co */
static inline accum32_t _foc_vd_ff_direct(accum32_t L_pu, fract16_t omega, fract16_t iq) { return -((int64_t)omega * L_pu * iq) / ((int64_t)FRACT16_SCALE * FRACT16_SCALE); }
static inline accum32_t _foc_vq_ff_direct(accum32_t Ld_pu, accum32_t psi_pu, fract16_t omega, fract16_t id) { return ((int64_t)omega * Ld_pu * id) / ((int64_t)FRACT16_SCALE * FRACT16_SCALE) + ((int64_t)omega * psi_pu) / FRACT16_SCALE; }


/******************************************************************************/
/*!
    @brief  dq voltage model — back-EMF in the estimated dq frame, from the
            steady-state stator voltage equation (did/dt = diq/dt = 0).

    Lq on both axes (EEMF form). Rs + ω·Lq·J is invariant under rotation, so the
    saliency lands in E and ê_d stays a pure phase detector, IPM included:
        ê_d = vd − Rs·id − foc_vd_ff(ω·Lq, iq)      = vd − Rs·id + ω·Lq·iq    ≈ −E·sin(θ − θ̂)
        ê_q = vq − Rs·iq − foc_vq_ff(ω·Lq, 0, id)   = vq − Rs·iq − ω·Lq·id    ≈  E·cos(θ − θ̂)
        E = ω·(ψ_f + (Ld − Lq)·id), signed with ω

    Caller precomputes ω·Lq in v_pu basis, as for the decoupling feedforward.
    No internal state, no di/dt term — cheapest of the dq-frame trackers. Needs Rs and Lq;
    a Lq error offsets θ̂ in proportion to the load current.
*/
/******************************************************************************/
/* One rotating axis at the operating point, signals DC: v = Rs·i + v_ff + e. v_ff couples in the other axis, [foc_vd_ff] [foc_vq_ff] */
static inline accum32_t foc_vemf_dq_axis(accum32_t Rs_pu, accum32_t v, accum32_t v_ff, accum32_t i) { return v - accum32_mul(Rs_pu, i) - v_ff; }

static inline accum32_t foc_vemf_d(accum32_t Rs_pu, accum32_t omega_Lq, accum32_t vd, accum32_t id, accum32_t iq) { return foc_vemf_dq_axis(Rs_pu, vd, foc_vd_ff(omega_Lq, iq), id); }
static inline accum32_t foc_vemf_q(accum32_t Rs_pu, accum32_t omega_Lq, accum32_t vq, accum32_t id, accum32_t iq) { return foc_vemf_dq_axis(Rs_pu, vq, foc_vq_ff(omega_Lq, 0, id), iq); }



/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    limit around d
    mag_limit^2 - d^2 = q^2
    no sqrt on back-calculation limit
*/
static inline struct foc_dq foc_circle_limit(fract16_t d, fract16_t q, ufract16_t magnitude_limit)
{
    uint32_t mag_limit_squared = (uint32_t)magnitude_limit * magnitude_limit;
    uint32_t d_squared = (int32_t)d * d;
    uint32_t q_squared = (int32_t)q * q;

    /* abs(d) > d_limit. gcc optimizes away when d is 0 */
    if (d_squared > mag_limit_squared) { return (struct foc_dq) { .d = math_sign(d) * magnitude_limit, .q = 0 }; }
    /* |Vdq| > magnitude_limit */
    if (d_squared + q_squared > mag_limit_squared) { return (struct foc_dq) { .d = d, .q = math_sign(q) * fixed_sqrt(mag_limit_squared - d_squared) }; }
    return (struct foc_dq) { .d = d, .q = q };
}

/* saturate into the fract16 domain only; foc_circle_limit applies the radius */
static inline struct foc_dq foc_circle_limit_ff(int32_t d, int32_t q, ufract16_t magnitude_limit)
{
    return foc_circle_limit(fract16_sat(d), fract16_sat(q), magnitude_limit);
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
static inline accum32_t foc_power_d(fract16_t vd, fract16_t id) { return fract16_mul(vd, id); }
static inline accum32_t foc_power_q(fract16_t vq, fract16_t iq) { return fract16_mul(vq, iq); }
static inline accum32_t foc_active_power(fract16_t vd, fract16_t id, fract16_t vq, fract16_t iq) { return foc_power_d(vd, id) + foc_power_q(vq, iq); }
static inline accum32_t foc_reactive_power(fract16_t vd, fract16_t id, fract16_t vq, fract16_t iq) { return fract16_mul(vq, id) - fract16_mul(vd, iq); }

/*
    i_dc = 3/2·(v_d·i_d + v_q·i_q) / V_dc, by the reciprocal 1 / V_dc.
    |v_dq| < 2·V_dc keeps power·v_dc_inv within 32 bits. The _ form omits the 3/2, keeping the sign within fract16 range.
*/
static inline accum32_t _foc_i_dc(accum32_t power, accum32_t v_dc_inv) { return fract16_mul(power, v_dc_inv); }
static inline accum32_t foc_i_dc(accum32_t power, accum32_t v_dc_inv) { return _foc_i_dc(power, v_dc_inv) * 3 / 2; }

/* 2/3·V_dc·i_dc, the power that carries i_dc */
static inline accum32_t foc_power_of_i_dc(ufract16_t i_dc, ufract16_t v_dc) { return fract16_mul(i_dc, fract16_mul(v_dc, FRACT16_2_DIV_3)); }

/* |i_q| = power / |v_q|. None once the power is spent, open toward v_q = 0 */
static inline ufract16_t foc_iq_of_power(accum32_t power, fract16_t vq) { return math_min(fract16_div(math_max(power, 0), math_max(math_abs(vq), 1)), FRACT16_MAX); }

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
    v_q = 0 takes the v_q > 0 orientation. A zero key would close the window at rest, where i_q = 0 holds v_q = 0.
*/
static inline interval_t foc_iq_limits(interval_t i_dc, ufract16_t v_dc, accum32_t power_d, fract16_t vq)
{
    ufract16_t iq_dis = foc_iq_discharge_limit(i_dc.high, v_dc, power_d, vq);
    ufract16_t iq_chg = foc_iq_charge_limit(-i_dc.low, v_dc, power_d, vq);
    return interval_of_sign((vq < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE, iq_dis, iq_chg);
}


/*
    The phase current that bounds i_dc at every operating point, linear SVPWM |v_dq| <= V_dc/√3:
        i_dc = 3/2·(v_d·i_d + v_q·i_q) / V_dc <= √3/2·|i_dq|   =>   |i_dq| <= 2/√3·i_dc
    Exact at full modulation and unity power factor, conservative below.
*/
static inline ufract16_t foc_i_phase_of_i_dc(ufract16_t i_dc) { return math_min(fract16_mul(i_dc, FRACT16_2_DIV_SQRT3), FRACT16_MAX); }

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
    return (motion != SIGN_ZERO) ? interval_of_sign(motion, along, against) : interval_symmetric(0, along);
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
//     return interval_of_sign((e < 0) ? SIGN_NEGATIVE : SIGN_POSITIVE, along, math_min(regen, plug));
// }


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
static inline sign_t _foc_vmotion_sign(accum32_t omega_psi, int32_t iqReq)
{
    sign_t motionSign = math_sign(omega_psi);
    return (motionSign != SIGN_ZERO) ? motionSign : math_sign(iqReq);
}

/*
    The v_q band. The hard band, the q circle ∩ the policy shape v_limit, holds 0 and is never empty.
    The window is clamped into it, so beyond the circle (|e_q| > circle + window) v_q stays at the edge nearest the back-EMF.
    A window wider than |e_q| + circle is the identity.
*/
static inline interval_t foc_vq_band(ufract16_t vq_circle_limit, interval_t v_limit, interval_t window)
{
    return interval_clamp_each(interval_intersect(interval_symmetric(0, vq_circle_limit), v_limit), window);
}

/*
    The window alone in the circle, at the edge nearest ω·ψ when the window lies beyond it.
    circle 50, ω·ψ 80, window 10: [70:90] in [−50:50] gives [50:50]
*/
// static inline interval_t foc_vq_band_w(ufract16_t vq_circle_limit, accum32_t omega_psi, ufract16_t window)
// {
//     return interval_clamp_each(interval_symmetric(0, vq_circle_limit), interval_symmetric(omega_psi, window));
// }

/*
    Dynamic v_q sides, per cycle from the motion (sign of ω·ψ) and the request (sign of i_q).
    Extents are FRACT16_MAX: the circle bounds the magnitude. Behaviors compared in Motor/Docs/VLimitPolicy.md.
*/
/* v_q never opposes the back-EMF. Standstill takes the request's side, closed without a request */
static inline interval_t _foc_v_no_plugging(accum32_t omega_psi, int32_t iq_req) { return interval_half_plane(_foc_vmotion_sign(omega_psi, iq_req), FRACT16_MAX); }

/* Only a braking request, i_q against the motion, is held to the motion's side */
static inline interval_t _foc_v_brake_overlay(accum32_t omega_psi, int32_t iq_req)
{
    sign_t motion = _foc_vmotion_sign(omega_psi, iq_req);
    return (motion * iq_req < 0) ? interval_half_plane(motion, FRACT16_MAX) : interval_symmetric(0, FRACT16_MAX);
}

/* Braking and coast held to the motion's side, standstill to the request's. Motoring keeps both sides */
static inline interval_t _foc_v_regen_only(accum32_t omega_psi, int32_t iq_req)
{
    return (math_sign(omega_psi) * math_sign(iq_req) <= 0) ? _foc_v_no_plugging(omega_psi, iq_req) : interval_symmetric(0, FRACT16_MAX);
}

/* v_q forced against the back-EMF, plug braking. Keyed by the motion alone, closed at standstill */
static inline interval_t _foc_v_plugging_only(accum32_t omega_psi) { return interval_half_plane((sign_t)(0 - math_sign(omega_psi)), FRACT16_MAX); }

/* [foc_vq_band] with a dynamic side over the static shape. Both hold 0, so the hard band stays non-empty */
static inline interval_t _foc_v_dynamic_band(ufract16_t vq_circle_limit, interval_t v_limit, interval_t side, interval_t window)
{
    return foc_vq_band(vq_circle_limit, interval_intersect(v_limit, side), window);
}

