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
    @file   foc_sensorless_math.h
    @author FireSourcery
    @brief  Sensorless FOC pure math — back-EMF observer, sliding-mode observer,
            and the map from the EMF to the rotor angle. The phase detector is AnglePll.h.

    Stateless per-tick step functions that compose into a sensorless rotor
    position observer; state is owned by the caller. The plant is stator_math.h,
    the correction laws smo_math.h, the smoothing Math/Filter. All signals share the
    fract16 per-unit basis used by foc_math.h / motor_electrical_math.h:

    Two families are included, distinguished by working frame:

      αβ stationary  — voltage-model and SMO; signals sinusoidal at ω_e;
                       position recovered downstream (PLL on (e_α,e_β) or atan2).
      dq estimated   — EEMF observer and direct-dq PLL; signals DC at lock;
                       the observer is closed-loop in θ̂.

    Excluded: open-loop I/F startup ramp and saliency-based estimation
    (HFI / square-wave / INFORM) — separate algorithm classes.
*/
/******************************************************************************/
#include "foc_math.h"
#include "stator_math.h"
#include "smo_math.h"
#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*!
    @brief  First-order discrete low-pass step.

        y[k+1] = y[k] + k_lp · (x[k] - y[k]),       k_lp = dt / (τ + dt)

    Used to extract the equivalent control of z (≈ ê) from the SMO output and
    to smooth ê before the angle tracker. Choose τ such that the cutoff sits
    above the maximum tracked electrical frequency yet below the SMO chatter band.
*/
/******************************************************************************/
static inline fract16_t lpf_step(fract16_t k_lp, fract16_t y, fract16_t x) { return fract16_sat((accum32_t)y + fract16_mul(k_lp, x - y)); }

/******************************************************************************/
/*!
    @brief  EEMF phase-error extraction (estimated dq frame).

    Takes the LPF'd EEMF (ê_d, ê_q) — typically lpf_step on (zd, zq) from
    the observer output. The same map takes it to the rotor vector relative to θ̂,
    for [angle_pll_error_normalized_relative].

    Sign of ê_q encodes rotation direction (positive for forward, negative for
    reverse). Caller supplies `sign` to disambiguate; pass +1 forward, -1 reverse.
*/
/******************************************************************************/
/*
    Normalized phase error ≈ −sin(θ − θ̂) in either direction, independent of |E|.
    |ê| strips the magnitude of E, not its sign: `sign` restores the direction, e.g. sign(ω̂).
    The signed ê_q would also restore it, but would make θ̂ = θ + π a second stable lock.
    0 when |ê| is below the noise floor (lock indeterminate).
*/
/* The rotor vector −j·sign(ω)·e, along θ with length |ω|·ψ */
static inline struct fract16_xy foc_rotor_of_emf(sign_t sign, fract16_t e_x, fract16_t e_y) { return (struct fract16_xy) { .x = sign * e_y, .y = -sign * e_x }; }

/******************************************************************************/
/*!
    @brief  Electrical speed estimate from EMF magnitude.

        |e| = |ω_e| · ψ_f       →       |ω̂_e| = |e| / ψ_f

    With Psi_pu per psi_pu_turn, the result is the electrical
    angle increment per control cycle:

        |Δθ̂_e| [angle16/dt] = |e_pu| / Psi_pu

    Sign of ω is not recoverable from |e|; combine with foc_pll_error sign
    or the open-loop reference. Returns 0 when Psi_pu is zero (uninitialized).
*/
/******************************************************************************/
// static inline angle16_t foc_speed_of_emf(accum32_t Psi_pu, ufract16_t e_mag) { return (angle16_t)fract16_div(e_mag, Psi_pu); }


/******************************************************************************/
/*!
    @brief  Sliding-Mode switching function — bounded sign with linear region.

        sat(x, thr) = clamp(x / thr, -1, +1)

    Replaces hard sign() inside the SMO to suppress chatter; thr is the
    boundary-layer width in the same per-unit basis as x. thr > 0.
*/
/******************************************************************************/
static inline fract16_t smo_sat(fract16_t thr, fract16_t x) { return fract16_sat(fract16_div(x, thr)); }
/* σ, saturated. |î − i| reaches 2.0 */
static inline fract16_t smo_sigma(fract16_t i_est, fract16_t i_meas) { return fract16_sat((accum32_t)i_est - i_meas); }
/* First-order law. z = K_smo · sat(σ, thr) */
static inline fract16_t smo_z(fract16_t K_smo, fract16_t thr, fract16_t sigma) { return fract16_mul(K_smo, smo_sat(thr, sigma)); }
// static inline fract16_t smo_axis_z(fract16_t K_smo, fract16_t thr, fract16_t i_est, fract16_t i_meas) { return smo_z(K_smo, thr, smo_sigma(i_est, i_meas)); }

/******************************************************************************/
/*!
    @brief  Sliding-Mode Current Observer — single-axis step (αβ frame).

    Plant (per axis), the current model [stator_i_next]:
        di/dt = (Ls)⁻¹ · ( v - Rs·i - e )

    Discrete observer, at the sample i_meas[k], with v[k] the command made on it, held across the next period:
        z[k]    = K_smo · sat( î[k] - i_meas[k], thr )
        î[k+1]  = î[k] + G_pu · ( v[k] - Rs_pu·î[k] - z[k] )      z in place of e

        G_pu = dt · V_max / (Ls · I_max) = 1 / Ls_pu_rad = π / Ls_pu_turn

    The same recurrence as î[k] then z[k], bounded at the end of the period instead: that form takes
    the v applied, stored a tick; this one the v just commanded, so it runs in the same call as the capture.


    The switching variable z drives (î - i) → 0; its slow component
    (extracted by lpf_step) is the back-EMF estimate ê used by the
    angle tracker. The cutoff sits above the maximum tracked electrical
    frequency, below the chatter band.

    @param  G_pu      observer integrator gain = 1 / Ls_pu_rad
    @param  K_smo     sliding gain; > peak EMF in pu so z dominates ê
    @param  thr       boundary-layer width (smo_sat); typ. 0.05..0.2 of i_max
*/
/******************************************************************************/


/******************************************************************************/
/*!
    @brief  Extended-EMF (EEMF) Observer — sliding-mode current observer in
            the estimated dq frame.

    Plant in estimated dq, EEMF form (Lq used on BOTH axes; the saliency
    (Ld−Lq) contribution is absorbed into the EEMF disturbance):

        vd = Rs·îd + Lq·dîd/dt − ω·Lq·îq + e_d
        vq = Rs·îq + Lq·dîq/dt + ω·Lq·îd + e_q

    In TRUE dq the disturbance is purely on q:
        e_d = 0
        e_q = ω·(ψ_f + (Ld−Lq)·id) − (Ld−Lq)·diq/dt
    For misalignment Δθ = θ − θ̂, the same disturbance appears in estimated dq as:
        ê_d_apparent ≈ E·sin(Δθ) ≈ E·Δθ       (phase detector!)
        ê_q_apparent ≈ E·cos(Δθ) ≈ E

    Discrete SMO step:
        îd[k] = îd[k-1] + G_pu · ( vd − Rs·îd[k-1] + ω·Lq·iq[k] − zd[k-1] )
        îq[k] = îq[k-1] + G_pu · ( vq − Rs·îq[k-1] − ω·Lq·id[k] − zq[k-1] )
        zd[k] = K_smo · sat( îd[k] − id[k], thr )
        zq[k] = K_smo · sat( îq[k] − iq[k], thr )
    The coupling on the measured i: each axis a scalar SMO [smo_axis_z], its boundary-layer offset î − i kept off the other axis.

        G_pu = dt · V_max / (Lq · I_max) = 1 / Lq_pu

    (zd, zq) drive (î − i) → 0; their LPF'd values (lpf_step) are (ê_d, ê_q):
        ê_d → 0      at lock
        ê_q → ω·(ψ_f + (Ld−Lq)·id)

    Already in the frame of θ̂: the rotor vector [foc_rotor_of_emf] of (ê_d, ê_q) is relative to θ̂.

    Advantage over αβ SMO: signals are DC at lock, so the equivalent-control
    LPF cleans noise without the chatter/bandwidth tradeoff that plagues the
    sinusoidal αβ case. Native IPM handling via the EEMF formulation.
    Cost: closed-loop in θ̂ (observer depends on ω̂, θ̂) — convergence is harder
    to prove and requires reasonable startup alignment.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief  Super-Twisting Algorithm (STA) — 2nd-order sliding-mode observer.

    Continuous-output alternative to the classical smo_z(K·sat) — replaces
    the discontinuous switching variable with one produced by integrating
    sign(σ). The output z is continuous, so no equivalent-control LPF is
    needed downstream.

    Per axis, with σ = î − i_meas:
        z[k]   = w[k] + λ · √|σ| · sign(σ)        (continuous correction)
        w[k+1] = w[k] + (α·dt) · sign(σ)          (integrator state)

    The current-observer update is identical to the classical SMO; only z
    changes:
        î[k+1] = î[k] + G_pu · ( v − Rs·î − z )

    At σ → 0 (sliding), the √|σ| term vanishes and z → w; both converge to
    the disturbance ê. Feed w (cleaner) into the PLL discriminator.

    Tuning (Moreno 2008), L = Lipschitz bound on dê/dt in pu:
        λ      > √L                practical: 1.5 · √L
        α · dt > L · dt            practical: 1.1 · L · dt
    Caller supplies α already multiplied by dt.
*/
/******************************************************************************/
/* sqrt(|x|) · sign(x) — STA forcing primitive. Guards x = INT16_MIN. */
// static inline fract16_t sta_sqrt_signed(fract16_t x)
// {
//     fract16_t mag_sqrt = (fract16_t)fract16_sqrt(fract16_abs_sat(x));
//     return (x >= 0) ? mag_sqrt : -mag_sqrt;
// }

// /* STA continuous switching output. w is the current integrator state. */
// static inline fract16_t sta_z(fract16_t lambda, fract16_t w, fract16_t sigma) { return fract16_sat((accum32_t)w + fract16_mul(lambda, sta_sqrt_signed(sigma))); }

// /* STA integrator update: w[k+1] = w[k] + (α·dt) · sign(σ). */
// static inline fract16_t sta_w_step(fract16_t alpha_dt, fract16_t w, fract16_t sigma) { return fract16_sat((accum32_t)w + (accum32_t)alpha_dt * math_sign(sigma)); }

// struct sta_axis { fract16_t z, w; };

// /* Combined STA single-axis step — yields (z, w_next) from (w, î, i_meas). */
// static inline struct sta_axis sta_axis_step(fract16_t lambda, fract16_t alpha_dt, fract16_t w, fract16_t i_est, fract16_t i_meas)
// {
//     fract16_t sigma = smo_sigma(i_est, i_meas);
//     return (struct sta_axis) { .z = sta_z(lambda, w, sigma), .w = sta_w_step(alpha_dt, w, sigma), };
// }