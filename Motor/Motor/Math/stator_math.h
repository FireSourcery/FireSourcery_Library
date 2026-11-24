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
    @file   stator_math.h
    @author FireSourcery
    @brief  Stator voltage equation of one axis across one control period.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"


/******************************************************************************/
/*!
    @brief  Voltage-Model Back-EMF Estimator

        v = R·i + L·di/dt + e
        e = ω·ψ = v - Rs·i - Ls·di/dt

    Discrete one-step form, caller supplies the precomputed Δi = i[k] - i[k-1]:
        ê[k] = v[k] - Rs_pu·i[k] - Ls_pu_rad·Δi[k]

    L at the rad base, Fs / ω_base = 1. A turn-base L carries 1 / π: L_rad = L_turn / π.

    Stationary axes don't couple: the equation holds per axis in abc, and in αβ with L = Lq, where ē is the active-flux EMF:
        ψ = Lq·i + ψ_a·e^{jθ}           ψ_a = ψ + (Ld − Lq)·i_d, the active flux, along d
        ē = d(ψ_a·e^{jθ})/dt            ω·ψ_a along q, and (Ld − Lq)·di_d/dt along d while i_d moves
    Exact in any frame, at any ω̂. The extended EMF is the other placement, Ld on both axes and ω·Lq across: its transient, −(Ld − Lq)·di_q/dt, lies along q.
    Solved for v, the drive. Solved for e, the voltage model. Solved for i, the current model.
*/
/******************************************************************************/
/* The drop across the winding: R·ī across the resistance, L·Δi across the inductance */
static inline accum32_t stator_v_drop(accum32_t r, accum32_t l, fract16_t i_prev, fract16_t i) { return accum32_mul(r, i) + accum32_mul(l, i - i_prev); }

/* The v held across the period that moves i_prev to i against e. Across the next period, the deadbeat command */
static inline accum32_t stator_v(accum32_t r, accum32_t l, fract16_t i_prev, fract16_t i, accum32_t e) { return stator_v_drop(r, l, i_prev, i) + e; }

/*
    Voltage model. ē is the period average.
    R on the end sample instead offsets ē by R·Δi / 2, in proportion to speed.
*/
static inline accum32_t stator_emf(accum32_t r, accum32_t l, fract16_t i_prev, fract16_t i, accum32_t v) { return v - stator_v_drop(r, l, i_prev, i); }

/*
    L·di/dt
*/
static inline accum32_t stator_v_l(accum32_t r, fract16_t i, accum32_t v, accum32_t e) { return v - fract16_mul(r, i) - e; }

/******************************************************************************/
/*!
    Current model. Forward Euler from i at the start of the period, e given:
        i[k+1] = i[k] + G·(v − R·i[k] − e),     G = 1 / L_rad = Ts · V_base / (L · I_base)
*/
/******************************************************************************/
static inline accum32_t stator_i_next(accum32_t r, accum32_t g, fract16_t i, accum32_t v, accum32_t e) { return (accum32_t)i + accum32_mul(g, stator_v_l(r, i, v, e)); }


/*!
    dq frame with ff term
*/
/* One rotating axis at the operating point, signals DC: v = Rs·i + v_ff + e. v_ff couples in the other axis, [foc_vd_ff] [foc_vq_ff] */
// static inline accum32_t stator_vd_ff(accum32_t omega_Lq, accum32_t iq) { return -accum32_mul(omega_Lq, iq); }
// static inline accum32_t stator_vq_ff(accum32_t omega_Ld, accum32_t omega_psi, accum32_t id) { return accum32_mul(omega_Ld, id) + omega_psi; }
// static inline accum32_t stator_dq_axis(accum32_t Rs_pu, accum32_t v, accum32_t v_ff, accum32_t i) { return v - accum32_mul(Rs_pu, i) - v_ff; }

// static inline accum32_t stator_vemf_d(accum32_t Rs_pu, accum32_t omega_Lq, accum32_t vd, accum32_t id, accum32_t iq) { return stator_dq_axis(Rs_pu, vd, foc_vd_ff(omega_Lq, iq), id); }
// static inline accum32_t stator_vemf_q(accum32_t Rs_pu, accum32_t omega_Lq, accum32_t vq, accum32_t id, accum32_t iq) { return stator_dq_axis(Rs_pu, vq, foc_vq_ff(omega_Lq, 0, id), iq); }
// static inline fract16_t stator_eemf_i(accum32_t Rs_pu, accum32_t G_pu, accum32_t v, accum32_t v_ff, fract16_t i_est_prev, fract16_t e) { return fract16_sat(stator_i_next(Rs_pu, G_pu, i_est_prev, v - v_ff, e)); }

/******************************************************************************/
/*!
    Power across the period, a voltage times ī. Each term of the equation splits the terminal power:
        v·ī = R·ī² + L·Δi·ī + ē·ī       winding loss, stored-energy rate L·Δ(i²)/2, air gap
    v·i units as [foc_active_power], the 3/2 belongs to the quantity.
*/
/******************************************************************************/
static inline accum32_t stator_power(accum32_t v, fract16_t i_prev, fract16_t i) { return fract16_mul(v, (i_prev + i) / 2); }

/******************************************************************************/
/*!
    Rotating frame. The reactance X = ω·L is the arc of the frame's turn per period at radius L:
    ω·Ts = Δθ in radians, Δθ · π / 2^15, and L at the rad base carries the Fs. One truncation.
*/
/******************************************************************************/
static inline accum32_t stator_reactance(accum32_t l, angle16_t delta) { return ((int64_t)l * delta * FRACT16_PI) >> (2 * FRACT16_N_BITS); }


/******************************************************************************/
/*!
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
    @brief  Sliding-Mode switching function — bounded sign with linear region.

    The innovation σ = î − i, the model's estimate against the measurement, drives a correction z
    that takes the place of the unknown disturbance in the model.

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
static inline fract16_t smo_z_step(fract16_t K_smo, fract16_t thr, fract16_t i_est, fract16_t i_meas) { return smo_z(K_smo, thr, smo_sigma(i_est, i_meas)); }


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
    @brief  EEMF front end — sliding-mode current observer in
            the estimated dq frame.

    Plant in estimated dq, Lq on BOTH axes, the active-flux form [stator_math.h];
    the saliency (Ld−Lq) lands in the disturbance:

        vd = Rs·îd + Lq·dîd/dt − ω·Lq·îq + e_d
        vq = Rs·îq + Lq·dîq/dt + ω·Lq·îd + e_q

    In TRUE dq:
        e_d = (Ld−Lq)·did/dt                    0 while id holds
        e_q = ω·(ψ_f + (Ld−Lq)·id) = ω·ψ_a
    For misalignment Δθ = θ − θ̂, with id held, the same disturbance appears in estimated dq as:
        ê_d_apparent ≈ −E·sin(Δθ) ≈ −E·Δθ     (phase detector!)
        ê_q_apparent ≈  E·cos(Δθ) ≈  E

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
    sinusoidal αβ case. Native IPM handling via the active-flux form; an id transient tilts θ̂.
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