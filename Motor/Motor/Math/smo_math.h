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
    @file   smo_math.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"


static inline accum32_t smo_g_pu_rpm(uint32_t polling_freq, uint32_t speed_max_rpm, uint32_t polePairs, uint32_t l_pu)
{
    return (uint64_t)FRACT16_PI * polePairs * speed_max_rpm  / ((uint64_t)30UL * polling_freq * l_pu);
}

static inline accum32_t smo_g_pu_of_erads(uint32_t polling_freq, uint32_t erads, uint32_t l_pu)
{
    return (uint64_t)erads * FRACT16_SCALE / (polling_freq * l_pu);
}

static inline accum32_t smo_g_pu_of_angle(uint32_t polling_freq, uint32_t angle, uint32_t l_pu)
{
    return (uint64_t)angle * FRACT16_SCALE / (l_pu);
}

/******************************************************************************/
/*!
    @brief  Sliding-Mode switching function — bounded sign with linear region.

        sat(x, thr) = clamp(x / thr, -1, +1)

    Replaces hard sign() inside the SMO to suppress chatter; thr is the
    boundary-layer width in the same per-unit basis as x. thr > 0.
*/
/******************************************************************************/
static inline fract16_t smo_sat(fract16_t thr, fract16_t x) { return fract16_sat(fract16_div(x, thr)); }

/******************************************************************************/
/*!
    @brief  Sliding-Mode Current Observer — single-axis step (αβ frame).

    Plant (per axis):
        di/dt = (Ls)⁻¹ · ( v - Rs·i - e )

    Discrete observer:
        z[k]    = K_smo · sat( î[k] - i_meas[k], thr )
        î[k+1]  = î[k] + G_int · ( v[k] - Rs_pu·î[k] - z[k] )

        G_int = dt · V_max / (Ls · I_max) = 1 / Ls_pu_angle16

    The switching variable z drives (î - i) → 0; its slow component
    (extracted by lpf_step) is the back-EMF estimate ê used by the
    angle tracker.

    @param  G_pu        bserver integrator gain = 1 / Ls_pu
    @param  K_smo     sliding gain; > peak EMF in pu so z dominates ê
    @param  thr       boundary-layer width (smo_sat); typ. 0.05..0.2 of i_max
*/
/******************************************************************************/
static inline fract16_t smo_z(fract16_t K_smo, fract16_t thr, fract16_t i_est, fract16_t i_meas) { return fract16_mul(K_smo, smo_sat(thr, i_est - i_meas)); }
static inline accum32_t smo_v_eff(fract16_t Rs_pu, accum32_t v, fract16_t i_est, fract16_t z) { return ((accum32_t)v - fract16_mul(Rs_pu, i_est) - z); }
// /* if G_pu > 1.0 */
static inline accum32_t _smo_i(accum32_t G_pu, fract16_t Rs_pu, accum32_t v, fract16_t i_est, fract16_t z) { return accum32_mul(G_pu, smo_v_eff(Rs_pu, v, i_est, z)); }
// static inline fract16_t smo_i(accum32_t G_pu, fract16_t Rs_pu, accum32_t v, fract16_t i_est, fract16_t z) { return fract16_sat((accum32_t)i_est + fract16_mul(G_pu, smo_v_eff(Rs_pu, v, i_est, z))); }
static inline fract16_t smo_i(accum32_t G_pu, fract16_t Rs_pu, accum32_t v, fract16_t i_est, fract16_t z) { return fract16_sat((accum32_t)i_est + _smo_i(G_pu, Rs_pu, v, i_est, z)); }

struct smo_axis { fract16_t i_est, z; };

static inline struct smo_axis smo_axis_step(fract16_t K_smo, fract16_t thr, accum32_t G_pu, fract16_t Rs_pu, accum32_t v, fract16_t i_est, fract16_t i_meas)
{
    fract16_t z = smo_z(K_smo, thr, i_est, i_meas);
    return (struct smo_axis) { .i_est = smo_i(G_pu, Rs_pu, v, i_est, z), .z = z, };
}

/******************************************************************************/
/*!
    @brief  First-order discrete low-pass step.

        y[k+1] = y[k] + k_lp · ( x[k] - y[k] ),       k_lp = dt / (τ + dt)

    Used to extract the equivalent control of z (≈ ê) from the SMO output and
    to smooth ê before the angle tracker. Choose τ such that the cutoff sits
    above the maximum tracked electrical frequency yet below the SMO chatter band.
*/
/******************************************************************************/
static inline fract16_t lpf_step(fract16_t k_lp, fract16_t y, fract16_t x) { return fract16_sat((accum32_t)y + fract16_mul(k_lp, x - y)); }



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
static inline fract16_t sta_sqrt_signed(fract16_t x)
{
    fract16_t mag_sqrt = (fract16_t)fract16_sqrt(fract16_abs_sat(x));
    return (x >= 0) ? mag_sqrt : -mag_sqrt;
}

/* STA continuous switching output. w is the current integrator state. */
static inline fract16_t sta_z(fract16_t lambda, fract16_t w, fract16_t sigma) { return fract16_sat((accum32_t)w + fract16_mul(lambda, sta_sqrt_signed(sigma))); }

/* STA integrator update: w[k+1] = w[k] + (α·dt) · sign(σ). */
static inline fract16_t sta_w_step(fract16_t alpha_dt, fract16_t w, fract16_t sigma) { return fract16_sat((accum32_t)w + (accum32_t)alpha_dt * math_sign(sigma)); }

struct sta_axis { fract16_t z, w; };

/* Combined STA single-axis step — yields (z, w_next) from (w, î, i_meas). */
static inline struct sta_axis sta_axis_step(fract16_t lambda, fract16_t alpha_dt, fract16_t w, fract16_t i_est, fract16_t i_meas)
{
    fract16_t sigma = fract16_sat((accum32_t)i_est - i_meas);
    return (struct sta_axis) { .z = sta_z(lambda, w, sigma), .w = sta_w_step(alpha_dt, w, sigma), };
}