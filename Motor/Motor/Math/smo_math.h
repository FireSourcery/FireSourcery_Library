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
    @brief  Sliding-mode correction laws.

    The innovation σ = î − i, the model's estimate against the measurement, drives a correction z
    that takes the place of the unknown disturbance in the model. The model is the caller's.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"



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