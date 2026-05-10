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
            PLL phase detector, and EMF-to-angle/speed extraction.

    Stateless per-tick step functions that compose into a sensorless rotor
    position observer; state is owned by the caller. All signals share the
    fract16 per-unit basis used by foc_math.h / motor_params_math.h:

        v_pu   = V / V_max                  i_pu   = I / I_max
        Rs_pu  = Rs · I_max / V_max
        Ls_pu = Ls · I_max · Fs / V_max            (multiplies Δi_pu → v_pu)
        Psi_pu = ψ_f · π · Fs · 32768 / V_max       (psi_vfract16_per_angle16)

    The αβ stationary frame is the working frame for sensorless observers —
    rotor position is the unknown, so dq is unavailable until lock.

    This file does not include startup support (open-loop I/F ramp) or zero/
    low-speed saliency-based estimation (HFI); those are separate algorithms
    that live alongside, not inside, the observer.
*/
/******************************************************************************/
#include "foc_math.h"
#include "Math/Fixed/fract16.h"

/* αβ stationary-frame pair (mirrors foc_dq / foc_abc). */
struct foc_alphabeta { fract16_t alpha, beta; };


/******************************************************************************/
/*!
    @brief  Voltage-Model Back-EMF Estimator (αβ stationary frame)

        e = v - Rs·i - Ls·di/dt

    Discrete one-step form, caller supplies the precomputed Δi = i[k] - i[k-1]:
        ê[k] = v[k] - Rs_pu·i[k] - Ls_pu·Δi[k]

    Output ê is in v_pu units. Recover angle via foc_pll_error + integrator,
    or foc_emf_to_angle (atan2). Accuracy degrades at low speed because |e| → 0;
    use SMO + LPF for noisier low-speed regimes, and switch to open-loop / HFI
    starting near ω_e = 0.
*/
/******************************************************************************/
// static inline fract16_t foc_emf_axis(fract16_t Rs_pu, fract16_t Ls_pu, fract16_t v, fract16_t i_prev, fract16_t i)
// {
//     return fract16_sat((accum32_t)v - fract16_mul(Rs_pu, i) - fract16_mul(Ls_pu, i_prev - i));
// }

static inline fract16_t foc_emf_axis(fract16_t Rs_pu, fract16_t Ls_pu, fract16_t v, fract16_t i, fract16_t di)
{
    return fract16_sat((accum32_t)v - fract16_mul(Rs_pu, i) - fract16_mul(Ls_pu, di));
}

static inline struct foc_alphabeta foc_emf_alphabeta
(
    fract16_t Rs_pu, fract16_t Ls_pu,
    fract16_t v_alpha, fract16_t v_beta,
    fract16_t i_alpha, fract16_t i_beta,
    fract16_t di_alpha, fract16_t di_beta
)
{
    return (struct foc_alphabeta)
    {
        .alpha = foc_emf_axis(Rs_pu, Ls_pu, v_alpha, i_alpha, di_alpha),
        .beta = foc_emf_axis(Rs_pu, Ls_pu, v_beta, i_beta, di_beta),
    };
}


/******************************************************************************/
/*!
    @brief  Sliding-Mode switching function — bounded sign with linear region.

        sat(x, thr) = clamp(x / thr, -1, +1)

    Replaces hard sign() inside the SMO to suppress chatter; thr is the
    boundary-layer width in the same per-unit basis as x. thr > 0.
*/
/******************************************************************************/
static inline fract16_t foc_smo_sat(fract16_t x, fract16_t thr)
{
    return fract16_sat(fract16_div(x, thr));
}


static inline fract16_t foc_smo_z(fract16_t K_smo, fract16_t thr, fract16_t i_est, fract16_t i_meas)
{
    return fract16_mul(K_smo, fract16_sat(fract16_div(i_est - i_meas, thr)));
}

static inline fract16_t foc_smo_i(fract16_t G_int_pu, fract16_t Rs_pu, fract16_t v, fract16_t i_est, fract16_t i_meas, fract16_t z)
{
    fract16_t v_eff = fract16_sat((accum32_t)v - fract16_mul(Rs_pu, i_est) - z);
    return fract16_sat((accum32_t)i_est + fract16_mul(G_int_pu, v_eff));
}


/******************************************************************************/
/*!
    @brief  Sliding-Mode Current Observer — single-axis step (αβ frame).

    Plant (per axis):
        di/dt = (Ls)⁻¹ · ( v - Rs·i - e )

    Discrete observer:
        z[k]    = K_smo · sat( î[k] - i_meas[k], thr )
        î[k+1]  = î[k] + G_int · ( v[k] - Rs_pu·î[k] - z[k] )

        G_int = dt · V_max / (Ls · I_max) = 1 / Ls_pu

    The switching variable z drives (î - i) → 0; its slow component
    (extracted by foc_lpf_step) is the back-EMF estimate ê used by the
    angle tracker.

    @param  G_int_pu  observer integrator gain = 1 / Ls_pu
    @param  K_smo     sliding gain; > peak EMF in pu so z dominates ê
    @param  thr       boundary-layer width (foc_smo_sat); typ. 0.05..0.2 of i_max
*/
/******************************************************************************/
struct foc_smo_axis { fract16_t i_est, z; };

static inline struct foc_smo_axis foc_smo_axis_step
(
    fract16_t Rs_pu, fract16_t G_int_pu, fract16_t K_smo, fract16_t thr,
    fract16_t i_est, fract16_t i_meas, fract16_t v
)
{
    fract16_t z = fract16_mul(K_smo, foc_smo_sat(i_est - i_meas, thr));
    fract16_t v_eff = fract16_sat((accum32_t)v - fract16_mul(Rs_pu, i_est) - z);
    return (struct foc_smo_axis)
    {
        .i_est = fract16_sat((accum32_t)i_est + fract16_mul(G_int_pu, v_eff)),
        .z = z,
    };
}

/* αβ-vector form of the SMO step. */
struct foc_smo_alphabeta { fract16_t i_alpha, i_beta, z_alpha, z_beta; };

static inline struct foc_smo_alphabeta foc_smo_alphabeta_step
(
    fract16_t Rs_pu, fract16_t G_int_pu, fract16_t K_smo, fract16_t thr,
    fract16_t i_alpha_est, fract16_t i_beta_est,
    fract16_t i_alpha, fract16_t i_beta,
    fract16_t v_alpha, fract16_t v_beta
)
{
    struct foc_smo_axis a = foc_smo_axis_step(Rs_pu, G_int_pu, K_smo, thr, i_alpha_est, i_alpha, v_alpha);
    struct foc_smo_axis b = foc_smo_axis_step(Rs_pu, G_int_pu, K_smo, thr, i_beta_est,  i_beta,  v_beta);
    return (struct foc_smo_alphabeta) { .i_alpha = a.i_est, .i_beta = b.i_est, .z_alpha = a.z, .z_beta = b.z };
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
static inline fract16_t foc_lpf_step(fract16_t y, fract16_t x, fract16_t k_lp)
{
    return fract16_sat((accum32_t)y + fract16_mul(k_lp, x - y));
}


/******************************************************************************/
/*!
    @brief  PLL Phase Detector — d-axis projection of estimated EMF.

    With d aligned to rotor flux ψ_f, back-EMF lies on the q-axis, so:
        e_d  =   e_α·cos(θ̂) + e_β·sin(θ̂)
        e_q  =  -e_α·sin(θ̂) + e_β·cos(θ̂)
    e_d → 0 at lock; near lock e_d ≈ |e|·sin(θ - θ̂) ≈ |e|·Δθ.

    Sign of e_d flips with rotation direction (|e| ∝ ω). Caller-side PI/integrator
    must be polarity-aware; for bidirectional operation feed sign(ω̂)·e_d or use
    foc_pll_error_normalized which strips |e|.
*/
/******************************************************************************/
static inline fract16_t foc_pll_error(fract16_t e_alpha, fract16_t e_beta, fract16_t sin, fract16_t cos)
{
    return fract16_sat(fract16_mul(e_alpha, cos) + fract16_mul(e_beta, sin));
}

/*!
    Speed-normalized phase error ≈ sin(Δθ), independent of |e|.
    Returns 0 when |e| is below the noise floor (lock indeterminate).
*/
static inline fract16_t foc_pll_error_normalized(ufract16_t e_floor, fract16_t e_alpha, fract16_t e_beta, fract16_t sin, fract16_t cos)
{
    ufract16_t mag = fract16_vector_magnitude(e_alpha, e_beta);
    return (mag <= e_floor) ? 0 : (fract16_t)fract16_div(foc_pll_error(e_alpha, e_beta, sin, cos), mag);
}


/******************************************************************************/
/*!
    @brief  Direct rotor angle from estimated back-EMF (atan2-based).

    For CCW rotation (ω > 0), with d aligned to rotor flux:
        e_α =  ω·ψ·sin(θ),     e_β = -ω·ψ·cos(θ)
    therefore:
        θ = atan2( e_α, -e_β)               (ω > 0)
        θ = atan2(-e_α,  e_β)               (ω < 0)

    Caller supplies `sign` of the rotation (e.g. from PLL ω̂ or open-loop ramp);
    pass +1 forward, -1 reverse.

    More accurate than the small-signal PLL detector but has no inherent
    smoothing — pair with foc_lpf_step on e_αβ before calling for noisy
    SMO output.
*/
/******************************************************************************/
static inline angle16_t foc_emf_to_angle(sign_t sign, fract16_t e_alpha, fract16_t e_beta)
{
    return (sign >= 0) ? fract16_atan2(e_alpha, -e_beta) : fract16_atan2(-e_alpha, e_beta);
}


/******************************************************************************/
/*!
    @brief  Electrical speed estimate from EMF magnitude.

        |e| = |ω_e| · ψ_f       →       |ω̂_e| = |e| / ψ_f

    With Psi_pu per psi_vfract16_per_angle16, the result is the electrical
    angle increment per control cycle:

        |Δθ̂_e| [angle16/poll] = |e_pu| / Psi_pu

    Sign of ω is not recoverable from |e|; combine with foc_pll_error sign
    or the open-loop reference. Returns 0 when Psi_pu is zero (uninitialised).
*/
/******************************************************************************/
static inline angle16_t foc_speed_of_emf(fract16_t Psi_pu, ufract16_t e_mag)
{
    return (Psi_pu == 0) ? 0 : (angle16_t)fract16_div(e_mag, Psi_pu);
}
