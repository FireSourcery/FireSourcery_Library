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
#include "Math/Fixed/fract16.h"


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
static inline fract16_t foc_pll_error(fract16_t e_alpha, fract16_t e_beta, fract16_t sin, fract16_t cos) { return fract16_sat(fract16_mul(e_alpha, cos) + fract16_mul(e_beta, sin)); }

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
        e_α =  ω·ψ·sin(θ),
        e_β = -ω·ψ·cos(θ)
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
static inline angle16_t foc_theta_of_emf(sign_t sign, fract16_t e_alpha, fract16_t e_beta) { return (sign >= 0) ? fract16_atan2(e_alpha, -e_beta) : fract16_atan2(-e_alpha, e_beta); }

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
static inline angle16_t foc_speed_of_emf(accum32_t Psi_pu, ufract16_t e_mag) { return (angle16_t)fract16_div(e_mag, Psi_pu); }


/*
    Turn base, ω_base = π / Ts
    G = Ts / L · (V_base/I_base)
*/
static inline accum32_t foc_g_pu_of_l_turn(uint32_t l_pu_turn) { return (uint64_t)FRACT16_PI * FRACT16_SCALE / l_pu_turn; }

/*
    1 / (L_pu · Fs / ω_base)
*/
static inline accum32_t foc_g_pu_of_l_pu(uint32_t angle_speed_base, uint32_t l_pu)
{
    return (uint64_t)angle_speed_base * FRACT16_SCALE * FRACT16_SCALE * ANGLE16_PER_REVOLUTION / ((uint64_t)ANGLE32_PER_RADIAN * l_pu);
}

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
static inline accum32_t foc_dq_emf_d(fract16_t Rs_pu, accum32_t omega_Lq, fract16_t vd, fract16_t id, fract16_t iq) { return (accum32_t)vd - fract16_mul(Rs_pu, id) - foc_vd_ff(omega_Lq, iq); }
static inline accum32_t foc_dq_emf_q(fract16_t Rs_pu, accum32_t omega_Lq, fract16_t vq, fract16_t id, fract16_t iq) { return (accum32_t)vq - fract16_mul(Rs_pu, iq) - foc_vq_ff(omega_Lq, 0, id); }


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
        zd = K_smo · sat( îd − id, thr )
        zq = K_smo · sat( îq − iq, thr )
        îd[k+1] = îd[k] + G_pu · ( vd − Rs·îd + ω·Lq·îq − zd )
        îq[k+1] = îq[k] + G_pu · ( vq − Rs·îq − ω·Lq·îd − zq )

        G_pu = dt · V_max / (Lq · I_max) = 1 / Lq_pu

    (zd, zq) drive (î − i) → 0; their LPF'd values (foc_lpf_step) are (ê_d, ê_q):
        ê_d → 0      at lock
        ê_q → ω·(ψ_f + (Ld−Lq)·id)

    Recover the position error via foc_eemf_dq_to_angle (atan2 on (ê_d, ê_q),
    no sin/cos needed — already in dq) or use ê_d directly as a PLL phase signal.

    Advantage over αβ SMO: signals are DC at lock, so the equivalent-control
    LPF cleans noise without the chatter/bandwidth tradeoff that plagues the
    sinusoidal αβ case. Native IPM handling via the EEMF formulation.
    Cost: closed-loop in θ̂ (observer depends on ω̂, θ̂) — convergence is harder
    to prove and requires reasonable startup alignment.
*/
/******************************************************************************/
// static inline fract16_t foc_eemf_zd(fract16_t K_smo, fract16_t thr, fract16_t id_est, fract16_t id) { return  smo_z(K_smo, thr, id_est, id); }
// static inline fract16_t foc_eemf_zq(fract16_t K_smo, fract16_t thr, fract16_t iq_est, fract16_t iq) { return  smo_z(K_smo, thr, iq_est, iq); }

// static inline fract16_t foc_smo_i_decouple(accum32_t G, fract16_t Rs, fract16_t v, accum32_t v_ff, fract16_t i_est, fract16_t z) { return  smo_i(G, Rs, ((accum32_t)v - v_ff), i_est, z); }
// /* vd_ff on iq_est */
// static inline fract16_t foc_eemf_id(accum32_t G_pu, fract16_t Rs_pu, fract16_t vd, accum32_t vd_ff, fract16_t id_est, fract16_t zd) { return smo_i_decouple(G_pu, Rs_pu, vd, vd_ff, id_est, zd); }
// /* vq_ff on id_est */
// static inline fract16_t foc_eemf_iq(accum32_t G_pu, fract16_t Rs_pu, fract16_t vq, accum32_t vq_ff, fract16_t iq_est, fract16_t zq) { return  smo_i_decouple(G_pu, Rs_pu, vq, vq_ff, iq_est, zq); }


/******************************************************************************/
/*!
    @brief  EEMF phase-error extraction (estimated dq frame).

    Takes the LPF'd EEMF (ê_d, ê_q) — typically foc_lpf_step on (zd, zq) from
    the observer output — and returns either the raw small-signal phase signal
    (ê_d), a magnitude-normalised phase signal (≈ sin(Δθ)), or the full angle
    correction Δθ to apply to θ̂.

    Sign of ê_q encodes rotation direction (positive for forward, negative for
    reverse). Caller supplies `sign` to disambiguate; pass +1 forward, -1 reverse.
*/
/******************************************************************************/
/*
    Normalized phase error ≈ −sin(θ − θ̂) in either direction, independent of |E|.
    |ê| strips the magnitude of E, not its sign: `sign` restores the direction, e.g. sign(ω̂).
    The signed ê_q would also restore it, but would make θ̂ = θ + π a second stable lock.
    Returns 0 when |ê| is below the noise floor (lock indeterminate).
*/
static inline fract16_t foc_eemf_dq_error_normalized(sign_t sign, ufract16_t e_floor, fract16_t ed, fract16_t eq)
{
    ufract16_t mag = fract16_vector_magnitude(ed, eq);
    return (mag <= e_floor) ? 0 : (fract16_t)(sign * fract16_div_sat(ed, mag));
}

/* Full angle correction Δθ to add to θ̂. Large-signal, no small-Δθ assumption. */
// static inline angle16_t foc_eemf_dq_to_angle(sign_t sign, fract16_t ed, fract16_t eq)
// {
//     return (sign >= 0) ? fract16_atan2(ed, eq) : fract16_atan2(-ed, -eq);
// }


