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
    @file   motor_params_math.h
    @author FireSourcery
    @brief  Pure math for PMSM electrical parameters — identification from raw
            measurements, unit conversion, and per-unit encoding for the
            control-loop basis.

    Parameters covered: Rs, Ls / Ld / Lq, Ke, ψ_f, Kv, Kt.

    Convention:
        T_em    = (3/2) · P · ψ_f · iq                  [Nm]
        Kt      = (3/2) · P · ψ_f                       [Nm/A]
        vq      = Rs·iq + Lq·diq/dt + ω_e·Ld·id + ω_e·ψ_f
        vd      = Rs·id + Ld·did/dt − ω_e·Lq·iq
        ω_elec  = P · ω_mech
        Ke_elec = ψ_f                                   [V·s/rad_elec, peak per-phase]
        Ke_mech = P · Ke_elec                           [V·s/rad_mech]
        Kv      = 60 / (2π · Ke_mech)                   [RPM/V, peak per-phase basis]

    Three representation domains:
        SI:    Ω, H, V·s/rad, Wb, Nm/A
        mU:    mΩ, µH, µV·s/rad, µWb, µNm/A     (integer storage, ~0.1% precision)
        PU:    fract16-scaled to the control basis (V_max, I_max, polling_freq)

    Per-unit encoding:
        i_pu  = I / I_max
        v_pu = V / V_max
        Rs_pu = Rs · I_max / V_max                              [≈ pu impedance]
        L_pu  = π · L · I_max · Fs / V_max                      [ω·L per angle16 step]
        ψ_pu  = π · ψ_f · Fs / V_max                            [BEMF per angle16 step]

    All PU coefficients are stored × FRACT16_SCALE so fract16_mul(angle16_step, X_pu)
    produces the v_pu drop directly. PU coefficients commonly exceed FRACT16_MAX
    for low-Kv / low-L motors — return type uint32 keeps them un-saturated.

    Pure functions: textbook formula, no guards, wider return preserves pre-sat value.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/angle_speed_math.h"

#include <stdint.h>
#include <assert.h>

/* Integer transcendentals × 1e6 for µ-unit arithmetic. */
#define MOTOR_PARAMS_2PI_uX   (6283185UL)
#define MOTOR_PARAMS_PI_uX    (3141593UL)
#define MOTOR_PARAMS_SQRT3_uX (1732051UL)
#define MOTOR_PARAMS_SQRT2_uX (1414214UL)


/******************************************************************************/
/*!
    @brief  Ke / ψ_f / Kv — Back-EMF and rotor flux

        ψ_f      [Wb]       = peak rotor flux linkage seen by one phase
        Ke_elec  [V·s/rad]  = peak per-phase BEMF per electrical rad/s = ψ_f
        Ke_mech  [V·s/rad]  = Ke_elec · P
        Kv       [RPM/V]    = 60 / (2π · Ke_mech)               (peak per-phase basis)

    Identification:
        Spin test (generator at known speed):
            ψ_f = V_phase_pk / ω_elec
                = V_LL_pk / (√3 · ω_elec)
        Steady-state FOC:
            ψ_f = (vq − Rs·iq − ω_e·Ld·id) / ω_e

    PU encoding:  ψ_pu × FRACT16_SCALE = π · ψ_f · Fs / V_max
        fract16_mul(el_delta_angle16, ψ_pu) → BEMF in v_pu.
*/
/******************************************************************************/
static inline uint32_t psi_pu_of_wb(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)FRACT16_PI * psi_Wb * polling_freq / ((uint64_t)v_max_V * scale); }

static inline uint32_t psi_pu_of_uwb(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_uWb) { return (uint64_t)MOTOR_PARAMS_PI_uX * psi_uWb * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_V * 1000000UL * 1000000UL); }
static inline uint32_t psi_uwb_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_pu) { return (uint64_t)psi_pu * v_max_V * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_PI_uX * polling_freq * FRACT16_SCALE); }

static inline uint32_t psi_pu_of_mvrads(uint32_t polling_freq, uint16_t v_max_volts, uint32_t mV_per_rads) { return ((uint64_t)MOTOR_PARAMS_PI_uX * mV_per_rads * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_volts * 1000000UL * 1000UL)); }

/* ψ_f[µWb] = V[mV]·1e6 / ω[mrad/s]      (mV·s/mrad ≡ Wb; ×1e6 → µWb) */
static inline uint32_t psi_uwb_of_emf_peak(uint32_t omega_elec_mrads, uint32_t v_phase_mV_pk) { return (uint64_t)v_phase_mV_pk * 1000000UL / omega_elec_mrads; }
static inline uint32_t psi_uwb_of_emf_ll_peak(uint32_t omega_elec_mrads, uint32_t v_ll_mV_pk) { return (uint64_t)v_ll_mV_pk * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_SQRT3_uX * omega_elec_mrads); }

/* Online ψ_f under steady-state FOC. Numerator built in µV (mΩ·mA = µV exactly).
   Caller ensures vq·1e3 ≥ v_r + v_l (physically required for ψ > 0). */
static inline uint32_t psi_uwb_of_running(uint32_t rs_mOhm, uint32_t ld_uH, uint32_t omega_e_mrads, uint32_t vq_mV, uint32_t id_mA, uint32_t iq_mA)
{
    uint64_t v_r = (uint64_t)rs_mOhm * iq_mA;                                    /* µV */
    uint64_t v_l = (uint64_t)omega_e_mrads * ld_uH * id_mA / 1000000UL;          /* µV */
    return ((uint64_t)vq_mV * 1000UL - v_r - v_l) * 1000UL / omega_e_mrads;      /* µWb */
}

// static inline uint32_t psi_uwb_of_kv(uint16_t kv_rpm_per_V, uint8_t polePairs) { return (uint64_t)60UL * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_2PI_uX * kv_rpm_per_V * polePairs); }
// static inline uint16_t kv_of_psi_uwb(uint32_t psi_uWb, uint8_t polePairs) { return (uint64_t)60UL * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_2PI_uX * psi_uWb * polePairs); }

/* PU identification — direct from FOC-loop quantities. Spin-test form is the
   literal inverse of the runtime BEMF computation fract16_mul(omega_step, psi_pu).
       ψ_pu = v_emf_pu · 32768 / omega_step  =  fract16_div(v_emf_pu, omega_step)   [spin test]
       ψ_pu = residual_v_pu / omega_step,  residual = vq − Rs·iq − ω·Ld·id          [online] */
static inline uint32_t psi_pu_of_emf(fract16_t v_emf_pu, fract16_t omega_step) { return fract16_div(v_emf_pu, omega_step); }

static inline uint32_t psi_pu_of_running(fract16_t rs_pu, fract16_t ld_pu, fract16_t omega_step, fract16_t vq_pu, fract16_t id_pu, fract16_t iq_pu)
{
    fract16_t omega_Ld = fract16_mul(omega_step, ld_pu);
    fract16_t residual = fract16_sat((accum32_t)vq_pu - fract16_mul(rs_pu, iq_pu) - fract16_mul(omega_Ld, id_pu));
    return fract16_div(residual, omega_step);
}

/******************************************************************************/
/*!
    @brief  Kv-based storage — {Kv, polePairs} as the canonical motor parameter.

    Direct Kv-keyed derivations skip the µWb intermediate:
        ψ_pu_stored = 30 · Fs · 32768 / (V_max · Kv · P)
        Kt_FOC      = 90 / (2π · Kv) · 1e6  µNm/A    (pole pairs cancel)
        rpm         = Kv · V_phase_pk                 (linear at any v ≤ V_max)
        rpm_max     = Kv · V_max                      (no-load top speed at v_pu = 1)

    Vendor-spec basis conversions (vendor → internal phase-peak):
        Kv_phase_pk = √3 · Kv_LL_pk     (V_LL_pk = √3 · V_phase_pk)
        Kv_pk       = Kv_rms / √2       (V_pk    = √2 · V_rms)

    Identities derived from (Kv, P):
        Ke_mech = 60 / (2π · Kv)                             [V·s/rad_mech]
        Ke_elec = ψ_f = Ke_mech / P                          [V·s/rad_elec = Wb]
        ψ_pu    = π · ψ_f · Fs / V_max
                = 30 · Fs / (Kv · P · V_max)                 (× FRACT16_SCALE; π cancels)
        Kt_FOC  = (3/2) · P · ψ_f = 45 / (π · Kv)            [Nm/A]   (P cancels)
        Kt_mtr  = 60 / (2π · Kv) = Ke_mech                   [Nm/A_rms]

    Direct PU shortcuts collapse the µWb intermediate and (for ψ_pu) cancel π:
        ψ_pu × FRACT16_SCALE = π·ψ_f·Fs/V_max   = 30·Fs / (Kv·P·V_max) × FRACT16_SCALE
*/
/*
    Kv [RPM/V]
    Kv [RPM/V, peak per-phase] is the most commonly published rotor parameter.
    Storing (Kv, P) instead of (ψ_uWb, P) or (ψ_pu, P) avoids µ-unit redundancy and matches how datasheets list motors —
    Storing {Kv, V_max, I_max, Fs, P} fully determines the rotor half of the
    model; only Rs and Ld/Lq need independent storage.
*/
/******************************************************************************/
/* Speed envelope from Kv. */
static inline uint32_t rpm_of_kv_v(uint16_t kv, uint32_t v_V) { return (uint32_t)kv * v_V; }
static inline uint32_t v_of_kv_rpm(uint16_t kv, uint32_t rpm) { return rpm / kv; }

/* Kv basis conversions — parse vendor specs into internal phase-peak. */
// static inline uint16_t kv_phase_pk_of_kv_ll_pk(uint16_t kv_ll_pk) { return (uint64_t)kv_ll_pk * MOTOR_PARAMS_SQRT3_uX / 1000000UL; }
// static inline uint16_t kv_ll_pk_of_kv_phase_pk(uint16_t kv_phase_pk) { return (uint64_t)kv_phase_pk * 1000000UL / MOTOR_PARAMS_SQRT3_uX; }
// static inline uint16_t kv_pk_of_kv_rms(uint16_t kv_rms) { return (uint64_t)kv_rms * 1000000UL / MOTOR_PARAMS_SQRT2_uX; }
// static inline uint16_t kv_rms_of_kv_pk(uint16_t kv_pk) { return (uint64_t)kv_pk * MOTOR_PARAMS_SQRT2_uX / 1000000UL; }

/* [V / (Rad/s)] base. generic for caller compose  */
static inline uint32_t ke_vrads_of(uint16_t kv, uint32_t scale) { return ((uint64_t)scale * FRACT16_SCALE * 60 / (2 * FRACT16_PI)) / (kv); }
static inline uint32_t psi_vrads_of(uint16_t kv, uint8_t polePairs, uint32_t scale) { return ke_vrads_of(kv, scale) / polePairs; }
static inline uint32_t ke_mvrads_of_kv(uint16_t kv) { return ke_vrads_of(kv, 1000); }
// static inline uint32_t ke_mech_uvsrad_of_kv(uint16_t kv_rpm_per_V) { return (uint64_t)60UL * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_2PI_uX * kv_rpm_per_V); }
// static inline uint32_t ke_elec_uvsrad_of_kv(uint16_t kv_rpm_per_V, uint8_t polePairs) { return ke_mech_uvsrad_of_kv(kv_rpm_per_V) / polePairs; }

static inline uint32_t ke_pu_of_kv(uint32_t polling_freq, uint16_t v_max_V, uint16_t kv) { return (uint64_t)30UL * polling_freq * FRACT16_SCALE / ((uint32_t)kv * v_max_V); }

/* Ke_pu when speed full-scale is set independently of Kv·V_max:
       Ke_pu = v_emf_pu / speed_pu = speed_max / (Kv·V_max)
   Returns 1.0 (FRACT16_MAX) when speed_max = Kv·V_max (natural basis). */
static inline uint32_t ke_pu_speed_of_kv(uint16_t v_max_V, uint32_t speed_max_rpm, uint16_t kv) { return fract16_div(speed_max_rpm, rpm_of_kv_v(kv, v_max_V)); }

/*!
    The direct Kv→PU forms below are exact-integer (π cancels), so they are
    both cheaper and slightly more accurate than chaining through ψ_uWb.
*/
/* ψ_pu × FRACT16_SCALE directly from (Kv, P) — π cancels, no µWb intermediate. */
/* ψ_pu directly from Kv. ψ_pu = π·ψ_f·Fs/V_max·32768 with ψ_f = 60/(2π·Kv·P) reduces to 30·Fs·32768/(V·Kv·P). */
static inline uint32_t psi_pu_of_kv(uint32_t polling_freq, uint16_t v_max_V, uint16_t kv, uint8_t polePairs) { return (uint64_t)30UL * polling_freq * FRACT16_SCALE / ((uint32_t)kv * polePairs * v_max_V); }
static inline uint16_t kv_of_psi_pu(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_pu, uint8_t polePairs) { return (uint64_t)30UL * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_V * polePairs * psi_pu); }

// static inline accum32_t kt_nm_per_amp_of(uint16_t kv, scale) { return ((uint64_t)60 * FRACT16_SQRT3_DIV_2 * FRACT16_SCALE / kv * 2 * FRACT16_PI); }

/* FOC directly from Kv — pole pairs cancel: Kt = 1.5·P·ψ_f = 1.5·P·60/(2π·Kv·P) = 1.5·Ke_mech. */
// static inline uint32_t kt_unm_per_a_of_kv(uint16_t kv_rpm_per_V) { return 3UL * ke_mech_uvsrad_of_kv(kv_rpm_per_V) / 2UL; }
// static inline uint32_t kt_rms_unm_per_a_of_kv(uint16_t kv_rpm_per_V) { return ke_mech_uvsrad_of_kv(kv_rpm_per_V); }

/******************************************************************************/
/*!
    @brief  Kt — Torque Constant [Nm/A]

        FOC peak-iq:        Kt = (3/2) · P · ψ_f          T_em = Kt · iq
        Motor-constant:     Kt [Nm/A_rms] = 60 / (2π · Kv) = Ke_mech (numerically)
*/
/******************************************************************************/
static inline uint32_t kt_unm_per_a_of_psi(uint32_t psi_uWb, uint8_t polePairs) { return 3UL * psi_uWb * polePairs / 2UL; }



/******************************************************************************/
/*!
    @brief  Rs — Stator Resistance [Ω], per phase

    DC injection (di/dt = 0, EMF = 0 at standstill):
        Rs       = V / I                    (single conductor)
        Rs_phase = V_LL / (2·I)             (Y, two phases in series)

    Online under FOC DC excitation (iq=0, id=id_ref, ω=0):
        Rs_pu = vd_pu / id_pu               (ratio invariant under PU)

    PU encoding:  Rs_pu × FRACT16_SCALE = Rs · I_max / V_max
*/
/******************************************************************************/
#define MOTOR_R_FRACT16(V_Scale, I_Scale, R_Ohm, SI_Scale) ((R_Ohm) * I_Scale * FRACT16_SCALE / ((uint64_t)V_Scale * SI_Scale))
#define MOTOR_R_FRACT16_OF_MOHM(V_Scale, I_Scale, R_mOhm) MOTOR_R_FRACT16(V_Scale, I_Scale, R_mOhm, 1000UL)

// static inline uint32_t rs_mohm_of_vi(uint32_t v_mV, uint32_t i_mA) { return (uint64_t)v_mV * 1000UL / i_mA; }
static inline uint32_t rs_mohm_of_vi_ll_y(uint32_t v_mV_ll, uint32_t i_mA) { return (uint64_t)v_mV_ll * 1000UL / (2UL * i_mA); }

static inline uint32_t rs_pu_of_mohm(uint16_t v_max_V, uint16_t i_max_A, uint32_t rs_mOhm) { return (uint64_t)rs_mOhm * i_max_A * FRACT16_SCALE / ((uint32_t)v_max_V * 1000UL); }
static inline uint32_t rs_mohm_of_pu(uint16_t v_max_V, uint16_t i_max_A, uint32_t rs_pu)   { return (uint64_t)rs_pu * v_max_V * 1000UL / ((uint32_t)i_max_A * FRACT16_SCALE); }

static inline fract16_t rs_pu_of_vd_id(fract16_t vd_pu, fract16_t id_pu) { return fract16_div_sat(vd_pu, id_pu); }

/******************************************************************************/
/*!
    @brief  Ls / Ld / Lq — Stator Inductance [H], per phase

        Ls = Ld = Lq for SPM; Ld ≠ Lq for IPM (rotor alignment selects axis).

    Identification:
        HFI (ω·L ≫ R):  L = V_pk / (2π·f·I_pk)
        RL τ (step):    L = Rs · τ           (τ = 63.2% rise time)
        Voltage step:   L = V · Δt / Δi      (initial linear ramp, R drop ≪)

    PU encoding:  L_pu × FRACT16_SCALE = π · L · I_max · Fs / V_max
        fract16_mul(el_delta_angle16, L_pu) → ω_e·L·i_pu term in v_pu.
        The π factor absorbs the 2π·Fs/65536 angle16-step → rad/s conversion.
*/
/******************************************************************************/
#define MOTOR_L_FRACT16(Fs, V_Scale, I_Scale, L_Henries, SI_Scale) ((L_Henries) * I_Scale * Fs * FRACT16_PI / ((uint64_t)V_Scale * SI_Scale))
#define MOTOR_L_FRACT16_OF_UH(Fs, V_Scale, I_Scale, L_uHenries) MOTOR_L_FRACT16(Fs, V_Scale, I_Scale, L_uHenries, 1000000UL)

/* l_pu < UINT64_MAX / (v_max_V × scale) */
static inline uint32_t l_h_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_pu, uint32_t scale) { return (uint64_t)l_pu * v_max_V * scale / ((uint64_t)FRACT16_PI * i_max_A * polling_freq); }
static inline uint32_t l_pu_of_h(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_h, uint32_t scale) { return (uint64_t)l_h * i_max_A * polling_freq * FRACT16_PI / ((uint64_t)v_max_V * scale); }

// static inline uint32_t l_uh_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_pu) { return (uint64_t)l_pu * v_max_V * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_PI_uX * i_max_A * polling_freq * FRACT16_SCALE); }
// static inline uint32_t l_pu_of_uh(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_uH) { return (uint64_t)MOTOR_PARAMS_PI_uX * l_uH * i_max_A * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_V * 1000000UL * 1000000UL); }

// static inline uint32_t l_uh_of_hfi(uint32_t f_Hz, uint32_t v_mV_pk, uint32_t i_mA_pk) { return (uint64_t)v_mV_pk * 1000000UL * 1000000UL / ((uint64_t)MOTOR_PARAMS_2PI_uX * f_Hz * i_mA_pk); }
// static inline uint32_t l_uh_of_rs_tau_us(uint32_t rs_mOhm, uint32_t tau_us) { return (uint64_t)rs_mOhm * tau_us / 1000UL; }
// static inline uint32_t l_uh_of_rs_tau_cycles(uint32_t polling_freq, uint32_t rs_mOhm, uint32_t tau_cycles) { return (uint64_t)rs_mOhm * tau_cycles * 1000UL / polling_freq; }
// static inline uint32_t l_uh_of_step(uint32_t v_mV, uint32_t di_mA, uint32_t dt_us) { return (uint64_t)v_mV * dt_us / di_mA; }

/* PU identification — direct from FOC-loop quantities. Output uint32 (may exceed FRACT16_MAX).
       L_pu = (v/i)·Fs/(2f)·32768  =  fract16_div(v, i) · Fs/(2f)               [HFI]
       L_pu = Rs_pu · π · τ_cycles =  fract16_mul(Rs_pu, FRACT16_PI) · τ_cyc    [RL τ]
       L_pu = (v/di)·π·n_cycles·32768                                           [step] */
static inline uint32_t l_pu_of_hfi(uint32_t polling_freq, uint32_t f_Hz, ufract16_t v_pk_pu, ufract16_t i_pk_pu) { return (uint64_t)fract16_div(v_pk_pu, i_pk_pu) * polling_freq / (2UL * f_Hz); }
static inline uint32_t l_pu_of_rs_tau_cycles(fract16_t rs_pu, uint32_t tau_cycles) { return (uint64_t)fract16_mul(rs_pu, FRACT16_PI) * tau_cycles; }
static inline uint32_t l_pu_of_step(fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return (uint64_t)fract16_mul(fract16_div(v_pu, di_pu), FRACT16_PI) * dt_cycles; }



/******************************************************************************/
/*!
    @brief  Phase-basis amplitude conversions (Y-connected motors).

        V_phase_pk = V_LL_pk / √3        V_LL_pk = √3 · V_phase_pk
        V_pk       = √2 · V_rms          V_rms   = V_pk / √2
*/
/******************************************************************************/
static inline uint32_t v_phase_peak_of_ll_peak(uint32_t v_ll_pk) { return (uint64_t)v_ll_pk * 1000000UL / MOTOR_PARAMS_SQRT3_uX; }
static inline uint32_t v_ll_peak_of_phase_peak(uint32_t v_phase_peak) { return (uint64_t)v_phase_peak * MOTOR_PARAMS_SQRT3_uX / 1000000UL; }
static inline uint32_t v_peak_of_rms(uint32_t v_rms) { return (uint64_t)v_rms * MOTOR_PARAMS_SQRT2_uX / 1000000UL; }
static inline uint32_t v_rms_of_peak(uint32_t v_peak) { return (uint64_t)v_peak * 1000000UL / MOTOR_PARAMS_SQRT2_uX; }


/******************************************************************************/
/*!
    @brief  Voltage-Model Back-EMF Estimator

        e = v - Rs·i - Ls·di/dt

    Discrete one-step form, caller supplies the precomputed Δi = i[k] - i[k-1]:
        ê[k] = v[k] - Rs_pu·i[k] - Ls_pu·Δi[k]
*/
/******************************************************************************/
static inline accum32_t motor_v_stator(fract16_t Rs_pu, fract16_t Ls_pu, fract16_t i_prev, fract16_t i) { return fract16_mul(Rs_pu, i) + fract16_mul(Ls_pu, i_prev - i); }
static inline accum32_t motor_emf(fract16_t Rs_pu, fract16_t Ls_pu, fract16_t i_prev, fract16_t i, fract16_t v) { return v - motor_v_stator(Rs_pu, Ls_pu, i_prev, i); }
