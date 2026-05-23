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

    Three representation domains:
        SI :    Ω, H, V·s/rad, Wb, Nm/A
        SI storage:    mΩ, µH, µV·s/rad, µWb, µNm/A     (integer storage, ~0.1% precision)

    Pure functions: textbook formula, no guards, wider return preserves pre-sat value.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/angle_speed_math.h"

#include <stdint.h>
#include <assert.h>


/*
    base in SI
    Anchor for PU of 1.0f (PU exceeds 1.0 when value exceeds base)
    V_base [V]
    I_base [A]
    ω_base [rad/S electrical]
    ω_base [eRPM] = ω_base [rad/S] · 60 / 2π
    ω_base(kv) [rad/S] = (2π · p · Kv · V_base) / 60

    Derived (PU conditionally > 1.0):
    R_base [Ω]   = V_base / I_base
    R_pu = R / (V_base / I_base)

    T_base [N·m] = (3/2) · p · ψ_base · I_base
    P_base [W]   = (3/2) · V_base · I_base

    ψ and L are dependent on ω_base:
    ψ_base [Wb] = V_base / ω_base,  v = ω·ψ
    L_base [H]  = ψ_base / I_base = V_base / (ω_base · I_base)

    ω_pu = ω / ω_base
    ψ_pu = ψ / ψ_base   = ψ · ω_base / V_base
    L_pu = L / L_base   = L · I_base / ψ_base    = L · ω_base · I_base / V_base

    ψ == v_emf/ω:
    ψ_pu = (v_emf / ω) · (ω_base / V_base) = (v_emf / V_base) / (ω / ω_base)

    L == v_L / (ω · I):
    L_pu = (v_L / (ω · I)) · (ω_base · I_base / V_base)

    ff terms:
    ω_pu·ψ_pu = ω · ψ / V_base
    ω_pu·L_pu = ω · L · I_base / V_base
    ω_pu·L_pu·I_pu = ω · L · I / V_base


    ω_pu => ω_base anchor:
        1/2 motor rated => ω_pu = 0.5 at rated speed, 1.0 at max field weakening
        Fs angle16 => controller max, resolution reduced

    ψ_pu = 1.0, when V_base, ω_base are self-consistent, per motor, V_base = Kv · ω_base[rpm]
    L_pu < 1.0, when V_base / I_base < 1.0
*/


/* Compose across unit specializations(µWb→Wb, mrad / s→rad / s), inline across Q - format boundaries. */




/******************************************************************************/
/*!
    @brief ψ_f [Wb] — Back-EMF and rotor flux

    Identification:
        Spin test (generator at known speed):
            ψ_f = V_phase_pk / ω_elec = V_LL_pk / (√3 · ω_elec)
        Steady-state FOC:
            ψ_f = (vq − Rs·iq − ω_e·Ld·id) / ω_e

*/
/******************************************************************************/
/*
    PU identification — direct from FOC-loop quantities.
    Spin-test form is the literal inverse of the runtime BEMF computation fract16_mul(omega_step, psi_pu).
       ψ_pu = v_emf_pu · 32768 / omega_step  =  fract16_div(v_emf_pu, omega_step)   [spin test]
       ψ_pu = residual_v_pu / omega_step,  residual = vq − Rs·iq − ω·Ld·id          [online]
*/
static inline uint32_t psi_pu_of_emf(fract16_t v_emf_pu, fract16_t omega_step) { return fract16_div(v_emf_pu, omega_step); }

/* ψ_f[µWb] = V[mV]·1e6 / ω[mrad/s]      (mV·s/mrad ≡ Wb; ×1e6 → µWb) */
static inline uint32_t psi_uwb_of_emf(uint32_t v_phase_mV_pk, uint32_t omega_elec_mrads) { return (uint64_t)v_phase_mV_pk * 1000000UL / omega_elec_mrads; }
// static inline uint32_t psi_uwb_of_emf_ll(uint32_t omega_elec_mrads, uint32_t v_ll_mV_pk) { return (uint64_t)v_ll_mV_pk * 1000000UL * FRACT16_SCALE / ((uint64_t)FRACT16_SQRT3 * omega_elec_mrads); }

static inline uint32_t psi_pu_of_running(fract16_t rs_pu, fract16_t ld_pu, fract16_t omega_step, fract16_t vq_pu, fract16_t id_pu, fract16_t iq_pu)
{
    fract16_t omega_Ld = fract16_mul(omega_step, ld_pu);
    fract16_t residual = fract16_sat((accum32_t)vq_pu - fract16_mul(rs_pu, iq_pu) - fract16_mul(omega_Ld, id_pu));
    return fract16_div(residual, omega_step);
}

/*
    Online ψ_f under steady-state FOC. Numerator built in µV (mΩ·mA = µV exactly).
    Caller ensures vq·1e3 ≥ v_r + v_l (physically required for ψ > 0).
*/
static inline uint32_t psi_uwb_of_running(uint32_t rs_mOhm, uint32_t ld_uH, uint32_t omega_e_mrads, uint32_t vq_mV, uint32_t id_mA, uint32_t iq_mA)
{
    uint64_t v_r = (uint64_t)rs_mOhm * iq_mA;                                    /* µV */
    uint64_t v_l = (uint64_t)omega_e_mrads * ld_uH * id_mA / 1000000UL;          /* µV */
    return ((uint64_t)vq_mV * 1000UL - v_r - v_l) * 1000UL / omega_e_mrads;      /* µWb */
}

/*
    from a defined ratio
    ψ_pu = (v_emf / ω) · (ω_base / V_base)
    ψ_pu = (V_rated · ω_base) / (ω_rated · V_base)
    rads or rpm cancel
*/
static inline uint32_t psi_pu_of_vspeed(uint16_t v_base_V, uint32_t omega_base, uint16_t v_emf, uint32_t omega_emf)
{
    return (uint64_t)v_emf * omega_base * FRACT16_SCALE / ((uint64_t)omega_emf * v_base_V);
}


static inline uint32_t l_uh_of_hfi(uint32_t f_Hz, uint32_t v_mV_pk, uint32_t i_mA_pk) { return (uint64_t)v_mV_pk * 1000000UL * FRACT16_SCALE / ((uint64_t)FRACT16_PI * f_Hz * i_mA_pk); }
static inline uint32_t l_uh_of_rs_tau_us(uint32_t rs_mOhm, uint32_t tau_us) { return (uint64_t)rs_mOhm * tau_us / 1000UL; }
static inline uint32_t l_uh_of_rs_tau_cycles(uint32_t polling_freq, uint32_t rs_mOhm, uint32_t tau_cycles) { return (uint64_t)rs_mOhm * tau_cycles * 1000UL / polling_freq; }
static inline uint32_t l_uh_of_step(uint32_t v_mV, uint32_t di_mA, uint32_t dt_us) { return (uint64_t)v_mV * dt_us / di_mA; }


/******************************************************************************/
/*!
    @brief  Kt — Torque Constant [Nm/A]

        FOC peak-iq:        Kt = (3/2) · P · ψ_f          T_em = Kt · iq
        Motor-constant:     Kt [Nm/A_rms] = 60 / (2π · Kv) = Ke_mech (numerically)
*/
/******************************************************************************/
static inline uint32_t kt_unm_per_a_of_psi(uint32_t psi_uWb, uint8_t polePairs) { return 3UL * psi_uWb * polePairs / 2UL; }

// static inline accum32_t kt_nm_per_amp_of(uint16_t kv, scale) { return ((uint64_t)60 * FRACT16_SQRT3_DIV_2 * FRACT16_SCALE / kv * 2 * FRACT16_PI); }

/* FOC directly from Kv — pole pairs cancel: Kt = 1.5·P·ψ_f = 1.5·P·60/(2π·Kv·P) = 1.5·Ke_mech. */
// static inline uint32_t kt_unm_per_a_of_kv(uint16_t kv_rpm_per_V) { return 3UL * ke_mech_uvsrad_of_kv(kv_rpm_per_V) / 2UL; }
// static inline uint32_t kt_rms_unm_per_a_of_kv(uint16_t kv_rpm_per_V) { return ke_mech_uvsrad_of_kv(kv_rpm_per_V); }


/******************************************************************************/
/*!
    @brief  Rs — Stator Resistance [Ω], per phase

    DC injection (di/dt = 0, EMF = 0 at standstill):
        Rs       = V / I                    (single conductor)
        Rs_phase = V_LL / (2·I)             (Y, two phases in series)

    Online under FOC DC excitation (iq=0, id=id_ref, ω=0):
        Rs_pu = vd_pu / id_pu               (ratio invariant under PU)

    PU encoding:  (float)Rs_pu × FRACT16_SCALE = Rs · I_max / V_max
*/
/******************************************************************************/
#define MOTOR_R_FRACT16(V_Base, I_Base, R_Ohm, SI_Scale) ((R_Ohm) * I_Base * FRACT16_SCALE / ((uint64_t)V_Base * SI_Scale))
#define MOTOR_R_FRACT16_OF_MOHM(V_Base, I_Base, R_mOhm) MOTOR_R_FRACT16(V_Base, I_Base, R_mOhm, 1000UL)

/* R_base = V_base / I_base */
/* accum32_t for when v_base > i_base */
static inline accum32_t rs_pu_of_vd_id(fract16_t vd_pu, fract16_t id_pu) { return fract16_div_sat(vd_pu, id_pu); }

static inline uint32_t rs_mohm_of_vi(uint32_t v_mV, uint32_t i_mA) { return (uint64_t)v_mV * 1000UL / i_mA; }
static inline uint32_t rs_mohm_of_vi_ll_y(uint32_t v_mV_ll, uint32_t i_mA) { return (uint64_t)v_mV_ll * 1000UL / (2UL * i_mA); }

static inline uint32_t rs_pu_of_mohm(uint16_t v_max_V, uint16_t i_max_A, uint32_t rs_mOhm) { return (uint64_t)rs_mOhm * i_max_A * FRACT16_SCALE / ((uint32_t)v_max_V * 1000UL); }
static inline uint32_t rs_mohm_of_pu(uint16_t v_max_V, uint16_t i_max_A, uint32_t rs_pu) { return (uint64_t)rs_pu * v_max_V * 1000UL / ((uint32_t)i_max_A * FRACT16_SCALE); }



/******************************************************************************/
/*!
    @brief  Phase-basis amplitude conversions (Y-connected motors).

        V_phase_pk = V_LL_pk / √3        V_LL_pk = √3 · V_phase_pk
        V_pk       = √2 · V_rms          V_rms   = V_pk / √2
*/
/******************************************************************************/
static inline uint32_t v_phase_peak_of_ll(uint32_t v_ll_pk) { return (uint64_t)v_ll_pk * FRACT16_SCALE / FRACT16_SQRT3; }
static inline uint32_t v_ll_of_phase_peak(uint32_t v_phase_peak) { return (uint64_t)v_phase_peak * FRACT16_SQRT3 / FRACT16_SCALE; }
static inline uint32_t v_peak_of_rms(uint32_t v_rms) { return (uint64_t)v_rms * FRACT16_SQRT2 / FRACT16_SCALE; }
static inline uint32_t v_rms_of_peak(uint32_t v_peak) { return (uint64_t)v_peak * FRACT16_SCALE / FRACT16_SQRT2; }


/******************************************************************************/
/*!
    @brief  Voltage-Model Back-EMF Estimator

        e = v - Rs·i - Ls·di/dt

    Discrete one-step form, caller supplies the precomputed Δi = i[k] - i[k-1]:
        ê[k] = v[k] - Rs_pu·i[k] - Ls_pu·Δi[k]
*/
/******************************************************************************/
static inline accum32_t motor_v_stator(accum32_t Rs_pu, accum32_t Ls_pu, fract16_t i_prev, fract16_t i) { return fract16_mul(Rs_pu, i) + fract16_mul(Ls_pu, i_prev - i); }
static inline accum32_t motor_emf(accum32_t Rs_pu, accum32_t Ls_pu, fract16_t i_prev, fract16_t i, fract16_t v) { return v - motor_v_stator(Rs_pu, Ls_pu, i_prev, i); }



/******************************************************************************/
/*
    ω_base motor rated anchored:
    ω_base-anchored (Fs-independent, consumed by fract16_mul with ω_pu):
    ω_base = 2π · p · n_max / 60                        [rad/s electrical]
    ω_pu   = ω_e / ω_base                               (rated → 1.0)

    integrator: θ += ω_pu · (ω_base / (π · Fs))

    L_pu / ψ_pu are pure motor parameters
    Fs appears only at the boundary that converts el_delta_angle16 → ω_pu.
        L_pu   = L · I_max · ω_base / V_max                 (≈ 0.05–0.5)
        ψ_pu   = ψ_f · ω_base / V_max                       (≈ 0.7–1.1)

    ω_base anchor to V saturation, psi_pu = 1.0:

    e.g:
    V_base = 94 V                                      (hardware: V_bus_max)
    I_base = 600 A                                     (hardware: I_sense_fs)
    ψ_base = ψ_si = 15 mWb = 0.015 Wb                  (motor: forces ψ_pu = 1)
    ω_base = V_base / ψ_base = 94 / 0.015 = 6266.67 rad/s electrical                  (derived)
        →  8,549 rpm
    Dependent bases
    R_base = V_base / I_base       = 94 / 600         = 156.67 mΩ
    L_base = ψ_base / I_base       = 0.015 / 600      = 25 µH
    T_base = (3/2)·p·ψ_base·I_base = 13.5·p N·m       (p = pole pairs)
    P_base = (3/2)·V_base·I_base   = 1.5·94·600       = 84.6 kW

*/
/******************************************************************************/
/*!
    @brief  PU — _rads form: ω_base parameterized directly in mrad/s_e

    Bridge to/from mechanical: ω_base_mrads = π · P · n_rpm · 1000 / 30

    Encoding (× FRACT16_SCALE for fract16 storage):
    L_pu = L · I_max · ω_base_mrads / (1000 · V_max)
    ψ_pu = ψ_f · ω_base_mrads / (1000 · V_max)

    bridge from angle16:
    ω_pu = el_delta · π · Fs · 1000 / ω_base_mrads
*/
/******************************************************************************/
static inline uint32_t psi_pu_rads_of_wb(uint16_t v_max_V, uint32_t el_rads_base, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)psi_Wb * el_rads_base * FRACT16_SCALE / ((uint64_t)v_max_V * scale); }
static inline uint32_t psi_wb_of_pu_rads(uint16_t v_max_V, uint32_t el_rads_base, uint32_t psi_pu, uint32_t scale) { return (uint64_t)psi_pu * v_max_V * scale / ((uint64_t)el_rads_base * FRACT16_SCALE); }

static inline uint32_t psi_pu_mrads_of_uwb(uint16_t v_max_V, uint32_t el_rads_base, uint32_t psi_uWb) { return psi_pu_rads_of_wb(v_max_V, el_rads_base, psi_uWb, 1000UL * 1000000UL); }
static inline uint32_t psi_uwb_of_pu_mrads(uint16_t v_max_V, uint32_t el_rads_base, uint32_t psi_pu) { return psi_wb_of_pu_rads(v_max_V, el_rads_base, psi_pu, 1000UL * 1000000UL); }

/*
    ψ_pu × FRACT16_SCALE = π · P · ψ_f · speed_max_rpm / (30 · V_max)
    ψ_pu in speed_pu_rpm basis — numerically equals Ke_pu_rpm
    V_emf_pu = ψ_pu · speed_pu, speed_pu = rpm / speed_max_rpm.

    (speed_pu carries no mech/elec distinction; P appears in SI conversion, cancels via Kv).
*/
static inline uint32_t psi_pu_rpm_of_wb(uint16_t v_max_V, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)FRACT16_PI * polePairs * speed_max_rpm * psi_Wb / ((uint64_t)30UL * v_max_V * scale); }
static inline uint32_t psi_wb_of_pu_rpm(uint16_t v_max_V, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t psi_pu, uint32_t scale) { return (uint64_t)30UL * psi_pu * v_max_V * scale / ((uint64_t)FRACT16_PI * polePairs * speed_max_rpm); }

static inline uint32_t psi_pu_rpm_of_uwb(uint16_t v_max_V, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t psi_uWb) { return psi_pu_rpm_of_wb(v_max_V, speed_max_rpm, polePairs, psi_uWb, 1000000UL); }
static inline uint32_t psi_uwb_of_pu_rpm(uint16_t v_max_V, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t psi_pu) { return psi_wb_of_pu_rpm(v_max_V, speed_max_rpm, polePairs, psi_pu, 1000000UL); }

/******************************************************************************/
/*
    Kv [RPM/V] is the most commonly published rotor parameter.
    Storing (Kv, P) instead of (ψ_uWb, P) or (ψ_pu, P) avoids µ-unit redundancy and matches how datasheets list motors —
    Storing {Kv, V_max, I_max, Fs, P} fully determines the rotor half of the model; only Rs and Ld/Lq need independent storage.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief  Kv-based storage — {Kv, polePairs} as the canonical motor parameter.

    Identities derived from (Kv, P):
        Kv [RPM/V]
        Ke_mech [V·s/rad mech]  = (1 / Kv) · (60 / 2π)
        ψ_f [Wb = V·s/rad elec] = Ke_mech / P / √3  = 1 / (Kv·2π·P/60) / √3

        Kt_FOC  = (3/2) · P · ψ_f = 45 / (π · Kv)            [Nm/A]   (P cancels)
        Kt_mtr  = 60 / (2π · Kv) = Ke_mech                   [Nm/A_rms]

    Direct Kv-keyed derivations skip the µWb intermediate:
        Kt_FOC      = 90 / (2π · Kv) · 1e6  µNm/A    (pole pairs cancel)

    Vendor-spec basis conversions (vendor → internal phase-peak):
        Kv_phase_pk = √3 · Kv_LL_pk     (V_LL_pk = √3 · V_phase_pk)
        Kv_pk       = Kv_rms / √2       (V_pk    = √2 · V_rms)
*/
/******************************************************************************/
/* Speed envelope from Kv. */
static inline uint32_t rpm_of_kv_v(uint16_t kv, uint32_t v_V) { return (uint32_t)kv * v_V; }
static inline uint32_t v_of_kv_rpm(uint16_t kv, uint32_t rpm) { return rpm / kv; }

/* [V / (Rad/s)] base. generic for caller compose  */
/* V·s/rad */
static inline uint32_t ke_vrads_of(uint16_t kv, uint32_t scale) { return ((uint64_t)scale * FRACT16_SCALE * 60U / (2 * FRACT16_PI)) / (kv); }
static inline uint32_t ke_mvrads_of_kv(uint16_t kv) { return ke_vrads_of(kv, 1000U); }
static inline uint32_t ke_uvsrad_of_kv(uint16_t kv_rpm_per_V) { return ke_vrads_of(kv_rpm_per_V, 1000000U); }

/* psi as phase electtrical */
static inline uint32_t psi_wb_of_kv(uint16_t kv, uint8_t polePairs, uint32_t scale) { return ke_vrads_of(kv, scale) / polePairs / 2; }
static inline uint32_t psi_uwb_of_kv(uint16_t kv_rpm_per_V, uint8_t polePairs) { return psi_wb_of_kv(kv_rpm_per_V, polePairs, 1000000UL); }
// static inline uint16_t kv_of_psi_uwb(uint32_t psi_uWb, uint8_t polePairs) { return (uint64_t)60UL * 1000000UL * FRACT16_SCALE / ((uint64_t)FRACT16_SQRT3 * psi_uWb * polePairs); }

/* Kv basis conversions — parse vendor specs into internal phase-peak. */
// static inline uint16_t kv_phase_pk_of_kv_ll(uint16_t kv_ll_pk) { return v_ll_of_phase_peak(kv_ll_pk); }
// static inline uint16_t kv_ll_of_kv_phase_pk(uint16_t kv_phase_pk) { return v_phase_peak_of_ll(kv_phase_pk); }
static inline uint16_t kv_phase_pk_of_kv_ll(uint16_t kv_ll_pk) { return (uint64_t)kv_ll_pk * FRACT16_SQRT3 / FRACT16_SCALE; }
static inline uint16_t kv_ll_of_kv_phase_pk(uint16_t kv_phase_pk) { return (uint64_t)kv_phase_pk * FRACT16_SCALE / FRACT16_SQRT3; }
static inline uint16_t kv_pk_of_kv_rms(uint16_t kv_rms) { return (uint64_t)kv_rms * FRACT16_SCALE / FRACT16_SQRT2; }
static inline uint16_t kv_rms_of_kv_pk(uint16_t kv_pk) { return (uint64_t)kv_pk * FRACT16_SQRT2 / FRACT16_SCALE; }

/* v_pu / omega_pu */
static inline uint32_t ke_pu_rpm_of_kv(uint16_t v_max_V, uint32_t speed_max_rpm, uint16_t kv) { return speed_max_rpm * FRACT16_SCALE / (kv * v_max_V); }
static inline uint16_t kv_of_ke_pu_rpm(uint16_t v_max_V, uint32_t speed_max_rpm, uint32_t ke_pu) { return (uint64_t)speed_max_rpm * FRACT16_SCALE / ((uint64_t)ke_pu * v_max_V); }

/*
    ψ_f [Wb] = (1 / Kv) · (60 / 2π) / P / M = 60 / (2π · Kv · P · M)
    kv = ω_rpm / (v * M)

    With ω_base in rpm_mech and ψ_f from Kv, the 60/(2π·p) factors cancel:
    ψ_pu = ψ_f · ω_base / V_base
        = [60/(2π·Kv·P) / M] · [ω_base_rpm·(2π/60)·P] / V_base
        = base_rpm / M / (Kv · V_base)

    ψ_pu = v_emf_pu / ω_pu = (v_emf / ω) · (ω_base / V_base)

    rpm_max/(Kv·M) > V_base → ψ_pu > 1.0 (field-weakening region)
    rpm_max/(Kv·M) < V_base → ψ_pu < 1.0 (motor reaches mechanical limit before voltage saturation)
*/
/* alternatively, psi of ke use √3, psi of kv use /2 since kv is rpm observation at bus voltage */
static inline uint32_t psi_pu_rpm_of_kv(uint16_t v_max_V, uint32_t speed_max_rpm, uint16_t kv) { return ke_pu_rpm_of_kv(v_max_V, speed_max_rpm, kv) / 2; }
static inline uint16_t kv_of_psi_pu_rpm(uint16_t v_max_V, uint32_t speed_max_rpm, uint32_t psi_pu) { return kv_of_ke_pu_rpm(v_max_V, speed_max_rpm, psi_pu) * 2; }
// static inline uint32_t psi_pu_rpm_of_kv(uint16_t v_max_V, uint32_t speed_max_rpm, uint16_t kv) { return ke_pu_rpm_of_kv(v_max_V, speed_max_rpm, kv) * FRACT16_SCALE / FRACT16_SQRT3; }
// static inline uint16_t kv_of_psi_pu_rpm(uint16_t v_max_V, uint32_t speed_max_rpm, uint32_t psi_pu) { return kv_of_ke_pu_rpm(v_max_V, speed_max_rpm, psi_pu) * FRACT16_SQRT3 / FRACT16_SCALE; }

static inline uint32_t psi_pu_rpm_of_ke(uint16_t v_max_V, uint32_t speed_max_rpm, uint16_t ke, uint32_t scale) { return (uint64_t)ke * FRACT16_SCALE / FRACT16_SQRT3 / scale; }

/******************************************************************************/
/*!
    @brief  L_pu in ω_base-anchored basis — Fs-independent

    L_pu × FRACT16_SCALE = π · P · L · n_rated · I_max / (30 · V_max)
    L_pu  = L · I_max · ω_base / V_max

    v_pu = ω_pu · L_pu · i_pu,  ω_pu = ω_e / ω_base,  ω_base = π·P·n_rated_rpm/30

    Mirrors the ψ_pu_rpm family above. Together with rs_pu_of_mohm (Fs-free already),
    forms the full motor parameter set anchored at ω_base — every constant a pure
    motor property, portable across controllers at different Fs. The single
    Fs-dependent multiply lives at omega_pu_rpm_of_angle16 below.

    For typical motors L_pu ≈ 0.05–0.5 — well inside FRACT16_MAX, so subsequent
    fract16_mul(ω_pu, L_pu) cannot overflow int32 the way the angle16-anchored
    form can at high speed.
*/
/******************************************************************************/
#define MOTOR_L_PU(V_Base, I_Base, W_Scale_Radians, L_Henries, SI_Scale) ((uint64_t)L_Henries * I_Base * W_Scale * FRACT16_SCALE / ((uint64_t)V_Base * SI_Scale))

/*
    L_pu — scale absorbs both L unit (1/scale_L H) and ω_base unit (1/scale_ω rad/s_e):
    scale = scale_L · scale_ω        e.g. (µH, mrad/s_e) → 1000 · 1e6 = 1e9
*/
static inline uint32_t l_pu_rads_of_h(uint16_t v_max_V, uint16_t i_max_A, uint32_t base_rads_e, uint32_t l_h, uint32_t scale) { return (uint64_t)l_h * i_max_A * base_rads_e * FRACT16_SCALE / ((uint64_t)v_max_V * scale); }
static inline uint32_t l_h_of_pu_rads(uint16_t v_max_V, uint16_t i_max_A, uint32_t base_rads_e, uint32_t l_pu, uint32_t scale) { return (uint64_t)l_pu * v_max_V * scale / ((uint64_t)i_max_A * base_rads_e * FRACT16_SCALE); }
static inline uint32_t l_pu_mrads_of_uh(uint16_t v_max_V, uint16_t i_max_A, uint32_t base_mrads_e, uint32_t l_uH) { return l_pu_rads_of_h(v_max_V, i_max_A, base_mrads_e, l_uH, 1000UL * 1000000UL); }
static inline uint32_t l_uh_of_pu_mrads(uint16_t v_max_V, uint16_t i_max_A, uint32_t base_mrads_e, uint32_t l_pu) { return l_h_of_pu_rads(v_max_V, i_max_A, base_mrads_e, l_pu, 1000UL * 1000000UL); }

static inline uint32_t l_pu_rpm_of_h(uint16_t v_max_V, uint16_t i_max_A, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t l_h, uint32_t scale) { return (uint64_t)FRACT16_PI * polePairs * speed_max_rpm * l_h * i_max_A / ((uint64_t)30UL * v_max_V * scale); }
static inline uint32_t l_h_of_pu_rpm(uint16_t v_max_V, uint16_t i_max_A, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t l_pu, uint32_t scale) { return (uint64_t)30UL * l_pu * v_max_V * scale / ((uint64_t)FRACT16_PI * polePairs * speed_max_rpm * i_max_A); }
static inline uint32_t l_pu_rpm_of_uh(uint16_t v_max_V, uint16_t i_max_A, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t l_uH) { return l_pu_rpm_of_h(v_max_V, i_max_A, speed_max_rpm, polePairs, l_uH, 1000000UL); }
static inline uint32_t l_uh_of_pu_rpm(uint16_t v_max_V, uint16_t i_max_A, uint32_t speed_max_rpm, uint8_t polePairs, uint32_t l_pu) { return l_h_of_pu_rpm(v_max_V, i_max_A, speed_max_rpm, polePairs, l_pu, 1000000UL); }


/*
    ω_base-anchored basis —  Fs appears only for time-domain measurements (τ_cycles, dt_cycles); HFI is Fs-free.
    L_pu_rads = Rs_pu · ω_base · τ_cycles / Fs                                         [RL τ]
    L_pu_rads = (v/di) · ω_base · dt_cycles / Fs                                       [step]
    L_pu_rads = (v/i) · ω_base / (2π·f)                                                [HFI]
    scale is the ω unit factor: 1 for rad/s_e, 1000 for mrad/s_e (matches setter pattern).
*/
static inline uint32_t l_pu_rads_of_rs_tau_cycles(uint32_t polling_freq, uint32_t omega_base_e, uint32_t scale, fract16_t rs_pu, uint32_t tau_cycles) { return (uint64_t)rs_pu * omega_base_e * tau_cycles / ((uint64_t)scale * polling_freq); }
static inline uint32_t l_pu_rads_of_step(uint32_t polling_freq, uint32_t omega_base_e, uint32_t scale, fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return (uint64_t)fract16_div(v_pu, di_pu) * omega_base_e * dt_cycles / ((uint64_t)scale * polling_freq); }
static inline uint32_t l_pu_rads_of_hfi(uint32_t omega_base_e, uint32_t scale, uint32_t f_Hz, ufract16_t v_pk_pu, ufract16_t i_pk_pu) { return (uint64_t)fract16_div(v_pk_pu, i_pk_pu) * omega_base_e * FRACT16_SCALE / ((uint64_t)2 * FRACT16_PI * scale * f_Hz); }

/*
    rpm form — ω_base = π·P·n/30 substituted directly. Constants cancel:
    L_pu = Rs_pu · π·P·n · τ_cycles / (30 · Fs)                  [RL τ]
    L_pu = (v/di) · π·P·n · dt_cycles / (30 · Fs)                [step]
    L_pu = (v/i) · P·n / (60 · f)                                [HFI, π and FRACT16_SCALE cancel]
*/
static inline uint32_t l_pu_rpm_of_rs_tau_cycles(uint32_t polling_freq, uint32_t speed_max_rpm, uint8_t polePairs, fract16_t rs_pu, uint32_t tau_cycles) { return (uint64_t)rs_pu * FRACT16_PI * polePairs * speed_max_rpm * tau_cycles / ((uint64_t)30UL * FRACT16_SCALE * polling_freq); }
static inline uint32_t l_pu_rpm_of_step(uint32_t polling_freq, uint32_t speed_max_rpm, uint8_t polePairs, fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return (uint64_t)fract16_div(v_pu, di_pu) * FRACT16_PI * polePairs * speed_max_rpm * dt_cycles / ((uint64_t)30UL * FRACT16_SCALE * polling_freq); }
static inline uint32_t l_pu_rpm_of_hfi(uint32_t speed_max_rpm, uint8_t polePairs, uint32_t f_Hz, ufract16_t v_pk_pu, ufract16_t i_pk_pu) { return (uint64_t)fract16_div(v_pk_pu, i_pk_pu) * polePairs * speed_max_rpm / (60UL * f_Hz); }





/*
    ω_base angle16-anchored (Fs anchored):
    ω_base = π · Fs
    ψ_base = V_base / (π·Fs)
    L_base = V_base / (π·Fs · I_base)

    ω_pu = ω / (π · Fs)
    ψ_pu = ψ · (π · Fs / V_base)
    L_pu = L · (π · Fs · I_base / V_base)

    θ in angle16: 32768 ⇔ π rad
    integrator:   θ_pu += ω_pu [digital]
    ω_base [digital] = 32768 ⇔ π rad at Fs
    ω_du [digital] = ω [rad/s] · 32768 / (π·Fs)
        ω_du [digital] = 1 ⇒ 2π/65536 [rad/tick] = Fs · π/32768 [rad/s]

    corresponds with nyquist by construction, int16 wrap at 2π/2.

    motor independent: system speed max fixed at Fs·π [rad/s]
        at Fs = 20000:
        resolution ~2 eRads/s
        ω max [rad/s]: Fs·π = 62832 [rad/s] = 10000 [erps]
        ω_du [digital] = 1 ⇒ 1.9f [rad/s]

    ω_pu roughly 2–25% of resolution during run time [0:8192]
        1000 mech rpm * 4p => 1000*4*2π/60 = 418 [erad/s] => 218 [angle16]
        alternatively ω_base = π · Fs / k, k=2 => resolution 50%, corresponds with 1/4 cycle max

    with direct multiply only composite V terms store at > 1.0 pu, no intermediary 64-bit multiply
    ω_pu·L_pu·I_pu store at > 1.0 pu

    ω_pu, θ          : fract16 (Q15/Q16, identity scale to angle wrap)
    ψ_pu, L_pu, R_pu : accum32

    or ω_pu in Q16.16 * L_pu in Q16.15 => wide multiply full precision
    angle32_t * accum32_t =>  accum32_t
    angle16_t * accum32_t =>  accum32_t


    e.g:
    V_base = 94 [V]
    I_base = 600 [A]

    R_base = V_base / I_base = 156.7 [mΩ]
    R_s = 10 mΩ => R_pu ≈ 0.064f

    Fs = 20000
    ω_base = 20000π = 62832 [erad/s] = 600000 [erpm]
    ψ_base  = 1.49 [mWb]
    L_base  = 2.49 [uH]

    ω_pu = ω / (π · Fs)
    ψ_pu = ψ · (π · Fs / V_base) = ψ_mWb · 0.668f
    L_pu = L · (π · Fs · I_base / V_base) = L_uH · 0.4008511f

    exceeds base, > Q1.15 range
    ψ_si = 45 mWb => ψ_pu ≈ 32.0f
    L_si = 80 µH => L_pu ≈ 30.0f

    combined v term exceeds v_base
    ω·ψ at max = 2827.44[V]
    ω·L·I at max = 3015[V]

    ω_pu·ψ_pu = ω · ψ / V_base,             at ω_max => 30.0f
    ω_pu·L_pu = ω · L · I_base / V_base,    at I_max, ω_max => 30.0f
    ω_pu·L_pu·I_pu = ω · L · I / V_base

    Direct multiply, without q15 scale:
        ψ_du  = ψ · (π · Fs / V_base)
        L_du  = L · (π · I_base · Fs / V_base)

        truncate
        ψ_du ≈ 30
        L_du ≈ 32

        1000 rpm * 4p => ω_du = 218
            ω_du * L_du = 6976

    l_h: 80 uH
    i_max_A: 600 A
    Fs: 20 kHz
    v_max_V: 94 V
    ω_base_e:2094.4 (5000 mech rpm, 4 pole pairs)

    _l_pu_of_h       L_pu_a16 = 32 (real value 32.083, integer-truncated)
        1000 rpm => ω = 218  =>  ω * L_pu = 6976

    l_pu_rads_of_h   L_pu_rads = 35,045 (Q15 of real value 1.0695)
        1000 rpm => ω = 6553 => ω * L_pu = 7009

    n_rpm	ω_e	F1 el_Δ	F1 product	F2 ω_pu	F2 product	true Q15·Z_L	F1 err	F2 err
    100	    42	22	704	655	700	701	+0.4 %	−0.1 %
    500	    209	109	3488	3277	3504	3504	−0.5 %	0.00 %
    1000	419	218	6976	6554	7010	7009	−0.5 %	+0.01 %
    2000	838	437	13984	13107	14017	14018	−0.24 %	0.00 %
    5000	2094	1092	34944	32767†	35043	35044	−0.29 %	0.00 %
    7500	3142	1639	52448	49152‡	52564	52566	−0.22 %	0.00 %
    10000	4189	2185	69920	65535‡	70089	70089	−0.24 %	0.00


    //
    ω_base with k
    ( π·Fs/k to extend precision, k=2, 4..):
        ψ_du  = ψ · π · Fs / V_max / k
        L_du  = L · π · I_max · Fs / V_max / k

    theta += omega_pu >> LOG2_N

    L_base = V_base / (ω_base / k · I_base)
        k = 1 => 2.49 [uH]
        k = 2 => 4.98 [uH]
        k = 4 => 9.95 [uH]
        k = 8 => 19.9 [uH]
        k = 16 => 39.8 [uH]
        k = 32 => 79.6 [uH]
        k = 64 => 159.2 [uH]
        k = 128 => 318.4 [uH]

    #define _MOTOR_SPEED_TYPE_MAX_ERPM(PolePairs) ((uint32_t)(PolePairs) * MOTOR_SPEED_TYPE_MAX_RPM())
    #define _MOTOR_SPEED_TYPE_MAX_DIGIT(erpm)  ANGLE16_OF_RPM(MOTOR_CONTROL_FREQ, erpm)
    #define MOTOR_DIGITAL_SPEED_NORM 2
    #define MOTOR_DIGITAL_SPEED_MAX (1 < (MOTOR_DIGITAL_SPEED_BITS - 1))

    Angle Speed max in digital units / S
    #define ANGLE_SPEED_MAX(pollingFreq) (32768 * pollingFreq)
    #define MOTOR_ANG_SPEED_MAX (1 < (MOTOR_DIGITAL_SPEED_BITS - 1))

    32767 600000 erpm, 10000 erps, 150000 mech rpm
    16383 300000 erpm
    8192 150000 erpm
    4096 75000 erpm
    2048 37500 erpm
    1024 18750 erpm, 8 pole pairs 2344 rpm


    //
    store L and ψ as(factor, shift)
    L_pu = L_q15  · 2 ^ L_shift
    ψ_pu = ψ_q15  · 2 ^ ψ_shift

    ψ_q15 = ψ_si · π·Fs / (V_base · k_ψ) · 32768
    L_q15 = L_si · π·Fs · I_base / (V_base · k_L) · 32768
    n_ψ = log2(k_ψ)
    n_L = log2(k_L)

    //
    split multiply to Fs 1000:
    ω_base_1000 = π · 1000 = 3141.6[rad / s electrical]
    ψ_base_1000 = V_base / ω_base_1000 = 30[mWb]
    L_base_1000 = V_base / (ω_base_1000 · I_base) = 50[uH]

    Fs_base = π·1000  →  ω_base = 3141.6 rad / s    L_base = 50 µH    ψ_base = 30 mWb
    Fs_base = π·625   →  ω_base = 1963 rad / s      L_base = 80 µH    ψ_base = 48 mWb
    Fs_base = π·312.5 →  ω_base = 981 rad / s       L_base = 159 µH   ψ_base = 95 mWb

    1000 mechrpm * 4p = > ω_F1000_du ~= 4360
*/



/******************************************************************************/
/*!
    @brief  Ke / ψ_f / Kv — Back-EMF and rotor flux

    PU encoding:  (float)ψ_pu × FRACT16_SCALE = π · ψ_f · Fs / V_max
        fract16_mul(el_delta_angle16, ψ_pu) → BEMF in v_pu.
*/
/******************************************************************************/
static inline uint32_t psi_pu_of_wb(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)FRACT16_PI * psi_Wb * polling_freq / ((uint64_t)v_max_V * scale); }
static inline uint32_t psi_wb_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_pu, uint32_t scale) { return (uint64_t)psi_pu * v_max_V * scale / ((uint64_t)FRACT16_PI * polling_freq); }

static inline uint32_t psi_pu_of_uwb(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_uWb) { return psi_pu_of_wb(polling_freq, v_max_V, psi_uWb, 1000000UL); }
static inline uint32_t psi_uwb_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_pu) { return psi_wb_of_pu(polling_freq, v_max_V, psi_pu, 1000000UL); }

static inline uint32_t psi_pu_of_mvrads(uint32_t polling_freq, uint16_t v_max_volts, uint32_t mV_per_rads) { return psi_pu_of_wb(polling_freq, v_max_volts, mV_per_rads, 1000UL); }

/*
    Direct Kv→PU shortcut.
    Mechanical Ke (no √3, no P) is the convenient intermediate
    60 / (2π·Kv)
*/
static inline uint32_t ke_pu_angle16_of_kv(uint32_t polling_freq, uint16_t v_max_V, uint16_t kv) { return (uint64_t)30UL * polling_freq * FRACT16_SCALE / ((uint32_t)kv * v_max_V); }
static inline uint16_t kv_of_ke_pu_angle16(uint32_t polling_freq, uint16_t v_max_V, uint32_t ke_pu) { return (uint64_t)30UL * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_V * ke_pu); }

/*
    ψ_pu directly from Kv — phase-electrical, √3
    Kv is vendor-spec line-to-line, so √3 converts to ψ_f_phase. π cancels:

    ψ_f [Wb] = (1 / (Kv·P·√3)) · (60 / 2π) = 60 / (2π·P·Kv·√3)

    ψ_pu = (60 / (2π·P·Kv·√3)) · (π · Fs / V_base) = 30·Fs / (Kv·P·V_max·√3)

    ψ_pu × 32768 = 30·Fs / (Kv·P·V_max·√3) · 32768
*/
// static inline uint32_t psi_pu_of_kv(uint32_t polling_freq, uint16_t v_max_V, uint8_t polePairs, uint16_t kv) { return ke_pu_angle16_of_kv(polling_freq, v_max_V, kv) * FRACT16_1_DIV_SQRT3 / ((uint32_t)polePairs * FRACT16_SCALE); }
// static inline uint16_t kv_of_psi_pu(uint32_t polling_freq, uint16_t v_max_V, uint8_t polePairs, uint32_t psi_pu) { return kv_of_ke_pu_angle16(polling_freq, v_max_V, psi_pu * polePairs * FRACT16_SCALE / FRACT16_1_DIV_SQRT3); }
static inline uint32_t psi_pu_of_kv(uint32_t polling_freq, uint16_t v_max_V, uint8_t polePairs, uint16_t kv) { return ke_pu_angle16_of_kv(polling_freq, v_max_V, kv) / ((uint32_t)polePairs * 2); }
static inline uint16_t kv_of_psi_pu(uint32_t polling_freq, uint16_t v_max_V, uint8_t polePairs, uint32_t psi_pu) { return kv_of_ke_pu_angle16(polling_freq, v_max_V, psi_pu * polePairs * 2); }


/*
    PU encoding: ψ_pu = ψ_f · π · Fs / V_max
        el_delta_angle16 * ψ_pu  → BEMF in v_pu.
*/
// static inline uint32_t _psi_pu_of_wb(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)psi_Wb * FRACT16_PI * polling_freq / ((uint64_t)v_max_V * scale * FRACT16_SCALE); }
// // static inline uint32_t _psi_wb_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint32_t psi_pu, uint32_t scale) { return (uint64_t)psi_pu * v_max_V * scale * FRACT16_SCALE / ((uint64_t)FRACT16_PI * polling_freq); }
// static inline uint32_t _psi_pu_of_kv(uint32_t polling_freq, uint16_t v_max_V, uint8_t polePairs, uint16_t kv) { return (uint64_t)30UL * polling_freq / ((uint32_t)kv * v_max_V * polePairs * 2); }

/******************************************************************************/
/*!
    @brief  Ls / Ld / Lq — Stator Inductance [H], per phase

        Ls = Ld = Lq for SPM; Ld ≠ Lq for IPM (rotor alignment selects axis).

    Identification:
        HFI (ω·L ≫ R):  L = V_pk / (2π·f·I_pk)
        RL τ (step):     L = Rs · τ,           (τ = 63.2% rise time)
        Voltage step:    L = V · Δt / Δi,      (initial linear ramp, R drop ≪)

    PU encoding:  (float)L_pu × FRACT16_SCALE = π · L · I_max · Fs / V_max
        fract16_mul(el_delta_angle16, L_pu) → ω_e·L·i_pu term in v_pu.
        The π factor absorbs the 2π·Fs/65536 angle16-step → rad/s conversion.
*/
/******************************************************************************/
#define MOTOR_L_ANGLE16(Fs, V_Base, I_Base, L_SI, SI_Scale) ((uint64_t)(L_SI) * I_Base * Fs * FRACT16_PI / FRACT16_SCALE / ((uint64_t)V_Base * SI_Scale))
#define MOTOR_L_ANGLE16_OF_UH(Fs, V_Base, I_Base, L_uH) MOTOR_L_ANGLE16(Fs, V_Base, I_Base, L_uH, 1000000UL)

/*
    L: L_si · I_max · π · Fs / V_max × 32768
    fract16_mul(el_delta_angle16, L_pu) → ω_e·L·i_pu term in v_pu.
*/
static inline uint32_t l_pu_of_h(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_h, uint32_t scale) { return (uint64_t)l_h * i_max_A * polling_freq * FRACT16_PI / ((uint64_t)v_max_V * scale); }
static inline uint32_t l_h_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_pu, uint32_t scale) { return (uint64_t)l_pu * v_max_V * scale / ((uint64_t)FRACT16_PI * i_max_A * polling_freq); }

static inline uint32_t l_pu_of_uh(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_uH) { return l_pu_of_h(polling_freq, v_max_V, i_max_A, l_uH, 1000000UL); }
static inline uint32_t l_uh_of_pu(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_pu) { return l_h_of_pu(polling_freq, v_max_V, i_max_A, l_pu, 1000000UL); }

/*
    el_delta_angle16:
        el_delta_angle16 * polling_freq / 10430 => electrical radians/second
*/
/* el_delta_angle16 * L_pu → ω_e·L·i_pu term in v_pu. */
// static inline uint32_t _l_pu_of_h(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_h, uint32_t scale) { return l_h * i_max_A * (polling_freq * FRACT16_PI / FRACT16_SCALE) / v_max_V / scale; }
// static inline uint32_t _l_pu_of_uh(uint32_t polling_freq, uint16_t v_max_V, uint16_t i_max_A, uint32_t l_uH) { return _l_pu_of_h(polling_freq, v_max_V, i_max_A, l_uH, 1000000UL); }


/*
    PU identification — direct from FOC-loop quantities. Output uint32 (may exceed FRACT16_MAX).
    Angle16-anchored basis (Fs baked in via the L_pu encoding):
    L_pu = Rs_pu · π · τ_cycles =  fract16_mul(Rs_pu, FRACT16_PI) · τ_cyc    [RL τ]
    L_pu = (v/di)·π·n_cycles·32768                                           [step]
    L_pu = (v/i)·Fs/(2f)·32768  =  fract16_div(v, i) · Fs/(2f)               [HFI]
*/
static inline uint32_t l_pu_of_rs_tau_cycles(fract16_t rs_pu, uint32_t tau_cycles) { return (uint64_t)rs_pu * FRACT16_PI * tau_cycles / FRACT16_SCALE; }
static inline uint32_t l_pu_of_step(fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return (uint64_t)fract16_div(v_pu, di_pu) * FRACT16_PI * dt_cycles / FRACT16_SCALE; }
static inline uint32_t l_pu_of_hfi(uint32_t polling_freq, uint32_t f_Hz, ufract16_t v_pk_pu, ufract16_t i_pk_pu) { return (uint64_t)fract16_div(v_pk_pu, i_pk_pu) * polling_freq / (2UL * f_Hz); }