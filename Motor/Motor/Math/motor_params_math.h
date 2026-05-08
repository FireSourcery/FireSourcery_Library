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
    @brief  Motor parameter conversions between SI, milliunit, and fract16 domains.

    _fract16  — Q1.15 normalized to hardware full-scale [V_max, I_max, etc.]
    _rpm      — mechanical speed in RPM
    _rads     — angular velocity in rad/s
    kv        — motor speed constant [RPM / V]
    v_max     — hardware peak voltage [V] (ADC full-scale, not bus)
    i_max     — hardware peak current [A]
    polling_freq — control loop rate [Hz]

    Pure functions: raw math, no guards, wider return type to preserve pre-saturation value.
    _sat functions: assert on zero-divisor inputs, saturate output to type range.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/angle_speed_math.h"

#include <assert.h>

/******************************************************************************/
/*!
    Kv — Speed / Voltage
        Kv [RPM/V]: rpm = Kv × V
        The "unit" of Kv is integer RPM per Volt stored as uint16_t.
*/
/******************************************************************************/
/* Precomputed factor: RPM at full-scale voltage — use as denominator. */
static inline int32_t kv_rpm_per_unit(uint16_t v_max_volts, uint16_t kv) { return (int32_t)kv * v_max_volts; }

/* rpm_of_kv: Speed [RPM] = Kv × V.  v_fract16 in [0, FRACT16_MAX]. */
static inline int32_t rpm_of_kv_vfract16(uint16_t v_max_volts, uint16_t kv, int16_t v_fract16) { return fract16_mul(v_fract16, kv_rpm_per_unit(v_max_volts, kv)); }

/* vfract16_of_kv_rpm: V_fract16 = Speed / Kv.  rpm in [0, kv × v_max]. */
static inline int32_t vfract16_of_kv_rpm(uint16_t v_max_volts, uint16_t kv, int32_t rpm) { return fract16_div(rpm, kv_rpm_per_unit(v_max_volts, kv)); }

// static inline fract16_t vfract16_of_kv_rpm_sat(uint16_t v_max_volts, uint16_t kv, int32_t rpm)
// {
//     assert(kv != 0U && v_max_volts != 0U);
//     return (fract16_t)math_clamp(vfract16_of_kv_rpm(v_max_volts, kv, rpm), 0, FRACT16_MAX);
// }

static inline uint32_t motor_kt(uint16_t kv) { return fract16_div(60 * FRACT16_SCALE, kv * 2 * FRACT16_PI); } /* [Nm/A] = 60 / (2π × Kv) */

/******************************************************************************/
/*!
    Ke — Normalized Voltage-per-Speed (back-EMF constant)
    Ke_SI [V/(rad/s)] = 60 / (2π × Kv) [V/(rad/s)]

    Speed_fract16 domain:
        V_emf = speed_rpm / Kv
        Ke_SF = V_emf / V_max / (speed / speed_max)
        Ke_SF = speed_max_rpm / (Kv × V_max)   [= speed_max / kv_rpm_per_unit]

        Applied as: v_emf_fract16 = fract16_mul(speed_fract16, Ke_SF)
*/
/******************************************************************************/
/* Ke SI * FRACT16_SCALE */
static inline uint32_t ke_vrads_fract16(uint16_t kv) { return ((uint64_t)FRACT16_MAX * FRACT16_SCALE * 60 / (2 * FRACT16_PI)) / (kv); }

/* Ke_SI in millivolts per rad/s from Kv [RPM/V]: Ke_SI = 60000 / (2π × Kv) [mV/(rad/s)]. */
static inline uint32_t ke_mvrads(uint16_t kv) { return ((uint32_t)60 * 1000 * FRACT16_SCALE / (2 * FRACT16_PI)) / kv; }

static inline uint32_t ke_v_per_angle16(uint16_t kv) { return ((uint64_t)FRACT16_MAX * FRACT16_SCALE * 60 / ANGLE16_PER_REVOLUTION) / (kv); }

/*
    [VFract16 / SpeedFract16]
    V_emf = speed_rpm / Kv
    V_emf_fract16 / speed_fract16 = [speed_rpm / (Kv × V_max) × 32768] / [speed_rpm / speed_max × 32768]
        = speed_max / (Kv × V_max)
*/
static inline uint32_t ke_speed_fract16(uint16_t v_max_volts, uint16_t speed_max_rpm, uint16_t kv) { return fract16_div(speed_max_rpm, kv_rpm_per_unit(v_max_volts, kv)); }

/* [VFract16 / (Angle16/ControlCycle)] */
static inline uint32_t ke_vfract16_per_angle16(uint32_t polling_freq, uint16_t v_max_volts, uint16_t kv) { return (uint64_t)ke_v_per_angle16(kv) * polling_freq / v_max_volts; }


/******************************************************************************/
/*!
    Psi — Rotor Flux Linkage Ke in Electrical Domain
    ψ_f [Wb] = Ke_SI [V/(rad/s)] = 60 / (2π × Kv) / PolePairs
    Psi_fract16 = π × ψ_f [V/(elec rad/s)] × Fs / V_max

    Applied as: v_emf_fract16 = fract16_mul(el_delta_angle16, Psi_fract16)
        (el_delta × Psi) >> 15 = ψ_f × el_delta × π × Fs / V_max
*/
/******************************************************************************/
/* [V / (Rad/s)] * FRACT16_SCALE */
/* SI Ref */
static inline uint32_t psi_rads_fract16(uint16_t kv, uint8_t polePairs) { return ke_vrads_fract16(kv) / polePairs; }

/* = ke_SI × FRACT16_SCALE × Fs / (V_max × P) = ψ_f × Fs × 32768 / V_max */
// static inline uint32_t psi_vfract16_per_rads(uint32_t polling_freq, uint16_t v_max_volts, uint16_t kv, uint8_t polePairs)
// {
//     return (uint64_t)ke_vrads_fract16(kv) * polling_freq / v_max_volts / polePairs;
// }

/*
    = ψ_f × π × FRACT16_SCALE × Fs / V_max

    Derivation:
    ω_e = el_delta_angle16 × 2π × Fs / 65536

    V_emf = ψ_f × ω_e = ψ_f × el_delta × 2π × Fs / 65536
    v_emf_fract16 = V_emf / V_max × 32768
        = ψ_f × ω_e / V_max × 32768
        = ψ_f × el_delta_angle16 × π × Fs / V_max    [2π×32768/65536 = π]

    KPsi_required = ψ_f × π × Fs × 32768 / V_max
        = 30 × Fs × 32768 / (Kv × P × V_max)
*/
/* v_emf_fract16 = fract16_mul(el_delta_angle16, Psi_fract16) */
static inline uint32_t psi_vfract16_per_angle16(uint32_t polling_freq, uint16_t v_max_volts, uint16_t kv, uint8_t polePairs)
{
    // return (uint64_t)30U * polling_freq * FRACT16_SCALE / ((uint64_t)kv * polePairs * v_max_volts);
    return ke_vfract16_per_angle16(polling_freq, v_max_volts, kv) / polePairs;
}

// static inline fract16_t psi_vfract16_per_rads_sat(uint32_t polling_freq, uint16_t v_max_volts, uint16_t kv, uint8_t polePairs)
// {
//     assert(kv != 0U && v_max_volts != 0U);
//     return (fract16_t)math_min(psi_vfract16_per_rads(polling_freq, v_max_volts, kv, polePairs), (uint32_t)FRACT16_MAX);
// }

/* Psi_fract16 from Ke_SI [V/(rad/s)] stored as millivolts-per-rads (×1000 for integer). */
/* Psi = π × ke_SI × Fs / V_max = 3141593 × ke_mV × Fs / (V_max × 1e9) */
static inline uint32_t psi_vfract16_of_ke_mvrads(uint32_t polling_freq, uint16_t v_max_volts, uint32_t ke_mV_per_rads)
{
    return (uint32_t)((uint64_t)3141593ULL * ke_mV_per_rads * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_volts * 1000000ULL * 1000ULL));
}

// static inline fract16_t psi_vfract16_of_ke_mvrads_sat(uint32_t polling_freq, uint16_t v_max_volts, uint32_t ke_mV_per_rads)
// {
//     assert(v_max_volts != 0U);
//     return (fract16_t)math_min(psi_vfract16_of_ke_mvrads(polling_freq, v_max_volts, ke_mV_per_rads), (uint32_t)FRACT16_MAX);
// }




/******************************************************************************/
/*!
    Rs — Stator Resistance Normalization
        R_ref = V_max / I_max     [Ω per full-scale unit]
        Rs_fract16 = Rs [Ω] / R_ref = Rs × I_max × FRACT16_MAX / V_max
                   = Rs_mOhm × I_max × FRACT16_MAX / (V_max × 1000)
*/
/******************************************************************************/
/* Rs_fract16 from measured resistance in milliohms. Returns uint32 (unsaturated) for _sat to clamp. */
static inline uint32_t rs_fract16_of_mohms(uint16_t v_max_volts, uint16_t i_max_amps, uint16_t rs_mOhm)
{
    return (uint32_t)(((uint64_t)rs_mOhm * i_max_amps * FRACT16_SCALE) / ((uint32_t)v_max_volts * 1000U));
}

// static inline fract16_t rs_fract16_of_mohms_sat(uint16_t v_max_volts, uint16_t i_max_amps, uint16_t rs_mOhm)
// {
//     assert(v_max_volts != 0U);
//     return (fract16_t)math_min(rs_fract16_of_mohms(v_max_volts, i_max_amps, rs_mOhm), (uint32_t)FRACT16_MAX);
// }

/* Rs back to milliohms from stored fract16 value. Result always fits uint32 (≤ V_max×1000/I_max). */
static inline uint32_t rs_mohms_of_fract16(uint16_t v_max_volts, uint16_t i_max_amps, ufract16_t rs_f)
{
    return (uint32_t)(((uint64_t)rs_f * v_max_volts * 1000U) / (uint32_t)i_max_amps / FRACT16_SCALE);
}

// static inline uint32_t rs_mohms_of_fract16_sat(uint16_t v_max_volts, uint16_t i_max_amps, ufract16_t rs_f)
// {
//     assert(i_max_amps != 0U && rs_f >= 0);
//     return rs_mohms_of_fract16(v_max_volts, i_max_amps, rs_f);
// }

/* Rs_Fract16 = Vd_Fract16 / Id_Fract16 (normalized by R_REF = V_MAX/I_MAX). */
static inline int32_t rs_fract16(int32_t vd_fract16, int32_t id_fract16)
{
    return fract16_div(vd_fract16, id_fract16);
}

// static inline fract16_t rs_fract16_sat(int32_t vd_fract16, int32_t id_fract16)
// {
//     assert(id_fract16 != 0);
//     return (fract16_t)math_clamp(rs_fract16(vd_fract16, id_fract16), 0, FRACT16_MAX);
// }

/******************************************************************************/
/*!
    L — Inductance Normalization
        SI storage: L_uH [µH]
        In-loop form: KL_fract16 — dq cross-coupling decoupling coefficient.

        Applied as: omega_L_fract16 = fract16_mul(el_delta_angle16, KL_fract16)
            where el_delta_angle16 is the electrical angle step per control cycle [angle16/poll].

        Derivation:
            v_cross [V] = ω_e [rad/s] × L [H] × I [A]
            ω_e = el_delta × 2π × Fs / ANGLE16_PER_REVOLUTION
            KL = π × L [H] × I_max [A] × Fs [Hz] / V_max [V]     (dimensionless)

        L from RL time-constant:  τ = L/R  →  L_uH = Rs_mOhm × τ_cycles × 1000 / Fs
        L from HFI:               L_uH = V_hf / (2π × f_hf × I_hf) × 1e6
*/
/******************************************************************************/

/* L_uH from RL time-constant τ [control cycles]: L = Rs × τ  →  uH = mOhm × cycles × 1000 / Fs. */
static inline uint16_t l_uh_of_rs_tau(uint32_t polling_freq, uint16_t rs_mOhm, uint32_t tau_cycles)
{
    return ((uint64_t)rs_mOhm * tau_cycles * 1000U) / polling_freq;
}

// static inline uint16_t l_uh_of_rs_tau_sat(uint32_t polling_freq, uint16_t rs_mOhm, uint32_t tau_cycles)
// {
//     assert(polling_freq != 0U);
//     return (uint16_t)math_min(l_uh_of_rs_tau(polling_freq, rs_mOhm, tau_cycles), (uint32_t)UINT16_MAX);
// }

/* L_uH from HFI: L = V_hf / (2π × f_hf × I_hf). vhf_fract16, iq_amp_fract16 normalized to [V_max, I_max]. */
static inline uint32_t l_uh_of_hfi(uint16_t v_max_volts, uint16_t i_max_amps, uint32_t hf_freq, uint16_t vhf_fract16, uint32_t iq_amp_fract16)
{
    return (uint32_t)((uint64_t)vhf_fract16 * v_max_volts * 1000000ULL / ((uint64_t)iq_amp_fract16 * i_max_amps * 628318ULL * hf_freq / 100000ULL));

}
// static inline uint16_t l_uh_of_hfi_sat(uint16_t v_max_volts, uint16_t i_max_amps, uint32_t hf_freq, uint16_t vhf_fract16, uint32_t iq_amp_fract16)
// {
//     assert(iq_amp_fract16 != 0U && hf_freq != 0U);
//     return (uint16_t)math_min(l_uh_of_hfi(v_max_volts, i_max_amps, hf_freq, vhf_fract16, iq_amp_fract16), (uint32_t)UINT16_MAX);
// }


/*
    Decouple coefficients from Ld/Lq [uH]:
        KL_Fract16 = π × L[H] × I_max × Fs / V_max

    where fract16_mul(delta_angle16, KL) yields omega_e * L * I_MAX / V_MAX in fract16 voltage basis.

    KL_fract16 from inductance in µH. KL = π × L_uH × 1e-6 × I_max × Fs / V_max  = π×10^6 × L_uH × I_max × Fs / (V_max × 10^12)
*/
static inline uint32_t kl_fract16_of_uh(uint32_t polling_freq, uint16_t v_max_volts, uint16_t i_max_amps, uint16_t l_uH)
{
    return (uint32_t)((uint64_t)3141593ULL * l_uH * i_max_amps * polling_freq * FRACT16_SCALE / ((uint64_t)v_max_volts * 1000000ULL * 1000000ULL));
}

// static inline fract16_t kl_fract16_of_uh_sat(uint32_t polling_freq, uint16_t v_max_volts, uint16_t i_max_amps, uint16_t l_uH)
// {
//     assert(v_max_volts != 0U);
//     return (fract16_t)math_min(kl_fract16_of_uh(polling_freq, v_max_volts, i_max_amps, l_uH), (uint32_t)FRACT16_MAX);
// }



