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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*!
    _fract16  — Q1.15 normalized to hardware full-scale [V_max, I_max, etc.]
    _rpm      — mechanical speed in RPM
    _rads     — angular velocity in rad/s
    kv        — motor speed constant [RPM / V]
    v_max     — hardware peak voltage [V] (ADC full-scale, not bus)
    i_max     — hardware peak current [A]
    polling_freq — control loop rate [Hz]
*/
/******************************************************************************/
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
static inline int32_t rpm_of_kv_vfract16(uint16_t v_max_volts, uint16_t kv, int16_t v_fract16) { return fract16_mul(v_fract16, kv_rpm_per_unit(kv, v_max_volts)); }

/* v_fract16_of_kv_rpm: V_fract16 = Speed / Kv.  rpm in [0, kv × v_max]. */
static inline int32_t vfract16_of_kv_rpm(uint16_t v_max_volts, uint16_t kv, int32_t rpm) { return fract16_div(rpm, kv_rpm_per_unit(kv, v_max_volts)); }

/******************************************************************************/
/*!
    Ke — Normalized Voltage-per-Speed (back-EMF constant)

    Speed_fract16 domain:
        V_fract16 = fract16_mul(Ke_SF, Speed_fract16)
        Ke_SF = V_nominal_fract16 / Speed_rated_fract16
              = V_nominal_fract16 / (FRACT16_MAX/2)     [when max = 2×rated]
              = 2 × V_nominal_fract16
        Usage: V and Speed in the same normalized basis → Ke_SF is independent of Kv.

    Rad/s domain (normalized to V_max):
        Ke_rads_fract16 = (60 × FRACT16_MAX²) / (2π × Kv × V_max × FRACT16_MAX)
                        = (60 × FRACT16_MAX)  / (2π × Kv × V_max)
        Usage: fract16_mul(Ke_rads_fract16, omega_normalized_fract16) → V_fract16
               where omega_normalized_fract16 = ω [rad/s] normalized to some ω_max.
*/
/******************************************************************************/
/* Ke in Speed_fract16 domain. v_fract16 = fract16_mul(ke_sf, speed_fract16). */
static inline int32_t ke_speed_fract16(int16_t v_nominal_fract16, int16_t speed_rated_fract16_val) { return fract16_div(v_nominal_fract16, speed_rated_fract16_val); }

/* Ke in rad/s domain, normalized to V_max. Uses 64-bit intermediate to avoid overflow. */
static inline int32_t ke_rads_fract16(uint16_t v_max_volts, uint16_t kv) { return (int64_t)60 * FRACT16_MAX * FRACT16_MAX / ((int64_t)2 * FRACT16_PI * kv * v_max_volts); }

/* Ke_SI in millivolts per rad/s from Kv [RPM/V]: Ke_SI = 60000 / (2π × Kv) [mV/(rad/s)]. */
static inline uint32_t ke_mvrads_of_kv(uint16_t kv) { return (kv > 0U) ? (60000U * FRACT16_MAX / (2U * FRACT16_PI * kv)) : 0U; }


/******************************************************************************/
/*!
    Rs — Stator Resistance Normalization
        R_ref = V_max / I_max     [Ω per full-scale unit]
        Rs_fract16 = Rs [Ω] / R_ref = Rs × I_max × FRACT16_MAX / V_max
                   = Rs_mOhm × I_max × FRACT16_MAX / (V_max × 1000)

        Rs_fract16 is the normalized resistance used directly in PI feedforward:
            V_fract16 = fract16_mul(Rs_fract16, I_fract16)
*/
/******************************************************************************/
/* Rs_fract16 from measured resistance in milliohms. */
static inline fract16_t rs_fract16_of_mohms(uint16_t v_max_volts, uint16_t i_max_amps, uint16_t rs_mOhm) { return (fract16_t)math_min((uint32_t)(((uint64_t)rs_mOhm * i_max_amps * FRACT16_MAX) / ((uint32_t)v_max_volts * 1000U)), (uint32_t)FRACT16_MAX); }

/* Rs back to milliohms from stored fract16 value. */
static inline uint32_t rs_mohms_of_fract16(uint16_t v_max_volts, uint16_t i_max_amps, fract16_t rs_fract16) { return ((uint32_t)rs_fract16 * v_max_volts * 1000U) / ((uint32_t)i_max_amps * FRACT16_MAX); }

/* Rs_Fract16 = Vd_Fract16 / Id_Fract16 (normalized by R_REF = V_MAX/I_MAX). */
static inline fract16_t rs_fract16(int32_t vd_fract16, int32_t id_fract16) { return (fract16_t)math_clamp(fract16_div(vd_fract16, id_fract16), 0, FRACT16_MAX); }


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
    if (polling_freq == 0U) { return 0U; }
    uint32_t uH = ((uint32_t)rs_mOhm * tau_cycles * 1000U) / polling_freq;
    return (uint16_t)math_min(uH, (uint32_t)UINT16_MAX);
}

/* L_uH from high-frequency injection: L = V_hf / (2π × f_hf × I_hf). vhf_fract16, iq_amp_fract16 are normalized to [V_max, I_max] respectively. */
static inline uint16_t l_uh_of_hfi(uint16_t v_max_volts, uint16_t i_max_amps, uint32_t hf_freq, uint16_t vhf_fract16, uint32_t iq_amp_fract16)
{
    if (iq_amp_fract16 == 0U || hf_freq == 0U) { return 0U; }
    uint64_t num = (uint64_t)vhf_fract16 * v_max_volts * 1000000ULL;
    uint64_t den = (uint64_t)iq_amp_fract16 * i_max_amps * 628318ULL * hf_freq / 100000ULL;
    if (den == 0U) { return 0U; }
    return (uint16_t)math_min((uint32_t)(num / den), (uint32_t)UINT16_MAX);
}



/*
    Decouple coefficients from Ld/Lq [H]:
        KL_Fract16 = pi * L * I_MAX / (V_MAX * Ts)

    where fract16_mul(delta_angle16, KL) yields omega_e * L * I_MAX / V_MAX in fract16 voltage basis.
    With L in uH, V_MAX in volts, Ts = 1/MOTOR_CONTROL_FREQ s:
        KL = pi * (uH/1e6) * I_MAX * MOTOR_CONTROL_FREQ / V_MAX
           = pi * uH * I_MAX * MOTOR_CONTROL_FREQ / (V_MAX * 1e6)
*/
/* KL_fract16 from inductance in µH. KL = π × L_uH × 1e-6 × I_max × Fs / V_max  = π×10^6 × L_uH × I_max × Fs / (V_max × 10^12) */
static inline fract16_t kl_fract16_of_uh(uint32_t polling_freq, uint16_t v_max_volts, uint16_t i_max_amps, uint16_t l_uH)
{
    if (v_max_volts == 0U) { return 0; }
    uint64_t num = (uint64_t)3141593ULL * l_uH * i_max_amps * polling_freq;
    uint64_t den = (uint64_t)v_max_volts * 1000000ULL * 1000000ULL;
    return (fract16_t)math_min((uint32_t)(num / den), (uint32_t)FRACT16_MAX);
}

/******************************************************************************/
/*!
    KPsi — Rotor Flux Linkage / Ke in Control Domain
        ψ_f [Wb] = Ke_SI [V/(rad/s)] = 60 / (2π × Kv)
        KPsi_fract16 — back-EMF decoupling / feedforward coefficient.

        Applied as: v_emf_fract16 = fract16_mul(el_delta_angle16, KPsi_fract16)

        Derivation (same form as KL, substituting ψ_f for L × I_max):
            v_emf = ω_e × ψ_f = el_delta × π × Fs × ψ_f / V_max
            KPsi = π × ψ_f × Fs / V_max
                 = π × 60 / (2π × Kv) × Fs / V_max
                 = 30 × Fs / (Kv × V_max)
*/
/******************************************************************************/
/* KPsi_fract16 from Kv [RPM/V]. */
static inline fract16_t kpsi_fract16_of_kv(uint32_t polling_freq, uint16_t v_max_volts, uint16_t kv)
{
    if (kv == 0U || v_max_volts == 0U) { return 0; }
    uint32_t num = 30U * polling_freq;
    uint32_t den = (uint32_t)kv * v_max_volts;
    return (fract16_t)math_min(num / den, (uint32_t)FRACT16_MAX);
}

/* KPsi_fract16 from Ke_SI [V/(rad/s)] stored as millivolts-per-rads (×1000 for integer). */
static inline fract16_t kpsi_fract16_of_ke_mvrads(uint32_t polling_freq, uint16_t v_max_volts, uint32_t ke_mV_per_rads)
{
    if (v_max_volts == 0U) { return 0; }
    /* KPsi = π × ke_SI × Fs / V_max = 3141593 × ke_mV × Fs / (V_max × 1e9) */
    uint64_t num = (uint64_t)3141593ULL * ke_mV_per_rads * polling_freq;
    uint64_t den = (uint64_t)v_max_volts * 1000000ULL * 1000ULL;
    return (fract16_t)math_min((uint32_t)(num / den), (uint32_t)FRACT16_MAX);
}



/*
    Psi_f [Wb] = 60 / (2*pi * Kv_RpmPerVolt * PolePairs)
    KPsi_Fract16 = pi * Psi_f / (V_MAX * Ts) = pi * Psi_f * Fs / V_MAX
*/
// static fract16_t  kpsi_fract16(uint16_t kv_rpmPerVolt, uint8_t polePairs)
// {
//     uint32_t v_max = Phase_Calibration_GetVMaxVolts();
//     if (kv_rpmPerVolt == 0U || polePairs == 0U || v_max == 0U) { return 0; }
//     /* KPsi = pi * 60 * Fs / (2*pi * Kv * PolePairs * V_MAX) = 30 * Fs / (Kv * PolePairs * V_MAX) */
//     uint64_t num = (uint64_t)30U * MOTOR_CONTROL_FREQ * 32768ULL;
//     uint64_t den = (uint64_t)kv_rpmPerVolt * polePairs * v_max;
//     uint64_t k = num / den;
//     if (k > FRACT16_MAX) { k = FRACT16_MAX; }
//     return (fract16_t)k;
// }




// /******************************************************************************/
// /*!
//     Speed — Fract16 Normalization
//         Speed_fract16 = Speed_Rpm × FRACT16_MAX / SpeedTypeMax_Rpm
//         Convention: SpeedTypeMax = 2 × SpeedRated, so SpeedRated maps to FRACT16_MAX/2.
// */
// /******************************************************************************/
// /* speed_type_max_rpm: full-scale denominator = 2 × rated (leaves headroom for field-weakening). */
// static inline uint32_t speed_type_max_rpm(uint16_t speed_rated_rpm) { return (uint32_t)speed_rated_rpm * 2U; }

// /* rpm → fract16.  max_rpm is the full-scale denominator (e.g. speed_type_max_rpm). */
// static inline int16_t speed_fract16_of_rpm(int32_t rpm, uint32_t max_rpm) { return (int16_t)(((int64_t)rpm * FRACT16_MAX) / (int32_t)max_rpm); }

// /* fract16 → rpm. */
// static inline int32_t rpm_of_speed_fract16(int16_t speed_fract16, uint32_t max_rpm) { return ((int32_t)speed_fract16 * (int32_t)max_rpm) >> FRACT16_N_BITS; }

// /* Rated speed as fract16: FRACT16_MAX/2 when max = 2×rated. */
// static inline int16_t speed_rated_fract16(uint16_t speed_rated_rpm, uint32_t max_rpm) { return speed_fract16_of_rpm(speed_rated_rpm, max_rpm); }


