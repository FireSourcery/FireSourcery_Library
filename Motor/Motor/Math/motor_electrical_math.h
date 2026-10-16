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
    @file   motor_electrical_math.h
    @author FireSourcery
    @brief  Pure math for PMSM electrical parameters — identification from raw
            measurements, unit conversion, and per-unit encoding for the
            control-loop basis.

    Parameters covered: Rs, Ls / Ld / Lq, Ke, ψ_f, Kv, Kt.

    Representation domains:
        SI :    Ω, H, V·s/rad, Wb, Nm/A
        SI storage:    mΩ, µH, µV·s/rad, µWb, µNm/A     (integer storage, ~0.1% precision)

    Pure functions: textbook formula, no guards, wider return preserves pre-sat value.
    PU macros: textbook formula, unscaled [1]. The destination applies the representation:
        ACCUM32, FRACT16 for storage, Q15 in every build. ACCUM, FRACT [real.h] at runtime.
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/angle_speed_math.h"

#include <stdint.h>
#include <assert.h>


/******************************************************************************/
/*!
    @brief  Phase-basis amplitude conversions (Y-connected motors).

        V_phase    = V_LL / √3        V_LL = √3 · V_phase
        V_pk       = √2 · V_rms       V_rms = V_pk / √2
*/
/******************************************************************************/
static inline uint32_t v_phase_of_ll(uint32_t v_ll) { return (uint64_t)v_ll * FRACT16_SCALE / FRACT16_SQRT3; }
static inline uint32_t v_ll_of_phase(uint32_t v_phase) { return (uint64_t)v_phase * FRACT16_SQRT3 / FRACT16_SCALE; }
static inline uint32_t v_peak_of_rms(uint32_t v_rms) { return (uint64_t)v_rms * FRACT16_SQRT2 / FRACT16_SCALE; }
static inline uint32_t v_rms_of_peak(uint32_t v_peak) { return (uint64_t)v_peak * FRACT16_SCALE / FRACT16_SQRT2; }


/******************************************************************************/
/*!
    @brief  Voltage-Model Back-EMF Estimator

        v = R·i + L·di/dt + ω·ψ
        e = ω·ψ = v - Rs·i - Ls·di/dt

    Discrete one-step form, caller supplies the precomputed Δi = i[k] - i[k-1]:
        ê[k] = v[k] - Rs_pu·i[k] - Ls_pu·Δi[k]
*/
/******************************************************************************/
static inline accum32_t motor_v_stator(accum32_t Rs_pu, accum32_t Ls_pu, fract16_t i_prev, fract16_t i) { return accum32_mul(Rs_pu, i) + accum32_mul(Ls_pu, i - i_prev); }
/* Ls at the tick base, ω_base = π · Fs: Fs / ω_base = 1 / π */
// static inline accum32_t motor_v_stator_tick(accum32_t Rs_pu, accum32_t Ls_tick, fract16_t i_prev, fract16_t i) { return motor_v_stator(Rs_pu, accum32_mul(Ls_tick, FRACT16_1_DIV_PI), i_prev, i); }
static inline accum32_t motor_emf(accum32_t Rs_pu, accum32_t Ls_pu, fract16_t i_prev, fract16_t i, fract16_t v) { return v - motor_v_stator(Rs_pu, Ls_pu, i_prev, i); }

/******************************************************************************/
/*!
    @brief  Electrical angle and mechanical RPM conversions.

    el_rads / mech_rpm through angle freq
*/
/******************************************************************************/
/* ANGLE_DT_OF_RPM() * PolePairs */
static inline int32_t el_angle_dt_of_mech_rpm(uint32_t fs, uint8_t polePairs, int16_t rpm) { return angle_dt_of_rpm(fs, (int32_t)rpm * polePairs); }
static inline int32_t mech_rpm_of_el_angle_dt(uint32_t fs, uint8_t polePairs, angle_dt_t angle_dt) { return rpm_of_angle_dt(fs, angle_dt) / polePairs; }
/* Rounds to nearest, inverting a truncated freq */
// static inline uint32_t mech_rpm_of_el_angle_freq(uint8_t polePairs, angle_freq_t freq) { return ((uint64_t)freq * SECONDS_PER_MINUTE + ANGLE16_PER_REVOLUTION / 2U * polePairs) / ((uint64_t)ANGLE16_PER_REVOLUTION * polePairs); }
static inline uint32_t mech_rpm_of_el_angle_freq(uint8_t polePairs, angle_freq_t freq) { return ((uint64_t)freq * SECONDS_PER_MINUTE) / (ANGLE16_PER_REVOLUTION * polePairs); }


/******************************************************************************/
/*!
    @brief  Rs — Stator Resistance [Ω]

    DC injection (di/dt = 0, EMF = 0 at standstill):
        Rs       = V / I                    (single conductor)
        Rs_phase = V_LL / (2·I)             (Y, two phases in series)

    Online under FOC DC excitation (iq=0, id=id_ref, ω=0):
        Rs_pu = vd_pu / id_pu               (ratio invariant under PU)

    PU encoding:  Rs_pu  = Rs · I_base / V_base
*/
/******************************************************************************/
#define MOTOR_R_PU(V_Base, I_Base, R_Ohm)           ((float)(R_Ohm) * (I_Base) / (V_Base))
#define MOTOR_R_PU_OF_MOHM(V_Base, I_Base, R_mOhm)  MOTOR_R_PU(V_Base, I_Base, (R_mOhm) * 1.0E-3F)

static inline uint32_t rs_pu_of_ohm(uint16_t v_base_V, uint16_t i_base_A, uint32_t rs_ohm, uint32_t si_scale) { return (uint64_t)rs_ohm * i_base_A * FRACT16_SCALE / ((uint32_t)v_base_V * si_scale); }
static inline uint32_t rs_ohm_of_pu(uint16_t v_base_V, uint16_t i_base_A, uint32_t rs_pu, uint32_t si_scale) { return (uint64_t)rs_pu * v_base_V * si_scale / ((uint32_t)i_base_A * FRACT16_SCALE); }

static inline uint32_t rs_pu_of_mohm(uint16_t v_base_V, uint16_t i_base_A, uint32_t rs_mOhm) { return (uint64_t)rs_mOhm * i_base_A * FRACT16_SCALE / ((uint32_t)v_base_V * 1000UL); }
static inline uint32_t rs_mohm_of_pu(uint16_t v_base_V, uint16_t i_base_A, uint32_t rs_pu) { return (uint64_t)rs_pu * v_base_V * 1000UL / ((uint32_t)i_base_A * FRACT16_SCALE); }

static inline accum32_t rs_pu_of_vi(fract16_t vd_pu, fract16_t id_pu) { return fract16_div_sat(vd_pu, id_pu); }

static inline uint32_t rs_mohm_of_vi(uint32_t v_mV, uint32_t i_mA) { return (uint64_t)v_mV * 1000UL / i_mA; }
static inline uint32_t rs_mohm_of_vi_ll_y(uint32_t v_mV_ll, uint32_t i_mA) { return (uint64_t)v_mV_ll * 1000UL / (2UL * i_mA); }


/******************************************************************************/
/*!
    @brief  Speed base ω_base [angle16/s] — ψ_pu and L_pu scale with the base

        x_pu ∝ ω_base = F_base · π / 2^15 [rad/s]
        Tick base: F_base = angle_freq_nyquist(Fs), ω_base = π · Fs
*/
/******************************************************************************/
/* x_pu ∝ ω_base: time on top (ψ, L). Speed pu scales inversely */
static inline uint32_t pu_tau_rebase(uint32_t x_pu, angle_freq_t base_from, angle_freq_t base_to) { return (uint64_t)x_pu * base_to / base_from; }

/*!
    @brief [angle16/s · A/V]
    integer storage form. free of a speed base, Fs, and the board base.

    AngleFreqPerVolt  [angle16/s / V]
    AngleFreqPerOhm   [angle16/s · A/V],   ψ / L [A], the short-circuit current.
*/
#define MOTOR_ELECTRICAL_INV(X_SPerRad) ANGLE_FREQ_OF_RADS(1.0F / (X_SPerRad))
// static inline angle_freq_t inv_of_pu(angle_freq_t base, uint16_t x_base_si_factor, uint16_t x_base_si_divisor, uint32_t pu) { return ((uint64_t)base << 15) * x_base_si_factor / ((uint64_t)pu * x_base_si_divisor); }


/******************************************************************************/
/*!
    @brief ψ_f — Back-EMF and rotor flux [Wb] [Volt·sec/rad]

    ψ_pu — Back-EMF and rotor flux, per-unit at a speed base [angle16/s]
    ψ_pu = ψ_f · ω_base / V_base
        fract16_mul(ω_pu, ψ_pu) → v_emf_pu. Tick base: ω_pu ≡ Δθ [angle16/Ts]

    Identification:
        Spin test (generator at known speed):
            ψ_f = V_phase / ω_elec = V_LL_pk / (√3 · ω_elec)
        Steady-state FOC :
            ψ_f = (vq − Rs·iq − ω_e·Ld·id) / ω_e
*/
/******************************************************************************/
/* ψ [V·s/rad] · ω_base [rad/s] / V_base [V] = [1] */
#define MOTOR_PSI_PU(Freq_Base, V_Base, Psi_Webers) ((float)(Psi_Webers) * RADS_OF_ANGLE_FREQ(Freq_Base) / (V_Base))

/* ψ_pu_tick [Volt·Ts/angle16]. Tick base ω_base = π · Fs */
#define MOTOR_PSI_PU_TICK(Fs, V_Base, Psi_Webers)   ((float)(Psi_Webers) * PI_FLOAT * (Fs) / (V_Base))


/*
    1 angle16/Ts = 2π·Fs/65536 rad/s = π·Fs / 2^15: π·Fs folds into ψ_pu, 2^15 is the fract16_mul shift.
    Fs · FRACT16_PI = Fs · π · 2^15 = Fs · 2^30 / ANGLE16_PER_RADIAN
*/
static inline uint32_t psi_pu_tick_of_wb(uint32_t fs_hz, uint16_t v_base_V, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)psi_Wb * FRACT16_PI * fs_hz / ((uint64_t)v_base_V * scale); }
static inline uint32_t psi_wb_of_pu_tick(uint32_t fs_hz, uint16_t v_base_V, uint32_t psi_pu, uint32_t scale) { return (uint64_t)psi_pu * v_base_V * scale / ((uint64_t)FRACT16_PI * fs_hz); }



/* Q15 constant divides first, the variable divisor last: one truncation in effect */
/* angle_freq_t applies integer radians, alternatively truncate in Hz */
/* base / FRACT16_SCALE * FRACT16_PI == base / 2^16 * 2π * FRACT16_SCALE */
static inline uint32_t psi_pu_of_wb(uint16_t v_base_V, angle_freq_t base, uint32_t psi_Wb, uint32_t scale) { return (uint64_t)psi_Wb * base / FRACT16_SCALE * FRACT16_PI / ((uint64_t)v_base_V * scale); }
static inline uint32_t psi_wb_of_pu(uint16_t v_base_V, angle_freq_t base, uint32_t psi_pu, uint32_t scale) { return (uint64_t)psi_pu * v_base_V * scale / FRACT16_PI * FRACT16_SCALE / base; }


/*!
    @brief  BEMF speed per volt [angle16/s / Volt] — the motor constant as a rate, free of a speed base.

        AngleF(V) = V / ψ        linear in V. The rate F / V = 1 / ψ, V phase peak.
        F at a V: angle_freq_at_v. At V = V_base, ψ_pu = 1.0 at F
        ψ [V·s/rad] per volt is [s/rad]: angle_freq_of_s_per_rad, s_per_rad_of_angle_freq
*/
/*
    Angle Freq Per Volt: (1/ψ)·2^15/π [angle16/s / V]
    From the rate: ψ_pu = F_base / (AngleFreqPerVolt · V_base)
    Tick base: ψ_tick = Fs · 2^30 / (AngleFreqPerVolt · V_base)
*/
// #define MOTOR_PSI_INV(Psi_Webers) ANGLE_FREQ_OF_RADS(1.0F / (Psi_Webers))
#define MOTOR_ANGLE_FREQ_OF_PSI(Psi_Webers) ANGLE_FREQ_OF_RADS(1.0F / (Psi_Webers))

static inline uint32_t psi_pu_of_angle_freq(angle_freq_t base, uint16_t v_base_V, angle_freq_t rate) { return ((uint64_t)base << 15) / ((uint64_t)rate * v_base_V); }
static inline angle_freq_t angle_freq_of_psi_pu(angle_freq_t base, uint16_t v_base_V, uint32_t psi_pu) { return ((uint64_t)base << 15) / ((uint64_t)psi_pu * v_base_V); }

/* Rating: V_Emf phase peak at Hz electrical, or at Rpm mechanical */
/* 1/(V/Hz) */
#define MOTOR_ANGLE_FREQ_OF_V_HZ(V_Emf, Hz) ANGLE_FREQ((float)(Hz) / (V_Emf))


/*
    Kv [rpm/V], M = 2: phase peak = V_bus / 2 at Kv · V_bus rpm.
    AngleFreq per phase peak volt. (without svpwm)
*/
/* (double), so it equals angle_freq_of_kv for an integer Kv, and keeps a fractional Kv */
#define MOTOR_ANGLE_FREQ_OF_KV(Kv, PolePairs) ((angle_freq_t)((double)(Kv) * (PolePairs) * 2U * ANGLE16_PER_REVOLUTION / SECONDS_PER_MINUTE))

/* Kv rounds to nearest, inverting the truncated rate */
static inline angle_freq_t angle_freq_of_kv(uint16_t kv, uint8_t polePairs) { return angle_freq_of_rpm((uint32_t)kv * polePairs * 2U); }
static inline uint16_t kv_of_angle_freq(angle_freq_t rate, uint8_t polePairs) { return rpm_of_angle_freq(rate) / (polePairs * 2U); }
static inline uint16_t kv_of_angle_freq_round(angle_freq_t rate, uint8_t polePairs) { return (rpm_of_angle_freq(rate) + polePairs) / (polePairs * 2U); }
// static inline uint16_t kv_of_angle_freq(angle_freq_t rate, uint8_t polePairs) { return ((uint64_t)rate * SECONDS_PER_MINUTE + (uint64_t)polePairs * ANGLE16_PER_REVOLUTION) / ((uint64_t)polePairs * 2U * ANGLE16_PER_REVOLUTION); }


/*
    PU identification — direct from FOC-loop quantities.

    Spin-test form is the literal inverse of the runtime BEMF computation fract16_mul(omega_step, psi_pu).
       ψ_pu = v_emf_pu · 32768 / omega_step  =  fract16_div(v_emf_pu, omega_step)   [spin test]
       ψ_pu = residual_v_pu / omega_step,  residual = vq − Rs·iq − ω·Ld·id          [online]
*/
static inline uint32_t psi_pu_of_emf(fract16_t v_emf_pu, fract16_t omega_step) { return fract16_div(v_emf_pu, omega_step); }

/* Online ψ_f under steady-state FOC. */
static inline uint32_t psi_pu_of_running(fract16_t rs_pu, fract16_t ld_pu, fract16_t omega_step, fract16_t vq_pu, fract16_t id_pu, fract16_t iq_pu)
{
    fract16_t omega_Ld = fract16_mul(omega_step, ld_pu);
    fract16_t residual = fract16_sat((accum32_t)vq_pu - fract16_mul(rs_pu, iq_pu) - fract16_mul(omega_Ld, id_pu));
    return fract16_div(residual, omega_step);
}

/*
    Numerator built in µV (mΩ·mA = µV exactly).
    Caller ensures vq·1e3 ≥ v_r + v_l (physically required for ψ > 0).
*/
static inline uint32_t psi_uwb_of_emf(uint32_t v_phase_pk, uint32_t omega_rads) { return (uint64_t)v_phase_pk * 1000000UL / omega_rads; }
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
    e.g VNominal, SpeedRated
*/
static inline uint32_t psi_of_emf(fract16_t v_emf, fract16_t omega_rads, fract16_t scale) { return (v_emf * scale / omega_rads); }
static inline uint32_t psi_pu_of_emf_rate(uint16_t v_base_V, uint32_t omega_base, uint16_t v_emf, uint32_t omega_emf)
{
    return (uint64_t)v_emf * omega_base * FRACT16_SCALE / ((uint64_t)omega_emf * v_base_V);
}


/******************************************************************************/
/*!
    @brief  Kt — Torque Constant [Nm/A]

        FOC peak-iq:        Kt = (3/2) · P · ψ_f          T_em = Kt · iq
        Motor-constant:     Kt [Nm/A_rms] = 60 / (2π · Kv) = Ke_mech (numerically)
*/
/******************************************************************************/
// static inline uint32_t kt_unm_per_amp_of_psi(uint32_t psi_uWb, uint8_t polePairs) { return 3UL * psi_uWb * polePairs / 2UL; }


/******************************************************************************/
/*!
    @brief  Ls / Ld / Lq — Stator Inductance [H] [Volt·sec/rad/Amp]

        Ls = Ld = Lq for SPM; Ld ≠ Lq for IPM (rotor alignment selects axis).

    L_pu — Stator Inductance, per-unit at a speed base [angle16/s]
    L_pu = L · I_base · ω_base / V_base = ψ_pu(L · I_base)
        fract16_mul(ω_pu, L_pu) → ω_e·L·i_pu term in v_pu. Tick base: ω_pu ≡ Δθ [angle16/Ts]

    Identification:
        Voltage step:    L = V · Δt / Δi,      (initial linear ramp, R drop <<)
        RL τ (step):     L = Rs · τ,           (τ = 63.2% rise time)
        HFI (ω·L >> R):  L = V_pk / (2π·f·I_pk)
*/
/******************************************************************************/
/* L [V·s/(A·rad)] · I_base [A] · ω_base [rad/s] / V_base [V] = [1] */
#define _MOTOR_L_PU(W_Base, V_Base, I_Base, L_Henries) ((float)(L_Henries) * (I_Base) * (W_Base) / (V_Base))
#define MOTOR_L_PU(AngleFreq_Base, V_Base, I_Base, L_Henries) _MOTOR_L_PU(RADS_OF_ANGLE_FREQ(AngleFreq_Base), V_Base, I_Base, L_Henries)

/* L_pu_tick [Volt·Ts/angle16/Amp]. Tick base ω_base = π · Fs */
#define MOTOR_L_PU_TICK(Fs, V_Base, I_Base, L_Henries)   ((float)(L_Henries) * (I_Base) * PI_FLOAT * (Fs) / (V_Base))


static inline uint32_t l_pu_tick_of_h(uint32_t fs_hz, uint16_t v_base_V, uint16_t i_base_A, uint32_t l_h, uint32_t scale) { return (uint64_t)l_h * i_base_A * FRACT16_PI * fs_hz / ((uint64_t)v_base_V * scale); }
static inline uint32_t l_h_of_pu_tick(uint32_t fs_hz, uint16_t v_base_V, uint16_t i_base_A, uint32_t l_pu, uint32_t scale) { return (uint64_t)l_pu * v_base_V * scale / ((uint64_t)FRACT16_PI * fs_hz * i_base_A); }

/* caller compose with angle_freq_of_ */
static inline uint32_t l_pu_of_h(uint16_t v_base_V, uint16_t i_base_A, angle_freq_t base, uint32_t l_H, uint32_t scale) { return (uint64_t)l_H * i_base_A * base / FRACT16_SCALE * FRACT16_PI / ((uint64_t)v_base_V * scale); }
static inline uint32_t l_h_of_pu(uint16_t v_base_V, uint16_t i_base_A, angle_freq_t base, uint32_t l_pu, uint32_t scale) { return (uint64_t)l_pu * v_base_V * scale / FRACT16_PI * FRACT16_SCALE / ((uint64_t)i_base_A * base); }



/*!
    @brief  Reactance speed per ohm [angle16/s · A/V] — the inductance as a rate, free of a speed base, Fs, and the board base.

        AngleFreq(X) = X / L        linear in X = ω · L. The rate F / X = 1 / L
        AngleFreq at X = V_base / I_base: L_pu = 1.0 at F

        1 / L               [A / (V·s)]
        · 2^16 / 2π         [angle16 / rad], L per radian of ω
        AngleFreqPerOhm     [angle16/s · A/V]

        L [V·s/(A·rad)] per ohm is [s/rad]: angle_freq_of_s_per_rad, s_per_rad_of_angle_freq

    int32 holds L ≥ 4.86 µH.
    Per AngleFreqPerVolt [angle16/s / V], it is ψ / L [A], the short-circuit current.
*/
/*
    Angle Freq Per Ohm: (1/L)·2^16/2π [angle16/s · A/V]
    From the rate: L_pu = F_base / (AngleFreqPerOhm · V_base / I_base)
    Tick base: L_tick = Fs · 2^30 / (AngleFreqPerOhm · V_base / I_base)
*/
// #define MOTOR_ANGLE_FREQ_PER_OHM(L_Henries) ANGLE_FREQ_OF_RADS(1.0F / (L_Henries))
#define MOTOR_ANGLE_FREQ_OF_L(L_Henries) ANGLE_FREQ_OF_RADS(1.0F / (L_Henries)) /* 1/(2πfL) */

static inline uint32_t l_pu_of_angle_freq(angle_freq_t base, uint16_t v_base_V, uint16_t i_base_A, angle_freq_t rate) { return ((uint64_t)base << 15) * i_base_A / ((uint64_t)rate * v_base_V); }
static inline angle_freq_t angle_freq_of_l_pu(angle_freq_t base, uint16_t v_base_V, uint16_t i_base_A, uint32_t l_pu) { return ((uint64_t)base << 15) * i_base_A / ((uint64_t)l_pu * v_base_V); }

/*
    PU identification — direct from FOC-loop quantities. Output uint32 (may exceed FRACT16_MAX).

    L_pu = (v/di)·π·n_cycles·32768                                           [step]
    L_pu = Rs_pu · π · τ_cycles =  fract16_mul(Rs_pu, FRACT16_PI) · τ_cyc    [RL τ]
    L_pu = (v/i)·Fs/(2f)·32768  =  fract16_div(v, i) · Fs/(2f)               [HFI]
*/
static inline uint32_t l_pu_tick_of_step(fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return (uint64_t)fract16_div(v_pu, di_pu) * FRACT16_PI * dt_cycles / FRACT16_SCALE; }
static inline uint32_t l_pu_tick_of_rs_tau_cycles(fract16_t rs_pu, uint32_t tau_cycles) { return (uint64_t)rs_pu * FRACT16_PI * tau_cycles / FRACT16_SCALE; }
static inline uint32_t l_pu_tick_of_hfi(uint32_t fs_hz, uint32_t hfi_hz, ufract16_t v_pk_pu, ufract16_t i_pk_pu) { return (uint64_t)fract16_div(v_pk_pu, i_pk_pu) * fs_hz / (2UL * hfi_hz); }

/*
    RL τ form:   L = Rs · τ_cycles / Fs
    Step form:   L = V · dt_cycles / Fs / di
*/
// static inline uint32_t l_uh_of_step(uint32_t v_mV, uint32_t di_mA, uint32_t dt_us) { return (uint64_t)v_mV * dt_us / di_mA; }
// static inline uint32_t l_uh_of_rs_tau_us(uint32_t rs_mOhm, uint32_t tau_us) { return (uint64_t)rs_mOhm * tau_us / 1000UL; }
// static inline uint32_t l_uh_of_rs_tau_cycles(uint32_t fs_hz, uint32_t rs_mOhm, uint32_t tau_cycles) { return (uint64_t)rs_mOhm * tau_cycles * 1000UL / fs_hz; }
// static inline uint32_t l_uh_of_hfi(uint32_t hfi_hz, uint32_t v_mV_pk, uint32_t i_mA_pk) { return (uint64_t)v_mV_pk * 1000000UL * FRACT16_SCALE / ((uint64_t)2 * FRACT16_PI * hfi_hz * i_mA_pk); }

/*
    (L_ts · _fract16e_pi(L_ts)) < FRACT16_MAX
   fract16e Omegaψ = Δθ · ψ_ts · _fract16e_pi  < FRACT16_MAX, 1ms
   fract16e OmegaL = Δθ · L_ts · _fract16e_pi  < FRACT16_MAX, 1ms

    L_ts    334,651 = 10.21,    Δθ ≤ 6,417    ~29,400

    G = 1/L_ts, L_ts = Rs_pu·τ_cycles, L_ts = (v/Δi)·n
*/
/* L_pu_ts [Volt·Ts/angle16/Amp/π]. ω_base = Fs */
// #define MOTOR_L_PU_TS(Fs, V_Base, I_Base, L_Henries) _MOTOR_L_PU(Fs, V_Base, I_Base, L_Henries)
static inline uint32_t l_pu_ts_of_h(uint32_t fs_hz, uint16_t v_base_V, uint16_t i_base_A, uint32_t l_h, uint32_t scale) { return (uint64_t)l_h * i_base_A * fs_hz * FRACT16_SCALE / ((uint64_t)v_base_V * scale); }
static inline uint32_t l_h_of_pu_ts(uint32_t fs_hz, uint16_t v_base_V, uint16_t i_base_A, uint32_t l_pu, uint32_t scale) { return (uint64_t)l_pu * v_base_V * scale / ((uint64_t)FRACT16_SCALE * fs_hz * i_base_A); }

// /*
//     L_ts = (v/Δi)·n             [step]
//     L_ts = Rs_pu · τ_cycles     [RL τ]
//     L_ts = (v/i)·Fs/(2π·f)      [HFI]
// */
// static inline uint32_t l_pu_ts_of_step(fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return fract16_div(v_pu, di_pu) * dt_cycles; }
// // static inline uint32_t l_pu_ts_of_step(fract16_t v_pu, fract16_t di_pu, uint32_t dt_cycles) { return (uint64_t)v_pu * dt_cycles * FRACT16_SCALE / di_pu; }
// static inline uint32_t l_pu_ts_of_rs_tau_cycles(fract16_t rs_pu, uint32_t tau_cycles) { return rs_pu * tau_cycles; }
// static inline uint32_t l_pu_ts_of_hfi(uint32_t fs_hz, uint32_t hfi_hz, ufract16_t v_pk_pu, ufract16_t i_pk_pu) { return (uint64_t)fract16_div(v_pk_pu, i_pk_pu) * fs_hz * FRACT16_SCALE / (2UL * FRACT16_PI * hfi_hz); }





/******************************************************************************/
/*!
    @brief  Kv [rpm/V] — rotor rating as published

        Ke_mech [V·s/rad mech]  = 60 / (2π · Kv)
        ψ_f [Wb]                = Ke_mech / (P · M)
            M = 2:  Kv observed at bus voltage, phase-peak BEMF = V/2 at Kv · V. *_of_kv
            M = √3: line-to-line rating. psi_pu_of_ke
*/
/******************************************************************************/
/*
    derives base when V is Vbase
    selection between inverter max or vnominal
*/
#define MOTOR_EL_RPM_OF_KV(Kv, PolePairs, V_Volts) ((Kv) * (PolePairs) * (V_Volts))
#define MOTOR_EL_RADS_OF_KV(Kv, PolePairs, V_Volts, SI_Scale) ((uint64_t)MOTOR_EL_RPM_OF_KV(Kv, PolePairs, V_Volts) * (SI_Scale) * 2U * FRACT16_PI / 60U / FRACT16_SCALE)

#define MOTOR_SPEED_PU_OF_KV_RPM(VBase, Kv, Rpm) ((Rpm) * INT16_MAX / ((Kv) * (VBase)))

/* Speed envelope from Kv. */
static inline uint32_t rpm_of_kv_v(uint16_t kv, uint32_t volts) { return (uint32_t)kv * volts; }
static inline uint32_t v_of_kv_rpm(uint16_t kv, uint32_t rpm) { return rpm / kv; }

/* Ke [mV/krpm] in the Kv basis: Ke = 1000 / Kv */
static inline uint32_t rpm_of_ke_v(uint32_t ke_mV_per_krpm, uint32_t volts) { return (uint64_t)volts * 1000000UL / ke_mV_per_krpm; }

/* Kv basis conversions — parse vendor specs into internal phase-peak. */
static inline uint16_t kv_phase_of_kv_ll(uint16_t kv_ll_pk) { return v_ll_of_phase(kv_ll_pk); }
static inline uint16_t kv_ll_of_kv_phase(uint16_t kv_phase) { return v_phase_of_ll(kv_phase); }

/* [V·s/rad]. generic for caller compose  */
static inline uint32_t ke_vrads_of(uint16_t kv, uint32_t scale) { return ((uint64_t)scale * FRACT16_SCALE * 60U / (2 * FRACT16_PI)) / (kv); }
static inline uint32_t ke_mvrads_of_kv(uint16_t kv) { return ke_vrads_of(kv, 1000U); }
static inline uint32_t ke_uvsrad_of_kv(uint16_t kv_rpm_per_V) { return ke_vrads_of(kv_rpm_per_V, 1000000U); }


/*
    ψ_f [Wb] = (1 / (Kv·P·M)) · (60 / 2π)  = 60 / (2π · Kv · P · M)
    ψ_pu = ψ_f · ω_base / V_base = ω_base_rpm / (M · Kv · V_base)     [60/(2π·P) cancels]
*/
#define MOTOR_PSI_PU_OF_KV(V_Max, Speed_Max_Rpm, Kv) ((float)(Speed_Max_Rpm) / (2.0F * (Kv) * (V_Max)))

/* psi as phase electrical */
static inline uint32_t psi_wb_of_kv(uint16_t kv, uint8_t polePairs, uint32_t scale) { return ke_vrads_of(kv, scale) / polePairs / 2; }
static inline uint32_t psi_uwb_of_kv(uint16_t kv_rpm_per_V, uint8_t polePairs) { return psi_wb_of_kv(kv_rpm_per_V, polePairs, 1000000UL); }
// static inline uint16_t kv_of_psi_uwb(uint32_t psi_uWb, uint8_t polePairs) { return (uint64_t)60UL * 1000000UL * FRACT16_SCALE / ((uint64_t)FRACT16_SQRT3 * psi_uWb * polePairs); }

static inline uint32_t psi_pu_of_ke(uint16_t ke) { return v_phase_of_ll(ke); }
static inline uint16_t ke_of_psi_pu(uint32_t psi) { return v_ll_of_phase(psi); }

/*
    Direct Kv→PU shortcut.
    Mechanical Ke (no √3, no P) is the convenient intermediate
*/
static inline uint32_t ke_pu_tick_of_kv(uint32_t fs_hz, uint16_t v_base_V, uint16_t kv) { return (uint64_t)30UL * fs_hz * FRACT16_SCALE / ((uint32_t)kv * v_base_V); }
static inline uint16_t kv_of_ke_pu_tick(uint32_t fs_hz, uint16_t v_base_V, uint32_t ke_pu) { return (uint64_t)30UL * fs_hz * FRACT16_SCALE / ((uint64_t)v_base_V * ke_pu); }

/*
    ψ_pu_tick = Ke_mech / (P · M) · (π · Fs / V_base) = 30·Fs / (Kv·P·M·V_base)
*/
// static inline uint32_t psi_pu_tick_of_kv(uint32_t fs_hz, uint16_t v_base_V, uint8_t polePairs, uint16_t kv) { return ke_pu_tick_of_kv(fs_hz, v_base_V, kv) * FRACT16_1_DIV_SQRT3 / ((uint32_t)polePairs * FRACT16_SCALE); }
// static inline uint16_t kv_of_psi_pu_tick(uint32_t fs_hz, uint16_t v_base_V, uint8_t polePairs, uint32_t psi_pu) { return kv_of_ke_pu_tick(fs_hz, v_base_V, psi_pu * polePairs * FRACT16_SCALE / FRACT16_1_DIV_SQRT3); }
static inline uint32_t psi_pu_tick_of_kv(uint32_t fs_hz, uint16_t v_base_V, uint8_t polePairs, uint16_t kv) { return ke_pu_tick_of_kv(fs_hz, v_base_V, kv) / ((uint32_t)polePairs * 2); }
static inline uint16_t kv_of_psi_pu_tick(uint32_t fs_hz, uint16_t v_base_V, uint8_t polePairs, uint32_t psi_pu) { return kv_of_ke_pu_tick(fs_hz, v_base_V, psi_pu * polePairs * 2); }

/* FOC directly from Kv — pole pairs cancel: Kt = 1.5·P·ψ_f = 1.5·P·60/(2π·Kv·P) = 1.5·Ke_mech. */
// static inline accum32_t kt_nm_per_amp_of(uint16_t kv, scale) { return ((uint64_t)60 * FRACT16_SQRT3_DIV_2 * FRACT16_SCALE / kv * 2 * FRACT16_PI); }
// static inline uint32_t kt_unm_per_a_of_kv(uint16_t kv_rpm_per_V) { return 3UL * ke_mech_uvsrad_of_kv(kv_rpm_per_V) / 2UL; }
// static inline uint32_t kt_rms_unm_per_a_of_kv(uint16_t kv_rpm_per_V) { return ke_mech_uvsrad_of_kv(kv_rpm_per_V); }
