#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   angle_speed_math.h
    @author FireSourcery
    @brief  rotary, rotational angle speed math.
*/
/******************************************************************************/
#include "../Fixed/fract16.h"
#include <assert.h>



/*
    Canonical forms
        angle_freq_t    n.angle16/s [rot32/s]   Fs-free form
        angle_dt_t      dθ/dt [angle16/Ts]      control loop form
        angle_dt = angle_freq / Fs. angle_dt → angle_freq is exact.

    [Hz], [rpm], [rad/s] are views, converted through [angle_freq_t].
        SI ↔ angle_dt composes through angle_freq_t.
*/
#define PI_FLOAT (3.14159265358979323846F)
#define SECONDS_PER_MINUTE (60U)

#define ANGLE_DT_MAX (32767)
#define ANGLE_DT_MAX_RPS(Fs) (Fs / 2) /* Nyquist Equivalent */
#define ANGLE_DT_MAX_RADS(Fs) (Fs * PI_FLOAT)
#define ANGLE_DT_MAX_RPM(Fs) (Fs * 30)

/******************************************************************************/
/*!
    @brief [angle16/dt]

    t_base in control periods: dt = Ts = 1/Fs.
    dθ/dt = ω·Ts, expressly in binary angle measurement [angle16] per period, not radians.
*/
/******************************************************************************/
typedef angle16_t angle_dt_t;

#define ANGLE_DT(Fs, rps)   ((angle_dt_t)(int32_t)(((float)(rps) * ANGLE16_PER_REVOLUTION) / (Fs)))


/******************************************************************************/
/*!
    @brief  Angle Freq [angle16/s] — frequency storage, storage in per second base

        freq [angle16/s] = f [Hz] · 65536 = ω [rad/s] · 32768 / π
        [turn.angle16 / s]: [31:16] turns (Hz) | [15:0] angle16 (fraction of a turn)
        dθ/dt [angle16/Ts] = freq / Fs

    Common form of rpm, rad/s, and the Fs-anchored base. π enters only at the rad/s boundary.
        e.g. 200 Hz = 1256.6 rad/s => 13,107,200 => 655 [angle16/Ts] at Fs = 20 kHz
*/
/******************************************************************************/
typedef rot32_t angle_freq_t;

#define ANGLE_FREQ_TURNS_SHIFT (16U)
static_assert(ANGLE16_PER_REVOLUTION == (1UL << ANGLE_FREQ_TURNS_SHIFT), "angle16 is the fraction of a turn");

#define ANGLE_FREQ(Hz)           ((angle_freq_t)((float)(Hz) * (int32_t)ANGLE16_PER_REVOLUTION))

#define ANGLE_FREQ_OF_RADS(Rads) ((angle_freq_t)((float)(Rads) * ANGLE16_PER_REVOLUTION / (2 * PI_FLOAT)))
#define ANGLE_FREQ_OF_RPM(Rpm)   ((angle_freq_t)((int64_t)(Rpm) * ANGLE16_PER_REVOLUTION / SECONDS_PER_MINUTE))
// #define ANGLE_FREQ_MAX(Fs)       ((angle_freq_t)((Fs) * (ANGLE16_PER_REVOLUTION / 2U)))
#define ANGLE_FREQ_NYQUIST(Fs)   ((angle_freq_t)((Fs) * (ANGLE16_PER_REVOLUTION / 2U)))


static inline angle_freq_t angle_freq(int16_t hz, uangle16_t angle) { return (int32_t)hz * (int32_t)ANGLE16_PER_REVOLUTION + angle; }
static inline int16_t angle_freq_to_hz(angle_freq_t freq) { return freq / (int32_t)ANGLE16_PER_REVOLUTION; }

/* Fs-anchored base: ω_base = π·Fs, Δθ_base = 32768 [angle16/Ts] */
static inline angle_freq_t angle_freq_nyquist(uint32_t fs) { return fs * (ANGLE16_PER_REVOLUTION / 2U); }
// static inline angle_freq_t angle_freq_max(uint32_t fs) { return fs * (ANGLE16_PER_REVOLUTION / 2U); }



/*
    dθ/dt [angle16/dt] = ω [rad/s] * (65536 / 2π) / Fs
*/
#define ANGLE_DT_OF_FREQ(Fs, AngleFreq) ((angle_dt_t)((AngleFreq) / (int32_t)(Fs)))
#define ANGLE_DT_OF_RPM(Fs, Rpm)        ANGLE_DT_OF_FREQ(Fs, ANGLE_FREQ_OF_RPM(Rpm))
#define ANGLE_DT_OF_RADS(Fs, Rads)      ANGLE_DT_OF_FREQ(Fs, ANGLE_FREQ_OF_RADS(Rads))

static inline int32_t angle_dt_of_angle_freq(uint32_t fs, angle_freq_t freq) { return freq / (int32_t)fs; }
static inline angle_freq_t angle_freq_of_angle_dt(uint32_t fs, angle_dt_t angle_dt) { return (int32_t)angle_dt * (int32_t)fs; }



/******************************************************************************/
/*!
    @brief  SI views of [angle_freq_t]. π enters only at the rad/s boundary.
        Signed, truncating toward zero: -0.3 Hz reads 0 Hz.
*/
/******************************************************************************/
static inline angle_freq_t angle_freq_of_rpm(int32_t rpm) { return (int64_t)rpm * ANGLE16_PER_REVOLUTION / SECONDS_PER_MINUTE; }
static inline int32_t rpm_of_angle_freq(angle_freq_t freq) { return (int64_t)freq * SECONDS_PER_MINUTE / ANGLE16_PER_REVOLUTION; } /* add round up for user view if needed */

static inline angle_freq_t angle_freq_of_rads(int32_t rads, uint32_t scale) { return (int64_t)rads * ANGLE32_PER_RADIAN / ANGLE16_PER_REVOLUTION / scale; }
static inline int32_t rads_of_angle_freq(angle_freq_t freq, uint32_t scale) { return (int64_t)freq * ANGLE16_PER_REVOLUTION * scale / ANGLE32_PER_RADIAN; }

/******************************************************************************/
/*!
    @brief  SI views of [angle_dt_t], composed through [angle_freq_t]
*/
/******************************************************************************/
static inline int32_t angle_dt_of_rads(uint32_t fs, int32_t rads, uint16_t scale) { return angle_dt_of_angle_freq(fs, angle_freq_of_rads(rads, scale)); }
static inline int32_t rads_of_angle_dt(uint32_t fs, angle_dt_t angle_dt, uint16_t scale) { return rads_of_angle_freq(angle_freq_of_angle_dt(fs, angle_dt), scale); }

static inline int32_t angle_dt_of_rpm(uint32_t fs, int32_t rpm) { return angle_dt_of_angle_freq(fs, angle_freq_of_rpm(rpm)); }
static inline int32_t rpm_of_angle_dt(uint32_t fs, angle_dt_t angle_dt) { return rpm_of_angle_freq(angle_freq_of_angle_dt(fs, angle_dt)); }

/* Single division, for comparison */
static inline int32_t angle_dt_of_rpm_direct(uint32_t fs, int32_t rpm) { return (int64_t)rpm * ANGLE16_PER_REVOLUTION / ((int64_t)fs * SECONDS_PER_MINUTE); }

/* rps [0:Fs/2] */
static inline int32_t angle_dt_of_rps(uint32_t fs, int16_t rps) { return angle_dt_of_angle_freq(fs, angle_freq(rps, 0U)); }
static inline int32_t rps_of_angle_dt(uint32_t fs, angle_dt_t angle_dt) { return angle_freq_to_hz(angle_freq_of_angle_dt(fs, angle_dt)); }





/******************************************************************************/
/*!
    @brief runtime Per Unit conversion Boundary: angle_dt  ↔  ω_pu (ω_base-anchored, fract16-scaled)

    ω_pu × FRACT16_SCALE = angle_freq · FRACT16_SCALE / F_base
        = angle_dt · Fs · FRACT16_SCALE / F_base     [F_base in angle16/s, π-free]
*/
/******************************************************************************/
/*
    Angle Rate Ref — π-free
*/
/* angle_dt = ω_pu_fract16 · (rate_base / Fs) */
// static inline angle_dt_t angle_dt_of_angle_freq_pu(uint32_t fs, angle_freq_t base_rate, int16_t pu_fract16)
// {
//     return ((int64_t)pu_fract16 * base_rate) / ((int64_t)fs * FRACT16_SCALE);
// }

// /* ω_pu_fract16 = angle_dt · Fs / rate_base */
// static inline int32_t angle_freq_pu_of_angle_dt(uint32_t fs, angle_freq_t base_rate, angle_dt_t angle_dt)
// {
//     return ((int64_t)angle_dt * fs * FRACT16_SCALE) / base_rate;
// }

// /*
//     Angle Rate Ref — π-free
// */
// /* angle_dt = ω_pu_fract16 · (rate_base / Fs) */
// // static inline angle_dt_t angle_dt_of_angle_freq_pu(uint32_t fs, angle_freq_t base_rate, int16_t pu_fract16)
// // {
// //     return angle_dt_of_angle_freq(fs, (int64_t)pu_fract16 * base_rate / FRACT16_SCALE);
// // }

// // /* ω_pu_fract16 = angle_dt · Fs / rate_base */
// // static inline int32_t angle_freq_pu_of_angle_dt(uint32_t fs, angle_freq_t base_rate, angle_dt_t angle_dt)
// // {
// //     return (int64_t)angle_freq_of_angle_dt(fs, angle_dt) * FRACT16_SCALE / base_rate;
// // }

// /*
//     RPM Ref
// */
// // return angle_dt_of_rpm(fs, fract16_mul(base_rpm, rpm_fract16));
// static inline angle_dt_t angle_dt_of_pu_rpm(uint32_t fs, uint32_t base_rpm, int16_t pu_fract16)
// {
//     return ((int64_t)pu_fract16 * base_rpm) / ((SECONDS_PER_MINUTE / 2) * fs);
// }

// static inline int16_t pu_rpm_of_angle_dt(uint32_t fs, uint32_t base_rpm, angle_dt_t angle_dt)
// {
//     return ((int64_t)angle_dt * ((SECONDS_PER_MINUTE / 2) * fs)) / base_rpm;
// }

// /*
//     optional include rads scaling
//     static inline angle_dt_t angle_dt_of_pu_rads(uint32_t fs, uint32_t base_rads, rads_scale, int16_t rads_pu)
// */
// /* angle_dt = ω_pu_fract16 · ω_base · ANGLE16_PER_RADIAN / Fs */
// static inline angle_dt_t angle_dt_of_pu_rads(uint32_t fs, uint32_t base_rads, int16_t pu_fract16)
// {
//     return (int64_t)pu_fract16 * base_rads * ANGLE32_PER_RADIAN / ((int64_t)fs * ANGLE16_PER_REVOLUTION * FRACT16_SCALE);
// }

// /* ω_pu_fract16 = angle_dt · Fs / (ω_base · ANGLE16_PER_RADIAN) */
// static inline int32_t pu_rads_of_angle_dt(uint32_t fs, uint32_t base_rads, angle_dt_t angle_dt)
// {
//     return (int64_t)angle_dt * fs * ANGLE16_PER_REVOLUTION * FRACT16_SCALE / ((int64_t)ANGLE32_PER_RADIAN * base_rads);
// }


// /*
//     rpm and rad/s bases
// */
// static inline angle_dt_t angle_dt_of_pu_rpm(uint32_t fs, uint32_t base_rpm, int16_t pu_fract16) { return angle_dt_of_angle_freq_pu(fs, angle_freq_of_rpm(base_rpm), pu_fract16); }
// static inline int16_t pu_rpm_of_angle_dt(uint32_t fs, uint32_t base_rpm, angle_dt_t angle_dt) { return angle_freq_pu_of_angle_dt(fs, angle_freq_of_rpm(base_rpm), angle_dt); }

// static inline angle_dt_t angle_dt_of_pu_rads(uint32_t fs, uint32_t base_rads, int16_t pu_fract16) { return angle_dt_of_angle_freq_pu(fs, angle_freq_of_rads(base_rads, 1U), pu_fract16); }
// static inline int32_t pu_rads_of_angle_dt(uint32_t fs, uint32_t base_rads, angle_dt_t angle_dt) { return angle_freq_pu_of_angle_dt(fs, angle_freq_of_rads(base_rads, 1U), angle_dt); }
/******************************************************************************/
/*
    si to si
*/
/******************************************************************************/
#define RADS_PER_RPM_FLOAT (PI_FLOAT / 30.0F)
#define RADS_OF_RPM(rpm) ((float)(rpm) * RADS_PER_RPM_FLOAT)

static inline uint32_t rads_of_rpm(uint32_t rpm, uint32_t scale) { return (uint64_t)rpm * FRACT16_PI * scale / (30U * FRACT16_SCALE); }
static inline uint32_t rpm_of_rads(uint32_t rads, uint32_t scale) { return (uint64_t)rads * 30U * FRACT16_SCALE / ((uint64_t)FRACT16_PI * scale); }
static inline uint32_t mrads_of_rpm(uint32_t rpm) { return rads_of_rpm(rpm, 1000U); }
static inline uint32_t rpm_of_mrads(uint32_t mrads) { return rpm_of_rads(mrads, 1000U); }

/******************************************************************************/
/*!
    @brief  Electrical angle and mechanical RPM conversions for motors.
*/
/******************************************************************************/
/* ANGLE_DT_OF_RPM() * PolePairs */
static inline int32_t el_angle_dt_of_mech_rpm(uint32_t fs, uint8_t polePairs, int16_t rpm) { return angle_dt_of_rpm(fs, (int32_t)rpm * polePairs); }
static inline int32_t mech_rpm_of_el_angle_dt(uint32_t fs, uint8_t polePairs, angle_dt_t angle_dt) { return rpm_of_angle_dt(fs, angle_dt) / polePairs; }
/* Rounds to nearest, inverting a truncated freq */
static inline uint32_t mech_rpm_of_el_angle_freq(uint8_t polePairs, angle_freq_t freq) { return ((uint64_t)freq * SECONDS_PER_MINUTE + ANGLE16_PER_REVOLUTION / 2U * polePairs) / ((uint64_t)ANGLE16_PER_REVOLUTION * polePairs); }

static inline uint32_t el_rads_of_mech_rpm(uint8_t pole_pairs, uint32_t mech_rpm) { return rads_of_rpm(mech_rpm, pole_pairs); }
static inline uint32_t mech_rpm_of_el_rads(uint8_t pole_pairs, uint32_t el_rads) { return rpm_of_rads(el_rads, pole_pairs); }

