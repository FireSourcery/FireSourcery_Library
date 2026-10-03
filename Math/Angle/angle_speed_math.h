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
    Convert between ω/Fs [angle16/dt], [angle16/s] and standard representations
        AngleDt: dθ/dt [angle16/dt]
        AngleFreq: n.angle/s [rot32/s]
            [rpm], [rps], [rad/s]
    e.g.
        - angle_dt for control loops, internal calculations.
        - standard units for configuration.

    Implementation:
        composition along compile time optimizable path
        provides both a of_x conversion function, as well as a per_x conversion factor
*/

/*

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

/* via int32_t: float out of int16 range is UB, int32 → int16 wraps as angle16 */
#define ANGLE_DT(Fs, rps)   ((angle_dt_t)(int32_t)(((float)(rps) * ANGLE16_PER_REVOLUTION) / (Fs)))

/*
    dθ/dt [angle16/dt] = ω [rad/s] * (65536 / 2π) / Fs
*/
// #define ANGLE_DT_PER_RADS(Fs) (ANGLE16_PER_RADIAN / Fs)
// #define RADS_PER_ANGLE_DT(Fs) (Fs / ANGLE16_PER_RADIAN) /* Fs * π / 32768 */
#define ANGLE_DT_OF_RADS(Fs, RadPerSecond) ((RadPerSecond) * ANGLE16_PER_RADIAN / Fs)


/******************************************************************************/
/*
    Generic base - args scaled by caller
*/
/******************************************************************************/
/*
    Constant expression path (for #define composition, initializers)
*/
/* freq in turns per second base, wide unit Fs parameter for generic use. e.g. rps at Fs, rpm at Fs·60 */
/* alternatively split macro for float */
#define ANGLE_DT_OF(Fs, freq)       (((int64_t)(freq) * ANGLE16_PER_REVOLUTION) / (Fs))
// depreciate
/* discards fractional part */
#define ANGLE_FREQ_OF(Fs, AngleDt) (((int64_t)(AngleDt) * (Fs)) / ANGLE16_PER_REVOLUTION)

// #define ANGLE_FREQ_OF( freq) ((freq * ANGLE16_PER_REVOLUTION)

/* direct for comparison */
static inline int32_t _angle_dt_of_freq_direct(uint32_t fs, int32_t freq) { return ANGLE_DT_OF(fs, freq); }
static inline int32_t _angle_freq_of_dt_direct(uint32_t fs, int32_t angle_dt) { return ANGLE_FREQ_OF(fs, angle_dt); }


/*
    Factor (compile-time const or precomputed)
*/
// #define TS_FRACT32(Fs) (FRACT32_SCALE / (Fs))
/* effectively Ts_fract32 */
/* FRACT32_SCALE = ANGLE16_PER_REVOLUTION * FRACT16_SCALE */
#define ANGLE_DT_PER_RPS(Fs) ((uint32_t)ANGLE16_PER_REVOLUTION * FRACT16_SCALE / (Fs))
#define RPS_PER_ANGLE_DT(Fs) ((Fs) / (ANGLE16_PER_REVOLUTION / FRACT16_SCALE))

/*
    Runtime - Compile time optimizable
    Compiler may optimize when [fs] is const, run time without division
    assert(fs != 0);
    32768 == (INT32_MAX + 1) / ANGLE16_PER_REVOLUTION
*/
/* optionally without int64 cast for base rates */
static inline int32_t _angle_freq_of(uint32_t fs, int32_t angle_dt) { return angle_dt * (int32_t)RPS_PER_ANGLE_DT(fs) / FRACT16_SCALE; }

/* freq [0:Fs/2] */
static inline int32_t angle_dt_of(uint32_t fs, int32_t freq) { return freq * (int32_t)ANGLE_DT_PER_RPS(fs) / FRACT16_SCALE; }
/* keep (int64_t) for scaled Fs. e.g. rpm at Fs·60 */
static inline int32_t angle_freq_of(uint32_t fs, int32_t angle_dt) { return ANGLE_FREQ_OF(fs, angle_dt); }



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
#define ANGLE_FREQ_MAX(Fs)       ((angle_freq_t)((Fs) * (ANGLE16_PER_REVOLUTION / 2U)))

// #define ANGLE_FREQ(Turns, Angle16)      ((angle_freq_t)((int32_t)(Turns) * (int32_t)ANGLE16_PER_REVOLUTION + (uangle16_t)(Angle16)))
// #define ANGLE_FREQ_OF_HZ(Hz)            ANGLE_FREQ(Hz, 0)
// #define ANGLE_FREQ_OF_RADS(Rads, Scale) ((angle_freq_t)((int64_t)(Rads) * (ANGLE16_PER_REVOLUTION / 2U) * FRACT16_SCALE / ((int64_t)FRACT16_PI * (Scale))))
#define ANGLE_FREQ_NYQUIST(Fs)          ((angle_freq_t)((Fs) * (ANGLE16_PER_REVOLUTION / 2U)))

#define ANGLE_DT_OF_FREQ(Fs, Freq)      ((angle_dt_t)((Freq) / (int32_t)(Fs)))


// static inline angle_freq_t angle_freq(uint32_t hz) { return hz * ANGLE16_PER_REVOLUTION; }
// static inline angle_freq_t angle_freq(angle16_t hz, angle16_t angle) { return hz * ANGLE16_PER_REVOLUTION; }
static inline int16_t angle_freq_to_turns(angle_freq_t freq) { return freq >> ANGLE_FREQ_TURNS_SHIFT; }

static inline int32_t angle_dt_of_angle_freq(uint32_t fs, angle_freq_t freq) { return freq / fs; }
static inline angle_freq_t angle_freq_of_angle_dt(uint32_t fs, angle_dt_t angle_dt) { return (int32_t)angle_dt * (int32_t)fs; }

/*  */
static inline angle_freq_t angle_freq_of_rpm(uint32_t rpm) { return (uint64_t)rpm * ANGLE16_PER_REVOLUTION / SECONDS_PER_MINUTE; }
static inline uint32_t rpm_of_angle_freq(angle_freq_t freq) { return (uint64_t)freq * SECONDS_PER_MINUTE / ANGLE16_PER_REVOLUTION; }
static inline angle_freq_t angle_freq_of_rads(uint32_t rads, uint32_t scale) { return (uint64_t)rads * (ANGLE16_PER_REVOLUTION / 2U) * FRACT16_SCALE / ((uint64_t)FRACT16_PI * scale); }
static inline uint32_t rads_of_angle_freq(angle_freq_t freq, uint32_t scale) { return (uint64_t)freq * FRACT16_PI * scale / ((uint64_t)(ANGLE16_PER_REVOLUTION / 2U) * FRACT16_SCALE); }

/* Fs-anchored base: ω_base = π·Fs, Δθ_base = 32768 [angle16/Ts] */
static inline angle_freq_t angle_freq_nyquist(uint32_t fs) { return fs * (ANGLE16_PER_REVOLUTION / 2U); }



/******************************************************************************/
/*
    functions with signatures matching input range.
*/
/******************************************************************************/

/*
    from scaled SI units
    at Fs = 20 kHz, ANGLE16_PER_RADIAN ~= Fs / 2 => [rad/s] ~= [angle16/dt]
*/
static inline int32_t angle_dt_of_rads(uint32_t fs, int32_t rads, uint16_t scale) { return ((int64_t)rads * ANGLE16_PER_RADIAN) / fs / scale; }
static inline int32_t rads_of_angle_dt(uint32_t fs, angle_dt_t angle_dt, uint16_t scale) { return ((int64_t)angle_dt * fs * scale) / ANGLE16_PER_RADIAN; }

/******************************************************************************/
/* Rpm */
/******************************************************************************/
/*
    Example: Fs = 20000 (20kHz)
        minutes_fract32 = INT32_MAX / (60 * 20000) = 1789 (compile-time)

    angle_dt:
        = rpm * ANGLE16_PER_REVOLUTION / (60 * Fs)
        = rpm * minutes_fract32 / (INT32_MAX / ANGLE16_PER_REVOLUTION)
*/
#define ANGLE_DT_OF_RPM(Fs, rpm)      ANGLE_DT_OF((int64_t)Fs * SECONDS_PER_MINUTE, rpm)
#define RPM_OF_ANGLE_DT(Fs, angle_dt) ANGLE_FREQ_OF((int64_t)Fs * SECONDS_PER_MINUTE, angle_dt)

/* Alternative direct implementations for comparison */
static inline int32_t angle_dt_of_rpm_direct(uint32_t fs, int32_t rpm) { return ANGLE_DT_OF_RPM(fs, rpm); }

static inline int32_t angle_dt_of_rpm(uint32_t fs, int32_t rpm) { return angle_dt_of(fs * SECONDS_PER_MINUTE, rpm); }
static inline int32_t rpm_of_angle_dt(uint32_t fs, angle_dt_t angle_dt) { return angle_freq_of(fs * SECONDS_PER_MINUTE, angle_dt); }


/*
    Cycles Per Second
*/
/* rps [0:Fs/2] */
static inline int32_t angle_dt_of_rps(uint32_t fs, int16_t rps) { return angle_dt_of(fs, rps); }
static inline int32_t rps_of_angle_dt(uint32_t fs, angle_dt_t angle_dt) { return angle_dt * (int32_t)RPS_PER_ANGLE_DT(fs) / FRACT16_SCALE; }




/******************************************************************************/
/*!
    @brief  Per Unit conversion Boundary: angle_dt  ↔  ω_pu (ω_base-anchored, fract16-scaled)

        ω_pu × FRACT16_SCALE = angle_dt · 30 · Fs / (P · n_base_rpm)
                             = angle_dt · 2π·Fs/(65536·ω_base) · FRACT16_SCALE     [π cancels]
*/
/******************************************************************************/
/*
    RPM Ref
*/
// return angle_dt_of_rpm(fs, fract16_mul(base_rpm, rpm_fract16));
static inline angle_dt_t angle_dt_of_rpm_fract16(uint32_t fs, uint32_t base_rpm, int16_t pu_fract16)
{
    return ((int64_t)pu_fract16 * base_rpm) / ((SECONDS_PER_MINUTE / 2) * fs);
}

static inline int16_t rpm_fract16_of_angle_dt(uint32_t fs, uint32_t base_rpm, angle_dt_t angle_dt)
{
    return ((int64_t)angle_dt * ((SECONDS_PER_MINUTE / 2) * fs)) / base_rpm;
}

/*
    optional include rads scaling
    static inline angle_dt_t angle_dt_of_rads_fract16(uint32_t fs, uint32_t base_rads, rads_scale, int16_t rads_pu)
*/
/* angle_dt = ω_pu_fract16 · (ω_base / (π · Fs)) */
static inline angle_dt_t angle_dt_of_rads_fract16(uint32_t fs, uint32_t base_rads, int16_t pu_fract16)
{
    return ((int64_t)pu_fract16 * base_rads * FRACT16_SCALE) / ((int64_t)FRACT16_PI * fs);
    // return (pu_fract16 * base_rads) / ((int64_t)FRACT16_PI * fs / FRACT16_SCALE);
}

/* ω_pu_fract16 = angle_dt · π · Fs / ω_base */
static inline int32_t rads_fract16_of_angle_dt(uint32_t fs, uint32_t base_rads, angle_dt_t angle_dt)
{
    return (int64_t)angle_dt * FRACT16_PI * fs / (base_rads * FRACT16_SCALE);
}

/*
    Angle Rate Ref — π-free
*/
/* angle_dt = ω_pu_fract16 · (rate_base / Fs) */
static inline angle_dt_t angle_dt_of_angle_freq_pu(uint32_t fs, angle_freq_t base_rate, int16_t pu_fract16)
{
    return ((int64_t)pu_fract16 * base_rate) / ((int64_t)fs * FRACT16_SCALE);
}

/* ω_pu_fract16 = angle_dt · Fs / rate_base */
static inline int32_t angle_freq_pu_of_angle_dt(uint32_t fs, angle_freq_t base_rate, angle_dt_t angle_dt)
{
    return ((int64_t)angle_dt * fs * FRACT16_SCALE) / base_rate;
}


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

static inline uint32_t el_rads_of_mech_rpm(uint8_t pole_pairs, uint32_t mech_rpm) { return rads_of_rpm(mech_rpm, pole_pairs); }
static inline uint32_t mech_rpm_of_el_rads(uint8_t pole_pairs, uint32_t el_rads) { return rpm_of_rads(el_rads, pole_pairs); }

