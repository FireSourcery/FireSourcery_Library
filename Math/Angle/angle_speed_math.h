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
    Convert between [angle16/polling] and standard representations: [rad/s] or [turns/time].
        AngleSpeed: angle16[angle16/polling]
        AngleFreq: n [turns per seconds or minute]
    e.g.
        - angle16/polling for control loops, internal calculations.
        - standard units for configuration.

    Implementation:
        composition along compile time optimizable path
        provides both a of_x conversion function, as well as a per_x conversion factor
*/

/*

*/
#define PI_FLOAT (3.14159265358979323846F)

#define ANGLE_SPEED_MAX (32767)
#define ANGLE_SPEED_MAX_RPS(Fs) (Fs / 2) /* Nyquist Equivalent */
#define ANGLE_SPEED_MAX_RADS(Fs) (Fs * PI_FLOAT)
#define ANGLE_SPEED_MAX_RPM(Fs) (Fs * 30)


/******************************************************************************/
/*
    Generic base - args scaled by caller
*/
/******************************************************************************/
/*
    Constant expression path (for #define composition, initializers)
*/
// #define ANGLE_SPEED(Fs, rps)    (((float)rps) * ANGLE16_PER_REVOLUTION) / (Fs))
#define ANGLE_SPEED_OF(Fs, rps)    (((int64_t)(rps) * ANGLE16_PER_REVOLUTION) / (Fs))
#define ANGLE_FREQ_OF(Fs, angle16) (((int64_t)(angle16) * (Fs)) / ANGLE16_PER_REVOLUTION)

/* direct for comparison */
static inline int32_t _angle_speed_of_freq_direct(uint32_t fs, int32_t cps) { return ANGLE_SPEED_OF(fs, cps); }
static inline int32_t _angle_freq_of_speed_direct(uint32_t fs, int32_t angle_per_poll) { return ANGLE_FREQ_OF(fs, angle_per_poll); }


/*
    Factor (compile-time const or precomputed)
*/
#define POLLING_PERIOD_FRACT32(Fs) (FRACT32_SCALE / (Fs))
/* effecticely polling_period_fract32 */
/* FRACT32_SCALE = ANGLE16_PER_REVOLUTION * FRACT16_SCALE */
#define ANGLE_SPEED_PER_RPS(Fs) ((uint32_t)ANGLE16_PER_REVOLUTION * FRACT16_SCALE / (Fs))
#define RPS_PER_ANGLE_SPEED(Fs) ((Fs) / (ANGLE16_PER_REVOLUTION / FRACT16_SCALE))

/*
    Runtime - Compile time optimizable
    Compiler may optimize when [fs] is const, run time without division
    assert(fs != 0);
    32768 == (INT32_MAX + 1) / ANGLE16_PER_REVOLUTION
*/
/* optionally without int64 cast for base rates */
static inline int32_t _angle_freq_of(uint32_t fs, int32_t angle_per_poll) { return angle_per_poll * (int32_t)RPS_PER_ANGLE_SPEED(fs) / FRACT16_SCALE; }

/* cps [0:Fs/2] */
static inline int32_t angle_speed_of(uint32_t fs, int32_t cps) { return cps * (int32_t)ANGLE_SPEED_PER_RPS(fs) / FRACT16_SCALE; }
/* keep (int64_t) for scaled polling rate. e.g rpm */
static inline int32_t angle_freq_of(uint32_t fs, int32_t angle_per_poll) { return ANGLE_FREQ_OF(fs, angle_per_poll); }

// typedef struct angle_speed { angle16_t Angle; angle16_t Delta; } angle_speed_t;


/******************************************************************************/
/* Rads */
/******************************************************************************/
/*
    ω_du[angle16/poll] = ω[rad/s] * (65536 / 2π) / Fs
*/
#define ANGLE_SPEED_PER_RADIAN(Fs) ((float)ANGLE16_PER_RADIAN / Fs)
#define RADIAN_PER_ANGLE_SPEED(Fs) ((float)Fs / ANGLE16_PER_RADIAN) /* Fs * π / 32768 */

#define ANGLE_SPEED(Fs, RadPerSecond) ((float)(RadPerSecond) * ANGLE16_PER_RADIAN / Fs)

/*
    from scaled SI units
    ANGLE16_PER_RADIAN ~= PollingFreq / 2 => [rad/s] ~= [angle16/poll]
*/
static inline int32_t angle_of_rads(uint32_t Fs, int32_t rads, uint16_t scale) { return ((int64_t)rads * ANGLE16_PER_RADIAN) / Fs / scale; }
static inline int32_t rads_of_angle(uint32_t Fs, int16_t angle16, uint16_t scale) { return ((int64_t)angle16 * Fs * scale) / ANGLE16_PER_RADIAN; }





/******************************************************************************/
/*
    functions with signitures matching input range.
*/
/******************************************************************************/
/******************************************************************************/
/* Rpm */
/******************************************************************************/
/*
    angle16 per poll of rpm

    Example: Fs = 20000 (20kHz)
        minutes_fract32 = INT32_MAX / (60 * 20000) = 1789 (compile-time)

    angle16:
        = rpm * ANGLE16_PER_REVOLUTION / (60 * Fs)
        = rpm * minutes_fract32 / (INT32_MAX / ANGLE16_PER_REVOLUTION)
*/

#define SECONDS_PER_MINUTE (60U)

#define ANGLE16_OF_RPM(Fs, rpm)      ANGLE_SPEED_OF((int64_t)Fs * SECONDS_PER_MINUTE, rpm)
#define RPM_OF_ANGLE16(Fs, angle16)  ANGLE_FREQ_OF((int64_t)Fs * SECONDS_PER_MINUTE, angle16)

/* Alternative direct implementations for comparison */
static inline int32_t angle_of_rpm_direct(uint32_t Fs, int32_t rpm) { return ANGLE16_OF_RPM(Fs, rpm); }

static inline int32_t angle_of_rpm(uint32_t Fs, int32_t rpm) { return angle_speed_of(Fs * SECONDS_PER_MINUTE, rpm); }
static inline int32_t rpm_of_angle(uint32_t Fs, int16_t angle16) { return angle_freq_of(Fs * SECONDS_PER_MINUTE, angle16); }


/* ANGLE16_OF_RPM() * PolePairs */
static inline int32_t el_angle_of_mech_rpm(uint32_t Fs, uint8_t polePairs, int16_t rpm) { return angle_of_rpm(Fs, (int32_t)rpm * polePairs); }
static inline int32_t mech_rpm_of_el_angle(uint32_t Fs, uint8_t polePairs, int16_t angle16) { return rpm_of_angle(Fs, angle16) / polePairs; }


/*
    Cycles Per Second
*/
/* cps [0:POLLING_FREQ/2] */
static inline int32_t angle_of_cps(uint32_t Fs, int16_t cps) { return angle_speed_of(Fs, cps); }
static inline int32_t cps_of_angle(uint32_t Fs, int16_t angle16) { return angle16 * (int32_t)RPS_PER_ANGLE_SPEED(Fs) / FRACT16_SCALE; }
// static inline int32_t cps_of_angle(uint32_t Fs, int16_t angle16) { return _angle_freq_of(Fs, angle16); }


/******************************************************************************/
/*!
    @brief  Per Unit conversion Boundary: delta_angle16  ↔  ω_pu (ω_base-anchored, fract16-scaled)

        ω_pu × FRACT16_SCALE = delta · 30 · Fs / (P · n_rated_rpm)
                             = delta · 2π·Fs/(65536·ω_base) · FRACT16_SCALE     [π cancels]
*/
/******************************************************************************/
/*
    angle16_of_speed_pu_rpm
    RPM Ref
*/
// return angle_of_rpm(fs, fract16_mul(base_rpm, rpm_fract16));
static inline int16_t angle_of_rpm_fract16(uint32_t fs, uint32_t base_rpm, int16_t pu_fract16)
{
    return ((int32_t)pu_fract16 * base_rpm) / ((SECONDS_PER_MINUTE / 2) * fs);
}

static inline int16_t rpm_fract16_of_angle(uint32_t fs, uint32_t base_rpm, int16_t delta)
{
    return ((int64_t)delta * ((SECONDS_PER_MINUTE / 2) * fs)) / base_rpm;
}

/*
    angle16_of_speed_pu_rads
*/
/* delta = ω_pu_fract16 · (ω_base / (π · Fs)) */
static inline int16_t angle_of_rads_fract16(uint32_t fs, uint32_t base_rads, int16_t pu_fract16)
{
    return ((int64_t)pu_fract16 * base_rads * FRACT16_SCALE) / ((int64_t)FRACT16_PI * fs);
    // return ((int64_t)pu_fract16 * base_rads) / ((int64_t)FRACT16_PI * fs / FRACT16_SCALE);
}

/* ω_pu_fract16 = delta · π · Fs / ω_base */
static inline int32_t rads_fract16_of_angle(uint32_t fs, uint32_t base_rads, int16_t delta)
{
    return (int64_t)delta * FRACT16_PI * fs / (base_rads * FRACT16_SCALE);
}

/*
    optional include rads scaling
    // static inline int16_t angle_of_rads_fract16(uint32_t fs, uint32_t base_rads, rads_scale, int16_t rads_pu)
*/

/******************************************************************************/
/*
    si to si
*/
/******************************************************************************/
// #define RADS_PER_RPM_FRACT16 (FRACT16_PI / 30)
// #define RADS_OF_RPM(rpm) ((uint64_t)rpm * FRACT16_PI / (30U * FRACT16_SCALE))

#define RADS_PER_RPM_FLOAT (0.10471975512f) /* π/30, for rpm → rad/s */
#define RADS_OF_RPM(rpm) ((float)(rpm) * RADS_PER_RPM_FLOAT)

static inline uint32_t rads_of_rpm(uint32_t rpm, uint32_t scale) { return (uint64_t)rpm * FRACT16_PI * scale / (30U * FRACT16_SCALE); }
static inline uint32_t rpm_of_rads(uint32_t rads, uint32_t scale) { return (uint64_t)rads * 30U * FRACT16_SCALE / ((uint64_t)FRACT16_PI * scale); }
static inline uint32_t mrads_of_rpm(uint32_t rpm) { return rads_of_rpm(rpm, 1000U); }
static inline uint32_t rpm_of_mrads(uint32_t mrads) { return rpm_of_rads(mrads, 1000U); }


static inline uint32_t el_rads_of_mech_rpm(uint8_t pole_pairs, uint32_t mech_rpm) { return rads_of_rpm(mech_rpm, pole_pairs); }
static inline uint32_t mech_rpm_of_el_rads(uint8_t pole_pairs, uint32_t el_rads) { return rpm_of_rads(el_rads, pole_pairs); }