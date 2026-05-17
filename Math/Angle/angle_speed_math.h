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

/******************************************************************************/
/*
    Generic base - args scaled by caller
*/
/******************************************************************************/
/*
    Constant expression path (for #define composition, initializers)
*/
#define ANGLE_SPEED_OF(pollingFreq, cps)    (((int64_t)(cps) * ANGLE16_PER_REVOLUTION) / (pollingFreq))
#define ANGLE_FREQ_OF(pollingFreq, angle16) (((int64_t)(angle16) * (pollingFreq)) / ANGLE16_PER_REVOLUTION)

/* direct for comparison */
static inline int32_t _angle_speed_of_freq_direct(uint32_t polling_freq, int32_t cps) { return ANGLE_SPEED_OF(polling_freq, cps); }
static inline int32_t _angle_freq_of_speed_direct(uint32_t polling_freq, int32_t angle_per_poll) { return ANGLE_FREQ_OF(polling_freq, angle_per_poll); }


/*
    Factor (compile-time const or precomputed)
*/
#define POLLING_PERIOD_FRACT32(pollingFreq) (FRACT32_SCALE / (pollingFreq))
/* effecticely polling_period_fract32 */
/* FRACT32_SCALE = ANGLE16_PER_REVOLUTION * FRACT16_SCALE */
#define ANGLE_SPEED_PER_RPS(pollingFreq) ((uint32_t)ANGLE16_PER_REVOLUTION * FRACT16_SCALE / (pollingFreq))
#define RPS_PER_ANGLE_SPEED(pollingFreq) ((pollingFreq) / (ANGLE16_PER_REVOLUTION / FRACT16_SCALE))

/*
    Runtime - Compile time optimizable
    Compiler may optimize when [polling_freq] is const, run time without division
    assert(polling_freq != 0);
    32768 == (INT32_MAX + 1) / ANGLE16_PER_REVOLUTION
*/
/* optionally without int64 cast for base rates */
static inline int32_t _angle_freq_of(uint32_t polling_freq, int32_t angle_per_poll) { return angle_per_poll * (int32_t)RPS_PER_ANGLE_SPEED(polling_freq) / FRACT16_SCALE; }

/* cps [0:pollingFreq/2] */
static inline int32_t angle_speed_of(uint32_t polling_freq, int32_t cps) { return cps * (int32_t)ANGLE_SPEED_PER_RPS(polling_freq) / FRACT16_SCALE; }
/* keep (int64_t) for scaled polling rate. e.g rpm */
static inline int32_t angle_freq_of(uint32_t polling_freq, int32_t angle_per_poll) { return ANGLE_FREQ_OF(polling_freq, angle_per_poll); }

// typedef struct angle_speed { angle16_t Angle; angle16_t Delta; } angle_speed_t;

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

    Example: pollingFreq = 20000 (20kHz)
        minutes_fract32 = INT32_MAX / (60 * 20000) = 1789 (compile-time)

    angle16:
        = rpm * ANGLE16_PER_REVOLUTION / (60 * pollingFreq)
        = rpm * minutes_fract32 / (INT32_MAX / ANGLE16_PER_REVOLUTION)
*/

#define SECONDS_PER_MINUTE (60U)

#define ANGLE16_OF_RPM(pollingFreq, rpm)      ANGLE_SPEED_OF((int64_t)pollingFreq * SECONDS_PER_MINUTE, rpm)
#define RPM_OF_ANGLE16(pollingFreq, angle16)  ANGLE_FREQ_OF((int64_t)pollingFreq * SECONDS_PER_MINUTE, angle16)

/* Alternative direct implementations for comparison */
static inline int32_t angle_of_rpm_direct(uint32_t pollingFreq, int32_t rpm) { return ANGLE16_OF_RPM(pollingFreq, rpm); }

static inline int32_t angle_of_rpm(uint32_t pollingFreq, int32_t rpm) { return angle_speed_of(pollingFreq * SECONDS_PER_MINUTE, rpm); }
static inline int32_t rpm_of_angle(uint32_t pollingFreq, int16_t angle16) { return angle_freq_of(pollingFreq * SECONDS_PER_MINUTE, angle16); }


/* ANGLE16_OF_RPM() * PolePairs */
static inline int32_t el_angle_of_mech_rpm(uint32_t pollingFreq, uint8_t polePairs, int16_t rpm) { return angle_of_rpm(pollingFreq, (int32_t)rpm * polePairs); }
static inline int32_t mech_rpm_of_el_angle(uint32_t pollingFreq, uint8_t polePairs, int16_t angle16) { return rpm_of_angle(pollingFreq, angle16) / polePairs; }


/*
    Cycles Per Second
*/
/* cps [0:POLLING_FREQ/2] */
static inline int32_t angle_of_cps(uint32_t pollingFreq, int16_t cps) { return angle_speed_of(pollingFreq, cps); }
// static inline int32_t cps_of_angle(uint32_t pollingFreq, int16_t angle16) { return _angle_freq_of(pollingFreq, angle16); }
static inline int32_t cps_of_angle(uint32_t pollingFreq, int16_t angle16) { return angle16 * (int32_t)RPS_PER_ANGLE_SPEED(pollingFreq) / FRACT16_SCALE; }

/*
    Radians
    ANGLE16_PER_RADIAN == 10430 == 65536 / (2*PI)

    ANGLE16_PER_RADIAN ~= PollingFreq / 2 ~= [angle16/poll]
*/
/* from scaled storage */
static inline int32_t angle_of_rads(uint32_t pollingFreq, accum32_t rads, uint16_t scale) { return ((int64_t)rads * ANGLE16_PER_RADIAN) / pollingFreq / scale; }
static inline int32_t rads_of_angle(uint32_t pollingFreq, int16_t angle16, uint16_t scale) { return ((int64_t)angle16 * pollingFreq * scale) / ANGLE16_PER_RADIAN; }

/******************************************************************************/
/*
    fract16
*/
/******************************************************************************/
/*
    RPM Ref
*/
// return angle_of_rpm(pollingFreq, fract16_mul(speedRef_Rpm, rpm_fract16));
static inline int16_t angle_of_rpm_fract16(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t rpm_fract16)
{
    return ((int32_t)rpm_fract16 * speedRef_Rpm) / ((SECONDS_PER_MINUTE / 2) * pollingFreq);
}

static inline int16_t rpm_fract16_of_angle(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t angle16)
{
    return ((int64_t)angle16 * ((SECONDS_PER_MINUTE / 2) * pollingFreq)) / speedRef_Rpm;
}

// static inline fract16_t rads_pu_of_angle(uint32_t polling_freq, uint32_t speed_base_rads_e, uint16_t el_delta_angle16)
// {
//     /* ω_pu_fract16 = el_delta · π · Fs / ω_base */
//     return (uint64_t)el_delta_angle16 * FRACT16_PI * polling_freq / speed_base_rads_e;
// }



/******************************************************************************/
/*!
    @brief  Boundary: el_delta_angle16  ↔  ω_pu (ω_base-anchored, fract16-scaled)

        ω_pu × FRACT16_SCALE = el_delta · 30 · Fs / (P · n_rated_rpm)
                             = el_delta · 2π·Fs/(65536·ω_base) · FRACT16_SCALE     [π cancels]

    Single Fs-dependent multiply for the textbook-pu inner loop. Above this seam,
    speed lives in angle16-per-tick (right unit for θ += ω integration); below it,
    all torque/voltage math is Fs-free.

    At ω_e = ω_base the result equals FRACT16_SCALE exactly. Result is accum32_t —
    field-weakening can push ω_pu > 1.0; signed for reverse rotation.
*/
/******************************************************************************/
static inline accum32_t speed_pu_rpm_of_angle16(uint32_t polling_freq, uint32_t speed_max_rpm, uint8_t polePairs, fract16_t el_delta_angle16) { return (int64_t)el_delta_angle16 * 30 * polling_freq / ((int32_t)polePairs * speed_max_rpm); }
static inline int32_t angle16_of_speed_pu_rpm(uint32_t polling_freq, uint32_t speed_max_rpm, uint8_t polePairs, accum32_t speed_pu) { return (int64_t)speed_pu * polePairs * speed_max_rpm / (30 * (int32_t)polling_freq); }

/* Boundary: el_delta_angle16 ↔ ω_pu (signed; allows reverse rotation and ω_pu > 1.0 in FW) */
static inline accum32_t speed_pu_rads_of_angle16(uint32_t polling_freq, uint32_t speed_base_mrads_e, fract16_t el_delta_angle16) { return (int64_t)el_delta_angle16 * FRACT16_PI * polling_freq * 1000UL / ((int64_t)speed_base_mrads_e * FRACT16_SCALE); }
static inline int32_t angle16_of_speed_pu_rads(uint32_t polling_freq, uint32_t speed_base_mrads_e, accum32_t speed_pu) { return (int64_t)speed_pu * speed_base_mrads_e * FRACT16_SCALE / ((int64_t)FRACT16_PI * polling_freq * 1000UL); }


/******************************************************************************/
/*
    si to si
*/
/******************************************************************************/
static inline uint32_t rads_of_rpm(uint32_t rpm, uint32_t scale) { return (uint64_t)FRACT16_PI * rpm * scale / (30U * FRACT16_SCALE); }
static inline uint32_t rpm_of_rads(uint32_t rads, uint32_t scale) { return (uint64_t)rads * 30U * FRACT16_SCALE / ((uint64_t)FRACT16_PI * scale); }

static inline uint32_t el_rads_of_mech_rpm(uint32_t mech_rpm, uint8_t pole_pairs) { return rads_of_rpm(mech_rpm, pole_pairs); }
static inline uint32_t mech_rpm_of_el_rads(uint32_t el_rads, uint8_t pole_pairs) { return rpm_of_rads(el_rads, pole_pairs); }

static inline uint32_t mrads_of_rpm(uint32_t speed_rpm, uint8_t polePairs) { return (uint64_t)FRACT16_PI * speed_rpm * polePairs * 1000UL / (30UL * FRACT16_SCALE); }
static inline uint32_t rpm_of_mrads(uint32_t speed_base_mrads_e, uint8_t polePairs) { return (uint64_t)speed_base_mrads_e * 30UL * FRACT16_SCALE / ((uint64_t)FRACT16_PI * polePairs * 1000UL); }
