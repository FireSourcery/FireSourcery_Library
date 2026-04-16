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
    Convert between [angle16] per poll and standard representations.
    [angle16] as [DeltaAngle] at [PollingFreq].
    composition along compile time optimizable path
    provides both a of_x conversion function, as well as a per_x conversion factor
*/

/*

*/
// #define POLLING_PERIOD_FLOAT(pollingFreq) ((double)1.0F / (pollingFreq))
// #define ANGLE16_SPEED_UNIT(pollingFreq) (((double)ANGLE16_PER_REVOLUTION * POLLING_PERIOD_FLOAT(pollingFreq)))
// #define ANGLE16_FREQ_UNIT(pollingFreq) (((double)(pollingFreq) / ANGLE16_PER_REVOLUTION))

// /* Compile time init call */
// #define ANGLE_SPEED_OF(pollingFreq, turns) (int32_t)(turns * ANGLE_SPEED_UNIT(pollingFreq))  /* angle delta */
// #define ANGLE_FREQ_OF(pollingFreq, angle16) (int32_t)(angle16 * ANGLE_FREQ_UNIT(pollingFreq))  /* n turns */

/* Constant expression path (for #define composition, initializers) */
/*
    Compile time init
    args scaled by caller
*/
/* angle delta */
#define ANGLE_SPEED_OF(pollingFreq, cps)  (((int64_t)cps * ANGLE16_PER_REVOLUTION) / (pollingFreq))
/* n angle turns per time period */
#define ANGLE_FREQ_OF(pollingFreq, angle16)  (((int64_t)angle16 * (pollingFreq)) / ANGLE16_PER_REVOLUTION)

/* Factor (compile-time const or precomputed) */
#define POLLING_PERIOD_FRACT32(pollingFreq) (uint32_t)(FRACT32_SCALE / (pollingFreq))
/* effecticely polling_period_fract32 */
/* FRACT32_SCALE = ANGLE16_PER_REVOLUTION * FRACT16_SCALE */
#define ANGLE_PER_CPS_ACCUM32(pollingFreq) (int32_t)((uint32_t)ANGLE16_PER_REVOLUTION * FRACT16_SCALE / (pollingFreq))
#define CPS_PER_ANGLE_ACCUM32(pollingFreq) ((int32_t)(pollingFreq) / (ANGLE16_PER_REVOLUTION / FRACT16_SCALE))

// static inline int32_t _angle_speed_of (int32_t per_cycle, int32_t turns) { return turns * per_cycle / FRACT16_SCALE; }
// static inline int32_t _angle_freq_of (int32_t per_angle, int32_t angle16) { return angle16 * per_angle / FRACT16_SCALE; }

/* direct  for comparison */
static inline int32_t _angle_speed_of_cps(uint32_t polling_rate, int32_t cps) { return ANGLE_SPEED_OF(polling_rate, cps); }
static inline int32_t _cps_of_angle_speed(uint32_t polling_rate, int32_t angle_per_poll) { return ANGLE_FREQ_OF(polling_rate, angle_per_poll); }

/*
    Runtime - Compile time optimizable
    Compiler may optimize when [polling_rate] is const, run time without division
    assert(polling_rate != 0);
    32768 == (INT32_MAX + 1) / ANGLE16_PER_REVOLUTION
*/
/* cps [0:pollingFreq/2] */
static inline int32_t angle_speed_of(uint32_t polling_rate, int32_t cps) { return cps * ANGLE_PER_CPS_ACCUM32(polling_rate) / FRACT16_SCALE; }

/* optionally without int64 cast for base rates */
static inline int32_t _angle_freq_of(uint32_t polling_rate, int32_t angle_per_poll) { return angle_per_poll * CPS_PER_ANGLE_ACCUM32(polling_rate) / FRACT16_SCALE; }

/* (int64_t) for scaled polling rate. e.g rpm */
static inline int32_t angle_freq_of(uint32_t polling_rate, int32_t angle_per_poll) { return _cps_of_angle_speed(polling_rate, angle_per_poll); }


// typedef struct angle_speed { angle16_t Angle; angle16_t Delta; } angle_speed_t;
/******************************************************************************/
/*
    Rpm
*/
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
static inline int32_t cps_of_angle(uint32_t pollingFreq, int16_t angle16) { return _angle_freq_of(pollingFreq, angle16); }


/*
    Radians
    ANGLE16_PER_RADIAN == 10430 == 65536 / (2*PI)
*/
static inline int32_t angle_of_rads_direct(uint32_t pollingFreq, int32_t rads) { return ((int32_t)rads * ANGLE16_PER_RADIAN) / pollingFreq; }
static inline int32_t rads_of_angle_direct(uint32_t pollingFreq, int16_t angle16) { return (angle16 * pollingFreq) / ANGLE16_PER_RADIAN; }



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

