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
*/

/*

*/
#define ANGLE_SPEED_UNIT(pollingFreq) (((double)ANGLE16_PER_REVOLUTION / (pollingFreq)))
#define ANGLE_FREQ_UNIT(pollingFreq) (((double)(pollingFreq) / ANGLE16_PER_REVOLUTION))


/*

*/
#define ANGLE_SPEED_OF(pollingFreq, cycles)  (((int64_t)cycles * ANGLE16_PER_REVOLUTION) / (pollingFreq))  /* angle delta */
#define ANGLE_FREQ_OF(pollingFreq, angle16)  (((int64_t)angle16 * (pollingFreq)) / ANGLE16_PER_REVOLUTION) /* n cycles */

/* r speed */
static inline int32_t _angle_of_cycles_direct(uint32_t polling_rate, int32_t cycles_per_poll) { return ANGLE_SPEED_OF(polling_rate, cycles_per_poll); }
static inline int32_t _cycles_of_angle_direct(uint32_t polling_rate, int32_t angle_per_poll) { return ANGLE_FREQ_OF(polling_rate, angle_per_poll); }

// #define POLLING_PERIOD_FRACT32(pollingFreq) (INT32_MAX / (pollingFreq))

/* effecticely polling_period_fract32 */
/* INT32_MAX / (pollingFreq) = (ANGLE16_PER_REVOLUTION * 32768) / (pollingFreq) */
#define ANGLE_SPEED_UNIT_ACCUM32(pollingFreq) (int32_t)((uint32_t)ANGLE16_PER_REVOLUTION * (uint32_t)FRACT16_SCALE / (pollingFreq))
#define ANGLE_FREQ_UNIT_ACCUM32(pollingFreq) ((int32_t)(pollingFreq) / (ANGLE16_PER_REVOLUTION / FRACT16_SCALE))

/*
    Runtime
    Compiler may optimize when [polling_rate] is const, run time without division
    assert(polling_rate != 0);
    32768 == (INT32_MAX + 1) / ANGLE16_PER_REVOLUTION
*/
/* cycle per poll [0:polling_rate/2] */
static inline int32_t _angle_speed_of(uint32_t polling_rate, int32_t cycles_per_poll) { return cycles_per_poll * ANGLE_SPEED_UNIT_ACCUM32(polling_rate) / FRACT16_SCALE; }
/* without int64 cast for base rates */
static inline int32_t _angle_freq_of(uint32_t polling_rate, int32_t angle_per_poll) { return angle_per_poll * ANGLE_FREQ_UNIT_ACCUM32(polling_rate) / FRACT16_SCALE; }

static inline int32_t angle_speed_of(uint32_t polling_rate, int32_t cycles_per_poll) { return _angle_speed_of(polling_rate, cycles_per_poll); }
/* (int64_t) for scaled polling rate. e.g rpm */
static inline int32_t angle_freq_of(uint32_t polling_rate, int32_t angle_per_poll) { return _cycles_of_angle_direct(polling_rate, angle_per_poll); }

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
static inline int32_t angle_of_rpm_direct(uint32_t pollingFreq, int32_t rpm) { return _angle_of_cycles_direct(pollingFreq * SECONDS_PER_MINUTE, rpm); }

static inline int32_t angle_of_rpm(uint32_t pollingFreq, int32_t rpm) { return _angle_speed_of(pollingFreq * SECONDS_PER_MINUTE, rpm); }
static inline int32_t rpm_of_angle(uint32_t pollingFreq, int16_t angle16) { return _cycles_of_angle_direct(pollingFreq * SECONDS_PER_MINUTE, angle16); }



/* ANGLE16_OF_RPM() * PolePairs */
static inline int32_t el_angle_of_mech_rpm(uint32_t pollingFreq, uint8_t polePairs, int16_t rpm) { return angle_of_rpm(pollingFreq, (int32_t)rpm * polePairs); }
static inline int32_t mech_rpm_of_el_angle(uint32_t pollingFreq, uint8_t polePairs, int16_t angle16) { return rpm_of_angle(pollingFreq, angle16) / polePairs; }


/******************************************************************************/
/*
    Cycles Per Second
*/
/******************************************************************************/
/*   */
static inline int32_t angle_of_cps_direct(uint32_t pollingFreq, int32_t cps) { return ((int32_t)cps * ANGLE16_PER_REVOLUTION) / pollingFreq; }

/* cps [0:POLLING_FREQ/2] */
static inline int32_t angle_of_cps(uint32_t pollingFreq, int16_t cps) { return _angle_speed_of(pollingFreq, cps); }
static inline int32_t cps_of_angle(uint32_t pollingFreq, int16_t angle16) { return _cycles_of_angle_direct(pollingFreq, angle16); }


/******************************************************************************/
/*
    Radians
    ANGLE16_PER_RADIAN == 10430 == 65536 / (2*PI)
*/
/******************************************************************************/
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

