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
    @file   math_speed.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"


// math_angular, angle_speed

typedef int32_t angle_accum32_t;

/*
    [Delta Angle] at [Polling Freq]. Angle Per Poll

    max_rpm = (max_angle16 * SECONDS_PER_MINUTE * pollingFreq) / ANGLE16_PER_REVOLUTION
        => 20khz pollingFreq: ~ 600000

    mechanical rpm of electrical rpm: ~ 600000 / polePairs
        2 pole pairs:  300,003 RPM mechanical
        3 pole pairs:  200,002 RPM mechanical
        4 pole pairs:  150,001 RPM mechanical
        40 pole pairs:  150,00 RPM mechanical

    max angle => 32768, .5 cycles per poll
    assert(angle16 * pollingFreq < 32768 * ANGLE16_PER_REVOLUTION);
*/
#define _POLLING_FREQ_MAX (20000U)
static_assert(_POLLING_FREQ_MAX < ANGLE16_PER_REVOLUTION);


/*
    Rpm
*/
#define SECONDS_PER_MINUTE (60U)

#define SPEED_ANGLE16_PER_RPM(pollingFreq) (((double)ANGLE16_PER_REVOLUTION / (SECONDS_PER_MINUTE * pollingFreq)))
#define SPEED_RPM_PER_ANGLE16(pollingFreq) (((double)pollingFreq * SECONDS_PER_MINUTE / ANGLE16_PER_REVOLUTION))

#define SPEED_ANGLE16_OF_RPM(pollingFreq, rpm)      (((int64_t)rpm * ANGLE16_PER_REVOLUTION) / (SECONDS_PER_MINUTE * pollingFreq))
#define SPEED_RPM_OF_ANGLE16(pollingFreq, angle16)  (((int64_t)angle16 * pollingFreq * SECONDS_PER_MINUTE) / ANGLE16_PER_REVOLUTION)

static inline int32_t speed_angle16_of_rpm_direct(uint32_t pollingFreq, int32_t rpm) { return SPEED_ANGLE16_OF_RPM(pollingFreq, rpm); }
static inline int32_t speed_rpm_of_angle16_direct(uint32_t pollingFreq, int16_t angle16) { return SPEED_RPM_OF_ANGLE16(pollingFreq, angle16); }



/* Alternative direct implementations for comparison */
/*
    Cycles Per Second
*/
// static inline int32_t angle16_of_cps_direct(uint32_t pollingFreq, int32_t cps) { return ((int32_t)cps * ANGLE16_PER_REVOLUTION) / pollingFreq; }
// static inline int32_t angle16_to_cps_direct(uint32_t pollingFreq, int16_t angle16) { return ((int64_t)angle16 * pollingFreq) / ANGLE16_PER_REVOLUTION; }
// static inline int32_t speed_cps_of_direct(uint32_t pollingFreq, int16_t angle16) { return ((int64_t)angle16 * pollingFreq) / ANGLE16_PER_REVOLUTION; }

static inline int32_t speed_angle16_of_cps_direct(uint32_t pollingFreq, int32_t cps) { return ((int32_t)cps * ANGLE16_PER_REVOLUTION) / pollingFreq; }
static inline int32_t speed_cps_of_angle16_direct(uint32_t pollingFreq, int16_t angle16) { return ((int64_t)angle16 * pollingFreq) / ANGLE16_PER_REVOLUTION; }


// static inline fract16_t angle16_unit_per_speed_with(uint32_t pollingFreq, uint32_t unitAugment, uint32_t perTimeAugment) {

/*
    minutes_fract32 = INT32_MAX / (60 * pollingFreq)
    (60 * pollingFreq) > 65536
    minutes_fract32 < 65536

    Example: pollingFreq = 20000 (20kHz)
    minutes_fract32 = INT32_MAX / (60 * 20000) = 2147483647 / 1200000 = 1789 (compile-time)

    rpm * (1/60) * ANGLE16_PER_REVOLUTION / pollingFreq
    rpm * ANGLE16_PER_REVOLUTION / (60 * pollingFreq)
*/
static inline int32_t _angle16_of_rpm(uint32_t minutes_fract32, int32_t rpm) { return (int64_t)rpm * minutes_fract32 / (ANGLE16_PER_REVOLUTION / 2); }
static inline int32_t _rpm_of_angle16(uint32_t pollingFreq_perMinute, int16_t angle16) { return ((int64_t)angle16 * pollingFreq_perMinute) / ANGLE16_PER_REVOLUTION; }

/*
    pollingFreq as const
    pollingFreq != 0
*/
static inline int32_t speed_angle16_of_rpm(uint32_t pollingFreq, int16_t rpm)     { return _angle16_of_rpm((INT32_MAX / SECONDS_PER_MINUTE / pollingFreq), rpm); }
static inline int32_t speed_rpm_of_angle16(uint32_t pollingFreq, int16_t angle16) { return _rpm_of_angle16((pollingFreq * SECONDS_PER_MINUTE), angle16); }

static inline int32_t speed_el_angle16_of_mech_rpm(uint32_t pollingFreq, uint8_t polePairs, int16_t rpm) { return _angle16_of_rpm((INT32_MAX / SECONDS_PER_MINUTE / pollingFreq), (int32_t)rpm * polePairs); }
static inline int32_t speed_mech_rpm_of_el_angle16(uint32_t pollingFreq, uint8_t polePairs, int16_t angle16) { return _rpm_of_angle16((pollingFreq * SECONDS_PER_MINUTE), angle16) / polePairs; }

/*
   Cycles Per Second
*/
static inline int32_t speed_angle16_of_cps(uint32_t pollingFreq, int32_t cps) { return (INT32_MAX / pollingFreq) * cps / (ANGLE16_PER_REVOLUTION / 2); }
static inline int32_t speed_cps_of_angle16(uint32_t pollingFreq, int16_t angle16) { return speed_cps_of_angle16_direct(pollingFreq, angle16); }

/*
    Radians
    ANGLE16_PER_RADIAN == 10430 == 65536 / (2*PI)
*/
static inline int32_t speed_angle16_of_rads_direct(uint32_t pollingFreq, int32_t rads) { return ((int32_t)rads * ANGLE16_PER_RADIAN) / pollingFreq; }
static inline int32_t speed_rads_of_angle16_direct(uint32_t pollingFreq, int16_t angle16) { return (angle16 * pollingFreq) / ANGLE16_PER_RADIAN; }


/*

*/
static inline int16_t speed_angle16_of_fract16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * speedRef_Rpm) / ((SECONDS_PER_MINUTE / 2) * pollingFreq); }

static inline int16_t speed_fract16_of_angle16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t angle16)
    { return ((int32_t)angle16 * SECONDS_PER_MINUTE * pollingFreq) / (2 * speedRef_Rpm); }


/*
   Pulse Encoder
*/
// static inline int32_t speed_angle16_of_counts(uint32_t pollingFreq, uint32_t countsPerRevolution, uint32_t countFreq)
