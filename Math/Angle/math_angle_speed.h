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
    @brief  rotory, rotational angle speed math.
*/
/******************************************************************************/
#include "../Fixed/fract16.h"


/*
    [Delta Angle] at [Polling Freq]. Angle Per Poll

    [max angle]: 32768 <=> Electrical Cycles < pollingFreq/2 <=> .5 cycles per poll

    [max rpm] = (max_angle16 * SECONDS_PER_MINUTE * pollingFreq) / ANGLE16_PER_REVOLUTION
        => 20khz pollingFreq: ~ 600000

    mechanical rpm of electrical rpm: ~ 600000 / polePairs
        2 pole pairs:  300,003 RPM mechanical
        4 pole pairs:  150,001 RPM mechanical
        40 pole pairs:  150,00 RPM mechanical
*/
#define _POLLING_FREQ_MAX (20000U)
static_assert(ANGLE16_PER_REVOLUTION * _POLLING_FREQ_MAX < 32768 * ANGLE16_PER_REVOLUTION);

/******************************************************************************/
/*
    [Rpm] Convert between [angle16] and RPM representations.
*/
/******************************************************************************/
#define SECONDS_PER_MINUTE (60U)

#define ANGLE16_PER_RPM(pollingFreq) (((double)ANGLE16_PER_REVOLUTION / (SECONDS_PER_MINUTE * pollingFreq)))
#define RPM_PER_ANGLE16(pollingFreq) (((double)pollingFreq * SECONDS_PER_MINUTE / ANGLE16_PER_REVOLUTION))

#define ANGLE16_OF_RPM(pollingFreq, rpm)      (((int64_t)rpm * ANGLE16_PER_REVOLUTION) / (SECONDS_PER_MINUTE * pollingFreq))
#define RPM_OF_ANGLE16(pollingFreq, angle16)  (((int64_t)angle16 * pollingFreq * SECONDS_PER_MINUTE) / ANGLE16_PER_REVOLUTION)

/* Alternative direct implementations for comparison */
static inline int32_t angle_of_rpm_direct(uint32_t pollingFreq, int32_t rpm) { return ANGLE16_OF_RPM(pollingFreq, rpm); }
static inline int32_t rpm_of_angle_direct(uint32_t pollingFreq, int16_t angle16) { return RPM_OF_ANGLE16(pollingFreq, angle16); }

/*
    [minutes_fract32] = INT32_MAX / (60 * pollingFreq)
    minutes_fract32 = (ANGLE16_PER_REVOLUTION * 32768) / (60 * pollingFreq)

    Example: pollingFreq = 20000 (20kHz)
    minutes_fract32 = INT32_MAX / (60 * 20000) = 2147483647 / 1200000 = 1789 (compile-time)

    angle16:
        = rpm * (1/60) * ANGLE16_PER_REVOLUTION / pollingFreq
        = rpm * ANGLE16_PER_REVOLUTION / (60 * pollingFreq)
        = rpm * minutes_fract32 / (ANGLE16_PER_REVOLUTION / 2)
*/
static inline int32_t _angle_of_speed_rpm(uint32_t minutes_fract32, int32_t rpm) { return ((int64_t)rpm * minutes_fract32) / (ANGLE16_PER_REVOLUTION / 2); }
static inline int32_t _speed_rpm_of_angle(uint32_t polling_freq_minutes, int32_t angle16) { return ((int64_t)angle16 * polling_freq_minutes) / ANGLE16_PER_REVOLUTION; }

/*
    pollingFreq as const
    pollingFreq != 0
*/
static inline int32_t angle_of_rpm(uint32_t pollingFreq, int16_t rpm) { return _angle_of_speed_rpm((INT32_MAX / SECONDS_PER_MINUTE / pollingFreq), rpm); }
static inline int32_t rpm_of_angle(uint32_t pollingFreq, int16_t angle16) { return _speed_rpm_of_angle((pollingFreq * SECONDS_PER_MINUTE), angle16); }

/* call private helper with [int32_t rpm] */
/* ANGLE16_OF_RPM() * PolePairs */
static inline int32_t el_angle_of_mech_rpm(uint32_t pollingFreq, uint8_t polePairs, int16_t rpm) { return _angle_of_speed_rpm((INT32_MAX / SECONDS_PER_MINUTE / pollingFreq), (int32_t)rpm * polePairs); }
static inline int32_t mech_rpm_of_el_angle(uint32_t pollingFreq, uint8_t polePairs, int16_t angle16) { return _speed_rpm_of_angle((pollingFreq * SECONDS_PER_MINUTE), (int32_t)angle16 * polePairs); }


/******************************************************************************/
/*
    fract16
*/
/******************************************************************************/
/*
    fract16 of rated
    Independent of PollingFreq
*/
/* ANGLE_PER_REVOLUTION / FRACT16_MAX == 2 */
static inline int16_t speed_fract16_of_angle_direct(angle16_t speedRef_degPerPoll, int16_t angle16) { return ((int32_t)angle16 * FRACT16_MAX) / speedRef_degPerPoll; }

/* speedRefInv_fract32 as cycles per degree */
/* alternatively with norm shift */
/* overflow note: angle16 < speedRefInv_fract32 */
static inline int16_t speed_fract16_of_angle(uint32_t speedRefInv_fract32, int16_t angle16) { return ((int32_t)angle16 * speedRefInv_fract32) >> 16U; }
static inline int16_t angle_of_speed_fract16(angle16_t speedRef_degPerPoll, int16_t speed_fract16) { return fract16_mul(speed_fract16, speedRef_degPerPoll); }

/*
    RPM Ref
*/
#define SPEED_FRACT16_OF_RPM(SpeedMaxRpm, speed_rpm)      (speed_rpm * INT16_MAX / SpeedMaxRpm)
#define SPEED_RPM_OF_FRACT16(SpeedMaxRpm, speed_fract16)  (speed_fract16 * SpeedMaxRpm / 32768)

static inline int16_t angle_of_speed_fract16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t speed_fract16)
{
    return ((int32_t)speed_fract16 * speedRef_Rpm) / ((SECONDS_PER_MINUTE / 2) * pollingFreq);
    // return angle_of_rpm(pollingFreq, fract16_mul(speedRef_Rpm, speed_fract16));
}

static inline int16_t speed_fract16_of_angle16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t angle16)
{
    return ((int32_t)angle16 * (SECONDS_PER_MINUTE / 2) * pollingFreq) / speedRef_Rpm;
}



/******************************************************************************/
/*
    Cycles Per Second
*/
/******************************************************************************/
static inline int32_t angle_of_cps_direct(uint32_t pollingFreq, int32_t cps) { return ((int32_t)cps * ANGLE16_PER_REVOLUTION) / pollingFreq; }
static inline int32_t cps_of_angle_direct(uint32_t pollingFreq, int16_t angle16) { return ((int64_t)angle16 * pollingFreq) / ANGLE16_PER_REVOLUTION; }

static inline int32_t angle_of_cps(uint32_t pollingFreq, int32_t cps) { return (INT32_MAX / pollingFreq) * cps / (ANGLE16_PER_REVOLUTION / 2); }
static inline int32_t cps_of_angle(uint32_t pollingFreq, int16_t angle16) { return cps_of_angle_direct(pollingFreq, angle16); }


/******************************************************************************/
/*
    Radians
    ANGLE16_PER_RADIAN == 10430 == 65536 / (2*PI)
*/
/******************************************************************************/
static inline int32_t angle_of_rads_direct(uint32_t pollingFreq, int32_t rads) { return ((int32_t)rads * ANGLE16_PER_RADIAN) / pollingFreq; }
static inline int32_t rads_of_angle_direct(uint32_t pollingFreq, int16_t angle16) { return (angle16 * pollingFreq) / ANGLE16_PER_RADIAN; }


// static inline fract16_t angle16_unit_per_speed_with(uint32_t pollingFreq, uint32_t unitAugment, uint32_t perTimeAugment) {