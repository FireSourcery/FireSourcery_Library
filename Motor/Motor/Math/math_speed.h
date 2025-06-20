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
// angle16_speed

#define ANGLE16_PER_REVOLUTION  65536U
#define SECONDS_PER_MINUTE      60U

#define INV_60_FRACT32  0x044444444UL  // 1/60 in 32-bit fixed point

/*
    minutes_fract32 = INT32_MAX / (60 * pollingFreq)
        < 65536
*/
/* shift 16 - 1, ANGLE16_MAX = 2*32678 */
static inline uint32_t angle16_of_rpm(uint32_t minutes_fract32, uint16_t rpm) { return (uint32_t)rpm * minutes_fract32 >> 15; }
static inline uint32_t rpm_of_angle16(uint32_t pollsPerMinute, uint16_t angle16) { return ((uint64_t)angle16 * pollsPerMinute) / ANGLE16_PER_REVOLUTION; }
// static inline uint32_t angle16_to_rpm(uint32_t pollingFreq, uint16_t angle16) { return (angle16 * pollingFreq) / (ANGLE16_PER_REVOLUTION / SECONDS_PER_MINUTE); }

static inline uint32_t angle16_el_speed_of_mech_rpm(uint32_t minutes_fract32, uint8_t polePairs, uint16_t rpm) { return (uint32_t)rpm * minutes_fract32 * polePairs >> 15; }



/* Control Angle */
static inline uint32_t speed_angle16_of_rpm(uint32_t pollingFreq, uint16_t rpm)         { return ((uint32_t)rpm * 65536U) / (60U * pollingFreq); }
static inline uint32_t speed_rpm_of_angle16(uint32_t pollingFreq, uint16_t angle16)     { return (angle16 * pollingFreq / (65536U / 60U)); }

static inline uint32_t speed_el_angle16_of_mech_rpm(uint32_t pollingFreq, uint8_t polePairs, uint16_t rpm) { return speed_angle16_of_rpm(pollingFreq / polePairs, rpm); }

static inline uint32_t speed_angle16_of_cps(uint32_t pollingFreq, uint32_t cps) { return ((uint32_t)cps * 65536U) / pollingFreq; }

/* elangle16 * pollingFreq < 32768 * 65536U */
/* max elangle => .5 cycles per poll */
static inline uint32_t speed_cps_of_angle16(uint32_t pollingFreq, uint16_t angle16) { return (angle16 * pollingFreq) / 65536U; }

/* 10430 = 65536 / (2*PI) */
static inline uint32_t speed_angle16_of_rads(uint32_t pollingFreq, uint32_t rads) { return ((uint32_t)rads * ANGLE16_PER_RAD) / pollingFreq; }
static inline uint32_t speed_rads_of_angle16(uint32_t pollingFreq, uint16_t angle16) { return (angle16 * pollingFreq) / ANGLE16_PER_RAD; }


static inline int16_t speed_angle16_of_fract16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * speedRef_Rpm) / (60 / 2 * pollingFreq); }

static inline int16_t speed_fract16_of_angle16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t angle16)
    { return ((uint32_t)angle16 * 60 * pollingFreq) / (2 * speedRef_Rpm); }


// static inline fract16_t speed_unit_angle16_per_rpm(uint32_t pollingFreq, uint32_t speedMaxRef_Rpm) {speed_angle16_of_fract16_rpm(pollingFreq, speedMaxRef_Rpm, FRACT16_MAX);}


// static inline accum32_t Motor_Speed_Fract16OfRpm(const Motor_State_T * p_motor, int16_t speed_rpm)        { return speed_rpm * INT16_MAX / Motor_GetSpeedRatedRef_Rpm(p_motor); }
// static inline int16_t   Motor_Speed_RpmOfFract16(const Motor_State_T * p_motor, accum32_t speed_fract16)  { return speed_fract16 * Motor_GetSpeedRatedRef_Rpm(p_motor) / 32768; }
