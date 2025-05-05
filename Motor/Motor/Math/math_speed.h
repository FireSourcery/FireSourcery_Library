/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MATH_MOTOR_SPEED_H
#define MATH_MOTOR_SPEED_H

#include "Math/Fixed/fract16.h"

/* Control Angle */
static inline uint32_t speed_angle16_of_rpm(uint32_t pollingFreq, uint16_t rpm)         { return ((uint32_t)rpm * 65536U) / (60U * pollingFreq); }
static inline uint32_t speed_rpm_of_angle16(uint32_t pollingFreq, uint16_t angle16)     { return (angle16 * pollingFreq / (65536U / 60U)); }

static inline uint32_t speed_el_angle16_of_mech_rpm(uint32_t pollingFreq, uint8_t polePairs, uint16_t rpm) { return speed_angle16_of_rpm(pollingFreq / polePairs, rpm); }

static inline uint32_t speed_angle16_of_cps(uint32_t pollingFreq, uint32_t cps) { return ((uint32_t)cps * 65536U) / pollingFreq; }

/* 10430 = 65536 / (2*PI) */
static inline uint32_t speed_angle16_of_rads(uint32_t pollingFreq, uint32_t rads) { return ((uint32_t)rads * 10430U) / pollingFreq; }
static inline uint32_t speed_rads_of_angle16(uint32_t pollingFreq, uint16_t angle16) { return (angle16 * pollingFreq) / 10430U; }


static inline uint16_t speed_angle16_of_fract16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t speed_fract16)
    { return ((int32_t)speed_fract16 * speedRef_Rpm) / (60 / 2 * pollingFreq); }

static inline uint16_t speed_fract16_of_angle16_rpm(uint32_t pollingFreq, uint32_t speedRef_Rpm, int16_t angle16)
    { return ((uint32_t)angle16 * 60 * pollingFreq) / (2 * speedRef_Rpm); }


// static inline fract16_t speed_unit_angle16_per_rpm(uint32_t pollingFreq, uint32_t speedMaxRef_Rpm) {speed_angle16_of_fract16_rpm(pollingFreq, speedMaxRef_Rpm, FRACT16_MAX);}



#endif