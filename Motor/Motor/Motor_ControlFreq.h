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
    @file   _Motor_ControlFreq.h
    @author FireSourcery
    @brief  Time/Clock References
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include <assert.h>

/*
    Part of Motor.h
*/

/******************************************************************************/
/*
    Clock/Timer Reference
*/
/******************************************************************************/
#ifndef MOTOR_CONTROL_FREQ
#define MOTOR_CONTROL_FREQ (20000U) /* Hz */
#endif

#define MOTOR_CONTROL_PERIOD_US (1000000U / MOTOR_CONTROL_FREQ) /* us */

/*
    Max representable speed

    [max angle per poll]: 32768 <=> 1/2 electrical cycle <=> pollingFreq/2 electrical cps

    [rpm] = [angle] * pollingFreq / ANGLE16_PER_REVOLUTION * SECONDS_PER_MINUTE
        => 20 kHz pollingFreq => ~600000 erpm

    mechanical rpm of electrical rpm: ~600000 / polePairs
        2 pole pairs:  300,003 RPM mechanical
        4 pole pairs:  150,001 RPM mechanical
        40 pole pairs:  150,00 RPM mechanical
*/
// #define MOTOR_ERPM_MAX (MOTOR_CONTROL_FREQ * SECONDS_PER_MINUTE / 2)


/* MOTOR_OUTER_LOOP_FREQ 1000U */
/* MOTOR_INNER_LOOP_FREQ 20000U */

#ifndef MOTOR_I_LOOP_FREQ
#define MOTOR_I_LOOP_FREQ MOTOR_CONTROL_FREQ /* CONTROL_FREQ / ANALOG_DIVIDER) */
#endif

#ifndef MOTOR_SPEED_LOOP_FREQ
#define MOTOR_SPEED_LOOP_FREQ (1000U)
#endif

/*
    Motor_Config use
*/
#define CYCLES_OF_MS(Freq, Milliseconds)    ((uint32_t)((uint32_t)Milliseconds * Freq / 1000U))
#define MS_OF_CYCLES(Freq, Cycles)          ((uint32_t)((uint32_t)Cycles * 1000U / Freq))

#define MOTOR_CONTROL_CYCLES(Milliseconds)  CYCLES_OF_MS(MOTOR_CONTROL_FREQ, Milliseconds)
#define MOTOR_CONTROL_TIME_MS(Cycles)       MS_OF_CYCLES(MOTOR_CONTROL_FREQ, Cycles)

/* Ramp Time */
#define MOTOR_TORQUE_CYCLES(Milliseconds)   CYCLES_OF_MS(MOTOR_CONTROL_FREQ, Milliseconds)
#define MOTOR_TORQUE_TIME_MS(Cycles)        MS_OF_CYCLES(MOTOR_CONTROL_FREQ, Cycles)

#define MOTOR_I_CYCLES(Milliseconds)        CYCLES_OF_MS(MOTOR_I_LOOP_FREQ, Milliseconds)
#define MOTOR_I_TIME_MS(Cycles)             MS_OF_CYCLES(MOTOR_I_LOOP_FREQ, Cycles)

#define MOTOR_SPEED_CYCLES(Milliseconds)    CYCLES_OF_MS(MOTOR_SPEED_LOOP_FREQ, Milliseconds)
#define MOTOR_SPEED_TIME_MS(Cycles)         MS_OF_CYCLES(MOTOR_SPEED_LOOP_FREQ, Cycles)


/* perserve invariance */
#define MOTOR_SPEED_RAMP_CYCLES(DeltaOutput, UnitPerS) ((uint32_t)MOTOR_SPEED_LOOP_FREQ * (DeltaOutput) / (UnitPerS))
#define MOTOR_TORQUE_RAMP_CYCLES(DeltaOutput, UnitPerS) ((uint32_t)MOTOR_CONTROL_FREQ * (DeltaOutput) / (UnitPerS))

#define MOTOR_SPEED_RAMP_COEF_OF_MS(RangeFract16, DurationMs)   RAMP_COEF_OF_DURATION_MS(MOTOR_SPEED_LOOP_FREQ, RangeFract16, DurationMs)
#define MOTOR_SPEED_RAMP_COEF_OF_SLOPE(Fract16PerS)             RAMP_COEF_OF_SLOPE(MOTOR_SPEED_LOOP_FREQ, Fract16PerS)
#define MOTOR_TORQUE_RAMP_COEF_OF_MS(RangeFract16, DurationMs)  RAMP_COEF_OF_DURATION_MS(MOTOR_CONTROL_FREQ, RangeFract16, DurationMs)
#define MOTOR_TORQUE_RAMP_COEF_OF_SLOPE(Fract16PerS)            RAMP_COEF_OF_SLOPE(MOTOR_CONTROL_FREQ, Fract16PerS)

/*
    Local Conversion Units
*/
static inline uint32_t _Motor_MillisOf(uint32_t controlCycles) { return MOTOR_CONTROL_TIME_MS(controlCycles); }
static inline uint32_t _Motor_ControlCyclesOf(uint32_t millis) { return MOTOR_CONTROL_CYCLES(millis); }


