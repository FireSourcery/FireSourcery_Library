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
    @file   MotorTimeReference.h
    @author FireSourcery
    @brief  Global Static, for all Motor instances

*/
/******************************************************************************/
#ifndef MOTOR_TIME_REFERENCE_H
#define MOTOR_TIME_REFERENCE_H

#include "Math/Fixed/fract16.h"

/******************************************************************************/
/*
    Clock/Timer Reference
*/
/******************************************************************************/
#ifndef MOTOR_CONTROL_FREQ
#define MOTOR_CONTROL_FREQ (20000U) /* Hz */
#endif

/* INNER_LOOP */
/* OUTER_LOOP */
/* INNER_CONTROL_FREQ 20000U */
/* OUTER_CONTROL_FREQ 1000U */

#ifndef MOTOR_I_LOOP_FREQ
#define MOTOR_I_LOOP_FREQ (10000U) /* CONTROL_FREQ / ANALOG_DIVIDER) */
#endif

#ifndef MOTOR_SPEED_LOOP_FREQ
#define MOTOR_SPEED_LOOP_FREQ (1000U)
#endif

#define CYCLES_OF_MS(Freq, Milliseconds)    ((uint32_t)((uint32_t)Milliseconds * Freq / 1000U))
#define MS_OF_CYCLES(Freq, Cycles)          ((uint32_t)((uint32_t)Cycles * 1000U / Freq))

#define MOTOR_CONTROL_CYCLES(Milliseconds)  CYCLES_OF_MS(MOTOR_CONTROL_FREQ, Milliseconds)
#define MOTOR_CONTROL_TIME_MS(Cycles)       MS_OF_CYCLES(MOTOR_CONTROL_FREQ, Cycles)

#define MOTOR_TORQUE_CYCLES(Milliseconds)   CYCLES_OF_MS(MOTOR_CONTROL_FREQ, Milliseconds)
#define MOTOR_TORQUE_TIME_MS(Cycles)        MS_OF_CYCLES(MOTOR_CONTROL_FREQ, Cycles)

#define MOTOR_I_CYCLES(Milliseconds)        CYCLES_OF_MS(MOTOR_I_LOOP_FREQ, Milliseconds)
#define MOTOR_I_TIME_MS(Cycles)             MS_OF_CYCLES(MOTOR_I_LOOP_FREQ, Cycles)

#define MOTOR_SPEED_CYCLES(Milliseconds)    CYCLES_OF_MS(MOTOR_SPEED_LOOP_FREQ, Milliseconds)
#define MOTOR_SPEED_TIME_MS(Cycles)         MS_OF_CYCLES(MOTOR_SPEED_LOOP_FREQ, Cycles)

#ifndef MOTOR_ANALOG_DIVIDER_MASK
// #define MOTOR_ANALOG_DIVIDER_MASK (0U) /* 0 -> 1x, 1 -> 2x, 2 -> 4x, 3 -> 8x */
#define MOTOR_ANALOG_DIVIDER_MASK ((MOTOR_CONTROL_FREQ / MOTOR_I_LOOP_FREQ) - 1U) /* must be pow2 */
#endif

//alternatively
// typedef const struct MotorTimeReference
// {
//     const uint32_t MOTOR_CONTROL_FREQ;
//     const uint32_t MOTOR_CURRENT_LOOP_FREQ;
// }
// MotorTimeReference_T;
// MotorRef_Clock_T;

static inline bool MotorTimeRef_IsAnalogCycle(uint32_t timerCounter) { return ((timerCounter & MOTOR_ANALOG_DIVIDER_MASK) == 0UL); }

/*
    Local Conversion Units
*/
static inline uint32_t _Motor_MillisOf(uint32_t controlCycles) { return MOTOR_CONTROL_TIME_MS(controlCycles); }
static inline uint32_t _Motor_ControlCyclesOf(uint32_t millis) { return MOTOR_CONTROL_CYCLES(millis); }

#endif