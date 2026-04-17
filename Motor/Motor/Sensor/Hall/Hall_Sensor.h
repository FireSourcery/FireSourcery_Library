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
    @file   Hall_RotorSensor.h
    @author FireSourcery
    @brief  Implement the RotorSensor interface for Hall sensors.
            Composes: AngleCounter_T (counter + frequency + interpolation via Angle_T Base),
            PulseTimer (Timer HAL wrap), Hall_T (sensor decode).
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "Hall.h"
#include "Transducer/Pulse/PulseTimer.h"
#include "Math/Angle/AngleCounter.h"

typedef const struct Hall_RotorSensor
{
    const RotorSensor_T BASE;       /* P_STATE->AngleSpeed is the final output interface */
    const Hall_T HALL;
    const PulseTimer_T TIMER;       /* Timer HAL wrap */
    AngleCounter_T * P_COUNTER;     /* Counter + frequency + interpolation state */
    uint32_t POLLING_FREQ;          /* Control loop frequency [Hz] for angle delta conversion */
}
Hall_RotorSensor_T;

extern const RotorSensor_VTable_T HALL_VTABLE;

#define HALL_ROTOR_SENSOR_INIT(HallStruct, TimerStruct, p_Counter, PollingFreq, p_State) (Hall_RotorSensor_T) \
{                                                                                               \
    .BASE           = ROTOR_SENSOR_INIT(&HALL_VTABLE, p_State),                                 \
    .HALL           = (HallStruct),                                                             \
    .TIMER          = (TimerStruct),                                                            \
    .P_COUNTER      = (p_Counter),                                                              \
    .POLLING_FREQ   = (PollingFreq),                                                            \
}


// typedef struct Hall_SensorState
// {
//     Hall_State_T Hall;
//     PulseTimer_State_T Timer;
//     AngleCounter_T Counter; /* Counter + frequency + interpolation state */
// }
// Hall_SensorState_T;

// typedef const struct Hall_RotorSensor1
// {
//     const RotorSensor_T BASE;       /* P_STATE->AngleSpeed is the final output interface */
//     const Hall_T HALL;
//     const PulseTimer_T TIMER;       /* Timer HAL wrap */
//     Hall_SensorState_T * P_STATE;
// }
// Hall_RotorSensor1_T;

// #define HALL_ROTOR_SENSOR_INIT(PinA, PinB, PinC, p_NvmConfig, p_TimerHal, TimerFreq, SampleFreq, p_RotorBaseState, p_HallSensorState) (Hall_RotorSensor1_T) \
// {                                                                                               \
//     .BASE           = ROTOR_SENSOR_INIT(&HALL_VTABLE, p_RotorBaseState),                                 \
//     .HALL           = HALL_INIT(PinA, PinB, PinC, p_NvmConfig, &(p_HallSensorState->Hall)),                  \
//     .TIMER          = PULSE_TIMER_INIT(p_TimerHal, TimerFreq, SampleFreq, &(p_HallSensorState->Timer)),   \
// }

