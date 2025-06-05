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
    @file   Hall_MotorSensor.h
    @author FireSourcery
    @brief  Implement the MotorSensor interface for Hall sensors.
*/
/******************************************************************************/
#include "../MotorSensor.h"
#include "Hall.h"
#include "Transducer/Encoder/Encoder.h" /* alternatively, split encoder math */
#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"


/* typedef const struct Hall_Sensor */
typedef const struct Hall_MotorSensor
{
    const MotorSensor_T MOTOR_SENSOR; /* _SUPER */
    const Hall_T HALL;
    const Encoder_T * const P_ENCODER;
}
Hall_MotorSensor_T;

extern const MotorSensor_VTable_T HALL_VTABLE;

// #define _HALL_GENERIC_INIT(p_State) MOTOR_SENSOR_INIT(&HALL_VTABLE, p_State)

#define HALL_MOTOR_SENSOR_INIT(HallStruct, p_Encoder, p_State) \
{ \
    .MOTOR_SENSOR = MOTOR_SENSOR_INIT(&HALL_VTABLE, p_State), \
    .HALL = (HallStruct), \
    .P_ENCODER = (p_Encoder), \
}

#define HALL_MOTOR_SENSOR_INIT_FROM(PinA, PinB, PinC, p_HallConfig, p_Encoder, p_InterfaceState) \
    HALL_MOTOR_SENSOR_INIT(HALL_INIT_CONSTEXPR(PinA, PinB, PinC, HALL_STATE_ALLOC(), p_HallConfig), (p_Encoder), p_InterfaceState)
