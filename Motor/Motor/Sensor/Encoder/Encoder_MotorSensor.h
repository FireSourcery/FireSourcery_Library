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
    @file   Encoder_MotorSensor.h
    @author FireSourcery
    @brief  Implement the MotorSensor interface for Encoder sensors.
*/
/******************************************************************************/
#include "../MotorSensor.h"
#include "Transducer/Encoder/Encoder.h" /* alternatively, split encoder math */
#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"


/* typedef const struct Encoder_Sensor */
typedef const struct Encoder_MotorSensor
{
    const MotorSensor_T MOTOR_SENSOR;
    const Encoder_T ENCODER;
}
Encoder_MotorSensor_T;

extern const MotorSensor_VTable_T ENCODER_VTABLE;

#define ENCODER_MOTOR_SENSOR_INIT(EncoderStruct, p_State) \
{ \
    .MOTOR_SENSOR = MOTOR_SENSOR_INIT(&ENCODER_VTABLE, p_State), \
    .ENCODER = EncoderStruct, \
}


// void Encoder_MotorSensor_Init(const MotorSensor_T * p_sensor);
// void Encoder_MotorSensor_CaptureAngle(const MotorSensor_T * p_sensor);
// void Encoder_MotorSensor_CaptureSpeed(const MotorSensor_T * p_sensor);
