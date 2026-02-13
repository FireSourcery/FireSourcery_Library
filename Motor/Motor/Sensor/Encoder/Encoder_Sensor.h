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
    @file   Encoder_RotorSensor.h
    @author FireSourcery
    @brief  Implement the RotorSensor interface for Encoder sensors.
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "Transducer/Encoder/Encoder.h" /* alternatively, split encoder math */
#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Encoder/Encoder_ISR.h"


/* typedef const struct Encoder_Sensor */
typedef const struct Encoder_RotorSensor
{
    const RotorSensor_T BASE;
    const Encoder_T ENCODER;    // RotorSensor_Calibration_T ENCODER_CALIBRATION; /* Calibration Function Pointer */
}
Encoder_RotorSensor_T;

extern const RotorSensor_VTable_T ENCODER_VTABLE;

#define ENCODER_ROTOR_SENSOR_INIT(EncoderStruct, p_State) \
{ \
    .BASE = ROTOR_SENSOR_INIT(&ENCODER_VTABLE, p_State), \
    .ENCODER = EncoderStruct, \
}


// void Encoder_RotorSensor_Init(const RotorSensor_T * p_sensor);
// void Encoder_RotorSensor_CaptureAngle(const RotorSensor_T * p_sensor);
// void Encoder_RotorSensor_CaptureSpeed(const RotorSensor_T * p_sensor);
