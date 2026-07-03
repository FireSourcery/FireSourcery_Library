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
    @file   Motor_Sensor.h
    @author FireSourcery
    @brief  Sensor and SensorTable Implementation and Interface
*/
/******************************************************************************/
#include "../Motor.h"

// #if defined(MOTOR_SENSOR_HALL_ENABLE)
#include "Hall/Motor_Hall.h"
#include "Hall/Motor_HallEncoder.h"
// #endif
#if defined(MOTOR_SENSOR_ENCODER_ENABLE)
#include "Encoder/Motor_Encoder.h"
#endif

/*
    Include all compile time sensor options
*/
#include "RotorSensor_Table.h"

/* dependent part of Motor_Var   */

/*
    Init with Motor_Context_T
    Init using shared State
*/
#define MOTOR_SENSOR_TABLE_INIT_EMPTY(MotorStateStruct) ROTOR_SENSOR_INIT_AS_EMPTY(&((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_HALL(MotorStateStruct, HallStruct, PulseStruct, PollingFreq) HALL_ROTOR_SENSOR_INIT(HallStruct, PulseStruct, PollingFreq, &((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_ENCODER(MotorStateStruct, EncoderStruct) ENCODER_ROTOR_SENSOR_INIT(EncoderStruct, &((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_SENSORLESS(MotorStateStruct) SENSORLESS_SENSOR_INIT(&((MotorStateStruct).SensorState), &((MotorStateStruct).FocSensorless))



