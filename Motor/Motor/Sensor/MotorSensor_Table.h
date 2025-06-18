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
    @brief  Sensor Abstraction/Selection Interface
*/
/******************************************************************************/
#include "Hall/Hall_MotorSensor.h"
#include "Encoder/Encoder_MotorSensor.h"
#include "MotorSensor.h"

/*
    Perserve order for serialization
*/
// typedef enum Motor_SensorId
typedef enum MotorSensor_Id
{
    MOTOR_SENSOR_MODE_HALL,
    MOTOR_SENSOR_MODE_ENCODER,
    MOTOR_SENSOR_MODE_SIN_COS,
    MOTOR_SENSOR_MODE_SENSORLESS,
    // MOTOR_SENSOR_MODE_EXTERN,
    // MOTOR_SENSOR_MODE_VTABLE,
    MOTOR_SENSOR_MODE_COUNT,
}
MotorSensor_Id_T;

// typedef struct MotorSensor
// {
//     MotorSensor_T * p_Active;
//     SpeedAngle_T SpeedAngle;
// }
// MotorSensor_State_T;

/*
    alternatively as Motor_SensorTable_T, part of Motor_T
*/
/* include all using preprocessor conditions */
/* There can only be 1 sensor of each type per motor, with fixed corresponding Id */
/* Mux/Repo */

// typedef const struct Motor_SensorTable
typedef const struct MotorSensor_Table
{
    const MotorSensor_T EMPTY;
// #if defined(CONFIG_MOTOR_SENSOR_HALL_ENABLE)
    const Hall_MotorSensor_T HALL;
// #endif
    const Encoder_MotorSensor_T ENCODER;
}
MotorSensor_Table_T;

/*
    Init with Motor_State_T
*/
// #define MOTOR_SENSOR_TABLE_INIT_EMPTY(p_MotorState) MOTOR_SENSOR_INIT_AS_EMPTY(&((p_MotorState)->SensorState))
// #define MOTOR_SENSOR_TABLE_INIT_HALL(p_MotorState, HallStruct, p_Encoder) HALL_MOTOR_SENSOR_INIT(HallStruct, p_Encoder, &((p_MotorState)->SensorState))

/*
    Init using shared State
*/
#define MOTOR_SENSOR_TABLE_INIT_EMPTY(MotorStateStruct) MOTOR_SENSOR_INIT_AS_EMPTY(&((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_HALL(MotorStateStruct, HallStruct, p_Encoder) HALL_MOTOR_SENSOR_INIT(HallStruct, p_Encoder, &((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_ENCODER(MotorStateStruct, EncoderStruct) ENCODER_MOTOR_SENSOR_INIT(EncoderStruct, &((MotorStateStruct).SensorState))


/*
    get by const id instead of generic mux
*/
static inline MotorSensor_T * MotorSensor_Of(const MotorSensor_Table_T * p_table, MotorSensor_Id_T id)
{
    switch (id)
    {
        case MOTOR_SENSOR_MODE_HALL: return &p_table->HALL.MOTOR_SENSOR;
        case MOTOR_SENSOR_MODE_ENCODER: return &p_table->ENCODER.MOTOR_SENSOR;
        default: return &p_table->EMPTY;
    }
}

// static inline Hall_T * Motor_SensorTable_GetHall(const MotorSensor_Table_T * p_table)
// {
//     return &p_table->HALL.HALL;
// }
