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
#include "Hall/Hall_Sensor.h"
#include "Encoder/Encoder_Sensor.h"
#include "RotorSensor.h"





#if     defined(MOTOR_SENSOR_SIN_COS_ENABLE)
#elif   defined(MOTOR_SENSOR_SIN_COS_DISABLE)
#else
#define MOTOR_SENSOR_SIN_COS_DISABLE
#endif

#if     defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
#elif   defined(MOTOR_SENSOR_SENSORLESS_DISABLE)
#else
#define MOTOR_SENSOR_SENSORLESS_DISABLE
#endif

/*
    [RotorSensor_Id]
    TypeId
    Generic access
    Preserve order for serialization
*/
typedef enum RotorSensor_Id
{
    ROTOR_SENSOR_ID_HALL,
    ROTOR_SENSOR_ID_ENCODER,
    ROTOR_SENSOR_ID_SIN_COS,
    ROTOR_SENSOR_ID_SENSORLESS,
    // ROTOR_SENSOR_ID_EXTERN,
    // ROTOR_SENSOR_ID_VTABLE,
    ROTOR_SENSOR_ID_COUNT,
}
RotorSensor_Id_T;


/*
    [RotorSensor_Table_T] Repo
    Holds all selectable sensors. Map common AngleSpeed State
    include all using preprocessor conditions
    There can only be 1 sensor of each type per motor, with fixed corresponding Id
*/
// alternatively as [Motor_SensorTable_T], part of Motor_T
// RotorSensor_T ** P_SENSOR_TABLE; may simplify external addition of sensor
typedef const struct RotorSensor_Table
{
    const RotorSensor_T EMPTY;
// #if defined(MOTOR_SENSOR_HALL_ENABLE)
    const Hall_RotorSensor_T HALL;    // const Hall_T HALL;
// #endif
    const Encoder_RotorSensor_T ENCODER; // const Encoder_T ENCODER;
}
RotorSensor_Table_T;

// typedef void(*RotorSensor_Calibration_T)(const void * p_motor, const struct RotorSensor * p_sensor, int cmdId);
// typedef enum Motor_Sensor_CalibrationStatus
// {
// }
// Motor_Sensor_CalibrationStatus_T;

// #define ROTOR_SENSOR_TABLE_INIT_EMPTY(p_State) ROTOR_SENSOR_INIT_AS_EMPTY(p_State)
// #define ROTOR_SENSOR_TABLE_INIT_HALL(p_State, HallStruct, p_Encoder) HALL_ROTOR_SENSOR_INIT(HallStruct, p_Encoder, p_State)
// #define ROTOR_SENSOR_TABLE_INIT_ENCODER(p_State, EncoderStruct) ENCODER_ROTOR_SENSOR_INIT(EncoderStruct, p_State)


/*
    get by const id, mux with named indexs
*/
static inline RotorSensor_T * RotorSensor_Of(const RotorSensor_Table_T * p_table, RotorSensor_Id_T id)
{
    switch (id)
    {
        case ROTOR_SENSOR_ID_HALL: return &p_table->HALL.MOTOR_SENSOR;
        case ROTOR_SENSOR_ID_ENCODER: return &p_table->ENCODER.MOTOR_SENSOR;
        default: return &p_table->EMPTY;
    }
}

static inline bool RotorSensor_Validate(const RotorSensor_Table_T * p_table, const RotorSensor_T * p_sensor, RotorSensor_Id_T id)
{
    if (id >= ROTOR_SENSOR_ID_COUNT) { return false; }
    if (RotorSensor_Of(p_table, id) != p_sensor) { return false; }
    return true;
}

