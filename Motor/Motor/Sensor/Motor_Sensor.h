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

/*
    Include all compile time sensor options
*/
#include "Hall/Motor_Hall.h"
#include "Encoder/Motor_Encoder.h"

#include "RotorSensor_Table.h"

/* dependent part of Motor_Var   */

/*
    Init with Motor_State_T
    Init using shared State
*/
#define MOTOR_SENSOR_TABLE_INIT_EMPTY(MotorStateStruct) ROTOR_SENSOR_INIT_AS_EMPTY(&((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_HALL(MotorStateStruct, HallStruct, PulseStruct, PollingFreq) HALL_ROTOR_SENSOR_INIT(HallStruct, PulseStruct, PollingFreq, &((MotorStateStruct).SensorState))
#define MOTOR_SENSOR_TABLE_INIT_ENCODER(MotorStateStruct, EncoderStruct) ENCODER_ROTOR_SENSOR_INIT(EncoderStruct, &((MotorStateStruct).SensorState))


/******************************************************************************/
/*
    Sensor ISR
*/
/******************************************************************************/
/* Optionally use Hall ISR */
static inline void Motor_HallEncoderA_ISR(const Motor_T * p_dev)
{
#ifdef MOTOR_SENSOR_ENCODER_ENABLE
    Encoder_OnPhaseA_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#endif
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderB_ISR(const Motor_T * p_dev)
{
#ifdef MOTOR_SENSOR_ENCODER_ENABLE
    Encoder_OnPhaseB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#endif
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderAB_ISR(const Motor_T * p_dev)
{
#ifdef MOTOR_SENSOR_ENCODER_ENABLE
    Encoder_OnPhaseAB_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
#endif
#if defined(MOTOR_HALL_MODE_ISR)
    if (p_dev->P_MOTOR->Config.SensorMode == ROTOR_SENSOR_ID_HALL) { Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL); }
#endif
}

static inline void Motor_HallEncoderCZ_ISR(const Motor_T * p_dev)
{
    switch (p_dev->P_MOTOR->Config.SensorMode)
    {
        #ifdef MOTOR_SENSOR_ENCODER_ENABLE
        case ROTOR_SENSOR_ID_ENCODER:
            Encoder_OnIndex_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
            break;
        #endif

        #ifdef MOTOR_HALL_MODE_ISR
        case ROTOR_SENSOR_ID_HALL:
            Encoder_OnPhaseC_Hall_ISR(&p_dev->SENSOR_TABLE.ENCODER.ENCODER);
            Hall_CaptureAngle_ISR(&p_dev->SENSOR_TABLE.HALL.HALL);
            break;
        #endif
        default: break;
    }
}

/******************************************************************************/
/*
    Handled by RotorSensor_Table
    Instead of using SensorTable Ids, This way it takes only one field to associate properties.
*/
/******************************************************************************/
typedef enum Motor_VarType_Sensor
{
    MOTOR_VAR_TYPE_HALL_STATE,
    MOTOR_VAR_TYPE_HALL_CONFIG,
    MOTOR_VAR_TYPE_HALL_CMD,
    MOTOR_VAR_TYPE_ENCODER_STATE,
    MOTOR_VAR_TYPE_ENCODER_CONFIG,
    MOTOR_VAR_TYPE_ENCODER_CMD,
}
Motor_VarType_Sensor_T;


extern int Motor_VarType_Sensor_Get(const Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId);
extern void Motor_VarType_Sensor_Set(const Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId, int varValue);
extern bool Motor_VarType_Sensor_CheckSet(const Motor_T * p_motor, Motor_VarType_Sensor_T typeId);


/******************************************************************************/
/*!
    Calibration State Access
*/
/******************************************************************************/
// extern void Motor_Sensor_CalibrationCmd_Call(const Motor_T * p_motor, RotorSensor_Id_T varId, int varValue);
// typedef enum Motor_Sensor_CalibrationStatus
// {
// }
// Motor_Sensor_CalibrationStatus_T;

// static bool Motor_Sensor_Validate(const Motor_T * p_motor, RotorSensor_Id_T id, int varValue)
// {
//     if (!RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->p_ActiveSensor, id)) return;
//     if (p_motor->P_MOTOR->Config.SensorMode != id) return;
// }
