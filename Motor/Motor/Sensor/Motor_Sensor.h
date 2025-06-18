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

/*
    Include all sensor types here.
*/
#include "Hall/Hall_MotorSensor.h"
#include "Hall/Motor_Hall.h"

#include "Encoder/Encoder_MotorSensor.h"
#include "Encoder/Motor_Encoder.h"

#include "MotorSensor_Table.h"
#include "MotorSensor.h"

/* Part of Motor */
typedef const struct Motor Motor_T;
typedef struct Motor_State Motor_State_T;


// alternatively,
// static inline MotorSensor_T * _Motor_Sensor_Get(const Motor_T * p_motor)
// {
//     return MotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_ACTIVE->Config.SensorMode);
// }

extern void Motor_Sensor_ResetUnits(Motor_State_T * p_motor);


// extern angle16_t Motor_PollSensorAngle(const Motor_T * p_motor);
// extern angle16_t Motor_CaptureAngle(const Motor_T * p_motor);
// extern bool Motor_PollCaptureSpeed(const Motor_T * p_motor);
// extern int32_t Motor_PollSensorSpeed(const Motor_T * p_motor);
// extern angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor);
// extern void Motor_ZeroSensor(const Motor_T * p_motor);
// extern bool Motor_IsClosedLoop(const Motor_T * p_motor);
// extern bool Motor_VerifySensorCalibration(const Motor_T * p_motor);
// extern void Motor_SetSensorDirection(const Motor_T * p_motor, Motor_Direction_T direction);

// extern void Motor_ResetUnitsHallEncoder(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsEncoder(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsSinCos(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsAngleSpeed_Mech(Motor_State_T * p_motor);
// extern void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_State_T * p_motor);

/******************************************************************************/
/*!
    Id Access
*/
/******************************************************************************/
// typedef enum Motor_SensorTable_VarType
// {
//     MOTOR_SENSOR_TABLE_VAR_TYPE_HALL_STATE,
//     MOTOR_SENSOR_TABLE_VAR_TYPE_HALL_CONFIG,
//     MOTOR_SENSOR_TABLE_VAR_TYPE_ENCODER_STATE,
//     MOTOR_SENSOR_TABLE_VAR_TYPE_ENCODER_CONFIG,
// }
// Motor_SensorTable_VarType_T;

int Motor_SensorTable_VarId_Get(const Motor_T * p_motor, MotorSensor_Id_T typeId, int varId);
void Motor_SensorTable_VarId_Set(const Motor_T * p_motor, MotorSensor_Id_T typeId, int varId, int varValue);
int Motor_SensorTable_ConfigId_Get(const Motor_T * p_motor, MotorSensor_Id_T typeId, int varId);
void Motor_SensorTable_ConfigId_Set(const Motor_T * p_motor, MotorSensor_Id_T typeId, int varId, int varValue);

void Motor_SensorTable_CalibrationCmd_Call(const Motor_T * p_motor, MotorSensor_Id_T varId, int varValue);
