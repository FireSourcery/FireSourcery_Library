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
    @file   Motor_Sensor.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Motor_Sensor.h"
#include "../Motor_StateMachine.h"

/*
    requires [Motor] StateMachine outside of Sensor Interface
*/

/******************************************************************************/
/*!

*/
/******************************************************************************/
// static inline RotorSensor_T * Sensor(const Motor_T * p_motor) { return RotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->Config.SensorMode); }

/******************************************************************************/
/*
    [VarType_Sensor]
*/
/******************************************************************************/
// #include "Sensor/Motor_Sensor.h" /* for calibration cmd */

int Motor_VarType_Sensor_Get(const Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId)
{
    if (p_motor == NULL) { return 0; }
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HALL_STATE:     return Hall_VarId_Get(&p_motor->SENSOR_TABLE.HALL.HALL, varId);
        case MOTOR_VAR_TYPE_HALL_CONFIG:    return _Hall_ConfigId_Get(&p_motor->SENSOR_TABLE.HALL.HALL.P_STATE->Config, varId);
        #if defined(MOTOR_SENSOR_ENCODER_ENABLE)
        case MOTOR_VAR_TYPE_ENCODER_STATE:  return Encoder_ModeDT_VarId_Get(p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE, varId);
        case MOTOR_VAR_TYPE_ENCODER_CONFIG: return _Encoder_ConfigId_Get(&p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE->Config, varId);
        #endif
        default: return 0;
    }
}

void Motor_VarType_Sensor_Set(const Motor_T * p_motor, Motor_VarType_Sensor_T typeId, int varId, int varValue)
{
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HALL_CONFIG:      _Hall_ConfigId_Set(&p_motor->SENSOR_TABLE.HALL.HALL.P_STATE->Config, varId, varValue);            break;
        case MOTOR_VAR_TYPE_HALL_CMD:         Motor_Hall_Cmd(p_motor, varId, varValue); break;
        case MOTOR_VAR_TYPE_HALL_STATE:                  break;
        #if defined(MOTOR_SENSOR_ENCODER_ENABLE)
        case MOTOR_VAR_TYPE_ENCODER_CONFIG:   _Encoder_ConfigId_Set(&p_motor->SENSOR_TABLE.ENCODER.ENCODER.P_STATE->Config, varId, varValue);   break;
        case MOTOR_VAR_TYPE_ENCODER_STATE:               break;
        case MOTOR_VAR_TYPE_ENCODER_CMD:                 break;
        #endif
        default: break;
    }
}

bool Motor_VarType_Sensor_CheckSet(const Motor_T * p_motor, Motor_VarType_Sensor_T typeId)
{
    if (p_motor == NULL) { return false; }
    switch (typeId)
    {
        case MOTOR_VAR_TYPE_HALL_STATE:     return false;
        case MOTOR_VAR_TYPE_HALL_CONFIG:    return Motor_IsConfig(p_motor);
        case MOTOR_VAR_TYPE_HALL_CMD:       return Motor_IsState(p_motor, MOTOR_STATE_ID_CALIBRATION);
        case MOTOR_VAR_TYPE_ENCODER_STATE:  return false;
        case MOTOR_VAR_TYPE_ENCODER_CONFIG: return Motor_IsConfig(p_motor);
        case MOTOR_VAR_TYPE_ENCODER_CMD:    return Motor_IsState(p_motor, MOTOR_STATE_ID_CALIBRATION);
        default: return false;
    }
}
