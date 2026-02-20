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

static inline RotorSensor_T * Sensor(const Motor_T * p_motor) { return RotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR_STATE->Config.SensorMode); }

/******************************************************************************/
/*
    Collective Calibration access
*/
/******************************************************************************/
/*!
    @param[in] RotorSensor_Id_T as varId. 1 less layer of nesting. handle in calling module.

    // varId as sensorId, varValue as cmdId
    // enter calibration state before calling
*/
//todo split
void Motor_Sensor_CalibrationCmd_Call(const Motor_T * p_motor, RotorSensor_Id_T varId, int varValue)
{
    // If the sensor is not the selected [p_ActiveSensor], do not calibrate.
    if (!RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR_STATE->p_ActiveSensor, varId)) return;
    if (p_motor->P_MOTOR_STATE->Config.SensorMode != varId) return;

    // RotorSensor_Calibrate(p_motor->P_MOTOR_STATE->p_ActiveSensor, p_motor, varId, varValue);
    switch (varId)
    {
        case ROTOR_SENSOR_ID_HALL:
            (void)varValue; // unused
            assert(p_motor->P_MOTOR_STATE->p_ActiveSensor == (RotorSensor_T *)&p_motor->SENSOR_TABLE.HALL);
            assert(p_motor->P_MOTOR_STATE->Config.SensorMode == ROTOR_SENSOR_ID_HALL);
            Motor_Hall_Calibrate(p_motor);
            break;
        case ROTOR_SENSOR_ID_ENCODER:
            // switch (varValue)
            // {

            // }
            // Encoder_Calibrate(&p_motor->SENSOR_TABLE.ENCODER.ENCODER);
            break;
        #if defined(MOTOR_SENSOR_SIN_COS_ENABLE)
        case ROTOR_SENSOR_ID_SIN_COS:
            // SinCos_Calibrate(&p_motor->SENSOR_TABLE.SIN_COS.SIN_COS);
            break;
        #endif
        #if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
        case ROTOR_SENSOR_ID_SENSORLESS:
            // Sensorless_Calibrate(&p_motor->SENSOR_TABLE.SENSORLESS.SENSORLESS);
            break;
        #endif
        default: break;
    }
}

/******************************************************************************/
/*!
    Optionally include Var
*/
/******************************************************************************/

// void Motor_VarType_Sensor_Set(const Motor_T * p_motor, Motor_VarType_T typeId, int varId, int varValue)
// {
//     switch (typeId)
//     {
//         case MOTOR_VAR_TYPE_HALL_CONFIG:      Hall_ConfigId_Set(&p_motor->SENSOR_TABLE.HALL.HALL, varId, varValue);            break;
//         case MOTOR_VAR_TYPE_HALL_CMD:         Motor_Hall_Cmd_Call(p_motor, varId, varValue); break; // Hall cmd may not require config struct access

//         case MOTOR_VAR_TYPE_ENCODER_CONFIG:   Encoder_ConfigId_Set(&p_motor->SENSOR_TABLE.ENCODER.ENCODER, varId, varValue);   break;
//         case MOTOR_VAR_TYPE_HALL_STATE:                  break;
//         case MOTOR_VAR_TYPE_ENCODER_STATE:               break;
//             // case ROTOR_SENSOR_ID_SENSORLESS: Sensorless_ConfigId_Set(&p_motor->SENSOR_TABLE.SENSORLESS.SENSORLESS, varId, varValue); break;
//             // case ROTOR_SENSOR_ID_SIN_COS:    SinCos_ConfigId_Set(&p_motor->SENSOR_TABLE.SIN_COS.SIN_COS, varId, varValue);    break;
//         default: break;
//     }
// }


// bool Motor_Sensor_VarType_CheckSet(const Motor_T * p_motor, Motor_VarType_T typeId, int varId, int varValue)
// {
//     if (p_motor == NULL) { return false; }

//     switch (typeId)
//     {
//         case MOTOR_VAR_TYPE_HALL_STATE:                 return false;
//         case MOTOR_VAR_TYPE_HALL_CONFIG:                return Motor_IsConfig(p_motor);
//         case MOTOR_VAR_TYPE_HALL_CMD:                   return Motor_IsCalibration(p_motor) && RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR_STATE->p_ActiveSensor, ROTOR_SENSOR_ID_HALL);
//         case MOTOR_VAR_TYPE_ENCODER_CONFIG:             return Motor_IsConfig(p_motor);
//         case MOTOR_VAR_TYPE_ENCODER_STATE:              return false;
//         default: return false;
//     }
//     return false;
// }