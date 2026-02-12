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



