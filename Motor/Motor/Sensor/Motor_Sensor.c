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
#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
#include "Sensorless/Motor_Sensorless.h"
#endif

/*
    requires [Motor] StateMachine outside of Sensor Interface
*/

/******************************************************************************/
/*!

*/
/******************************************************************************/
// static inline RotorSensor_T * Sensor(Motor_T * p_motor) { return RotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->Config.SensorMode); }

/*
    The chain that acquires the angle from standstill, of the selected sensor. NULL where the sensor reads the angle directly.
    Encoder's is [Motor_Encoder_GetStartUpState], its declaration commented in Motor_Encoder.h.
*/
State_T * Motor_Sensor_StartUpState(Motor_T * p_motor)
{
    switch (p_motor->P_MOTOR->Config.SensorMode)
    {
    #if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
        case ROTOR_SENSOR_ID_SENSORLESS: return Motor_Sensorless_StartUpState(p_motor);
    #endif
        default: return NULL;
    }
}

