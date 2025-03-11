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
    @file   MotorSensor.h
    @author FireSourcery
    @version V0
    @brief  Position Sensor Interface
*/
/******************************************************************************/
#ifndef MOTOR_SENSOR_H
#define MOTOR_SENSOR_H

#include <stdint.h>
#include <stdbool.h>


typedef uint16_t(*MotorSensor_Angle_T)(void * p_sensor);
typedef int32_t(*MotorSensor_Speed_T)(void * p_sensor);  // + capture

typedef void(*MotorSensor_Proc_T)(void * p_sensor);
typedef bool(*MotorSensor_Test_T)(const void * p_sensor);
typedef int (*MotorSensor_Get_T)(const void * p_sensor);
typedef void(*MotorSensor_Set_T)(void * p_sensor, int value);

struct MotorSensor;
typedef void(*MotorSensor_Capture_T)(struct MotorSensor * p_sensor);
// typedef void(*MotorSensor_Capture_T)(struct MotorSensor_State * p_sensor);

typedef struct MotorSensor_VTable_T
{
    MotorSensor_Proc_T Init;
    MotorSensor_Test_T VerifyCalibration;
    MotorSensor_Angle_T PollStateAngle; /* periodic update */
    MotorSensor_Angle_T CaptureAngle;
    MotorSensor_Speed_T CaptureSpeed;
    // MotorSensor_Angle_T GetElectricalAngle;
    // MotorSensor_Angle_T GetMechanicalAngle;
    MotorSensor_Proc_T SetInitial;
    MotorSensor_Test_T IsFeedbackAvailable;
    MotorSensor_Get_T GetDirection;
    MotorSensor_Set_T SetDirection;
    MotorSensor_Proc_T ResetUnits;
}
MotorSensor_VTable_T;

typedef struct MotorSensor_State
{
    angle16_t MechanicalAngle;
    angle16_t ElectricalAngle;  /* Angle Feedback. Shared E-Cycle edge detect, User output */
    int32_t Speed_Fract16;      /* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */
}
MotorSensor_State_T;

typedef struct MotorSensor
{
    void * const p_Sensor;
    MotorSensor_VTable_T * const p_VTable;
    MotorSensor_State_T State;
}
MotorSensor_T;



// int32_t MotorSensor_CaptureAngle(MotorSensor_T * p_sensor)
// {
//     p_sensor->p_VTable->CaptureAngle(p_sensor->p_Sensor, &p_sensor->State); /* set */
// }

#endif
