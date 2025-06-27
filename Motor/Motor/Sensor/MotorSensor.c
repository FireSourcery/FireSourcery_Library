// sensor interface

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
    @file   MotorSensor.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "MotorSensor.h"

/******************************************************************************/
/*
    Empty Instance
*/
/******************************************************************************/
static void Empty_Init(const MotorSensor_T * p_sensor) { (void)p_sensor; }
static void Empty_CaptureAngle(const MotorSensor_T * p_sensor) { (void)p_sensor; }
static void Empty_CaptureSpeed(const MotorSensor_T * p_sensor) { (void)p_sensor; }
static void Empty_ResetUnits(const MotorSensor_T * p_sensor, const void * p_config) { (void)p_sensor; }
static bool Empty_IsFeedbackAvailable(const MotorSensor_T * p_sensor) { (void)p_sensor; return false; }
static bool Empty_VerifyCalibration(const MotorSensor_T * p_sensor) { (void)p_sensor; return false; }
static int Empty_GetDirection(const MotorSensor_T * p_sensor) { (void)p_sensor; return 0; }
static void Empty_SetDirection(const MotorSensor_T * p_sensor, int value) { (void)p_sensor; (void)value; }

const MotorSensor_VTable_T MOTOR_SENSOR_VTABLE_EMPTY =
{
    .INIT = (MotorSensor_Proc_T)Empty_Init,
    .CAPTURE_ANGLE = (MotorSensor_Proc_T)Empty_CaptureAngle,
    .CAPTURE_SPEED = (MotorSensor_Proc_T)Empty_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (MotorSensor_Test_T)Empty_IsFeedbackAvailable,
    .SET_DIRECTION = (MotorSensor_Set_T)Empty_SetDirection,
    .GET_DIRECTION = (MotorSensor_Get_T)Empty_GetDirection,
    // .SET_INITIAL = (MotorSensor_Proc_T)SetInitial,
    .INIT_UNITS_FROM = (MotorSensor_InitFrom_T)Empty_ResetUnits,
    .VERIFY_CALIBRATION = (MotorSensor_Test_T)Empty_VerifyCalibration,
};

