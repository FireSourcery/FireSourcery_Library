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
    @file   RotorSensor.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
/******************************************************************************/
#include "RotorSensor.h"

/******************************************************************************/
/*
    Empty Instance
*/
/******************************************************************************/
static void Empty_InitFrom(const RotorSensor_T * p_sensor, const void * p_config) { (void)p_sensor; }
static void Empty_Proc(const RotorSensor_T * p_sensor) { (void)p_sensor; }
static bool Empty_Test(const RotorSensor_T * p_sensor) { (void)p_sensor; return false; }
static int Empty_Get(const RotorSensor_T * p_sensor) { (void)p_sensor; return 0; }
static void Empty_Set(const RotorSensor_T * p_sensor, int value) { (void)p_sensor; (void)value; }

const RotorSensor_VTable_T MOTOR_SENSOR_VTABLE_EMPTY =
{
    .INIT = (RotorSensor_Proc_T)Empty_Proc,
    .CAPTURE_ANGLE = (RotorSensor_Proc_T)Empty_Proc,
    .CAPTURE_SPEED = (RotorSensor_Proc_T)Empty_Proc,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Empty_Test,
    .SET_DIRECTION = (RotorSensor_Set_T)Empty_Set,
    .ZERO_INITIAL = (RotorSensor_Proc_T)Empty_Proc,
    .INIT_UNITS_FROM = (RotorSensor_InitFrom_T)Empty_InitFrom,
    .VERIFY_CALIBRATION = (RotorSensor_Test_T)Empty_Test,
};
