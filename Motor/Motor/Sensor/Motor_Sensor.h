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
    @brief
*/
/******************************************************************************/

/*
    Include all sensor Calibration
*/
// #include "Hall/Hall_Sensor.h"
#include "Hall/Motor_Hall.h"

// #include "Encoder/Encoder_Sensor.h"
#include "Encoder/Motor_Encoder.h"

#include "RotorSensor_Table.h"
// #include "RotorSensor.h"

/* Part of Motor */
/* non inline, include by Motor */
typedef const struct Motor Motor_T;
typedef struct Motor_State Motor_State_T;

/******************************************************************************/
/*!
    Id Access
*/
/******************************************************************************/
void Motor_Sensor_CalibrationCmd_Call(const Motor_T * p_motor, RotorSensor_Id_T varId, int varValue);


