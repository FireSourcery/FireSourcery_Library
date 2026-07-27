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
#include "../StateMachine/Motor_StateMachine.h"

/*
    requires [Motor] StateMachine outside of Sensor Interface
*/

/******************************************************************************/
/*!

*/
/******************************************************************************/
// static inline RotorSensor_T * Sensor(Motor_T * p_motor) { return RotorSensor_Of(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->Config.SensorMode); }

// void Motor_Sensor_InitUnits(Motor_Context_T * p_motor)
// {
//     RotorSensor_Config_T config =
//     {
//         .PolePairs = p_motor->Config.SpeedRating.PolePairs,
//         .SpeedTypeMax_Angle16 = _Motor_GetSpeedTypeMax_Angle(&p_motor->Config.SpeedRating),
//         .SpeedTypeMax_Rpm = _Motor_GetSpeedTypeMax_Rpm(&p_motor->Config.SpeedRating),
//     };

//     RotorSensor_InitUnitsFrom(p_motor->p_ActiveSensor, &config);
// }

