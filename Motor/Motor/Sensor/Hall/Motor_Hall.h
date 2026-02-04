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
    @file   Motor_Hall.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/


#if     defined(MOTOR_HALL_MODE_POLLING)
#elif   defined(MOTOR_HALL_MODE_ISR)
#else
#define MOTOR_HALL_MODE_POLLING
#endif


/* Part of Motor */
/* include for Calibration state */
typedef const struct Motor Motor_T;
typedef struct Motor_State Motor_State_T;

extern void Motor_Hall_Calibrate(const Motor_T * p_motor);
// extern void Motor_Hall_Cmd(const Motor_T * p_motor, int cmdId);

