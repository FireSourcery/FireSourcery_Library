/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	MotAnalogUser_Motor.h
    @author FireSoucery
    @brief  1 instance for all motor, input output control
    @version V0
*/
/**************************************************************************/
#ifndef MOT_ANALOG_USER_MOTOR_H
#define MOT_ANALOG_USER_MOTOR_H

#include "Motor/Motor/Motor.h"

void MotAnalogUser_Motor_Write(const MotAnalogUser_T * input, Motor_T * p_motorDest, uint8_t motorCount);

#endif
