/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	MotorStateMachine.h
    @author FireSoucery
    @brief  MotorStateMachine
    @version V0
*/
/*******************************************************************************/
#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

//#define CONFIG_STATE_MACHINE_INPUT_ENUM_USER_DEFINED
//#define CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
//#define CONFIG_STATE_MACHINE_MULTITHREADED_OS_HAL
//#include "OS/StateMachine/StateMachine.h"



typedef enum StateMachine_Input_Tag
{
	MOTOR_TRANSISTION_CALIBRATION_COMPLETE,
	MOTOR_TRANSISTION_FOC_ALIGN,
	MOTOR_TRANSISTION_FOC_ALIGN_COMPLETE,

	MOTOR_TRANSISTION_FAULT,
	MOTOR_TRANSISTION_FAULT_CLEAR,

	STATE_INPUT_BUTTON_NEXT,
	STATE_INPUT_ID_A,
	BUTTON_FUNCTION_NEXT,
	BUTTON_FUNCTION_PREV,

	MOTOR_STATUS_NO_OP = 0xFFu,
	STATE_INPUT_RESERVED_NO_OP = 0xFFu,
} MotorStateMachine_Input_T;

#endif
