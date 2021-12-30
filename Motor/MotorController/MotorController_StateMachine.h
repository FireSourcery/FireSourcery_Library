/******************************************************************************/
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
/******************************************************************************/
/******************************************************************************/
/*!
    @file 	MotorStateMachine.h
    @author FireSoucery
    @brief  MotorStateMachine
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_STATE_MACHINE_H
#define MOTOR_CONTROLLER_STATE_MACHINE_H

#include "Utility/StateMachine/StateMachine.h"

typedef enum MotorController_StateMachine_Input_Tag
{
//	MCSM_TRANSITION_INIT,
	MCSM_TRANSITION_STOP,
//	MCSM_TRANSITION_RUN,
	MCSM_TRANSITION_FAULT,

	MCSM_INPUT_DIRECTION,
	MCSM_INPUT_ACCELERATE,
	MCSM_INPUT_DECELERATE,
	MCSM_INPUT_FLOAT,
	MCSM_INPUT_CHECK_STOP,
	MCSM_INPUT_SAVE_PARAMS,
}
MotorController_StateMachine_Input_T;

extern const StateMachine_Machine_T MCSM_MACHINE;

#define MOTOR_CONTROLLER_STATE_MACHINE_CONFIG(p_MotorController) STATE_MACHINE_CONFIG(&MCSM_MACHINE, p_MotorController, false)

#endif
