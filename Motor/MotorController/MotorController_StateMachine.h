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
	@file 	MotorController_StateMachine.h
	@author FireSoucery
	@brief  MotorController_StateMachine
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_STATE_MACHINE_H
#define MOTOR_CONTROLLER_STATE_MACHINE_H

#include "MotorController.h"
#include "Utility/StateMachine/StateMachine.h"

#define MCSM_TRANSITION_TABLE_LENGTH 	(10U)

typedef enum MotorController_StateMachine_Input_Tag
{
	MCSM_INPUT_FAULT,
	MCSM_INPUT_DIRECTION,
	MCSM_INPUT_THROTTLE,
	MCSM_INPUT_BRAKE,
	//	MCSM_INPUT_FLOAT, 
	MCSM_INPUT_RELEASE_THROTTLE, /* Release Control */
	MCSM_INPUT_RELEASE_BRAKE, /* Release Control */
	MCSM_INPUT_NULL,	/* No Input */ 
	MCSM_INPUT_NEUTRAL,
	MCSM_INPUT_SET_NEUTRAL,
	MCSM_INPUT_SAVE_PARAMS,
}
MotorController_StateMachine_Input_T;

typedef enum MotorController_StateMachine_StateId_Tag
{
	MCSM_STATE_ID_INIT,
	MCSM_STATE_ID_STOP,
	MCSM_STATE_ID_RUN,
	MCSM_STATE_ID_FAULT,
}
MotorController_StateMachine_StateId_T;

extern const StateMachine_Machine_T MCSM_MACHINE;

#define MOTOR_CONTROLLER_STATE_MACHINE_CONFIG(p_MotorController) STATE_MACHINE_CONFIG(&MCSM_MACHINE, p_MotorController, false)

#endif
