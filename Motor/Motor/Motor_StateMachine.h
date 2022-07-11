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
#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

#include "Utility/StateMachine/StateMachine.h"

#define MSM_TRANSITION_TABLE_LENGTH 	(11U)

typedef enum MotorStateMachine_Input_Tag
{
	//	MSM_TRANSITION_INIT,
	//	MSM_TRANSITION_STOP,
	//	MSM_TRANSITION_ALIGN,
	//	MSM_TRANSITION_OPEN_LOOP,
	//	MSM_TRANSITION_RUN,
	//	MSM_TRANSITION_FREEWHEEL,
	//	MSM_TRANSITION_FAULT,
	MSM_INPUT_FAULT,  		/* Toggle Fault */
	MSM_INPUT_CONTROL, 		/* FeedbackMode update, transitions to Run State (Active Control) */
	MSM_INPUT_FLOAT, 		/* Release Control */
	// MSM_INPUT_GROUND,
	MSM_INPUT_DIRECTION,
	MSM_INPUT_CALIBRATION,
}
MotorStateMachine_Input_T;

typedef enum Motor_StateMachine_StateId_Tag
{
	MSM_STATE_ID_INIT,
	MSM_STATE_ID_STOP,
	MSM_STATE_ID_ALIGN,
	MSM_STATE_ID_OPEN_LOOP,
	MSM_STATE_ID_RUN,
	MSM_STATE_ID_FREEWHEEL,
	MSM_STATE_ID_CALIBRATION,
	MSM_STATE_ID_FAULT,
}
Motor_StateMachine_StateId_T;

extern const StateMachine_Machine_T MSM_MACHINE;

#define MOTOR_STATE_MACHINE_INIT(p_Motor) STATE_MACHINE_INIT(&MSM_MACHINE, p_Motor, false)

#endif
