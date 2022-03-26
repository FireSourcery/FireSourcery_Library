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
    @file 	MCStateMachine.c
    @author FireSoucery
    @brief  MCStateMachine
    @version V0
*/
/******************************************************************************/
#include "MotorController_StateMachine.h"
#include "MotorController.h"
#include "Utility/StateMachine/StateMachine.h"

static const StateMachine_State_T MCSM_STATE_INIT;
static const StateMachine_State_T MCSM_STATE_STOP;
static const StateMachine_State_T MCSM_STATE_RUN;
static const StateMachine_State_T MCSM_STATE_FAULT;

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
const StateMachine_Machine_T MCSM_MACHINE =
{
	.P_STATE_INITIAL 			= &MCSM_STATE_INIT,
	.TRANSITION_TABLE_LENGTH 	= MCSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(Motor_T * p_motor) 		{return &MCSM_STATE_FAULT;}

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
//	[MCSM_INPUT_TRANSITION_FAULT]		= (StateMachine_Transition_T)TransitionFault,
};

static void Init_Entry(MotorController_T * p_motorController)
{
//	MotorController_InitReboot(p_motorController);
}

static void Init_Proc(MotorController_T * p_motorController)
{
	StateMachine_ProcTransition(&p_motorController->StateMachine, &MCSM_STATE_STOP);
}

static const StateMachine_State_T MCSM_STATE_INIT =
{
	.P_TRANSITION_TABLE		= &INIT_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Init_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Init_Proc,
};

/******************************************************************************/
/*!
    @brief  Stop State

    Enters upon all motors reading 0 speed
*/
/******************************************************************************/
static StateMachine_State_T * Stop_InputThrottle(MotorController_T * p_motorController)
{
	return &MCSM_STATE_RUN;
}

static StateMachine_State_T * Stop_InputDirection(MotorController_T * p_motorController)
{
	return (MotorController_ProcDirection(p_motorController) == true) ? 0U : &MCSM_STATE_FAULT;
}

static StateMachine_State_T * Stop_InputSaveParams(MotorController_T * p_motorController)
{
	MotorController_SaveParameters_Blocking(p_motorController);
	return 0U;
}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_THROTTLE]  		= (StateMachine_Transition_T)Stop_InputThrottle,
	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Stop_InputDirection,
	[MCSM_INPUT_SAVE_PARAMS] 	= (StateMachine_Transition_T)Stop_InputSaveParams,

//	MCSM_INPUT_FAULT,
//	MCSM_INPUT_DIRECTION,
//	MCSM_INPUT_THROTTLE,
//	MCSM_INPUT_BRAKE,
//	MCSM_INPUT_FLOAT,
//	MCSM_INPUT_CHECK_STOP,
//	MCSM_INPUT_SAVE_PARAMS,
};

static void Stop_Entry(MotorController_T * p_motorController)
{
	MotorController_DisableMotorAll(p_motorController); //disable all motor may be in decel control mode
}

static void Stop_Proc(MotorController_T * p_motorController)
{

}

static const StateMachine_State_T MCSM_STATE_STOP =
{
	.P_TRANSITION_TABLE 	= &STOP_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Stop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Stop_Proc,
};

/******************************************************************************/
/*!
    @brief  State

    During Run State Motor Controller accepts speed inputs
    motors may be in active control or freewheel
*/
/******************************************************************************/
static StateMachine_State_T * Run_InputDirection(MotorController_T * p_motorController)
{
	if (p_motorController->MainDirection != p_motorController->UserDirection)
	{
		MotorController_Beep(p_motorController);
	}

	MotorController_ProcDirection(p_motorController); 	// if motor is in freewheel state, sets flag to remain in freewheel state
	return 0U;
}

static StateMachine_State_T * Run_InputThrottle(MotorController_T * p_motorController)
{
	MotorController_ProcUserCmdThrottle(p_motorController);
	return 0U;
}

static StateMachine_State_T * Run_InputBrake(MotorController_T * p_motorController)
{
	MotorController_ProcUserCmdBrake(p_motorController);
//	return (MotorController_CheckMotorAllStop(p_motorController) == true) ? &MCSM_STATE_STOP : 0U;
	return 0U;
}

static StateMachine_State_T * Run_InputCheckStop(MotorController_T * p_motorController)
{
	return (MotorController_CheckMotorAllStop(p_motorController) == true) ? &MCSM_STATE_STOP : 0U;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Run_InputDirection,
	[MCSM_INPUT_THROTTLE] 		= (StateMachine_Transition_T)Run_InputThrottle,
	[MCSM_INPUT_BRAKE]  		= (StateMachine_Transition_T)Run_InputBrake,
	[MCSM_INPUT_CHECK_STOP] 	= (StateMachine_Transition_T)Run_InputCheckStop,

//	[MCSM_INPUT_FLOAT] 	= (StateMachine_Transition_T)Run_InputFloat,
};

static void Run_Entry(MotorController_T * p_motorController)
{

}

static void Run_Proc(MotorController_T * p_motorController)
{

}

static const StateMachine_State_T MCSM_STATE_RUN =
{
	.P_TRANSITION_TABLE 	= &RUN_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Run_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Run_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
};

static void Fault_Entry(MotorController_T * p_motorController)
{
	MotorController_DisableMotorAll(p_motorController);
}

static void Fault_Proc(MotorController_T * p_motorController)
{
	MotorController_Beep(p_motorController);
	//if fault clears
	StateMachine_ProcTransition(&p_motorController->StateMachine, &MCSM_STATE_STOP);
}

static const StateMachine_State_T MCSM_STATE_FAULT =
{
	.P_TRANSITION_TABLE 	= &FAULT_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
