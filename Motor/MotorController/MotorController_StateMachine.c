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

#define MCSM_TRANSITION_TABLE_LENGTH 	(8U)

static const StateMachine_State_T MCSM_STATE_INIT;
static const StateMachine_State_T MCSM_STATE_STOP;
static const StateMachine_State_T MCSM_STATE_RUN;
static const StateMachine_State_T MCSM_STATE_FAULT;

/******************************************************************************/
/*!
    @brief Common Unconditional Transitions
*/
/******************************************************************************/
static StateMachine_State_T * TransitionInit(Motor_T * p_motor) 		{return &MCSM_STATE_INIT;}
static StateMachine_State_T * TransitionStop(Motor_T * p_motor) 		{return &MCSM_STATE_STOP;}
static StateMachine_State_T * TransitionRun(Motor_T * p_motor) 			{return &MCSM_STATE_RUN;}
static StateMachine_State_T * TransitionFault(Motor_T * p_motor) 		{return &MCSM_STATE_FAULT;}

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

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_TRANSITION_FAULT] 	= (StateMachine_Transition_T)TransitionFault,
	[MCSM_TRANSITION_STOP] 		= (StateMachine_Transition_T)TransitionStop,
};

static void Init_Entry(MotorController_T * p_motorController)
{
//	MotorController_InitReboot(p_motorController);
}

static void Init_Proc(MotorController_T * p_motorController)
{
	StateMachine_Semisynchronous_ProcInput(&p_motorController->StateMachine, MCSM_TRANSITION_STOP);
}

static const StateMachine_State_T MCSM_STATE_INIT =
{
	.P_TRANSITION_TABLE		= &INIT_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Init_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Init_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static StateMachine_State_T * Stop_InputAccelerate(MotorController_T * p_motorController)
{
	return &MCSM_STATE_RUN;
}

static StateMachine_State_T * Stop_InputDirection(MotorController_T * p_motorController)
{
	return (MotorController_ProcDirection(p_motorController) == true) ? 0U : &MCSM_STATE_FAULT;
}

static StateMachine_State_T * Stop_InputSaveParams(MotorController_T * p_motorController)
{
	MotorController_ProcSaveParameters_Blocking(p_motorController);
	return 0U;
}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_TRANSITION_FAULT]		= (StateMachine_Transition_T)TransitionFault,

	[MCSM_INPUT_ACCELERATE]  	= (StateMachine_Transition_T)Stop_InputAccelerate,
	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Stop_InputDirection,
	[MCSM_INPUT_SAVE_PARAMS] 	= (StateMachine_Transition_T)Stop_InputSaveParams,
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
*/
/******************************************************************************/
static StateMachine_State_T * Run_InputDirection(MotorController_T * p_motorController)
{
	MotorController_Beep(p_motorController);
	return 0U;
}

static StateMachine_State_T * Run_InputAccelerate(MotorController_T * p_motorController)
{
	MotorController_ProcUserCmdAccelerate(p_motorController);
	return 0U;
}

static StateMachine_State_T * Run_InputDecelerate(MotorController_T * p_motorController)
{
	MotorController_ProcUserCmdDecelerate(p_motorController);
	return (MotorController_CheckMotorAllStop(p_motorController) == true) ? &MCSM_STATE_STOP : 0U;
}

static StateMachine_State_T * Run_InputCheckStop(MotorController_T * p_motorController)
{
	return (MotorController_CheckMotorAllStop(p_motorController) == true) ? &MCSM_STATE_STOP : 0U;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_TRANSITION_FAULT] 	= (StateMachine_Transition_T)TransitionFault,

	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Run_InputDirection,
	[MCSM_INPUT_ACCELERATE]  	= (StateMachine_Transition_T)Run_InputAccelerate,
	[MCSM_INPUT_DECELERATE]  	= (StateMachine_Transition_T)Run_InputDecelerate,
	[MCSM_INPUT_CHECK_STOP] 	= (StateMachine_Transition_T)Run_InputCheckStop,
};

static void Run_Entry(MotorController_T * p_motorController)
{

}

static void Run_Proc(MotorController_T * p_motorController)
{

}

static const StateMachine_State_T MCSM_STATE_RUN =
{
	.P_TRANSITION_TABLE 		= &RUN_TRANSITION_TABLE[0U],
	.ON_ENTRY 					= (StateMachine_Output_T)Run_Entry,
	.OUTPUT 					= (StateMachine_Output_T)Run_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_TRANSITION_STOP] 		= (StateMachine_Transition_T)TransitionStop,
};

static void Fault_Entry(MotorController_T * p_motorController)
{
	MotorController_DisableMotorAll(p_motorController);
}

static void Fault_Proc(MotorController_T * p_motorController)
{
	//if fault clears	StateMachine_Semisynchronous_ProcTransition(&p_motorController->StateMachine, MCSM_TRANSITION_STOP);
}

static const StateMachine_State_T MCSM_STATE_FAULT =
{
	.P_TRANSITION_TABLE 	= &FAULT_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
