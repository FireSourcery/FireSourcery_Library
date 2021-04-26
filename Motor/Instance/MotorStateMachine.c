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
    @file 	MotorStateMachine.c
    @author FireSoucery
    @brief  MotorStateMachine
    @version V0
*/
/*******************************************************************************/

#include "MotorStateMachine.h"

#include "OS/StateMachine/StateMachine.h"

#include "../Motor.h"
#include "../Motor_FOC.h"

#define MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES 			(10)
#define MOTOR_STATE_MACHINE_INPUT_SELF_TRANSITION_COUNT 	(2)
#define MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES 			(MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES + MOTOR_STATE_MACHINE_INPUT_SELF_TRANSITION_COUNT)

StateMachine_T MotorStateMachine;


//extern const State_T MOTOR_STATE_INIT;
//extern const State_T MOTOR_STATE_FAULT;
//
//extern const State_T MOTOR_STATE_FOC_INIT;
//extern const State_T MOTOR_STATE_FOC_CALIBRATE;
//extern const State_T MOTOR_STATE_FOC_ALIGN;
//extern const State_T MOTOR_STATE_FOC_RUN;
//
//void MotorState_FaultEntry(Motor_T * p_motor)
//{
//
//}
//
//void MotorState_FaultOutput(Motor_T * p_motor)
//{
//	StateMachine_Semisynchronous_ProcInput(&MotorStateMachine, MOTOR_TRANSISTION_FAULT);
//}
//
//void (* const MOTOR_STATE_INIT_FUNCTION_MAP[MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES])(void * p_motor) =
//{
//	[MOTOR_TRANSISTION_FAULT] = MotorState_FaultEntry,
//	[MOTOR_TRANSISTION_FOC_ALIGN] = MotorState_FaultEntry,
//	[MOTOR_TRANSISTION_FOC_ALIGN_COMPLETE] = MotorState_FaultEntry,
//};
//
//const State_T * const MOTOR_STATE_INIT_TRANSITION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_TRANSISTION_FAULT] 				= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSISTION_FOC_ALIGN] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSISTION_FOC_ALIGN_COMPLETE] 	= &MOTOR_STATE_FAULT,
//};
//
//const State_T MOTOR_STATE_FAULT =
//{
//	.p_FunctionMap =	MOTOR_STATE_FAULT_FUNCTION_MAP,
//	.pp_TransitionMap = MOTOR_STATE_FAULT_TRANSITION_MAP,
//	.Entry = MotorState_FaultEntry,
//	.Output = MotorState_FaultOutput,
//};
//
//
//void (* const MOTOR_STATE_INIT_FUNCTION_MAP[MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES])(void * p_motor) =
//{
//	[MOTOR_TRANSISTION_FAULT] = MotorState_FaultEntry,
//	[MOTOR_TRANSISTION_FOC_ALIGN] = MotorState_FaultEntry,
//	[MOTOR_TRANSISTION_FOC_ALIGN_COMPLETE] = MotorState_FaultEntry,
//};
//
//const State_T * const MOTOR_STATE_INIT_TRANSITION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_TRANSISTION_FAULT] 				= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSISTION_FOC_ALIGN] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSISTION_FOC_ALIGN_COMPLETE] 	= &MOTOR_STATE_FAULT,
//};
//
//const State_T MOTOR_STATE_INIT =
//{
//	.p_FunctionMap =	&MOTOR_STATE_INIT_FUNCTION_MAP,
//	.pp_TransitionMap = &MOTOR_STATE_INIT_TRANSITION_MAP,
//	.Entry = 0,
//	.Output = 0,
//};
//
//
//
//void MotorState_FocAlignEntry(Motor_T * p_motor)
//{
//	Motor_FOC_ActivateAlign(p_motor);
//}
//
//void MotorState_FocAlignState(Motor_T * p_motor)
//{
//	if (p_motor->Timer_ControlFreq <= 0)
//	{
//		StateMachine_Semisynchronous_ProcInput(&MotorStateMachine, MOTOR_TRANSISTION_FOC_ALIGN_COMPLETE);
//	}
//	else
//	{
//		p_motor->Timer_ControlFreq--;
//	}
//}
//
//void (* const MOTOR_STATE_FOC_ALIGN_FUNCTION_MAP[MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES])(void * p_motor) =
//{
//	[MOTOR_STATE_INPUT_FAULT] = MotorState_FaultEntry,
//	[MOTOR_STATE_INPUT_ALIGN_COMPLETE] = 0,
//};
//
//const State_T * const MOTOR_STATE_FOC_ALIGN_TRANSITION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_STATE_INPUT_FAULT] = &MOTOR_STATE_FAULT,
//};
//
//const State_T MOTOR_STATE_FOC_ALIGN =
//{
//	.p_FunctionMap =	MOTOR_STATE_FOC_RUN_FUNCTION_MAP,
//	.pp_TransitionMap = MOTOR_STATE_FOC_RUN_TRANSITION_MAP,
//	.Entry = MotorState_FocAlignEntry,
//	.Output = MotorState_FocAlignOutput,
//};
//
//
//void (* const MOTOR_STATE_FOC_ALIGN_FUNCTION_MAP[MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES])() 				= { [MOTOR_STATE_INPUT_FAULT] = 0, 	};
//const State_T * const MOTOR_STATE_FOC_RUN_TRANSITION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] 	= { [MOTOR_STATE_INPUT_FAULT] = &MOTOR_STATE_FAULT, 		};
//
//const State_T MOTOR_STATE_FOC_RUN =
//{
//	.p_FunctionMap =	MOTOR_STATE_FOC_RUN_FUNCTION_MAP,
//	.pp_TransitionMap = MOTOR_STATE_FOC_RUN_TRANSITION_MAP,
//	.Entry = MotorState_FocRunEntry,
//	.Loop = MotorState_FocRunLoop,
//};
//
//
//void MotorState_FocRunEntry(Motor_T * p_motor)
//{
//	Motor_FOC_StartRun( p_motor);
//}
//
//StateMachine_Input_T MotorState_FocRunLoop(Motor_T * p_motor)
//{
//	StateMachine_Input_T stateInput;
//
//	if(Motor_FOC_PollRun(p_motor))
//	{
//		stateInput = x;
//	}
//	else
//	{
//		stateInput = 0xFF;
//	}
//
//	return stateInput;
//}
//
//void (* const MOTOR_STATE_FOC_ALIGN_FUNCTION_MAP[MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES])() 				= { [MOTOR_STATE_INPUT_FAULT] = 0, 	};
//const State_T * const MOTOR_STATE_FOC_RUN_TRANSITION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] 	= { [MOTOR_STATE_INPUT_FAULT] = &MOTOR_STATE_FAULT, 		};
//
//const State_T MOTOR_STATE_FOC_RUN =
//{
//	.p_FunctionMap =	MOTOR_STATE_FOC_RUN_FUNCTION_MAP,
//	.pp_TransitionMap = MOTOR_STATE_FOC_RUN_TRANSITION_MAP,
//	.Entry = MotorState_FocRunEntry,
//	.Loop = MotorState_FocRunLoop,
//};
//
//
//void (* const P_MOTOR_STATE_FOC_ALIGN_FUNCTION_MAP)[MOTOR_STATE_MACHINE_FUNCTION_MAP_ENTRIES] 				=
//{
//	[MOTOR_STATE_TRANSISTION_INPUT_FAULT] = &TransitionToFault,
//};
//const State_T * const PP_MOTOR_STATE_FOC_ALIGN_TRANSITION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] 	=
//{
//	[MOTOR_STATE_TRANSISTION_INPUT_FAULT] = &MOTOR_STATE_FAULT,
//};
//
//const State_T MOTOR_STATE_FOC_ALIGN =
//{
//	.p_FunctionMap =	P_MOTOR_STATE_FOC_ALIGN_FUNCTION_MAP,
//	.pp_TransitionMap = PP_MOTOR_STATE_FOC_ALIGN_TRANSITION_MAP,
//	.Entry = Motor_FocStartAlign,
//	.Loop = Motor_FocAlign,
//};
//
//
//void MotorStateMachine_Init(Motor_T * p_motor)
//{
//	StateMachine_Init
//	(
//		&(p_motor->StateMachine),
//		&MOTOR_STATE_INIT,
//		MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES,
//		MOTOR_STATE_MACHINE_INPUT_SELF_TRANSITION_COUNT,
//		p_motor
//	);
//}

