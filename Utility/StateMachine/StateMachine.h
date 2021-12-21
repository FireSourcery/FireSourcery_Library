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
	@file 	StateMachine.c
	@author FireSoucery
	@brief 	StateMachine module
	@version V0
*/
/******************************************************************************/
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t statemachine_input_t;
typedef uint8_t statemachine_tinput_t;
typedef uint8_t statemachine_oinput_t;
typedef void (* StateMachine_Output_T)(void * p_context);

struct StateMachine_State_Tag;

typedef const struct StateTransition_Tag
{
	const struct StateMachine_State_Tag * const P_STATE;	/* next state */
	const StateMachine_Output_T ON_TRANSITION; 	/* Process on transition */
}
StateMachine_Transition_T;

/*
 *	Array Implementation - 2D input table
 *  Map allocates for all possible transitions/inputs for each state, valid and invalid
 * 	Allocates space for fault transition for invalid inputs
 *  Array index is input, eliminates search, only space efficient when inputs are common across many states.
 *  States belonging to the same state machine must have same size maps
 *  User define const states at compile time.
 */
typedef const struct StateMachine_State_Tag
{
	/*
	 * Transition Input Map
	 * Inputs with without state transition, self transitions.
	 * Inputs with associated p_FunctionMap and pp_TransitionStateMap are transition function
	 */
	const StateMachine_Transition_T * const P_TRANSITION_TABLE;

	/*
	 * Nontransition (Output only) Input Map - Mealy machine style outputs.
	 * Separate table for inputs without transitions, self transitions bypass check , less memory use, user must operate 2 input variables.
	 */
	const StateMachine_Output_T * const P_OUTPUT_TABLE;
	const StateMachine_Output_T OUTPUT_SYNCHRONOUS;		/* Synchronous output / Common output to all inputs for asynchronous case */
	const StateMachine_Output_T ON_ENTRY;				/* common to all transition to current state, including self transition */
}
StateMachine_State_T;

typedef struct StateMachine_Machine_Tag
{
	const StateMachine_State_T * const P_STATE_INITIAL;
	/* Shared table length for all states */
	const uint8_t TRANSITION_TABLE_LENGTH;		/* Total state count */
	const uint8_t OUTPUT_TABLE_LENGTH;
}
StateMachine_Machine_T;

typedef const struct StateMachine_Config_Tag
{
	const StateMachine_Machine_T * const P_MACHINE;
	void * const P_CONTEXT;		/* Share functions data for all state */
#if defined(CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	const bool USE_CRITICAL;
#endif
}
StateMachine_Config_T;

typedef struct StateMachine_Tag
{
	const StateMachine_Config_T CONFIG;
	const StateMachine_State_T * p_StateActive; 	//uint8_t CurrentStateID

	/* for Synchronous Machine */
	uint8_t InputTransition;
	uint8_t InputOutput;
	bool IsSetInputTransition;
	bool IsSetInputOutput;

#if defined(CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	volatile critical_mutex_t Mutex;
#endif
}
StateMachine_T;

#if defined(CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	#define STATE_MACHINE_CONFIG_CRITICAL(IsMultithreaded) .USE_CRITICAL = IsMultithreaded,
#else
	#define STATE_MACHINE_CONFIG_CRITICAL(IsMultithreaded)
#endif

#define STATE_MACHINE_CONFIG(p_Machine, p_Context, IsMultithreaded)	\
{															\
	.CONFIG = 												\
	{														\
		.P_MACHINE = p_Machine,								\
		.P_CONTEXT = p_Context,								\
		STATE_MACHINE_CONFIG_CRITICAL(IsMultithreaded)		\
	}														\
}


extern void StateMachine_Init(StateMachine_T * p_stateMachine);
extern void StateMachine_Reset(StateMachine_T * p_stateMachine);
extern void StateMachine_Synchronous_Proc(StateMachine_T * p_stateMachine);
extern void StateMachine_Synchronous_SetTransition(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Synchronous_SetOutput(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Asynchronous_ProcTransition(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Asynchronous_ProcOutput(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Semisynchronous_ProcOutput(StateMachine_T * p_stateMachine, uint8_t input);

#endif
