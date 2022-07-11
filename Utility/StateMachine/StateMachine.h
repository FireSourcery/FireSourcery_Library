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

struct StateMachine_State_Tag;
#define STATE_MACHINE_INPUT_NULL (0xFFU)
typedef uint8_t statemachine_input_t;	/* Input ID. Index into transition table. User may overwrite with enum */
typedef uint8_t statemachine_state_t;	/* State ID. User may overwrite with enum */

typedef void (*StateMachine_Output_T)(void * p_context); /* Synchronous State Output */

/*
	Input is passed via context
*/
typedef struct StateMachine_State_Tag * (* StateMachine_Transition_T)(void * p_context);

/*!
	Additional input passed as arguments

	@return pointer to new state. 0 for self transition skipping entry function
*/
typedef struct StateMachine_State_Tag * (* StateMachine_TransitionExt_T)(void * p_context, uint32_t inputExt);

/*!
	user implement case switch calls _StateMachine_ProcStateTransition
	@return user status
*/
typedef bool (*StateMachine_TransitionFunction_T)(void * p_context, statemachine_input_t inputId, uint32_t inputExt);

/*
	Array Implementation - 2D input table
	Map allocates for all possible transitions/inputs for each state, valid and invalid
	Allocates space for fault transition for invalid inputs
	Array index is input, eliminates search, only space efficient when inputs are common across many states.
	States belonging to the same state machine must have same size maps

	Pointer to array of functions that return a pointer to the next state (No null pointer check, user must supply empty table)
	Not accept input => define null pointer.
	Nontransition (Output only / Mealy machine style outputs), does not proc entry function => function return 0
	Selftransition, proc entry function => function return pointer to self
*/
typedef const struct StateMachine_State_Tag
{
	const statemachine_state_t ID;
	const StateMachine_Transition_T * const P_TRANSITION_TABLE;
	const StateMachine_TransitionExt_T * const P_TRANSITION_EXT_TABLE; 	/* extended inputs */
	const StateMachine_TransitionFunction_T TRANSITION_FUNCTION;		/* Single function, user provide switch case */
	const StateMachine_Output_T OUTPUT;		/* Synchronous output. Asynchronous case, proc on input only. No null pointer check, user must supply empty function */
	const StateMachine_Output_T ENTRY;		/* Common to all transition to current state, including self transition */
	const StateMachine_Output_T EXIT;
#ifdef CONFIG_STATE_MACHINE_MENU_ENABLE
	const struct StateMachine_State_Tag * P_NEXT_MENU;
	const struct StateMachine_State_Tag * P_PREV_MENU;
#endif
}
StateMachine_State_T;

typedef struct StateMachine_Machine_Tag
{
	const StateMachine_State_T * const P_STATE_INITIAL;
	const uint8_t TRANSITION_TABLE_LENGTH;			/* Total input count. Shared table length for all states, i.e. all states allocate for all inputs*/
}
StateMachine_Machine_T;

typedef const struct StateMachine_Config_Tag
{
	const StateMachine_Machine_T * const P_MACHINE; 	/* Const definition of state transition behaviors */
	void * const P_CONTEXT;								/* Mutable state information per state machine */
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
	const bool USE_CRITICAL;
#endif
}
StateMachine_Config_T;

typedef struct StateMachine_Tag
{
	const StateMachine_Config_T CONFIG;
	const StateMachine_State_T * p_StateActive;
	statemachine_input_t SyncInput;
	uint32_t SyncInputExt;
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
	volatile critical_mutex_t Mutex;
#endif
}
StateMachine_T;

#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
#define _STATE_MACHINE_INIT_CRITICAL(UseCritical) .USE_CRITICAL = UseCritical,
#else
#define _STATE_MACHINE_INIT_CRITICAL(UseCritical)
#endif

#define STATE_MACHINE_INIT(p_Machine, p_Context, UseCritical)	\
{																\
	.CONFIG = 													\
	{															\
		.P_MACHINE = p_Machine,									\
		.P_CONTEXT = p_Context,									\
		_STATE_MACHINE_INIT_CRITICAL(UseCritical)				\
	}															\
}

static inline statemachine_state_t StateMachine_GetActiveStateId(StateMachine_T * p_stateMachine) { return p_stateMachine->p_StateActive->ID; }

extern void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState);
extern void StateMachine_Init(StateMachine_T * p_stateMachine);
extern void StateMachine_Reset(StateMachine_T * p_stateMachine);
extern void StateMachine_Sync_Proc(StateMachine_T * p_stateMachine);
extern bool StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, uint8_t input);
extern bool StateMachine_Sync_SetInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt);
extern bool StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, uint8_t input);
extern bool StateMachine_Async_ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt);
extern void StateMachine_Semi_ProcOutput(StateMachine_T * p_stateMachine);
extern bool StateMachine_Semi_ProcInput(StateMachine_T * p_stateMachine, uint8_t input);
extern bool StateMachine_Semi_ProcInputExt(StateMachine_T * p_stateMachine, uint8_t input, uint32_t inputExt);
extern bool StateMachine_Semi_ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt);

#endif
