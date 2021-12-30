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
typedef struct StateMachine_State_Tag *  (* StateMachine_Transition_T)(void * p_context);
typedef void 							 (* StateMachine_Output_T)(void * p_context);
typedef uint8_t statemachine_input_t;	/* User may overwrite with enum */

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
	 * Pointer to array of functions that return a pointer to the next state
	 * No null pointer check, user must supply empty table
	 *
	 * Nontransition (Output only) Input Map - Mealy machine style outputs return 0
	 */
	const StateMachine_Transition_T * const P_TRANSITION_TABLE;

	const StateMachine_Output_T OUTPUT;		/* Synchronous output / Common output to all inputs for asynchronous case. No null pointer check, user must supply empty function */
	const StateMachine_Output_T ON_ENTRY;	/* common to all transition to current state, including self transition */
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
#if defined(CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	const bool USE_CRITICAL;
#endif
}
StateMachine_Config_T;

typedef struct StateMachine_Tag
{
	const StateMachine_Config_T CONFIG;
	const StateMachine_State_T * p_StateActive;
	uint8_t Input; 	/* for Synchronous Machine only */

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
extern void StateMachine_Asynchronous_ProcInput(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, uint8_t input);
extern void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine);

#endif
