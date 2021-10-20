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

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	#include "System/Critical/Critical.h"
#elif defined(ONFIG_STATE_MACHINE_MULTITHREADED_USER_DEFINED)
	extern inline void Critical_AquireMutex(critical_mutex_t);
	extern inline void Critical_ReleaseMutex(critical_mutex_t);
#endif

#include <stdint.h>
#include <stdbool.h>

//typedef uint8_t statemachine_transition_t;
//typedef uint8_t statemachine_selftransition_t;

//typedef void (* StateMachine_TransitionFunction_T)(volatile void * p_context);
//typedef void (* StateMachine_Function_T)(volatile void * p_context);
typedef void (* StateMachine_StateFunction_T)(volatile void * p_context);

struct State_Tag;

typedef const struct StateTransition_Tag
{
	const struct State_Tag * const  P_STATE;			/* next state */
	const StateMachine_StateFunction_T ON_TRANSITION; 	/* Process on transition */
}
StateMachine_Transition_T;

/*
 *	Array Implementation - 2D input table
 *  Map allocates for all possible transitions/inputs for each state, valid and invalid
 *  only space efficient when inputs are common across many states.
 *  can assign fault transition to invalid inputs
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
	StateMachine_Transition_T * P_TRANSITION_TABLE;

	/*
	 * Nontransition (Output only) Input Map - Mealy machine style outputs.
	 * Separate table for inputs without transitions, self transitions bypass check , less memory use, user must operate 2 input variables.
	 */
	StateMachine_StateFunction_T * P_OUTPUT_TABLE;
	StateMachine_StateFunction_T OUTPUT_COMMON;		/* Synchronous output / Common output to all inputs for asynchronous case */
	StateMachine_StateFunction_T ON_ENTRY;			/* common to all transition to current state, including self transition */
}
StateMachine_State_T;

//typedef struct StateMachine_Tag
//{
//	const StateMachine_State_T * const P_STATE_INITIAL;
//	/* Shared table length for all states */
//	const uint8_t TRANSITION_TABLE_LENGTH;		/* Total state count */
//	const uint8_t OUTPUT_TABLE_LENGTH;
//}
//StateMachine_Machine_T;

//typedef const struct StateMachine_Config_Tag
//{
//
//} StateMachine_Config_T;

typedef struct StateMachine_Tag
{
//	const StateMachine_Machine_T * const P_MACHINE;
	const StateMachine_State_T * const P_STATE_INITIAL;
	/* Shared table length for all states */
	const uint8_t TRANSITION_TABLE_LENGTH;		/* Total state count */
	const uint8_t OUTPUT_TABLE_LENGTH;

	volatile void * const P_FUNCTIONS_CONTEXT;		/* Share functions data for all state */

	const StateMachine_State_T * volatile p_StateActive; 	//uint8_t CurrentStateID

	/* for Synchronous Machine */
	volatile uint8_t InputTransition;
	volatile uint8_t InputOutput;
	volatile bool IsSetInputTransition;
	volatile bool IsSetInputOutput;

#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_MULTITHREADED_USER_DEFINED)
	const bool IS_MULTITHREADED;
	volatile critical_mutex_t Mutex;
#endif
}
StateMachine_T;

#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_MULTITHREADED_USER_DEFINED)
	#define STATE_MACHINE_CONFIG_MULTITHREADED(IsMultithreaded) .IS_MULTITHREADED = IsMultithreaded,
#else
	#define STATE_MACHINE_CONFIG_MULTITHREADED(IsMultithreaded)
#endif

#define STATE_MACHINE_CONFIG(p_StateInitial, TransitionTableLength, OutputTableLength, p_FunctionContext, IsMultithreaded)	\
{																		\
	.P_STATE_INITIAL			= p_StateInitial,						\
	.TRANSITION_TABLE_LENGTH	= TransitionTableLength,				\
	.OUTPUT_TABLE_LENGTH		= OutputTableLength,					\
	.P_FUNCTIONS_CONTEXT		= p_FunctionContext,					\
	STATE_MACHINE_CONFIG_MULTITHREADED(IsMultithreaded)					\
}

#endif
