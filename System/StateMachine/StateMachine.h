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
//#elif defined(ONFIG_STATE_MACHINE_MULTITHREADED_USER_DEFINED)
//	extern inline void Critical_MutexAquire(critical_mutex_t);
//	extern inline void Critical_MutexRelease(critical_mutex_t);
#endif

#include <stdint.h>
#include <stdbool.h>

//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
//	#define STATE_MACHINE_TRANSITION_INPUT_COUNT (p_stateMachine->TransitionInputCount)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (p_stateMachine->TotalInputCount)
//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (STATE_MACHINE_TRANSITION_INPUT_COUNT + STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT)
//#endif

//typedef uint8_t statemachine_input_transition_t;
//typedef uint8_t statemachine_input_selftransition_t;

//typedef void (* StateMachine_TransitionFunction_T)(volatile void * p_typeData);
typedef void (* StateMachine_StateFunction_T)(volatile void * p_typeData);
//typedef void (* StateMachine_Function_T)(volatile void * p_typeData);

struct State_Tag;

/*
 * Array Implementation - 2D input table
 *  Map allocates for all possible transitions/inputs for each state, valid and invalid
 *  only space efficient when inputs are common across many states.
 *  can assign fault transition to invalid inputs
 *  States belonging to the same state machine must have same size maps
 *
 * List Implementation
 *	using list and check condition will have no wasted space
 *	must check all entries and end of list. user provide function to validate transition
 *
 * User define const states. No init function.
 */
typedef const struct State_Tag
{
	//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY //module memory allocation, all state machine must be same size
	//	const struct State_Tag * const pp_TransitionStateMap[STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT];
	//	void (* const p_TransitionFunctionMap[STATE_MACHINE_TRANSITION_INPUT_COUNT ])(void * userData);
	//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL)

	/*
	 * Parallel array
	 * 	Allows 1 share input var, can return state input
	 * 		 * Inputs with functions without state transition, bypass self transitions.
	 * Inputs with associated p_FunctionMap and pp_TransitionStateMap are transition function
	 */

	/*
	 * Input to Transition Map
	 */
//	struct StateTransition
//	{
	const struct State_Tag (*const (*const PP_INPUT_TRANSITION_STATE_MAP));
	void (*(*const P_INPUT_TRANSITION_FUNCTION_MAP))(volatile void * p_typeData);
//	};

	/*
	 * Separate table for inputs without transitions less memory use, user must operate 2 input variables.
	 *
	 * Input to Output Function Map - Mealy machine style outputs.
	 * Theoretical State Machine -> all out outputs without state change transition to self, without common entry function
	 * bypass self transitions,
	 */
	void (*(*const P_INPUT_OUTPUT_FUNCTION_MAP))(volatile void * p_typeData);
	void (*TRANSITION_ENTRY)(volatile void * p_typeData); 	//common to all transition to current state
	void (*OUTPUT)(volatile void * p_typeData); 				 //synchronous output, or common output to all inputs for asynchronous case
//	void (* const Exit)(void);
} State_T;

typedef struct StateMachine_Tag
{
	const State_T * p_StateInitial;
	const State_T * volatile p_StateActive; 	//uint8_t CurrentStateID?
//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
	uint8_t InputTransitionMapLength;		//state count
	uint8_t InputOutputMapLength;
	//uint8_t TotalInputCount;
//#endif
	volatile void * p_TypeData; //StateMachine Instance Type data

	/* for Synchronous Machine */
	volatile uint8_t InputTransition;
	volatile uint8_t InputOutput;
	volatile bool IsSetInputTransition;
	volatile bool IsSetInputOutput;

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	volatile critical_mutex_t Mutex;
#endif
} StateMachine_T;

#endif
