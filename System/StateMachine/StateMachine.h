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

typedef void (* StateMachine_TransitionFunction_T)(volatile void * p_userData);
typedef void (* StateMachine_StateFunction_T)(volatile void * p_userData); //SelfTransition

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
		const struct State_Tag (*const (*const pp_TransitionStateMap));
		void (*(*const p_TransitionFunctionMap))(volatile void *userData);
//	};

	/* Input to Output Function Map - Mealy machine style outputs.
	 * Theoretical State Machine -> all out outputs without state change transition to self.
	 * save memory without this way,   user must operate 2 input variables.
	 */
	void (*(*const p_SelfTransitionFunctionMap))(volatile void *userData);

	void (* Entry)(volatile void * userData); 	//common to all transition to current state, factored segment
	void (* Output)(volatile void * userData);  //common output to all state input self transition, factored out segment
	//	void (* const Exit)(void); //should exit function be state feedback input
} State_T;

typedef struct StateMachine_Tag
{
	const State_T * p_StateInitial;
	const State_T * volatile p_StateActive; 	//uint8_t CurrentStateID?
//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
	uint8_t TransitionInputCount;		//state count
	uint8_t SelfTransitionInputCount;
	//uint8_t TotalInputCount;
//#endif
	volatile void * volatile p_UserData;

	/* for Synchronous Machine */
	volatile uint8_t InputTransition;
	volatile uint8_t InputSelfTransition;
	volatile bool IsSetInputTransition;
	volatile bool IsSetInputSelfTransition;

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	volatile critical_mutex_t Mutex;
#endif
} StateMachine_T;




#endif
