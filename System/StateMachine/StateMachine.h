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

/*
 * Inputs shared between all instances of state machine. boundary checking wont work, must be per statemchaines input, or enum must overlap
 */
#ifdef CONFIG_STATE_MACHINE_INPUT_ENUM_USER_DEFINED
/*
 * User provide typedef enum StateMachine_Input_Tag { ..., STATE_INPUT_RESERVED_NO_OP = 0xFFu } StateMachine_Input_T
 * #include defs before including State.h
 */
	#define STATE_MACHINE_RESERVED_INPUT_NO_OP (0xFFu)
//if multiple state machines
#elif defined (CONFIG_STATE_MACHINE_INPUT_UINT8)
//multiple state machines must use?
	#define STATE_MACHINE_RESERVED_INPUT_NO_OP (0xFFu)
	typedef uint8_t StateMachine_Input_T;
#endif


#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_OS_HAL
	#include "System/Critical/Critical.h"
#elif defined(ONFIG_STATE_MACHINE_MULTITHREADED_USER_DEFINED)
	extern inline void Critical_Enter(void){}
	extern inline void Critical_Exit(void){}
#endif

//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
//	#define STATE_MACHINE_TRANSITION_INPUT_COUNT (p_stateMachine->TransitionInputCount)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (p_stateMachine->TotalInputCount)
//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (STATE_MACHINE_TRANSITION_INPUT_COUNT + STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT)
//#endif

struct State_Tag;

/*
 * Array Implementation - 2D input table
 * Map allocates for all possible transitions/inputs for each state, valid and invalid
 * only space efficient when inputs are common across many states.
 * can assign fault transition to invalid inputs
 *	//States belongin to the same state machine must have same size maps
 * List Implementation
 *	using list and check condition will have no wasted space
 *	must check all entries and end of list. user provide function to validate transition
 *
 * User define const states. No init function.
 */
typedef const struct State_Tag
{
	/*
	 * Input to Transition Map
	 * 	 struct StateTransition
	 *	 {
	 *		 const struct State * const p_TargetState;
	 *	  	 void (* const OnTransition)(void * userData);
	 *	 } InputTransitionMap[StateMachine_Input_TRANSITION_COUNT];
	 *
	 * Input to Output Function Map - Mealy machine style outputs.
	 *   void (* const InputOutputMap[STATE_MACHINE_INPUT_OUTPUT_COUNT])(void * userData);
	 *
	 * Cannot determine return type is of which input type, non transition or transition, when using 2 tables
	 */

	/*
	 * Parallel array
	 * 	Allows 1 share input var, can return state input
	 * 		 * Inputs with functions without state transition, bypass self transitions.
	 *
	 * Inputs with associated p_FunctionMap and pp_TransitionMap are transition functions
	 * Remaining functions, only associated with p_FunctionMap, are output only functions i.e. mealy style outputs, self transition functions
	 * SelfTransitionInputCount == 0 for moore case
	 */

	void (*(* const p_FunctionMap))(volatile void * userData);
	const struct State_Tag (* const (* const pp_TransitionMap));

//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY
//
//	void (* const p_FunctionMap[STATE_MACHINE_TRANSITION_INPUT_COUNT + STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT])(void * userData);
//	const struct State_Tag * const pp_TransitionMap[STATE_MACHINE_TRANSITION_INPUT_COUNT];
//
//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL)

	void (* Entry)(volatile void * userData); //common to all transition to current state, factored segment
	void (* Output)(volatile void * userData);  //common output to all state input self transition, factored out segment
	//	void (* const Exit)(void); //todo?	//should exit function be state feedback input?
} State_T;

typedef struct StateMachine_Tag
{
	const State_T * p_StateInitial;
	const State_T * p_StateActive; 	//uint8_t CurrentStateID?
//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
	uint8_t TransitionInputCount;	//	uint8_t SelfTransitionInputCount;
	uint8_t TotalInputCount;
//#endif
	volatile void * p_UserData;
	volatile StateMachine_Input_T Input; /* for synchronous transition */
} StateMachine_T;


static inline void ProcTransition(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	Critical_Enter();
	if (p_stateMachine->p_StateActive->pp_TransitionMap[input] != 0)
	{
		/* proc input function */
		if (p_stateMachine->p_StateActive->p_FunctionMap[input] != 0)
		{
			p_stateMachine->p_StateActive->p_FunctionMap[input](p_stateMachine->p_UserData);
		}

		/* map new state */
		p_stateMachine->p_StateActive = p_stateMachine->p_StateActive->pp_TransitionMap[input];

		if (p_stateMachine->p_StateActive->Entry != 0)
		{
			p_stateMachine->p_StateActive->Entry(p_stateMachine->p_UserData);
		}
	}

	// if synchronous machine 	p_stateMachine->Input = reserved;
	Critical_Exit();
}

static inline void ProcSelfTransition(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
//	Critical_Enter();
	//need to block transition during processing
	if (p_stateMachine->p_StateActive->p_FunctionMap[input] != 0)
	{
		p_stateMachine->p_StateActive->p_FunctionMap[input](p_stateMachine->p_UserData);
	}
	if (p_stateMachine->p_StateActive->Output != 0)
	{
		p_stateMachine->p_StateActive->Output(p_stateMachine->p_UserData);
	}
//	Critical_Exit();
}

static inline void ProcFunctionMap(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
//	Critical_Enter();
	if (p_stateMachine->p_StateActive->p_FunctionMap[input] != 0)
	{
		p_stateMachine->p_StateActive->p_FunctionMap[input](p_stateMachine->p_UserData);
	}
//	Critical_Exit();
}

static inline void ProcOutput(StateMachine_T * p_stateMachine)
{
//	Critical_Enter();
	if (p_stateMachine->p_StateActive->Output != 0)
	{
		p_stateMachine->p_StateActive->Output(p_stateMachine->p_UserData);
	}
//	Critical_Exit();
}


/*
 * Semi synchronous
 * Synchronous state logic proc, asynchronous input proc
 */
static inline void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine)
{
	ProcOutput(p_stateMachine);
}

static inline void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
//	#define STATE_MACHINE_TRANSITION_INPUT_COUNT (p_stateMachine->TransitionInputCount)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (p_stateMachine->TotalInputCount)
//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (STATE_MACHINE_TRANSITION_INPUT_COUNT + STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT)
//#endif

	if (input < p_stateMachine->TransitionInputCount)
	{
		ProcTransition(p_stateMachine, input);
	}
	else if (input < p_stateMachine->TotalInputCount)
	{
		ProcFunctionMap(p_stateMachine, input);
	}
}

/*
 * Asynchronous Process
 * Asynchronous Machine has no periodic output
 */
static inline void StateMachine_Asynchronous_ProcMachine(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
//	StateMachine_Asynchronous_ProcInput();
	if (input < p_stateMachine->TransitionInputCount)
	{
		ProcTransition(p_stateMachine, input);
	}
	else if (input < p_stateMachine->TotalInputCount)
	{
		ProcSelfTransition(p_stateMachine, input);
	}
}


/*
 * Synchronous Process
 * should be single threaded proc
 * supports timer counter
 * Map decides if current state reacts to input
 */
static inline void StateMachine_Synchronous_ProcMachine(StateMachine_T * p_stateMachine)
{
	StateMachine_Input_T input;

	/* eliminate chance of missing next input */
	Critical_Enter();

	input = p_stateMachine->Input;
	p_stateMachine->Input = STATE_MACHINE_RESERVED_INPUT_NO_OP;

	Critical_Exit();

	if (input < p_stateMachine->TransitionInputCount)
	{
		ProcTransition(p_stateMachine, input);
	}
	else if (input < p_stateMachine->TotalInputCount)
	{
		ProcSelfTransition(p_stateMachine, input);
	}
	else
	{
		ProcOutput(p_stateMachine);
	}

}

static inline void StateMachine_Synchronous_SetInput(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	p_stateMachine->Input = input;
}

//static inline void State_InputAccumulated(StateMachine_t * stateMachine, StateInput_t input)
//{
//	for (uint8_t bitIdx = 0; bitIdx < STATE_INPUT_OUTPUT_COUNT; bitIdx++)
//	{
//		if (((input >> bitIdx) & (uint32_t)0x01) && stateMachine->p_StateActive->InputOutputMap[bitIdx] =! 0) stateMachine->p_StateActive->InputOutputMap[bitIdx]();
//	}
//}
//void State_SetInputAccumulated(StateMachine_t * stateMachine, uint8_t input)
//{
//	stateMachine->InputOutput |= ((uint32_t)0x01 << input);
//}

#endif
