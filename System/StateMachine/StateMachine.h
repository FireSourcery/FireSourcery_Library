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
//	extern inline void Critical_Enter(void);
//	extern inline void Critical_Exit(void);
#endif

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
//	#define STATE_MACHINE_RESERVED_INPUT_NO_OP (0xFFu)
//if multiple state machines
#elif defined (CONFIG_STATE_MACHINE_INPUT_UINT8)
//multiple state machines must use?
//	#define STATE_MACHINE_RESERVED_INPUT_NO_OP (0xFFu)
	typedef uint8_t StateMachine_Input_T;
#endif

typedef void (* StateMachine_TransitionFunction_T)(volatile void *userData);
typedef void (* StateMachine_StateFunction_T)(volatile void *userData);

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
	/*
	 * Parallel array
	 * 	Allows 1 share input var, can return state input
	 * 		 * Inputs with functions without state transition, bypass self transitions.
	 *
	 * Inputs with associated p_FunctionMap and pp_TransitionStateMap are transition functions
	 * Remaining functions, only associated with p_FunctionMap, are output only functions i.e. mealy style outputs, self transition functions
	 * SelfTransitionInputCount == 0 for moore case
	 */
	//	void (*(* const p_FunctionMap))(volatile void * userData);
	//	const struct State_Tag (* const (* const pp_TransitionStateMap));

	//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY //module memory allocation, all state machine must be same size
	//
	//	void (* const p_FunctionMap[STATE_MACHINE_TRANSITION_INPUT_COUNT + STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT])(void * userData);
	//	const struct State_Tag * const pp_TransitionStateMap[STATE_MACHINE_TRANSITION_INPUT_COUNT];
	//
	//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL)

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
	 * save memory without this way,   user operate 2 input variables.
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
	uint8_t TransitionInputCount;
	uint8_t SelfTransitionInputCount;
	//uint8_t TotalInputCount;
//#endif
	volatile void * volatile p_UserData;

	/* for Synchronous Machine */
	volatile StateMachine_Input_T InputTransition;
	volatile StateMachine_Input_T InputSelfTransition;
	volatile bool IsSetInputTransition;
	volatile bool IsSetInputSelfTransition;

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	   critical_mutex_t Mutex;
#endif
} StateMachine_T;


static inline void ProcStateMachineTransition(StateMachine_T * p_stateMachine, StateMachine_Input_T transitionInput)
{
	if (p_stateMachine->p_StateActive->pp_TransitionStateMap[transitionInput] != 0)
	{
		/* proc input function */
		if (p_stateMachine->p_StateActive->p_TransitionFunctionMap[transitionInput] != 0)
		{
			p_stateMachine->p_StateActive->p_TransitionFunctionMap[transitionInput](p_stateMachine->p_UserData);
		}

		/* map new state */
		p_stateMachine->p_StateActive = p_stateMachine->p_StateActive->pp_TransitionStateMap[transitionInput];

		/* proc new state entry function */
		if (p_stateMachine->p_StateActive->Entry != 0)
		{
			p_stateMachine->p_StateActive->Entry(p_stateMachine->p_UserData);
		}
	}
}

static inline void ProcStateMachineSelfTransition(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	//dont need critical section if no null pointers
	if (p_stateMachine->p_StateActive->p_TransitionFunctionMap[input] != 0)
	{
		p_stateMachine->p_StateActive->p_TransitionFunctionMap[input](p_stateMachine->p_UserData);
	}
}

static inline void ProcStateMachineOutput(StateMachine_T * p_stateMachine)
{
	if (p_stateMachine->p_StateActive->Output != 0)
	{
		p_stateMachine->p_StateActive->Output(p_stateMachine->p_UserData);
	}
}


/*
 * Semi synchronous
 * Synchronous state logic proc, asynchronous input proc
 */
static inline void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine)
{
	ProcStateMachineOutput(p_stateMachine);
}

static inline void StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
//#ifdef CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
//	#define STATE_MACHINE_TRANSITION_INPUT_COUNT (p_stateMachine->TransitionInputCount)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (p_stateMachine->TotalInputCount)
//#elif defined(CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY)
//	#define STATE_MACHINE_TOTAL_INPUT_COUNT (STATE_MACHINE_TRANSITION_INPUT_COUNT + STATE_MACHINE_SELF_TRANSITION_INPUT_COUNT)
//#endif

	if (input < p_stateMachine->TransitionInputCount)
	{
		if (Critical_MutexAquire(&p_stateMachine->Mutex))
		{
			ProcStateMachineTransition(p_stateMachine, input);
			Critical_MutexRelease(&p_stateMachine->Mutex);
		}
	}

}

static inline void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	if (input < p_stateMachine->SelfTransitionInputCount)
	{
		ProcStateMachineSelfTransition(p_stateMachine, input);
	}
}

/*
 * Asynchronous Process
 * Asynchronous Machine has no periodic output
 */
static inline void StateMachine_Asynchronous_ProcTransition(StateMachine_T *p_stateMachine, StateMachine_Input_T input)
{
	if (input < p_stateMachine->TransitionInputCount)
	{
		if (Critical_MutexAquire(&p_stateMachine->Mutex))
		{
			ProcStateMachineTransition(p_stateMachine, input);
			Critical_MutexRelease(&p_stateMachine->Mutex);
		}
	}
	ProcStateMachineOutput(p_stateMachine);
}

static inline void StateMachine_Asynchronous_ProcInput(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	if (input < p_stateMachine->SelfTransitionInputCount)
	{
		ProcStateMachineSelfTransition(p_stateMachine, input);
	}
	ProcStateMachineOutput(p_stateMachine);
}


/*
 * Synchronous Process
 * should be single threaded proc  to be Synchronous
 */
static inline void StateMachine_Synchronous_ProcMachine(StateMachine_T * p_stateMachine)
{
	if (p_stateMachine->IsSetInputTransition == true)
	{
		p_stateMachine->IsSetInputTransition = false;
		p_stateMachine->IsSetInputTransition = false;

		if (p_stateMachine->InputTransition < p_stateMachine->TransitionInputCount)
		{
			ProcStateMachineTransition(p_stateMachine, p_stateMachine->InputTransition);
		}
	}
	else if (p_stateMachine->IsSetInputSelfTransition == true)
	{
		p_stateMachine->IsSetInputTransition = false;

		if (p_stateMachine->InputSelfTransition < p_stateMachine->SelfTransitionInputCount)
		{
			ProcStateMachineSelfTransition(p_stateMachine, p_stateMachine->InputSelfTransition);
		}
	}

	ProcStateMachineOutput(p_stateMachine);
}

static inline void StateMachine_Synchronous_SetTransitionInput(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	p_stateMachine->InputTransition = input;
	p_stateMachine->IsSetInputTransition = true;
}

static inline void StateMachine_Synchronous_SetSelfTransitionInput(StateMachine_T * p_stateMachine, StateMachine_Input_T input)
{
	p_stateMachine->InputSelfTransition = input;
	p_stateMachine->IsSetInputSelfTransition = true;
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
