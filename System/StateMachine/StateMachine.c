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
	@brief 	StateMachine module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "StateMachine.h"

#include <stdint.h>

/*
 * States const strut should be compile time def
 */

void StateMachine_Init
(
	StateMachine_T * p_stateMachine,
	const State_T * p_stateInitial,
	uint8_t transitionInputCount,
	uint8_t selfTransitionInputCount,
	const void * p_userData
)
{
	p_stateMachine->p_StateInitial 				= p_stateInitial;
	p_stateMachine->p_StateActive 				= p_stateInitial;
	p_stateMachine->InputTransitionMapLength 	= transitionInputCount;
	p_stateMachine->InputOutputMapLength 		= selfTransitionInputCount;
	p_stateMachine->p_TypeData 					= p_userData;

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	p_stateMachine->Mutex = 1U;
#endif
}

static inline void ProcTransition(StateMachine_T * p_stateMachine, uint8_t transitionInput)
{
	/* check new state exists*/
	if (p_stateMachine->p_StateActive->PP_INPUT_TRANSITION_STATE_MAP != 0)
	{
		if (p_stateMachine->p_StateActive->PP_INPUT_TRANSITION_STATE_MAP[transitionInput] != 0)
		{

			/* proc input function */
			if (p_stateMachine->p_StateActive->P_INPUT_TRANSITION_FUNCTION_MAP != 0)
			{
				if (p_stateMachine->p_StateActive->P_INPUT_TRANSITION_FUNCTION_MAP[transitionInput] != 0)
				{
					p_stateMachine->p_StateActive->P_INPUT_TRANSITION_FUNCTION_MAP[transitionInput](p_stateMachine->p_TypeData);
				}
			}

			/* map new state */
			p_stateMachine->p_StateActive = p_stateMachine->p_StateActive->PP_INPUT_TRANSITION_STATE_MAP[transitionInput];

			/* proc new state common entry function */
			if (p_stateMachine->p_StateActive->TRANSITION_ENTRY != 0)
			{
				p_stateMachine->p_StateActive->TRANSITION_ENTRY(p_stateMachine->p_TypeData);
			}

		}
	}
}

/*
 * Input Output, bypass entry
 */
static inline void ProcInputOuput(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (p_stateMachine->p_StateActive->P_INPUT_OUTPUT_FUNCTION_MAP != 0)
	{
		if (p_stateMachine->p_StateActive->P_INPUT_OUTPUT_FUNCTION_MAP[input] != 0)
		{
			p_stateMachine->p_StateActive->P_INPUT_OUTPUT_FUNCTION_MAP[input](p_stateMachine->p_TypeData);
		}
	}
}

static inline void ProcOutput(StateMachine_T * p_stateMachine)
{
	if (p_stateMachine->p_StateActive->OUTPUT != 0)
	{
		p_stateMachine->p_StateActive->OUTPUT(p_stateMachine->p_TypeData);
	}
}


/*
 * Synchronous Process
 * always single threaded proc
 *
 * synchronous machine proc last set input
 */
void StateMachine_Synchronous_ProcMachine(StateMachine_T * p_stateMachine)
{
	if (p_stateMachine->IsSetInputTransition == true)
	{
		p_stateMachine->IsSetInputTransition = false;

		if (p_stateMachine->InputTransition < p_stateMachine->InputTransitionMapLength)
		{
			ProcTransition(p_stateMachine, p_stateMachine->InputTransition);
		}
	}
	else if (p_stateMachine->IsSetInputOutput == true)
	{
		p_stateMachine->IsSetInputTransition = false;

		if (p_stateMachine->InputOutput < p_stateMachine->InputOutputMapLength)
		{
			ProcInputOuput(p_stateMachine, p_stateMachine->InputOutput);
		}
	}

	ProcOutput(p_stateMachine);
}

//void StateMachine_Synchronous_SetInput(StateMachine_T * p_stateMachine, uint8_t input)
//{
//	p_stateMachine->InputTransition 		= input;
//	p_stateMachine->IsSetInputTransition 	= true;
//}

void StateMachine_Synchronous_SetInputTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	p_stateMachine->InputTransition 		= input;
	p_stateMachine->IsSetInputTransition 	= true;
	p_stateMachine->IsSetInputOutput 		= false;
}

void StateMachine_Synchronous_SetInputOutput(StateMachine_T * p_stateMachine, uint8_t input)
{
	p_stateMachine->InputOutput 			= input;
	p_stateMachine->IsSetInputOutput 		= true;
	p_stateMachine->IsSetInputTransition 	= false;
}


/*
 * Asynchronous Process
 * Asynchronous Machine has no periodic output
 */

static inline void ProcAsynchronousInputTransition(StateMachine_T *p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->InputTransitionMapLength)
	{
#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
		if (Critical_MutexAquire(&p_stateMachine->Mutex))
#endif
		{
			ProcTransition(p_stateMachine, input);
#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
			Critical_MutexRelease(&p_stateMachine->Mutex);
#endif
		}
	}
}

static inline void ProcAsynchronousInputOuput(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->InputOutputMapLength)
	{
		ProcInputOuput(p_stateMachine, input);
	}
}

void StateMachine_Asynchronous_ProcTransition(StateMachine_T *p_stateMachine, uint8_t input)
{
	ProcAsynchronousInputTransition(p_stateMachine, input);
	ProcOutput(p_stateMachine);
}

void StateMachine_Asynchronous_ProcInput(StateMachine_T * p_stateMachine, uint8_t input)
{
	ProcAsynchronousInputOuput(p_stateMachine, input);
	ProcOutput(p_stateMachine);
}

/*
 * Semi synchronous
 * Synchronous state logic proc, asynchronous input proc
 *
 * State Ouput
 */
void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine)
{
	ProcOutput(p_stateMachine);
}

void StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	ProcAsynchronousInputTransition(p_stateMachine, input);
}

/*
 * Input Output
 */
void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, uint8_t input)
{
	ProcAsynchronousInputOuput(p_stateMachine, input);
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

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
	if (Critical_MutexAquire(&p_stateMachine->Mutex))
	{
		p_stateMachine->p_StateActive 			= p_stateMachine->p_StateInitial;
		Critical_MutexRelease(&p_stateMachine->Mutex);
	}
}

