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
	p_stateMachine->TransitionInputCount 		= transitionInputCount;
	p_stateMachine->SelfTransitionInputCount 	= selfTransitionInputCount;
	p_stateMachine->p_UserData 					= p_userData;

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	p_stateMachine->Mutex = 1U;
#endif
}

static inline void ProcStateMachineTransition(StateMachine_T * p_stateMachine, uint8_t transitionInput)
{
	/* check new state exists*/
	if (p_stateMachine->p_StateActive->pp_TransitionStateMap != 0)
	{
		if (p_stateMachine->p_StateActive->pp_TransitionStateMap[transitionInput] != 0)
		{

			/* proc input function */
			if (p_stateMachine->p_StateActive->p_TransitionFunctionMap != 0)
			{
				if (p_stateMachine->p_StateActive->p_TransitionFunctionMap[transitionInput] != 0)
				{
					p_stateMachine->p_StateActive->p_TransitionFunctionMap[transitionInput](p_stateMachine->p_UserData);
				}
			}

			/* map new state */
			p_stateMachine->p_StateActive = p_stateMachine->p_StateActive->pp_TransitionStateMap[transitionInput];

			/* proc new state common entry function */
			if (p_stateMachine->p_StateActive->Entry != 0)
			{
				p_stateMachine->p_StateActive->Entry(p_stateMachine->p_UserData);
			}

		}
	}
}

static inline void ProcStateMachineSelfTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (p_stateMachine->p_StateActive->p_SelfTransitionFunctionMap != 0)
	{
		if (p_stateMachine->p_StateActive->p_SelfTransitionFunctionMap[input] != 0)
		{
			p_stateMachine->p_StateActive->p_SelfTransitionFunctionMap[input](p_stateMachine->p_UserData);
		}
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
 *
 * State Ouput
 */
void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine)
{
	ProcStateMachineOutput(p_stateMachine);
}

void StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->TransitionInputCount)
	{
#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
		if (Critical_MutexAquire(&p_stateMachine->Mutex))
#endif
		{
			ProcStateMachineTransition(p_stateMachine, input);
#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
			Critical_MutexRelease(&p_stateMachine->Mutex);
#endif
		}
	}
}

/*
 * Input Output
 */
void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, uint8_t input)
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
void StateMachine_Asynchronous_ProcTransition(StateMachine_T *p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->TransitionInputCount)
	{
#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
		if (Critical_MutexAquire(&p_stateMachine->Mutex))
#endif
		{
			ProcStateMachineTransition(p_stateMachine, input);
#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
			Critical_MutexRelease(&p_stateMachine->Mutex);
#endif
		}
	}
	ProcStateMachineOutput(p_stateMachine);
}

void StateMachine_Asynchronous_ProcInput(StateMachine_T * p_stateMachine, uint8_t input)
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
void StateMachine_Synchronous_ProcMachine(StateMachine_T * p_stateMachine)
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

void StateMachine_Synchronous_SetInput(StateMachine_T * p_stateMachine, uint8_t input)
{
	p_stateMachine->InputTransition 		= input;
	p_stateMachine->IsSetInputTransition 	= true;
}

void StateMachine_Synchronous_SetTransitionInput(StateMachine_T * p_stateMachine, uint8_t input)
{
	p_stateMachine->InputTransition 		= input;
	p_stateMachine->IsSetInputTransition 	= true;
}

void StateMachine_Synchronous_SetSelfTransitionInput(StateMachine_T * p_stateMachine, uint8_t input)
{
	p_stateMachine->InputSelfTransition 		= input;
	p_stateMachine->IsSetInputSelfTransition 	= true;
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

