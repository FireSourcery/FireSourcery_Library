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
#include <stdbool.h>

#ifdef CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED
	#include "System/Critical/Critical.h"
#elif defined(ONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	extern inline void Critical_AquireMutex(critical_mutex_t);
	extern inline void Critical_ReleaseMutex(critical_mutex_t);
#endif


static inline bool EnterCriticalCommon(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	return (p_stateMachine.CONFIG.USE_CRITICAL == true) ? Critical_Enter_Common(&p_stateMachine->Mutex) : true;
#else
	return true;
#endif
}

static inline void ExitCriticalCommon(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
		if (p_stateMachine.CONFIG.USE_CRITICAL == true) { Critical_Exit_Common(&p_stateMachine->Mutex) };
#endif
}

/*
 * Transition Input
 */
static inline void ProcTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (p_stateMachine->p_StateActive->P_TRANSITION_TABLE != 0)
	{
		/* proc input function */
		if (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[input].ON_TRANSITION != 0)
		{
			p_stateMachine->p_StateActive->P_TRANSITION_TABLE[input].ON_TRANSITION(p_stateMachine->CONFIG.P_CONTEXT);
		}

		/* check new state exists, map new state, else it's a self transition */
		if (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[input].P_STATE != 0)
		{
			p_stateMachine->p_StateActive = p_stateMachine->p_StateActive->P_TRANSITION_TABLE[input].P_STATE;
		}

		/* proc new state common entry function */
		if (p_stateMachine->p_StateActive->ON_ENTRY != 0)
		{
			p_stateMachine->p_StateActive->ON_ENTRY(p_stateMachine->CONFIG.P_CONTEXT);
		}
	}
}

/*
 * Nontransition (output only) Input, bypass entry
 */
static inline void ProcOutputInput(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (p_stateMachine->p_StateActive->P_OUTPUT_TABLE != 0)
	{
		if (p_stateMachine->p_StateActive->P_OUTPUT_TABLE[input] != 0)
		{
			p_stateMachine->p_StateActive->P_OUTPUT_TABLE[input](p_stateMachine->CONFIG.P_CONTEXT);
		}
	}
}

static inline void ProcOutputSynchronous(StateMachine_T * p_stateMachine)
{
	if (p_stateMachine->p_StateActive->OUTPUT_SYNCHRONOUS != 0)
	{
		p_stateMachine->p_StateActive->OUTPUT_SYNCHRONOUS(p_stateMachine->CONFIG.P_CONTEXT);
	}
}

static inline void ProcAsynchronousTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (EnterCriticalCommon(p_stateMachine))
	{
		ProcTransition(p_stateMachine, input);
		ExitCriticalCommon(p_stateMachine);
	}
}

/*
 * States const strut should be compile time def
 */
void StateMachine_Init(StateMachine_T * p_stateMachine)
{
	p_stateMachine->InputTransition = 0U;
	p_stateMachine->InputOutput = 0U;
	p_stateMachine->IsSetInputTransition = false;
	p_stateMachine->IsSetInputOutput = false;

#ifdef CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED
	p_stateMachine->Mutex = 1U;
#endif

	 StateMachine_Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
	if (EnterCriticalCommon(p_stateMachine))
	{
		p_stateMachine->p_StateActive = p_stateMachine->CONFIG.P_MACHINE->P_STATE_INITIAL;

		if (p_stateMachine->p_StateActive->ON_ENTRY != 0)
		{
			p_stateMachine->p_StateActive->ON_ENTRY(p_stateMachine->CONFIG.P_CONTEXT);
		}
		ExitCriticalCommon(p_stateMachine);
	}
}

/*
 * Synchronous Machine Process -
 * proc last set input, always single threaded proc
 */
//void StateMachine_Synchronous_Proc(StateMachine_T * p_stateMachine, void * p_userContext)
void StateMachine_Synchronous_Proc(StateMachine_T * p_stateMachine)
{
	if (p_stateMachine->IsSetInputTransition == true)
	{
		p_stateMachine->IsSetInputTransition = false;

		if (p_stateMachine->InputTransition < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH)
		{
			ProcTransition(p_stateMachine, p_stateMachine->InputTransition);
		}
	}
	else if (p_stateMachine->IsSetInputOutput == true)
	{
		p_stateMachine->IsSetInputOutput = false;

		if (p_stateMachine->InputOutput < p_stateMachine->CONFIG.P_MACHINE->OUTPUT_TABLE_LENGTH)
		{
			ProcOutputInput(p_stateMachine, p_stateMachine->InputOutput);
		}
	}

	ProcOutputSynchronous(p_stateMachine);
}

void StateMachine_Synchronous_SetTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH)
	{
		p_stateMachine->InputTransition 		= input;
		p_stateMachine->IsSetInputTransition 	= true;
		p_stateMachine->IsSetInputOutput 		= false;
	}
}

void StateMachine_Synchronous_SetOutput(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->OUTPUT_TABLE_LENGTH)
	{
		p_stateMachine->InputOutput 			= input;
		p_stateMachine->IsSetInputOutput 		= true;
		p_stateMachine->IsSetInputTransition 	= false;
	}
}

/*
 * Asynchronous Process
 * Asynchronous Machine has no synchronous periodic output
 */
void StateMachine_Asynchronous_ProcTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH)
	{
		ProcAsynchronousTransition(p_stateMachine, input);
	}
	ProcOutputSynchronous(p_stateMachine);
}

void StateMachine_Asynchronous_ProcOutput(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->OUTPUT_TABLE_LENGTH)
	{
		ProcOutputInput(p_stateMachine, input);
	}
	ProcOutputSynchronous(p_stateMachine);
}

/*
 * Semi synchronous
 * Synchronous state logic proc, asynchronous input proc
 */
void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine)
{
	ProcOutputSynchronous(p_stateMachine);
}

void StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH)
	{
		ProcAsynchronousTransition(p_stateMachine, input);
	}
}

void StateMachine_Semisynchronous_ProcOutput(StateMachine_T * p_stateMachine, uint8_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->OUTPUT_TABLE_LENGTH)
	{
		ProcOuputInput(p_stateMachine, input);
	}
}
