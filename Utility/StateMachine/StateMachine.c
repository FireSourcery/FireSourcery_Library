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
	@brief 	StateMachine
	@version V0
*/
/******************************************************************************/
#include "StateMachine.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED
	#include "System/Critical/Critical.h"
#elif defined(ONFIG_STATE_MACHINE_CRITICAL_USER_DEFINED)
	extern inline void Critical_Enter_Common(critical_mutex_t);
	extern inline void Critical_Exit_Common(critical_mutex_t);
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
 * Maps active state to new state
 */
static inline StateMachine_State_T * TransitionFunction(void * p_context, StateMachine_State_T * p_active, statemachine_input_t input)
{
	return (p_active->P_TRANSITION_TABLE[input] != 0U) ? p_active->P_TRANSITION_TABLE[input](p_context) : 0U;
}

static inline void ProcTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
{
	p_stateMachine->p_StateActive = p_newState;
	if (p_newState->ON_ENTRY != 0U)
	{
		p_newState->ON_ENTRY(p_stateMachine->CONFIG.P_CONTEXT);
	}
}

static inline void ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	StateMachine_State_T * p_newState = TransitionFunction(p_stateMachine->CONFIG.P_CONTEXT, p_stateMachine->p_StateActive, input);
	/* proc input function, check new state exists, else it's a self transition */

	/* Self transitions - User return 0 to bypass on entry, or return current state to run on entry function */
	if (p_newState != 0U)
	{
		ProcTransition(p_stateMachine, p_newState);
	}
}

/*
 * No null pointer check. User must supply empty for no op
 */
static inline void ProcOutput(StateMachine_T * p_stateMachine)
{
	p_stateMachine->p_StateActive->OUTPUT(p_stateMachine->CONFIG.P_CONTEXT);
}

/*
 * If multi threaded inputs asynch use cirtical
 */
static inline void ProcAsynchronousInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	if (input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH)
	{
		if (EnterCriticalCommon(p_stateMachine))
		{
			ProcInput(p_stateMachine, input);
			ExitCriticalCommon(p_stateMachine);
		}
	}
}

/*
 * States const strut should be compile time def
 */
void StateMachine_Init(StateMachine_T * p_stateMachine)
{
	p_stateMachine->Input  = 0U;
#ifdef CONFIG_STATE_MACHINE_CRITICAL_LIBRARY_DEFINED
	p_stateMachine->Mutex = 1U;
#endif
	StateMachine_Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
	if (EnterCriticalCommon(p_stateMachine) == true)
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
 * Unconditional Transition - user must ensure correctness, call from state output function only
 */
void StateMachine_ProcTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
{
	ProcTransition(p_stateMachine, p_newState);
}


/******************************************************************************/
/*
 * Synchronous Machine
 */
/******************************************************************************/
/*
 * proc last set input, always single threaded proc
 */
void StateMachine_Synchronous_Proc(StateMachine_T * p_stateMachine)
{
	if(p_stateMachine->Input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH)
	{
		ProcInput(p_stateMachine, p_stateMachine->Input);
	}
	p_stateMachine->Input = 0xFFU; //clear input
	ProcOutput(p_stateMachine);
}

void StateMachine_Synchronous_SetInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	p_stateMachine->Input = input;
}

/******************************************************************************/
/*
 * Asynchronous Machine
 */
/******************************************************************************/
/*
 * Asynchronous Machine has no synchronous periodic output
 */
void StateMachine_Asynchronous_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	ProcAsynchronousInput(p_stateMachine, input);
	ProcOutput(p_stateMachine);
}

/******************************************************************************/
/*
 * Semisynchronous Machine
 */
/******************************************************************************/
/*
 * Synchronous State Output
 */
void StateMachine_Semisynchronous_ProcOutput(StateMachine_T * p_stateMachine)
{
	ProcOutput(p_stateMachine);
}

/*
 * Asynchronous Input
 */
void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	ProcAsynchronousInput(p_stateMachine, input);
}

