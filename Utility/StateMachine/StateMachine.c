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
 * Proc transition function of input index. If new state exists, returns new state, else it's a self transition, returns 0
 */
static inline StateMachine_State_T * TransitionFunction(void * p_context, StateMachine_State_T * p_active, statemachine_input_t input)
{
	StateMachine_Transition_T transitionFunction = p_active->P_TRANSITION_TABLE[input];
	return (transitionFunction != 0U) ? transitionFunction(p_context) : 0U;
}

/*
 * Maps active state to new state
 */
static inline void ProcTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
{
	p_stateMachine->p_StateActive = p_newState;
	if (p_newState->ON_ENTRY != 0U)	{p_newState->ON_ENTRY(p_stateMachine->CONFIG.P_CONTEXT);}
}

static inline void ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	/* proc input function, check new state exists, else it's a self transition */
	StateMachine_State_T * p_newState = TransitionFunction(p_stateMachine->CONFIG.P_CONTEXT, p_stateMachine->p_StateActive, input);

	/* Self transitions - User return 0 to bypass on entry, or return current state to run on entry function */
	if (p_newState != 0U) {ProcTransition(p_stateMachine, p_newState);}
}

/*
 * No null pointer check. User must supply empty for no op
 */
static inline void ProcOutput(StateMachine_T * p_stateMachine)
{
	p_stateMachine->p_StateActive->OUTPUT(p_stateMachine->CONFIG.P_CONTEXT);
}

/*
 * If multi threaded inputs asynch use critical
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
 * Protected Function
 * Unconditional Transition - user must ensure correctness, call from state output function only
 */
void _StateMachine_ProcTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
{
	ProcTransition(p_stateMachine, p_newState);
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

/******************************************************************************/
/*
 * Synchronous Machine
 *
 * Does not need Critical Section if Proc thread is higher priority than Input Thread
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
		p_stateMachine->Input = 0xFFU; //clear input, reserved char
	}

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
void StateMachine_Semisynchronous_ProcState(StateMachine_T * p_stateMachine)
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

//void StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, statemachine_input_t inputTransition)
//{
//	ProcAsynchronousInput(p_stateMachine, inputTransition);
//}
//
//void StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t inputTransition, uint32_t inputVar)
//{
//	ProcAsynchronousInput(p_stateMachine, inputTransition, inputVar);
//}




#ifdef CONFIG_STATE_MACHINE_MENU_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	ProcAsynchronousInput(p_stateMachine, input);
}

StateMachine_State_T * StateMachine_Menu_GetPtrActive(StateMachine_T * p_stateMachine)
{
	return p_stateMachine->p_StateActive;
}

void StateMachine_Menu_SetMenu(Menu_T * target)
{
	p_MenuSelect = target;
}

void StateMachine_Menu_SetNext() //does not run entry funcion
{
	if(p_MenuSelect->NextMenu) p_MenuSelect = p_MenuSelect->NextMenu;
}

void StateMachine_Menu_StartMenu(Menu_T * target)
{
	p_MenuSelect = target;
	if(p_MenuSelect->InitFunction) p_MenuSelect->InitFunction();
}

void StateMachine_Menu_StartNext(StateMachine_T * p_stateMachine)
{
	//enter cirtical
	ProcTransition(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU);
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, statemachine_input_t num)
{
	ProcOutput(p_stateMachine);
}
#endif


/*
 * New
 */

/* proc input function, check new state exists, else it's a self transition */
/* Self transitions - User return 0 to bypass on entry, or return current state to run on entry function */
//static inline void ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputVar)
//{
//	StateMachine_TransitionInput_T transitionFunction = p_stateMachine->p_StateActive->TRANSITION_INPUT;
//	StateMachine_State_T * p_newState = (transitionFunction != 0U) ? transitionFunction(p_stateMachine->CONFIG.P_CONTEXT, input, inputVar) : 0U;
////	StateMachine_State_T * p_currentState = p_stateMachine->p_StateActive;
//
////	bool status = (p_newState != 0U);
//
//	if (p_newState != 0U) {ProcTransition(p_stateMachine, p_newState);}
//
////	return status;
//}

//return true if transition was sucessful
//bool StateMachine_Semisynchronous_ProcTransition(StateMachine_T * p_stateMachine, statemachine_input_t inputTransition)
//{
//	if (EnterCriticalCommon(p_stateMachine))
//	{
//		ProcTransitionFunction(p_stateMachine, inputMode);
//		ExitCriticalCommon(p_stateMachine);
//	}
//}

//bool StateMachine_Semisynchronous_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t inputTransition, uint32_t inputVar)
//{
//	if (EnterCriticalCommon(p_stateMachine))
//	{
//		ProcTransitionFunction(p_stateMachine, inputMode, inputVar);
//		ExitCriticalCommon(p_stateMachine);
//	}
//}


