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

#ifdef  CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE
#include "System/Critical/Critical.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
	Private Functions
*/
/******************************************************************************/
static inline bool EnterCritical(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
	return (p_stateMachine.CONFIG.USE_CRITICAL == true) ? Critical_AcquireEnter(&p_stateMachine->Mutex) : true;
#else
	(void)p_stateMachine;
	return true;
#endif
}

static inline void ExitCritical(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
	if(p_stateMachine.CONFIG.USE_CRITICAL == true) { Critical_ReleaseExit(&p_stateMachine->Mutex) };
#else
	(void)p_stateMachine;
#endif
}

/*
	No null pointer check. User must supply empty for no op
*/
static inline void ProcOutput(StateMachine_T * p_stateMachine)
{
	p_stateMachine->p_StateActive->OUTPUT(p_stateMachine->CONFIG.P_CONTEXT);
}

/******************************************************************************/
/* Input Id - Additional inputs is passed via context */
/******************************************************************************/
/*!
	TransitionFunction defined via P_TRANSITION_TABLE
	@param[out] pp_newReturn - 	returns pointer to new state, if it exists.
	@return false indicates not accepted input, transition does not exist, pp_newReturn is not set.
			true
				pp_newReturn == 0  no transition,  bypass exist and entry, indicates user defined non transition
				pp_newReturn != 0  transition, perform exist and entry. User may return same state
*/
static inline bool TransitionFunction(StateMachine_State_T ** pp_newReturn, StateMachine_State_T * p_active, void * p_context, statemachine_input_t input)
{
	StateMachine_Transition_T transition = p_active->P_TRANSITION_TABLE[input];
	bool isAccept = (transition != 0U);
	if(isAccept == true) { *pp_newReturn = transition(p_context); };
	return isAccept;
}

/*!
	@return 0 indicates not accepted input, transition does not exist.
*/
static inline bool ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	bool isAccept = (input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH);
	StateMachine_State_T * p_newState;
	if(isAccept == true) { isAccept = TransitionFunction(&p_newState, p_stateMachine->p_StateActive, p_stateMachine->CONFIG.P_CONTEXT, input); }
	if(isAccept == true) { if(p_newState != 0U) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); } }
	return isAccept;
}

/*
	If multi threaded inputs asynch use critical
*/
static inline bool ProcAsyncInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	bool isAccept;
	if(EnterCritical(p_stateMachine))
	{
		isAccept = ProcInput(p_stateMachine, input);
		ExitCritical(p_stateMachine);
	}
	return isAccept;
}

/******************************************************************************/
/* Input Ext - 1 additional input passed as argument  */
/******************************************************************************/
static inline bool TransitionFunctionExt(StateMachine_State_T ** pp_newReturn, StateMachine_State_T * p_active, void * p_context, statemachine_input_t input, uint32_t inputExt)
{
	StateMachine_TransitionExt_T transition = p_active->P_TRANSITION_EXT_TABLE[input];
	bool isAccept = (transition != 0U);
	if(isAccept == true) { *pp_newReturn = transition(p_context, inputExt); };
	return isAccept;
}

static inline bool ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	bool isAccept = (input < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH);
	StateMachine_State_T * p_newState;
	if(isAccept == true) { isAccept = TransitionFunctionExt(&p_newState, p_stateMachine->p_StateActive, p_stateMachine->CONFIG.P_CONTEXT, input, inputExt); }
	if(isAccept == true) { if(p_newState != 0U) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); } }
	return isAccept;
}


static inline bool ProcAsyncInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	bool isAccept;
	if(EnterCritical(p_stateMachine))
	{
		isAccept = ProcInputExt(p_stateMachine, input, inputExt);
		ExitCritical(p_stateMachine);
	}
	return isAccept;
}

/******************************************************************************/
/*!
	Protected Functions
*/
/******************************************************************************/
/*
	Unconditional Transition -	Maps active state to new state, input assumed error checked
	user must ensure correctness, call from state machine / output function only
*/
void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
{
	if(p_stateMachine->p_StateActive->EXIT != 0U) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONFIG.P_CONTEXT); }
	p_stateMachine->p_StateActive = p_newState;
	if(p_newState->ENTRY != 0U) { p_newState->ENTRY(p_stateMachine->CONFIG.P_CONTEXT); }
}

/******************************************************************************/
/*!
	Public Functions
*/
/******************************************************************************/
/*
	States const strut should be compile time def
*/
void StateMachine_Init(StateMachine_T * p_stateMachine)
{
	p_stateMachine->SyncInput = STATE_MACHINE_INPUT_NULL;
	p_stateMachine->SyncInputExt = STATE_MACHINE_INPUT_NULL;
#ifdef  CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE
	p_stateMachine->Mutex = 1U;
#endif
	StateMachine_Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
	if(EnterCritical(p_stateMachine) == true)
	{
		p_stateMachine->p_StateActive = p_stateMachine->CONFIG.P_MACHINE->P_STATE_INITIAL;

		if(p_stateMachine->p_StateActive->ENTRY != 0U)
		{
			p_stateMachine->p_StateActive->ENTRY(p_stateMachine->CONFIG.P_CONTEXT);
		}
		ExitCritical(p_stateMachine);
	}
}

/******************************************************************************/
/*
	Synchronous Machine
	Does not need Critical Section if Proc thread is higher priority than Input Thread
	proc last set input, always single threaded proc
*/
/******************************************************************************/
void StateMachine_Sync_Proc(StateMachine_T * p_stateMachine)
{
	ProcInput(p_stateMachine, p_stateMachine->SyncInput);
	p_stateMachine->SyncInput = STATE_MACHINE_INPUT_NULL; /* clear input, reserved char */
	p_stateMachine->SyncInputExt = STATE_MACHINE_INPUT_NULL;
	ProcOutput(p_stateMachine);
}

bool StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	bool isAccept = (p_stateMachine->SyncInput < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH);
	if(isAccept == true) { p_stateMachine->SyncInput = input; }
	return isAccept;
}

bool StateMachine_Sync_SetInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	bool isAccept = (p_stateMachine->SyncInput < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH);
	if(isAccept == true) { p_stateMachine->SyncInput = input; p_stateMachine->SyncInputExt = inputExt;}
	return isAccept;
}

/******************************************************************************/
/*
	Asynchronous Machine
	Asynchronous Machine has no synchronous periodic output
*/
/******************************************************************************/
bool StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	bool isAccept = ProcAsyncInput(p_stateMachine, input);
	ProcOutput(p_stateMachine);
	return isAccept;
}

bool StateMachine_Async_ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	bool isAccept = ProcAsyncInputExt(p_stateMachine, input, inputExt);
	ProcOutput(p_stateMachine);
	return isAccept;
}

/******************************************************************************/
/*
	Semisynchronous Machine
	Synchronous periodic output
	Asynchronous Input
*/
/******************************************************************************/
void StateMachine_Semi_ProcOutput(StateMachine_T * p_stateMachine)
{
	ProcOutput(p_stateMachine); //todo return user status
}

/*!
	proc user defined transition function via pointer table
	@return true if transition was accepted
*/
bool StateMachine_Semi_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	return ProcAsyncInput(p_stateMachine, input);
}

bool StateMachine_Semi_ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	return ProcAsyncInputExt(p_stateMachine, input, inputExt);
}

/*
	full user defined transition function
*/
bool StateMachine_Semi_ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	bool isAccept = false;

	if(p_stateMachine->p_StateActive->TRANSITION_FUNCTION != 0U)
	{
		isAccept = p_stateMachine->p_StateActive->TRANSITION_FUNCTION(p_stateMachine->CONFIG.P_CONTEXT, input, inputExt);
	}

	return isAccept;
}

/******************************************************************************/
/*
	Simple Menu
*/
/******************************************************************************/
#ifdef CONFIG_STATE_MACHINE_MENU_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	ProcAsyncInput(p_stateMachine, input);
}

StateMachine_State_T * StateMachine_Menu_GetPtrActive(StateMachine_T * p_stateMachine)
{
	return p_stateMachine->p_StateActive;
}

void StateMachine_Menu_SetMenu(StateMachine_T * p_stateMachine, uint8_t menuId)
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
	_StateMachine_ProcStateTransition(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU);
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, statemachine_input_t num)
{
	ProcOutput(p_stateMachine);
}
#endif
