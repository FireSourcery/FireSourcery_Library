/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
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
	No null pointer check. User ensure p_stateMachine->p_StateActive->OUTPUT is defined when using this interface. supply empty for no op
*/
static inline void ProcOutput(StateMachine_T * p_stateMachine)
{
	p_stateMachine->p_StateActive->OUTPUT(p_stateMachine->CONFIG.P_CONTEXT);
}

/******************************************************************************/
/*
	Input Id - Additional inputs is passed via context
	todo remove in favor of Ext
*/
/******************************************************************************/
static inline bool CheckInput(StateMachine_T * p_stateMachine, statemachine_input_t inputId)
{
	return ((inputId < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH) && (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId] != 0U));
}

static inline StateMachine_State_T * TransitionFunction(void * p_context, StateMachine_State_T * p_active, statemachine_input_t inputId)
{
	return p_active->P_TRANSITION_TABLE[inputId](p_context);
}

static inline void ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	StateMachine_State_T * p_newState = TransitionFunction(p_stateMachine->CONFIG.P_CONTEXT, p_stateMachine->p_StateActive, input);
	if(p_newState != 0U) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); }
}

/******************************************************************************/
/*
	Input Ext - 1 additional input passed as argument
*/
/******************************************************************************/
/*!
	@return false indicates not accepted input, transition does not exist.
			true indicates accepted input, state may transition or self transition (with or without entry and exit function).
*/
static inline bool CheckInputExt(StateMachine_T * p_stateMachine, statemachine_input_t inputId)
{
	return ((inputId < p_stateMachine->CONFIG.P_MACHINE->TRANSITION_TABLE_LENGTH) && (p_stateMachine->p_StateActive->P_TRANSITION_EXT_TABLE[inputId] != 0U));
}

/*!

*/
static inline StateMachine_State_T * TransitionFunctionExt(void * p_context, StateMachine_State_T * p_active, statemachine_input_t inputId, uint32_t inputExt)
{
	return p_active->P_TRANSITION_EXT_TABLE[inputId](p_context, inputExt);
}

/*!
	private procInput helper without input error checking
*/
static inline void ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t inputId, uint32_t inputExt)
{
	StateMachine_State_T * p_newState = TransitionFunctionExt(p_stateMachine->CONFIG.P_CONTEXT, p_stateMachine->p_StateActive, inputId, inputExt);
	if(p_newState != 0U) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); }
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
	p_stateMachine->SyncInputExt = 0U;
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
		if(p_stateMachine->p_StateActive->ENTRY != 0U) { p_stateMachine->p_StateActive->ENTRY(p_stateMachine->CONFIG.P_CONTEXT); }
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
	ProcInputExt(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputExt);
	p_stateMachine->SyncInput = STATE_MACHINE_INPUT_NULL; /* clear input, reserved char */
	ProcOutput(p_stateMachine);
}

bool StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	bool isAccept = CheckInput(p_stateMachine, input);
	if(isAccept == true) { p_stateMachine->SyncInput = input; }
	return isAccept;
}

bool StateMachine_Sync_SetInputExt(StateMachine_T * p_stateMachine, statemachine_input_t input, uint32_t inputExt)
{
	bool isAccept = CheckInputExt(p_stateMachine, input);
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
	bool isAccept = false;
	if(EnterCritical(p_stateMachine))
	{
		if(CheckInput(p_stateMachine, input) == true)
		{
			ProcInput(p_stateMachine, input);
			ProcOutput(p_stateMachine);
			isAccept = true;
		}
		ExitCritical(p_stateMachine);
	}
	return isAccept;
}

bool StateMachine_Async_ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t inputId, uint32_t inputExt)
{
	bool isAccept = false;
	if(EnterCritical(p_stateMachine))
	{
		if(CheckInputExt(p_stateMachine, inputId) == true)
		{
			ProcInputExt(p_stateMachine, inputId, inputExt);
			ProcOutput(p_stateMachine);
			isAccept = true;
		}
		ExitCritical(p_stateMachine);
	}
	return isAccept;
}

/******************************************************************************/
/*
	Semi-synchronous Machine
	Synchronous periodic output
	Asynchronous Input
*/
/******************************************************************************/
/*!
	Synchronous periodic output
*/
void StateMachine_Semi_ProcOutput(StateMachine_T * p_stateMachine)
{
	ProcOutput(p_stateMachine); //todo return user status
}

/*!
	proc user defined transition function via pointer table
	If multi threaded inputs asynch use critical
	@return true if transition was accepted
*/
bool StateMachine_Semi_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	bool isAccept = false;
	if(EnterCritical(p_stateMachine))
	{
		if(CheckInput(p_stateMachine, input) == true) { ProcInput(p_stateMachine, input); isAccept = true;}
		ExitCritical(p_stateMachine);
	}
	return isAccept;
}

bool StateMachine_Semi_ProcInputExt(StateMachine_T * p_stateMachine, statemachine_input_t inputId, uint32_t inputExt)
{
	bool isAccept = false;
	if(EnterCritical(p_stateMachine))
	{
		if(CheckInputExt(p_stateMachine, inputId) == true) { ProcInputExt(p_stateMachine, inputId, inputExt); isAccept = true;}
		ExitCritical(p_stateMachine);
	}
	return isAccept;
}

/*
	Full user defined transition function
	No null pointer check. user ensure p_stateMachine->p_StateActive->TRANSITION_FUNCTION is defined when using this interface
*/
bool StateMachine_Semi_ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_t inputId, uint32_t inputExt)
{
	bool isAccept = false;
	if(EnterCritical(p_stateMachine))
	{
		isAccept = p_stateMachine->p_StateActive->TRANSITION_FUNCTION(p_stateMachine->CONFIG.P_CONTEXT, inputId, inputExt);
		ExitCritical(p_stateMachine);
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
	StateMachine_Semi_ProcInput(p_stateMachine, input);
}

StateMachine_State_T * StateMachine_Menu_GetPtrActive(StateMachine_T * p_stateMachine)
{
	return p_stateMachine->p_StateActive;
}

void StateMachine_Menu_SetMenu(StateMachine_T * p_stateMachine, StateMachine_State_T * p_targetMenu)
{
	p_stateMachine->p_StateActive = p_targetMenu;
}

void StateMachine_Menu_StartMenu(StateMachine_T * p_stateMachine, StateMachine_State_T * p_targetMenu)
{
	_StateMachine_ProcStateTransition(p_stateMachine, p_targetMenu);
}

// does not run entry function
void StateMachine_Menu_SetNext(StateMachine_T * p_stateMachine)
{
	if(p_stateMachine->p_StateActive->P_NEXT_MENU != 0U) { StateMachine_Menu_SetMenu(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU); }
}

// run entry function
void StateMachine_Menu_StartNext(StateMachine_T * p_stateMachine)
{
	if(p_stateMachine->p_StateActive->P_NEXT_MENU != 0U) { StateMachine_Menu_StartMenu(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU); }
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, statemachine_input_t input)
{
	StateMachine_Semi_ProcInput(p_stateMachine, input);
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
	StateMachine_Semi_ProcOutput(p_stateMachine);
}
#endif
