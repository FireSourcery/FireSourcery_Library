/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   StateMachine.c
    @author FireSourcery
    @brief     StateMachine
    @version V0
*/
/******************************************************************************/
#include "StateMachine.h"

#ifdef  CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE
#include "System/Critical/Critical.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>



/******************************************************************************/
/*!
    Private Functions
*/
/******************************************************************************/
/*!
    If multi threaded inputs asynch use critical
*/
static inline bool EnterCritical(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
    return (p_stateMachine.CONST.USE_CRITICAL == true) ? Critical_AcquireEnter(&p_stateMachine->Mutex) : true;
#else
    (void)p_stateMachine;
    return true;
#endif
}

static inline void ExitCritical(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE)
    if (p_stateMachine.CONST.USE_CRITICAL == true) { Critical_ReleaseExit(&p_stateMachine->Mutex) };
#else
    (void)p_stateMachine;
#endif
}

static inline bool AcquireSignal(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE) && !defined(CONFIG_CRITICAL_USE_MUTEX)
    return (p_stateMachine.CONST.USE_CRITICAL == true) ? Critical_AcquireSignal(&p_stateMachine->Mutex) : true;
#else
    (void)p_stateMachine;
    return true;
#endif
}

static inline void ReleaseSignal(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE) && !defined(CONFIG_CRITICAL_USE_MUTEX)
    if (p_stateMachine.CONST.USE_CRITICAL == true) { Critical_ReleaseSignal(&p_stateMachine->Mutex) };
#else
    (void)p_stateMachine;
#endif
}


// static inline void StateOuput(const StateMachine_State_T * p_state, void * p_context)
// {
//     p_state->LOOP(p_context);
// }

/*!
    Transistion Function maps current state to new state for each input

    @return NULL for self-transition without processing ENTRY,
            self for self-transition processing ENTRY
            another State for transition to another State
*/
// static inline StateMachine_State_T * TransitionFunction(const StateMachine_State_T * p_active, void * p_context, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
// {
//     return p_active->P_TRANSITION_TABLE[inputId](p_context, inputValue);
// }


static inline void Reset(StateMachine_T * p_stateMachine)
{
    p_stateMachine->p_StateActive = p_stateMachine->CONST.P_MACHINE->P_STATE_INITIAL;
    if (p_stateMachine->p_StateActive->ENTRY != NULL) { p_stateMachine->p_StateActive->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
}

/*
    No null pointer check. User ensure LOOP is defined when using this interface. supply empty for no op
*/
static inline void ProcStateOuput(const StateMachine_T * p_stateMachine)
{
    p_stateMachine->p_StateActive->LOOP(p_stateMachine->CONST.P_CONTEXT);
}

// static inline void ProcStateTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
// {
//     if (p_stateMachine->p_StateActive->EXIT != NULL) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONST.P_CONTEXT); }
//     if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
//     p_stateMachine->p_StateActive = p_newState;
// }

/*!
    private helper without input error checking
*/
static inline void ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    // StateMachine_State_T * p_newState = TransitionFunction(p_stateMachine->p_StateActive, p_stateMachine->CONST.P_CONTEXT, inputId, inputValue);
    StateMachine_State_T * p_newState = p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId](p_stateMachine->CONST.P_CONTEXT, inputValue);
    if (p_newState != NULL) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); }
}

/*!
    @return false indicates not accepted input, transition does not exist, no mapped input function.
            true indicates accepted input, state may transition or self transition (with or without entry and exit function).
*/
static inline bool isAcceptInput(const StateMachine_T * p_stateMachine, statemachine_input_id_t inputId)
{
    return ((inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH) && (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId] != NULL));
}

/* ProcInput must lock for transition */
static bool ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    bool isAccept = isAcceptInput(p_stateMachine, inputId);
    if (isAccept == true) { ProcTransitionFunction(p_stateMachine, inputId, inputValue); }
    return isAccept;
}

/******************************************************************************/
/*!
    Protected Functions
*/
/******************************************************************************/
/*
    Unconditional Transition - Maps active state to new state.
        input assumed error checked, caller ensure correctness, call from state machine / output function only
*/
void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
    if (p_stateMachine->p_StateActive->EXIT != NULL) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONST.P_CONTEXT); }
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
    p_stateMachine->p_StateActive = p_newState;
    /*
        Async may selectively implement critical. If unprotected:
        Set p_StateActive after proc ENTRY.
            This way Async will not proc OUTPUT of p_newState until after ENTRY.
            May proc p_StateActive->OUTPUT (the prev State) after ENTRY. cannot reject during entry
            p_newState->OUTPUT after p_newState->ENTRY correctly
            p_StateActive->OUTPUT may overwrite p_newState->ENTRY incorrectly

        Alternatively:
        Set p_StateActive before ENTRY. This way Async might proc OUTPUT of p_newState before ENTRY.
            p_newState->ENTRY overwrite p_StateActive->OUTPUT correctly
            p_newState->OUTPUT prior to p_newState->ENTRY incorrectly
            if newState OUTPUT transitions:
                newState2 ENTRY
                newState ENTRY

    */
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
    p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL;
    p_stateMachine->SyncInputValue = 0U;
#ifdef  CONFIG_STATE_MACHINE_MULTITHREADED_ENABLE
    Critical_InitSignal(&p_stateMachine->Mutex);
#endif
    Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    if (EnterCritical(p_stateMachine) == true)
    {
        Reset(p_stateMachine);
        ExitCritical(p_stateMachine);
    }
}

/******************************************************************************/
/*
    Synchronous Machine
    Synchronous State Update - ProcStateOuput synchronous to timer
    Synchronous Proc User Input - Async user inputs proc synchronously during ProcStateOuput

    Proc last set input, always single threaded proc, inputs may overwrite
    Does not need Critical Section -
        Check accept on ProcState
        ProcState thread is higher priority than SetInput thread
        All SetInput calls are on the same thread

*/
/******************************************************************************/
void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->SyncInput != STATE_MACHINE_INPUT_ID_NULL)
    {
        ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL; /* Clear input */
    }

    ProcStateOuput(p_stateMachine);
}

bool StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    bool isAccept = false;
    isAccept = isAcceptInput(p_stateMachine, inputId); /* a immediate return, although may differ when checked again during ProcState */
    if (isAccept == true) { p_stateMachine->SyncInputValue = inputValue; p_stateMachine->SyncInput = inputId; }
    return isAccept;
}

/******************************************************************************/
/*
    Asynchronous Machine
    Async user input may change state immediately,
        optionally implement periodic ProcStateOuput

    Input/State on the same thread: same behavior as synchronous

    Input/State on different threads:
        Critical on input:
            prevent ProcStateOuput state transition in between checking input and proc input

    Selectively implement critical in calling layer, if protection not require for all inputs
*/
/******************************************************************************/
bool StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    bool isAccept = false;
    if (EnterCritical(p_stateMachine) == true) /* Using AcquireSignal require inputs to be on the same thread */
    {
        isAccept = ProcInput(p_stateMachine, inputId, inputValue);
        ExitCritical(p_stateMachine);
    }
    return isAccept;
}

/*
    Assuming ProcStateOuput runs higher priority than ProcInput, critical is implemented in ProcInput only
*/
void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireSignal(p_stateMachine) == true) /* Ensure any transition on input is completed */
    {
        ProcStateOuput(p_stateMachine);
        ReleaseSignal(p_stateMachine);
    }
}

/*
    As Async with StateOutput as common output for all inputs, and on input only.
    Critical only if inputs on different threads
*/
// bool StateMachine_Async_ProcInputWithState(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
// {
//     bool isAccept = false;
//     isAccept = ProcInput(p_stateMachine, inputId, inputValue);
//     if (isAccept == true) { ProcStateOuput(p_stateMachine); }
//     return isAccept;
// }

/******************************************************************************/
/*
    Hybrid Semi-synchronous Machine
    supports both operation as expense of superfluous condition checking
*/
/******************************************************************************/
void StateMachine_ProcState(StateMachine_T * p_stateMachine)
{
    StateMachine_Sync_ProcState(p_stateMachine);
}

bool StateMachine_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    return StateMachine_Async_ProcInput(p_stateMachine, inputId, inputValue);
}

bool StateMachine_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    return StateMachine_Sync_SetInput(p_stateMachine, inputId, inputValue);
}

/******************************************************************************/
/*
    Linked Menus
*/
/******************************************************************************/
#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t input, statemachine_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, statemachine_input_value_t inputValue)
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
    if (p_stateMachine->p_StateActive->P_NEXT_MENU != NULL) { StateMachine_Menu_SetMenu(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU); }
}

// run entry function
void StateMachine_Menu_StartNext(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->p_StateActive->P_NEXT_MENU != NULL) { StateMachine_Menu_StartMenu(p_stateMachine, p_stateMachine->p_StateActive->P_NEXT_MENU); }
}

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, statemachine_input_id_t input, statemachine_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, statemachine_input_value_t inputValue)
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
    StateMachine_ProcState(p_stateMachine);
}
#endif
