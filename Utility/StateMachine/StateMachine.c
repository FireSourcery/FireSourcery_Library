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

/******************************************************************************/
/*!
    Private Functions
*/
/******************************************************************************/
/*!
    Handle multithreaded async inputs
*/
/* Multithreaded may use disable interrupts, if-test, or spin-wait with thread scheduler */
static inline bool AcquireSignal_ISR(StateMachine_T * p_stateMachine)
{
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
}

/* Single threaded input always overwrite */
static inline void ReleaseSignal_ISR(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine;
#endif
}

/* Single threaded input always does not need to check */
static inline bool AcquireSignal_Input(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#else
    Critical_AcquireSignal(&p_stateMachine->InputSignal);
    return true;
#endif
}

static inline void ReleaseSignal_Input(StateMachine_T * p_stateMachine)
{
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
}

static inline void InitSignal(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    atomic_flag_clear(&p_stateMachine->InputSignal); /* ISR implemented with additional sentinel */
#else
    atomic_flag_test_and_set(&p_stateMachine->InputSignal); /* Disables the ISR */
#endif
}

// static inline bool AcquireCritical_Input(StateMachine_T * p_stateMachine)
// {
// #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
//     return Critical_AcquireEnter(&p_stateMachine->InputSignal); /* Multithreaded may use disable interrupts, if-test, or spin-wait with thread scheduler */
// #else
//     Critical_AcquireSignal(&p_stateMachine->InputSignal);
//     return true;
// #endif
// }
static inline bool AcquireCritical(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    return Critical_AcquireEnter(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine;
    return true;
#endif
}

static inline void ReleaseCritical(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    Critical_ReleaseExit(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine;
#endif
}


/* Always skip processing when not acquired */
static inline bool AcquireSignal(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine;
    return true;
#endif
}

static inline void ReleaseSignal(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine;
#endif
}


static inline void Reset(StateMachine_T * p_stateMachine)
{
    p_stateMachine->p_StateActive = p_stateMachine->CONST.P_MACHINE->P_STATE_INITIAL;
    if (p_stateMachine->p_StateActive->ENTRY != NULL) { p_stateMachine->p_StateActive->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
}

// static inline void ProcStateTransition(StateMachine_T * p_stateMachine, StateMachine_State_T * p_newState)
// {
//     if (p_newState != NULL)
//     {
//         if (p_stateMachine->p_StateActive->EXIT != NULL) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONST.P_CONTEXT); }
//         if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
//         p_stateMachine->p_StateActive = p_newState;
//     }
// }

/*
    No null pointer check. User ensure LOOP is defined when using this interface. supply empty for no op
*/
static inline void ProcStateOuput(const StateMachine_T * p_stateMachine)
{
    p_stateMachine->p_StateActive->LOOP(p_stateMachine->CONST.P_CONTEXT);
    // include transition in output loop
    // ProcStateTransition(p_stateMachine, p_stateMachine->p_StateActive->LOOP(p_stateMachine->CONST.P_CONTEXT));
}

/*!
    @return false indicates not accepted input, transition does not exist, no mapped input function.
            true indicates accepted input, state may transition or self transition (with or without entry and exit function).
*/
static inline bool isAcceptInput(const StateMachine_T * p_stateMachine, statemachine_input_id_t inputId)
{
    return ((inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH) && (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId] != NULL));
}

/*!
    Transistion Function maps current state to new state for each input

    @return NULL for self-transition without processing ENTRY,
            self for self-transition processing ENTRY
            another State for transition to another State
*/
static inline void ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    // StateMachine_State_T * p_newState = TransitionFunction(p_stateMachine->p_StateActive, p_stateMachine->CONST.P_CONTEXT, inputId, inputValue);
    StateMachine_State_T * p_newState = p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId](p_stateMachine->CONST.P_CONTEXT, inputValue);
    if (p_newState != NULL) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); }
}

/*
    ProcInput must lock
    A transition must not occur between [isAcceptInput] and [ProcTransition]
*/
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
        p_newState assumed to be valid, caller ensure correctness
        call from within [ProcState] high priority thread, or user handle critical
*/
void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    if (p_stateMachine->p_StateActive->EXIT != NULL) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONST.P_CONTEXT); }
#endif
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
    p_stateMachine->p_StateActive = p_newState;
    /*
        Async may selectively implement critical. If unprotected:
        Set p_StateActive after proc ENTRY.
            This way Async will not proc [p_newState->OUTPUT] until after [p_newState->ENTRY].
            May proc [p_StateActive->OUTPUT] (the prev State) after ENTRY. cannot reject during entry
            p_newState->OUTPUT after p_newState->ENTRY correctly
            p_StateActive->OUTPUT may overwrite p_newState->ENTRY incorrectly

            Alternatively:
            Set p_StateActive before ENTRY.
                This way Async might proc OUTPUT of p_newState before ENTRY.
                p_newState->ENTRY overwrite p_StateActive->OUTPUT correctly
                p_newState->OUTPUT prior to p_newState->ENTRY incorrectly
    */
}

// void StateMachine_ProcStateTransitionId(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId)

/*
    Selectively implement critical in calling layer, if protection not require for all inputs
    User ensure same thread or handle critical
*/
void _StateMachine_ProcState(StateMachine_T * p_stateMachine)
{
    ProcStateOuput(p_stateMachine);
}

bool _StateMachine_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    return ProcInput(p_stateMachine, inputId, inputValue);
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
    p_stateMachine->SyncInputValue = 0;
    InitSignal(p_stateMachine);
// #ifdef  CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE
//     Critical_InitSignal(&p_stateMachine->InputSignal);
// #endif
    Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    if (AcquireCritical(p_stateMachine) == true)
    {
        Reset(p_stateMachine);
        ReleaseCritical(p_stateMachine);
    }
}

/******************************************************************************/
/*
    Synchronous Machine
    Synchronous Proc State - [ProcState] synchronous to clock/timer
    Synchronous Proc Input - Async user [SetInput] proc synchronously during [ProcState]

    Proc last set input, always single threaded proc, inputs may overwrite

    Does not need EnterCritical/DisableIRQ, given:
        [ProcState] thread is higher priority than [SetInput] thread
        [SetInput] calls are on the same thread
        Check accept on [ProcState]

    Multiple [SetInput] calls in between [ProcState] needs to be handled by synchronization mechanism.
        [ProcState] may occur during [SetInput]
        [SyncInput] will not have cleared when a new [SyncInputValue] is set
    E.g.
        p_stateMachine->SyncInputValue = inputValue;
        p_stateMachine->SyncInput = inputId;
        p_stateMachine->SyncInputValue = inputValue;
        StateMachine_Sync_ProcState
        p_stateMachine->SyncInput = inputId;

    Atomic signal skip [ProcState], or spin-wait in low priority input thread

*/
/******************************************************************************/
static inline bool ProcSyncInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    bool isAccept;
    /*
        Multithreaded calls to SetInput must use additional sentinel value
        Signal flag must be cleared to provide indication to other inputs

        Single thread can use atomic flag as bufferHasData
    */
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    if (syncInput != STATE_MACHINE_INPUT_ID_NULL)
    {
        isAccept = ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL; /* Clear input */
    }
#else
    isAccept = ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
#endif
    return isAccept;
}

void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireSignal_ISR(p_stateMachine) == true)
    {
        ProcSyncInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        ReleaseSignal_ISR(p_stateMachine);
    }
    else
    {
        ProcStateOuput(p_stateMachine);
    }
}

/*!
    Multiple calls may overwrite each other, only the last set is processed.

    @return     isAcceptInput()
                a immediate return, although may differ when checked again during ProcState, due to transition or overwrite
*/
bool StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    bool isAccept = isAcceptInput(p_stateMachine, inputId);
    if (isAccept == true)
    {
        if (AcquireSignal_Input(p_stateMachine) == true) /* Disables [ProcState] [ProcInput] Interrupt */ // or AcquireCritical
        {
            p_stateMachine->SyncInputValue = inputValue;
            p_stateMachine->SyncInput = inputId;
            ReleaseSignal_Input(p_stateMachine);
        }
    }
    return isAccept;
}


/******************************************************************************/
/*
    Asynchronous Machine
    Async user input may change state immediately,
        optionally implement periodic ProcStateOuput

    Input/State on the same thread: same behavior as synchronous

    Input/State on different threads:
        Critical on input, Disable ProcState when input is processing

*/
/******************************************************************************/
/*
    Assuming ProcState runs higher priority than ProcInput, critical is implemented in ProcInput only
*/
void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireSignal(p_stateMachine) == true) /* Disabled when input is processing, ensure any transition is completed */
    {
        ProcStateOuput(p_stateMachine);
        ReleaseSignal(p_stateMachine);
    }
}

/*
    AcquireSignal blocks [ProcState] and other [ProcInput]
        inputs must be on the same thread or may skip input processing
*/
bool StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    bool isAccept = false;
    if (AcquireCritical(p_stateMachine) == true)
    {
        isAccept = ProcInput(p_stateMachine, inputId, inputValue);
        ReleaseCritical(p_stateMachine);
    }
    return isAccept;
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
    supports both operation at expense of additional condition check
*/
/******************************************************************************/
inline void StateMachine_ProcState(StateMachine_T * p_stateMachine)
{
    StateMachine_Sync_ProcState(p_stateMachine);
}

inline bool StateMachine_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    return StateMachine_Async_ProcInput(p_stateMachine, inputId, inputValue);
}

inline bool StateMachine_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
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
