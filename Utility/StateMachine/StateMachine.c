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
// static inline void InitSignal_SingleThreaded(StateMachine_T * p_stateMachine)
// {
//     atomic_flag_test_and_set(&p_stateMachine->InputSignal); /* Disables the ISR, when no input is set */
// }

// static inline bool AcquireSignal_ISR_SingleThreaded(StateMachine_T * p_stateMachine)
// {
//     return (atomic_flag_test_and_set(&p_stateMachine->InputSignal) == false); /*  */
// }

// static inline void ReleaseSignal_ISR_SingleThreaded(StateMachine_T * p_stateMachine)
// {
//     (void)p_stateMachine; /* Single threaded input always overwrite */
// }

// static inline bool AcquireSignal_Input_SingleThreaded(StateMachine_T * p_stateMachine)
// {
//     atomic_flag_test_and_set(&p_stateMachine->InputSignal); /* Single threaded input always overwrite. ISR does not need to block Input */
//     return true;
// }

// static inline void ReleaseSignal_Input_SingleThreaded(StateMachine_T * p_stateMachine)
// {
//     atomic_flag_clear(&p_stateMachine->InputSignal);
// }

/*
    Selection between DisableISR and Signal
    Multi-threaded may use disable interrupts, set_and_test signal, or spin-wait with thread scheduler
*/
static inline bool AcquireCritical_ISR(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
#else
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#endif
}

static inline void ReleaseCritical_ISR(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
#else
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#endif
}

static inline bool AcquireCritical_Input(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    _Critical_DisableIrq();
    return true;
#else
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#endif
}

static inline void ReleaseCritical_Input(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    _Critical_EnableIrq();
#else
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#endif
}


// static inline StateMachine_State_T * TransitionFunction(const StateMachine_State_T * p_active, void * p_context, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
// {
//     return p_active->P_TRANSITION_TABLE[inputId](p_context, inputValue);
// }

// static inline StateMachine_State_T * StateOuput(const StateMachine_State_T * p_state, void * p_context)
// {
//     return p_state->LOOP(p_context);
// }

// static inline StateMachine_State_T * StateTransition(const StateMachine_State_T * p_active, void * p_context, StateMachine_State_T * p_newState)
// {
//     if (p_active->EXIT != NULL) { p_active->EXIT(p_context); }
//     if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
//     return p_newState;
// }

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

/*!
    Transistion Function maps current state to new state for each input

    @return NULL for self-transition without processing ENTRY,
            self for self-transition processing ENTRY
            another State for transition to another State
*/
static inline void ProcTransitionFunction(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    StateMachine_State_T * p_newState = p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId](p_stateMachine->CONST.P_CONTEXT, inputValue);
    if (p_newState != NULL) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); }
    // ProcStateTransition(p_stateMachine, TransitionFunction(p_stateMachine->p_StateActive, p_stateMachine->CONST.P_CONTEXT, inputId, inputValue));
}

/*
    No null pointer check. User ensure LOOP is defined when using this interface. supply empty for no op
*/
static inline void ProcStateOuput(StateMachine_T * p_stateMachine)
{
    StateMachine_State_T * p_newState = p_stateMachine->p_StateActive->LOOP(p_stateMachine->CONST.P_CONTEXT);
    if (p_newState != NULL) { _StateMachine_ProcStateTransition(p_stateMachine, p_newState); }
}

/*
    ProcInput must lock
    A transition must not occur between [StateMachine_IsAcceptInput] and [ProcTransition]
*/
static void ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    if (StateMachine_IsAcceptInput(p_stateMachine, inputId) == true) { ProcTransitionFunction(p_stateMachine, inputId, inputValue); }
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
inline void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
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
            May proc [p_StateActive->OUTPUT] (the prev State) after ENTRY.
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
    User ensure same thread
    Selectively implement critical in calling layer, if not require for all inputs
*/
inline void _StateMachine_ProcStateOutput(StateMachine_T * p_stateMachine)
{
    ProcStateOuput(p_stateMachine);
}

// void _StateMachine_ProcState_OrSyncInput(StateMachine_T * p_stateMachine)
// {
//     if (p_stateMachine->SyncInput != STATE_MACHINE_INPUT_ID_NULL)
//     {
//         ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
//         p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL;
//     }
//     else
//     {
//         ProcStateOuput(p_stateMachine);
//     }
// }

/*
    it is possible to return a boolean status indicating if the input was accepted,
    however this status is only dependent on the active State, and not the logic of the State.
    the calling layer only needs to check the active state.
*/
inline void _StateMachine_ProcAsyncInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    ProcInput(p_stateMachine, inputId, inputValue);
}

inline void _StateMachine_ProcSyncInput(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->SyncInput != STATE_MACHINE_INPUT_ID_NULL) /* checked by isAccept input */
    {
        ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL;
    }
}

inline void _StateMachine_SetSyncInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    p_stateMachine->SyncInputValue = inputValue;
    p_stateMachine->SyncInput = inputId;
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
    _StateMachine_SetSyncInput(p_stateMachine, STATE_MACHINE_INPUT_ID_NULL, 0);
    // Sync_InitSignal(p_stateMachine);
    atomic_flag_clear(&p_stateMachine->InputSignal); /* Special case single threaded case, ISR will run null once */
    Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        Reset(p_stateMachine);
        ReleaseCritical_Input(p_stateMachine);
    }
}

/******************************************************************************/
/*
    [Synchronous Machine]
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
/*
    Special case for Single Threaded Input Sync Machines with ISR
*/
// static inline void Sync_InitSignal(StateMachine_T * p_stateMachine)
// {
// #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
//     atomic_flag_clear(&p_stateMachine->InputSignal); /* ISR implemented with additional sentinel */
// #else
//     atomic_flag_test_and_set(&p_stateMachine->InputSignal); /* Disables the ISR */
// #endif
// }

/*!
    Handle multithreaded [SetInput]
*/
static inline bool Sync_AcquireSignal_ISR(StateMachine_T * p_stateMachine)
{
    return Critical_AcquireSignal(&p_stateMachine->InputSignal); /*  */
}

static inline void Sync_ReleaseSignal_ISR(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine; /* Single threaded input always overwrite */
#endif
}

static inline bool Sync_AcquireSignal_Input(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    return Critical_AcquireSignal(&p_stateMachine->InputSignal); /* Multi-threaded do not proceed, if another thread holds the signal */
#else
    Critical_AcquireSignal(&p_stateMachine->InputSignal); /* Single threaded input always overwrite. ISR does not need to block Input */
    return true;
#endif
}

static inline void Sync_ReleaseSignal_Input(StateMachine_T * p_stateMachine)
{
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
}

/******************************************************************************/
/*! Public Functions */
/******************************************************************************/
void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine)
{
    if (Sync_AcquireSignal_ISR(p_stateMachine) == true)
    {
        /*
            Multi-threaded calls to [Sync_SetInput] use additional sentinel value
            [InputSignal] flag is cleared to provide indication to other inputs

            Single threaded calls can use atomic flag as bufferEmpty, bufferHasData when cleared
            [InputSignal] flag does not need to clear, as it is always overwritten by input
            [SyncInput] id does not need to clear
        */
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
        _StateMachine_ProcSyncInput(p_stateMachine);
    #else
        _StateMachine_ProcAsyncInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
    #endif
        Sync_ReleaseSignal_ISR(p_stateMachine);
    }
    else
    {
        /* [ProcStateOutput] without Signal/DisableISR is okay, when all Inputs are [Sync_SetInput] */
        _StateMachine_ProcStateOutput(p_stateMachine);
    }
}

/*!
    Multiple calls overwrite the previous, only the last set is processed.

    [StateMachine_IsAcceptInput] may provide an immediate return,
        although may differ when checked again during [ProcState], due to transition or overwrite
*/
void StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    if (Sync_AcquireSignal_Input(p_stateMachine) == true) /* Disables [ProcInput] portion of [Sync_ProcState] */
    {
        _StateMachine_SetSyncInput(p_stateMachine, inputId, inputValue);
        Sync_ReleaseSignal_Input(p_stateMachine);
    }
}


/******************************************************************************/
/*
    [Asynchronous Machine]
    Async user input may change state immediately,
        optionally implement periodic [ProcStateOuput]

    Input/State on the same thread: same behavior as synchronous

    Input/State on different threads:
        Critical on input, Disable ProcState when input is processing

*/
/******************************************************************************/
/*
    Assuming [ProcState] runs higher priority than [ProcInput], critical is implemented in [ProcInput] only
*/
void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireCritical_ISR(p_stateMachine) == true) /* Disabled when input is processing, ensure any transition is completed */
    {
        _StateMachine_ProcStateOutput(p_stateMachine);
        ReleaseCritical_ISR(p_stateMachine);
    }
}

/*
    [AcquireCritical] blocks [ProcState] and other [ProcInput]
        inputs must be on the same thread or may skip input processing
*/
void StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        _StateMachine_ProcAsyncInput(p_stateMachine, inputId, inputValue);
        ReleaseCritical_Input(p_stateMachine);
    }
}


/******************************************************************************/
/*
    [Combined Semi-Synchronous Machine]
    supports both operation at expense of additional condition check
*/
/******************************************************************************/
inline void StateMachine_ProcState(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE)
    /* [Async_ProcInput] disables ISR, [ProcStateOutput] is ok  */
    StateMachine_Sync_ProcState(p_stateMachine);
#else
    /* [ProcSyncInput] must also be blocked during [Async_ProcInput] in case of transition */
    if (Sync_AcquireSignal_ISR(p_stateMachine) == true)
    {
        _StateMachine_ProcSyncInput(p_stateMachine);
        _StateMachine_ProcStateOutput(p_stateMachine);
        Sync_ReleaseSignal_ISR(p_stateMachine);
    }
#endif
}

inline void StateMachine_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    StateMachine_Sync_SetInput(p_stateMachine, inputId, inputValue);
}

inline void StateMachine_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
{
    StateMachine_Async_ProcInput(p_stateMachine, inputId, inputValue);
}


/******************************************************************************/
/*
    Linked Menus
*/
/******************************************************************************/
#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t input, statemachine_input_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
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
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
    StateMachine_ProcState(p_stateMachine);
}
#endif
