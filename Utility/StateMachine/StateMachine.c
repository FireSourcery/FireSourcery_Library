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
    [Sync] Machine
    Special case where [ProcStateOutput] is not blocked by [SetInput]
*/
static inline bool AcquireSignal_ISR(StateMachine_T * p_stateMachine)
{
    return Critical_AcquireSignal(&p_stateMachine->InputSignal); /* Checks if an Input has the signal  */
}

static inline void ReleaseSignal_ISR(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine; /* Single threaded input always overwrite */
#endif
}

static inline bool AcquireSignal_Input(StateMachine_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    /* Multi-threaded do not proceed, if another thread holds the signal */
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#else
    /* Single threaded input always overwrite. ISR will run to completion. does not need to block Input */
    Critical_AcquireSignal(&p_stateMachine->InputSignal);
    return true;
#endif
}

static inline void ReleaseSignal_Input(StateMachine_T * p_stateMachine)
{
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
}

/*
    [Async] Machine
    Selection between DisableISR and Signal
    Multi-threaded may use disable interrupts, set_and_test signal, or spin-wait with thread scheduler
*/
static inline bool AcquireCritical_ISR(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    _Critical_DisableIrq();
    #endif
    return true;
#else
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#endif
}

static inline void ReleaseCritical_ISR(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    _Critical_EnableIrq();
    #endif
#else
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#endif
}

static inline bool AcquireCritical_Input(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
    _Critical_DisableIrq();
    return true;
#else
    return Critical_AcquireSignal(&p_stateMachine->InputSignal);
#endif
}

static inline void ReleaseCritical_Input(StateMachine_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
    _Critical_EnableIrq();
#else
    Critical_ReleaseSignal(&p_stateMachine->InputSignal);
#endif
}


/******************************************************************************/
/*!
    [StateMachine_State_T] Common
*/
/******************************************************************************/

/******************************************************************************/
/* Transition */
/******************************************************************************/
static inline void State_OnTransition(const StateMachine_State_T * p_state, const StateMachine_State_T * p_newState, void * p_context)
{
    if (p_newState != NULL)
    {
    #ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
        if (p_state->EXIT != NULL) { p_state->EXIT(p_context); }
    #endif
        if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
    }
}

/******************************************************************************/
/* Output */
/******************************************************************************/
static inline const StateMachine_State_T * State_TransitionOutput(const StateMachine_State_T * p_state, void * p_context)
{
    if (p_state->LOOP != NULL) { p_state->LOOP(p_context); }
    return (p_state->NEXT != NULL) ? p_state->NEXT(p_context) : NULL;
}

static inline const StateMachine_State_T * State_TransitionOutput_Top(const StateMachine_State_T * p_state, void * p_context)
{
    p_state->LOOP(p_context);
    return (p_state->NEXT != NULL) ? p_state->NEXT(p_context) : NULL;
}

/******************************************************************************/
/* Input */
/******************************************************************************/
static inline StateMachine_Input_T State_AcceptInput(const StateMachine_State_T * p_state, void * p_context, state_machine_input_t inputId)
{
   volatile StateMachine_Input_T result = NULL;
    if      (p_state->P_TRANSITION_TABLE != NULL)   { result = p_state->P_TRANSITION_TABLE[inputId]; }
    else if (p_state->TRANSITION_MAPPER != NULL)    { result = p_state->TRANSITION_MAPPER(p_context, inputId); }
    return result;
}

static inline StateMachine_Input_T State_AcceptInput_Table(const StateMachine_State_T * p_state, void * p_context, state_machine_input_t inputId)
{
    (void)p_context;
    return p_state->P_TRANSITION_TABLE[inputId];
}

static inline StateMachine_Input_T State_AcceptInput_Mapper(const StateMachine_State_T * p_state, void * p_context, state_machine_input_t inputId)
{
    return p_state->TRANSITION_MAPPER(p_context, inputId);
}

/*
    Resolve on transition action and return new [State]
*/
static inline StateMachine_State_T * ResolveInputHandler(StateMachine_Input_T transition, void * p_context, state_machine_value_t inputValue)
{
    return (transition != NULL) ? transition(p_context, inputValue) : NULL;
}

/*!
    Transistion Function maps current state to new state for each input
    Map (inputId, inputValue) => newState
    @return NULL for self-transition without processing ENTRY,
            self for self-transition processing ENTRY
            another State for transition to another State
*/
/*
    [ProcInput] must lock to prevent a transition occurring between [AcceptInput] and [ProcTransition]
    Store a local copy first. if a transition does occur, prevent NULL pointer, the previously selected transition will run.
*/
static inline StateMachine_State_T * State_TransitionFunction(const StateMachine_State_T * p_state, void * p_context, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    return ResolveInputHandler(State_AcceptInput(p_state, p_context, inputId), p_context, inputValue);
}


/******************************************************************************/
/*
    [const StateMachine_State_T **]
    convience wrapper with pointer State
    map compile time const to runtime variable
*/
/******************************************************************************/
static inline void State_Init(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
    *pp_currentState = p_newState;
}

static inline void State_Set(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    if ((*pp_currentState)->EXIT != NULL) { (*pp_currentState)->EXIT(p_context); }
#endif
    *pp_currentState = p_newState;
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
}

// (*pp_currentState) != NULL
// static inline void State_ProcTransition(const StateMachine_State_T ** pp_currentState, void * p_context, const StateMachine_State_T * p_newState)
static inline void State_ProcTransition(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
    if (p_newState != NULL) { State_Set(pp_currentState, p_newState, p_context); }
}

static inline void State_ProcOutput(const StateMachine_State_T ** pp_currentState, void * p_context)
{
    State_ProcTransition(pp_currentState, State_TransitionOutput(*pp_currentState, p_context), p_context);
}

static void State_ProcInput(const StateMachine_State_T ** pp_currentState, void * p_context, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    State_ProcTransition(pp_currentState, State_TransitionFunction(*pp_currentState, p_context, inputId, inputValue), p_context);
}

/* known mapper or table only */
static void State_ProcInput_ByTable(const StateMachine_State_T ** pp_currentState, void * p_context, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    State_ProcTransition(pp_currentState, ResolveInputHandler(State_AcceptInput_Table(*pp_currentState, p_context, inputId), p_context, inputValue), p_context);
}



/******************************************************************************/
/*!
    Protected Functions
*/
/******************************************************************************/

/******************************************************************************/
/*
    [StateMachine_T]
    Top Level State [p_StateActive]
    pre signal lock
    With runtime variable
    optionally skip some null ptr checks
*/
/******************************************************************************/
//todo combine
static inline void Reset(StateMachine_T * p_stateMachine)
{
    State_Init(&p_stateMachine->p_StateActive, p_stateMachine->CONST.P_MACHINE->P_STATE_INITIAL, p_stateMachine->CONST.P_CONTEXT);
}

static inline void _ProcTransition_Top(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
    assert(p_newState == NULL || p_newState->DEPTH == 0); /* Top level state */
    State_ProcTransition(pp_currentState, p_newState, p_context);
}

/* Top level check table only */
/* no null ptr check on P_TRANSITION_TABLE */
static inline StateMachine_Input_T AcceptInput(const StateMachine_T * p_stateMachine, state_machine_input_t inputId)
{
    return p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId];
}

/* check table only */
static inline StateMachine_State_T * TransitionFunction(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    // StateMachine_Input_T transition = AcceptInput(p_stateMachine, inputId);
    // return (transition != NULL) ? transition(p_stateMachine->CONST.P_CONTEXT, inputValue) : NULL;
    return ResolveInputHandler(AcceptInput(p_stateMachine, inputId), p_stateMachine->CONST.P_CONTEXT, inputValue);
}

/* (*pp_currentState) != NULL */
static inline void ProcInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    assert(inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH); /* inputId is known at compile time */
    _ProcTransition_Top(&p_stateMachine->p_StateActive, TransitionFunction(p_stateMachine, inputId, inputValue), p_stateMachine->CONST.P_CONTEXT);
}

static inline const StateMachine_State_T * TransitionOutput(const StateMachine_State_T * p_state, void * p_context)
{
    p_state->LOOP(p_context); /* Top level always defined */
    return (p_state->NEXT != NULL) ? p_state->NEXT(p_context) : NULL;
}

/*
    No null pointer check on LOOP.
*/
static inline void ProcStateOuput(StateMachine_T * p_stateMachine)
{
    _ProcTransition_Top(&p_stateMachine->p_StateActive, TransitionOutput(p_stateMachine->p_StateActive, p_stateMachine->CONST.P_CONTEXT), p_stateMachine->CONST.P_CONTEXT);
}


/*
    Unconditional Transition - Maps active state to new state.
    p_newState assumed to be valid, caller ensure correctness
    call from within [ProcState] high priority thread, or user handle critical
*/
inline void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
    // State_Set(&p_stateMachine->p_StateActive, p_newState, p_stateMachine->CONST.P_CONTEXT);
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    if (p_stateMachine->p_StateActive->EXIT != NULL) { p_stateMachine->p_StateActive->EXIT(p_stateMachine->CONST.P_CONTEXT); }
#endif
    p_stateMachine->p_StateActive = p_newState;
    if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_stateMachine->CONST.P_CONTEXT); }
    /*
        Async may selectively implement critical. If unprotected:
        Set p_StateActive after proc ENTRY.
            [p_newState->OUTPUT] will not proc until after [p_newState->ENTRY], correctly
            [p_StateActive->OUTPUT] (prevState) may proc after [p_newState->ENTRY], overwrite, incorrectly

        Set p_StateActive before ENTRY.
            [p_newState->ENTRY] will not be overwritten by [p_StateActive->OUTPUT], correctly
            [p_newState->OUTPUT] may proc before [p_newState->ENTRY], without setup, incorrectly
    */
}

/*
    User ensure same thread
    Selectively implement critical in calling layer, if not require for all inputs
*/
inline void _StateMachine_ProcStateOutput(StateMachine_T * p_stateMachine)
{
    ProcStateOuput(p_stateMachine);
}

inline void _StateMachine_ProcSyncInput(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->SyncInput != STATE_MACHINE_INPUT_ID_NULL) /* checked by isAccept input */
    {
        ProcInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL;
    }
}

inline void _StateMachine_SetSyncInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    p_stateMachine->SyncInputValue = inputValue;
    p_stateMachine->SyncInput = inputId;
}

/*
    it is possible to return a boolean status indicating if the input was accepted,
    however this status is only dependent on the active State, and not the logic of the State.
    the calling layer only needs to check the active state.
*/
inline void _StateMachine_ProcAsyncInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    ProcInput(p_stateMachine, inputId, inputValue);
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
    // atomic_flag_clear(&p_stateMachine->InputSignal); /* sync runs with 255 once */
    atomic_flag_test_and_set(&p_stateMachine->InputSignal);
    Reset(p_stateMachine);
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    if (AcquireSignal_Input(p_stateMachine) == true)
    {
        Reset(p_stateMachine);
        ReleaseSignal_Input(p_stateMachine);
    }
}

/******************************************************************************/
/*
    [Synchronous Machine]
    Synchronous Proc State - [ProcState] synchronous to clock/timer
    Synchronous Proc Input - Async user [SetInput] proc synchronously during [ProcState]
        Setting Inputs will never delay periodic Output Loop

    Proc last set input, always single threaded proc, inputs may overwrite unless queued

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
void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine)
{
    if (AcquireSignal_ISR(p_stateMachine) == true)
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
        /* Singled threaded case handled by signal flag */
        _StateMachine_ProcAsyncInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
    #endif
        ReleaseSignal_ISR(p_stateMachine);
    }
    /* Optionally Proc both */
    else
    {
        /* All Inputs are [Sync_SetInput], all transition are processed Sync, [ProcStateOutput] will not interrupt a transition  */
        _StateMachine_ProcStateOutput(p_stateMachine);
    }
}

/*!
    Multiple calls overwrite the previous, only the last set is processed.

    [StateMachine_IsAcceptInput] may provide an immediate return,
        although may differ when checked again during [ProcState], due to transition or overwrite
*/
void StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    /* Disables [ProcInput] portion of [Sync_ProcState]. */
    if (AcquireSignal_Input(p_stateMachine) == true)
    {
        _StateMachine_SetSyncInput(p_stateMachine, inputId, inputValue);
        ReleaseSignal_Input(p_stateMachine);
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
    when input can be higher priority this must block, prevent transition
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

    Signal Case
        may skip the next [ProcState]
        inputs must be on the same thread, or a non polling input maybe missed
*/
void StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
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
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    /* [Async_ProcInput] disables ISR, [ProcStateOutput] same as Sync case  */
    StateMachine_Sync_ProcState(p_stateMachine);
#elif defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)
    /* Both transition threads [ProcState]/[Async_ProcInput] run to completion */
    if (AcquireCritical_ISR(p_stateMachine) == true)
    {
        _StateMachine_ProcSyncInput(p_stateMachine);     /* Proc input must use sentinel */
        _StateMachine_ProcStateOutput(p_stateMachine);
        ReleaseCritical_ISR(p_stateMachine);
    }
#endif
}

/* Handles Top level transitions only */

inline void StateMachine_SetInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    StateMachine_Sync_SetInput(p_stateMachine, inputId, inputValue);
}

inline void StateMachine_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    StateMachine_Async_ProcInput(p_stateMachine, inputId, inputValue);
}

/* set with signal */
void StateMachine_SetStateValue(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_state, StateMachine_Set_T setter, state_machine_value_t value)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        if (StateMachine_IsActiveState(p_stateMachine, p_state) == true) { setter(p_stateMachine->CONST.P_CONTEXT, value); }
        ReleaseCritical_Input(p_stateMachine);
    }
}

/******************************************************************************/
/*
    [Hierarchical State Machine]
    State as Node/Tree
*/
/******************************************************************************/

/******************************************************************************/
/*
    Generic [Parent Node Tree] Relations
*/
/******************************************************************************/

/*
    Is [test] a Ancestor of [ref]
*/
static bool State_IsAncestor(const StateMachine_State_T * p_refState, const StateMachine_State_T * p_isAncestor)
{
    // if (p_refState == NULL) { return false; }
    // return (p_refState == p_isAncestor) ? true : State_IsAncestor(p_refState->P_PARENT, p_isAncestor);

    assert(p_refState != NULL);
    assert(p_isAncestor != NULL);

    bool isAncestor = false;
    for (const StateMachine_State_T * p_ref = p_refState; (p_ref->DEPTH > p_isAncestor->DEPTH); p_ref = p_ref->P_PARENT)
    {
        if (p_ref->P_PARENT == p_isAncestor) { isAncestor = true; break; } /* Compare the parent. p_ref->DEPTH returns 0 before p_ref reaches NULL */
    }
    return isAncestor;
}

/*

*/
static bool State_IsDescendant(const StateMachine_State_T * p_refState, const StateMachine_State_T * p_isDescendant)
{
    // if (p_isDescendant) == NULL) { return false; }
    // return (p_refState == p_isDescendant) ? true : State_IsDescendant(p_refState, p_isDescendant)->P_PARENT);

    assert(p_refState != NULL);
    assert(p_isDescendant != NULL);

    bool isDescendant = false;
    for (const StateMachine_State_T * p_test = p_isDescendant; (p_test->DEPTH > p_refState->DEPTH); p_test = p_test->P_PARENT)
    {
        if (p_test->P_PARENT == p_refState) { isDescendant = true; break; }
    }
    return isDescendant;
}

// static bool State_IsCousin(const StateMachine_State_T * p_refState, const StateMachine_State_T * p_isCousin)
// {

// }


/* State as a Branch from root to itself. */
/*

*/
static bool State_IsReachableBranch(const StateMachine_State_T * p_active, const StateMachine_State_T * p_test)
{
    return ((p_active == p_test) || State_IsAncestor(p_active, p_test) || State_IsDescendant(p_active, p_test));
}

static bool State_IsActiveBranch(const StateMachine_State_T * p_active, const StateMachine_State_T * p_test)
{
    return ((p_active == p_test) || State_IsAncestor(p_active, p_test));
}

static bool State_IsInactiveBranch(const StateMachine_State_T * p_active, const StateMachine_State_T * p_test)
{
    return State_IsDescendant(p_active, p_test);
}


static const StateMachine_State_T * State_CommonAncestorOf(const StateMachine_State_T * p_state1, const StateMachine_State_T * p_state2)
{
    if (p_state1 == NULL || p_state2 == NULL) { return NULL; }

    const StateMachine_State_T * p_iterator1 = p_state1;
    const StateMachine_State_T * p_iterator2 = p_state2;

    // Bring both nodes to the same level
    while (p_iterator1->DEPTH > p_iterator2->DEPTH) { p_iterator1 = p_iterator1->P_PARENT; }
    while (p_iterator2->DEPTH > p_iterator1->DEPTH) { p_iterator2 = p_iterator2->P_PARENT; }

    // Traverse upwards until both nodes meet
    while (p_iterator1 != p_iterator2)
    {
        p_iterator1 = p_iterator1->P_PARENT;
        p_iterator2 = p_iterator2->P_PARENT;
    }

    return p_iterator1;

    // if (p_state1 == NULL || p_state2 == NULL) { return NULL; }
    // if (p_state1 == p_state2) { return p_state1; }
    // if (p_state1->DEPTH > p_state2->DEPTH) { return State_CommonAncestorOf(p_state1->P_PARENT, p_state2); }
    // if (p_state1->DEPTH < p_state2->DEPTH) { return State_CommonAncestorOf(p_state1, p_state2->P_PARENT); }
    // return State_CommonAncestorOf(p_state1->P_PARENT, p_state2->P_PARENT);
}


// void State_CaptureTraverseUp(const StateMachine_State_T * p_state, const StateMachine_State_T ** p_buffer, uint8_t * p_count)
// {
//     if (p_state != NULL)
//     {
//         p_buffer[(*p_count)++] = p_state;
//         State_CaptureTraverseUp(p_state->P_PARENT, p_buffer, p_count);
//     }
// }

// void State_CaptureTraverseDown(const StateMachine_State_T * p_state, const StateMachine_State_T ** p_buffer, uint8_t * p_count)
// {
//     if (p_state != NULL)
//     {
//         State_CaptureTraverseDown(p_state->P_PARENT, p_buffer, p_count);
//         p_buffer[(*p_count)++] = p_state;
//     }
// }

// static inline void State_CaptureTraverseUp(const StateMachine_State_T * p_descendant, const StateMachine_State_T ** p_buffer, uint8_t * p_count)
// {
//     for (const StateMachine_State_T * p_iterator = p_descendant; p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
//     {
//         p_buffer[(*p_count)++] = p_iterator;
//     }
// }


// static inline void State_CaptureTraverseDown(const StateMachine_State_T * p_descendant, const StateMachine_State_T ** p_buffer, uint8_t * p_count)
// {
//     for (const StateMachine_State_T * p_iterator = p_descendant; p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
//     {
//         p_buffer[p_descendant->DEPTH - (*p_count)] = p_iterator;
//         (*p_count)++;
//     }
// }

/******************************************************************************/
/* State Tree Functions */
/******************************************************************************/

/******************************************************************************/
/* State Tree Transition */
/******************************************************************************/
/* Call Exit traversing up the tree */
static inline void TraverseExit(const StateMachine_State_T * p_start, const StateMachine_State_T * p_common, void * p_context)
{
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    for (const StateMachine_State_T * p_iterator = p_start; p_iterator != p_common; p_iterator = p_iterator->P_PARENT)
    {
        if (p_iterator->EXIT != NULL) { p_iterator->EXIT(p_context); }
    }
#endif
}

// static inline void TraverseEntry_ValidateDepth(const StateMachine_State_T * p_common, const StateMachine_State_T * p_end, void * p_context, uint8_t depth)
// {
//     if ((p_end != NULL) && (p_end != p_common))
//     {
//         TraverseEntry_ValidateDepth(p_common, p_end->P_PARENT, p_context, depth + 1);
//         if (p_end->ENTRY != NULL) { p_end->ENTRY(p_context); }
//     }
// }

/* Call Entry traversing down the tree */
static inline void TraverseEntry(const StateMachine_State_T * p_common, const StateMachine_State_T * p_end, void * p_context)
{
    if ((p_end != NULL) && (p_end != p_common))
    {
        TraverseEntry(p_common, p_end->P_PARENT, p_context);
        if (p_end->ENTRY != NULL) { p_end->ENTRY(p_context); }
    }
}

static inline void State_TraverseTransitionThrough(const StateMachine_State_T * p_start, const StateMachine_State_T * p_common, const StateMachine_State_T * p_end, void * p_context)
{
    TraverseExit(p_start, p_common, p_context);
    TraverseEntry(p_common, p_end, p_context);
}

/* call handle assignment */
static inline void State_TraverseTransition(const StateMachine_State_T * p_start, const StateMachine_State_T * p_end, void * p_context)
{
    const StateMachine_State_T * p_common = State_CommonAncestorOf(p_start, p_end);

    // if (p_end != NULL)
    {
        TraverseExit(p_start, p_common, p_context);
        TraverseEntry(p_common, p_end, p_context);
    }
}

/******************************************************************************/
/* State Branch Output */
/******************************************************************************/
/*
    Proc all Outputs.
    Top level determines the target State. Transition start from leaf State.
    optionally pass end to skip top levels
*/
/* traverse up only for now */
static inline const StateMachine_State_T * State_TraverseOutput(const StateMachine_State_T * p_start, const StateMachine_State_T * p_end, void * p_context)
{
    const StateMachine_State_T * p_next= NULL;
    for (const StateMachine_State_T * p_iterator = p_start; (p_iterator != NULL) && (p_iterator != p_end); p_iterator = p_iterator->P_PARENT)
    {
        p_next = State_TransitionOutput(p_iterator, p_context); /* allow top level overwrite */
    }
    return p_next;
}


/******************************************************************************/
/* State Branch Input */
/******************************************************************************/
/*
    Take the first transition that accepts the input
*/
static inline StateMachine_Input_T State_TraverseAcceptInput(const StateMachine_State_T * p_start, void * p_context, state_machine_input_t inputId)
{
    volatile StateMachine_Input_T result;
    for (const StateMachine_State_T * p_iterator = p_start; p_iterator != NULL; p_iterator = p_iterator->P_PARENT)
    {
        result = State_AcceptInput(p_iterator, p_context, inputId);
        if (result != NULL) { break; }
    }
    return result;
}

static inline StateMachine_State_T * State_TransitionFunction_Traverse(const StateMachine_State_T * p_start, void * p_context, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    return ResolveInputHandler(State_TraverseAcceptInput(p_start, p_context, inputId), p_context, inputValue);
}

/******************************************************************************/
/*
    [const StateMachine_State_T **]
*/
/******************************************************************************/
static void StateBranch_ProcTransition(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_newState, void * p_context)
{
    if (p_newState != NULL)
    {
        State_TraverseTransition(*pp_currentState, p_newState, p_context);
        *pp_currentState = p_newState;
    }
}

static void StateBranch_ProcTransitionThrough(const StateMachine_State_T ** pp_current, const StateMachine_State_T * p_common, const StateMachine_State_T * p_new, void * p_context)
{
    if (p_new != NULL)
    {
        State_TraverseTransitionThrough(*pp_current, p_common, p_new, p_context);
        *pp_current = p_new;
    }
}

static inline void StateBranch_ProcOutput(const StateMachine_State_T ** pp_currentState, const StateMachine_State_T * p_limit, void * p_context)
{
    StateBranch_ProcTransition(pp_currentState, State_TraverseOutput(*pp_currentState, p_limit, p_context), p_context);
}

static inline void StateBranch_ProcInput(const StateMachine_State_T ** pp_currentState, void * p_context, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    StateBranch_ProcTransition(pp_currentState, State_TransitionFunction_Traverse(*pp_currentState, p_context, inputId, inputValue), p_context);
}


static inline void StateBranch_ProcSyncInput(const StateMachine_State_T ** pp_currentState, void * p_context, state_machine_input_t inputId, state_machine_value_t inputValue)
{
    StateBranch_ProcTransition(pp_currentState, State_TransitionFunction_Traverse(*pp_currentState, p_context, inputId, inputValue), p_context);
}


/******************************************************************************/
/*
    [p_StateActive][p_SubState]
*/
/******************************************************************************/

/******************************************************************************/
/*
    SubState as fixed 2nd level
    independent top level
    Transition/Ouput without traversal

    caller call _StateMachine_EndSubState to end loop
*/
/******************************************************************************/
/*
    Set SubState without defined transition input/event.
    Directly reachable. Cannot be a cousin or sibling.
    Does not traverse Exit/Entry.
*/
void _StateMachine_SetSubState(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
    if (p_stateMachine->p_SubState == NULL)
    {
        assert(p_newState != NULL);
        State_Init(&p_stateMachine->p_SubState, p_newState, p_stateMachine->CONST.P_CONTEXT); /* error on NULL  */
    }
    else
    {
        State_ProcTransition(&p_stateMachine->p_SubState, p_newState, p_stateMachine->CONST.P_CONTEXT); /* ignores null */
    }
}

/* ExitBranch */
inline void _StateMachine_EndSubState(StateMachine_T * p_stateMachine)
{
    p_stateMachine->p_SubState = NULL;
    // p_stateMachine->p_SubState = p_stateMachine->p_StateActive; /* return to top level state */
}

/*
    Proc inside p_stateMachine->p_StateActive->LOOP
*/
void _StateMachine_ProcSubState(StateMachine_T * p_stateMachine)
{
    // if (p_stateMachine->p_SubState != NULL) { State_ProcOutput(&p_stateMachine->p_SubState, p_stateMachine->CONST.P_CONTEXT); }
    if (p_stateMachine->p_SubState != NULL && (p_stateMachine->p_SubState != p_stateMachine->p_StateActive)) { State_ProcOutput(&p_stateMachine->p_SubState, p_stateMachine->CONST.P_CONTEXT); }
}

void _StateMachine_ProcSubStateInput(StateMachine_T * p_stateMachine, state_machine_input_t id, state_machine_value_t value)
{
    if (p_stateMachine->p_SubState != NULL) { p_stateMachine->p_SubState = p_stateMachine->p_StateActive; }
    State_ProcInput(&p_stateMachine->p_SubState, p_stateMachine->CONST.P_CONTEXT, id, value);
}

/* SubStae must accept the input, does not traverse. alternatively includ 1 level parent */
void StateMachine_ProcSubStateInput(StateMachine_T * p_stateMachine, state_machine_input_t id, state_machine_value_t value)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        _StateMachine_ProcSubStateInput(p_stateMachine, id, value);
        ReleaseCritical_Input(p_stateMachine);
    }
}




/******************************************************************************/
/*
    SubState as a branch
*/
/******************************************************************************/
/*   */
bool StateMachine_IsReachableBranch(const StateMachine_T * p_stateMachine, const StateMachine_State_T * p_state)
{
    return State_IsReachableBranch(StateMachine_GetActiveBranch(p_stateMachine), p_state);
}

/* State is in the active branch */
bool StateMachine_IsActiveBranch(const StateMachine_T * p_stateMachine, const StateMachine_State_T * p_state)
{
    return (p_stateMachine->p_SubState == NULL) ? (p_stateMachine->p_StateActive == p_state) : State_IsActiveBranch(p_stateMachine->p_SubState, p_state);
    // return (p_stateMachine->p_StateActive == p_state) || State_IsActiveBranch(p_stateMachine->p_SubState, p_state);
}


/*
    traverse transition
    procBranchTransition
    Cousin State with Traverse Entry/Exit

    p_SubState = p_StateActive ends the branch

    p_newState != NULL
    p_SubState == NULL => transition from top level
*/
void _StateMachine_SetBranch(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
    if (StateMachine_IsReachableBranch(p_stateMachine, p_newState) == true)
    {
        StateBranch_ProcTransition(&p_stateMachine->p_SubState, p_newState, p_stateMachine->CONST.P_CONTEXT); // p_SubState null will repeat entry on top
    }
}

void _StateMachine_SetBranchOf(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_commonAncestor, const StateMachine_State_T * p_newState)
{
    if ((StateMachine_IsActiveBranch(p_stateMachine, p_commonAncestor) == true) && State_IsDescendant(p_commonAncestor, p_newState) == true)
    {
        StateBranch_ProcTransitionThrough(&p_stateMachine->p_SubState, p_commonAncestor, p_newState, p_stateMachine->CONST.P_CONTEXT);
    }
}

/*
    except Top State, for cases where implementation include [_StateMachine_ProcBranch_SubState] in only some [p_StateActive->LOOP]
*/
void _StateMachine_ProcBranch_Nested(StateMachine_T * p_stateMachine)
{
    StateBranch_ProcOutput(&p_stateMachine->p_SubState, p_stateMachine->p_StateActive, p_stateMachine->CONST.P_CONTEXT);
}

void _StateMachine_ProcBranchInput(StateMachine_T * p_stateMachine, state_machine_input_t id, state_machine_value_t value)
{
    if (p_stateMachine->p_SubState == NULL) { p_stateMachine->p_SubState = p_stateMachine->p_StateActive; }
    StateBranch_ProcInput(&p_stateMachine->p_SubState, p_stateMachine->CONST.P_CONTEXT, id, value);
}

void _StateMachine_ProcBranchSyncInput(StateMachine_T * p_stateMachine)
{
    if (p_stateMachine->SyncInput != STATE_MACHINE_INPUT_ID_NULL)
    {
        _StateMachine_ProcBranchInput(p_stateMachine, p_stateMachine->SyncInput, p_stateMachine->SyncInputValue);
        p_stateMachine->SyncInput = STATE_MACHINE_INPUT_ID_NULL;
    }
}

/* up to root */
void StateMachine_ProcBranch(StateMachine_T * p_stateMachine)
{
    if (AcquireCritical_ISR(p_stateMachine) == true)
    {
        _StateMachine_ProcBranchSyncInput(p_stateMachine);
        StateBranch_ProcOutput(&p_stateMachine->p_SubState, NULL, p_stateMachine->CONST.P_CONTEXT);
        ReleaseCritical_ISR(p_stateMachine);
    }
}

void StateMachine_ProcBranchInput(StateMachine_T * p_stateMachine, state_machine_input_t id, state_machine_value_t value)
{
    if (AcquireCritical_Input(p_stateMachine) == true)
    {
        _StateMachine_ProcBranchInput(p_stateMachine, id, value);
        ReleaseCritical_Input(p_stateMachine);
    }
}


/******************************************************************************/
/*
    State direct inputs. Reject if it is not reachable from active state.

    inputs that only map to 1 state, reduce table size
    per state inputs, only need to check id.
*/
/******************************************************************************/

/*
    Set the SubState if it can be reached directly up and down the branch
    without traversing the tree
    Effective as branch if Proc is [ProcBranch]
*/
void StateMachine_EnterSubState(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState)
{
    if (AcquireSignal_Input(p_stateMachine) == true) /* Disable ProcState or entry maybe overwritten */
    {
        if (StateMachine_IsReachableBranch(p_stateMachine, p_newState) == true)
        {
            _StateMachine_SetSubState(p_stateMachine, p_newState);
        }

        ReleaseSignal_Input(p_stateMachine);
    }
}

void StateMachine_EnterSubStateWith(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState, StateMachine_Set_T setter, state_machine_value_t inputValue)
{
    // if (StateMachine_IsReachableBranch(p_stateMachine, p_newState) == true) /* preemptive check before lock */
    {
        if (AcquireSignal_Input(p_stateMachine) == true)
        {
            if (StateMachine_IsReachableBranch(p_stateMachine, p_newState) == true)
            {
                if (setter != NULL) { setter(p_stateMachine->CONST.P_CONTEXT, inputValue); }
                _StateMachine_SetSubState(p_stateMachine, p_newState);
            }

            ReleaseSignal_Input(p_stateMachine);
        }
    }
}

void StateMachine_StartCmd(StateMachine_T * p_stateMachine, const StateMachine_Cmd_T * p_cmd, state_machine_value_t inputValue)
{
    StateMachine_EnterSubStateWith(p_stateMachine, p_cmd->P_INITIAL, p_cmd->CMD, inputValue);
}


/*
    Effective for all States descending from the selected State
    i.e. the active SubState is below the selected State, in the active branch.
*/
void StateMachine_SetBranchValueWith(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_state, StateMachine_Set_T setter, state_machine_value_t value)
{
    if (AcquireSignal_ISR(p_stateMachine) == true)
    {
        if (StateMachine_IsActiveBranch(p_stateMachine, p_state) == true) { setter(p_stateMachine->CONST.P_CONTEXT, value); }
        ReleaseSignal_ISR(p_stateMachine);
    }
}



/******************************************************************************/
/*
    Linked Menus
*/
/******************************************************************************/
#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
void StateMachine_Menu_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_t input, state_machine_value_t inputValue)
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

void StateMachine_Menu_ProcFunction(StateMachine_T * p_stateMachine, state_machine_input_t input, state_machine_value_t inputValue)
{
    StateMachine_ProcInput(p_stateMachine, input, inputValue)
}

void StateMachine_Menu_ProcLoop(StateMachine_T * p_stateMachine)
{
    StateMachine_ProcState(p_stateMachine);
}
#endif
