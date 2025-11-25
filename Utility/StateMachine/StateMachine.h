#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_StateMachine.h" /* As submodule implementation */
#include "_StateMachine_Lock.h"
#include "_StateMachine_Tree.h"
#include "State.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <assert.h>

/******************************************************************************/
/*
    [StateMachine_TransitionInput]
    Input passed to the StateMachine alternatively to mapped to a State.
    Compile-time defined valid transitions - effectively defines [State] local [state_input_t]

    Functionally InputCmds that map to a single State
    Transition within Signal Lock

    reverse map [Transition]/[Input] to a [State] or set of [State]s
    compare [p_transition->P_START] to [p_stateMachine->p_ActiveState]
    Apply transition to State

    [StateMachine_InvokeTransition]
    Directly invoke. defined at compile time as known valid transition
*/
/******************************************************************************/
// typedef const struct StateMachine_Transition
/* Transition_T or State_TransitionCmd_T */
typedef const struct StateMachine_TransitionInput
{
    State_T * P_START; /* From/Start. Starting State known to accept this input at compile time. */
    State_Input_T TRANSITION; /* To/Next. Does not return NULL */
}
StateMachine_TransitionInput_T;

// typedef const struct State_TransitionPair
// {
//     State_T * P_START;
//     State_T * P_NEXT;
//      bool (OnTransition)(void * p_context,  state_value_t inputValue);
// }
// State_TransitionPair_T;

// typedef const struct StateMachine_MultiTransitionInput
// {
//     const State_Input_T TRANSITION;
//     State_T * const * const PP_VALID_LIST;
//     const uint8_t VALID_COUNT;
// }
// StateMachine_MultiTransitionInput_T;

/******************************************************************************/
/*
    StateMachine_Def / StateMachine_Table
    Stateless Machine
    Interface VTable Analogous
*/
/******************************************************************************/
typedef const struct StateMachine_Machine
{
    State_T * P_STATE_INITIAL;
    uint8_t TRANSITION_TABLE_LENGTH;  /* state_input_t count. Shared table length for all states, i.e. all states allocate for all inputs */
    /* Optional */
    State_T * P_STATE_FAULT; /* Optional. for base fault functions */
    // const State_T STATES[]; /* auto assign a stateid this way */
    // alternatively, simplified  implementation
    // const State_Input_T * const * const PP_TRANSITION_TABLE; [state_t][state_input_t]
    // meta select handler mode
    // // typedef struct State * (*StateMachine_TransitionFunction_T)(struct State *, void * p_context, state_input_t inputId, state_value_t inputValue);
}
StateMachine_Machine_T;


/* Compile warnings */
// #ifdef STATE_MACHINE_CONTEXT_AS_CONST
// typedef const void state_machine_context_t;
// #else
// typedef void state_machine_context_t;
// #endif

/*
    [StateMachine_T] - as interface instance for P_CONTEXT as subtype
        StateMachine_Machine_T instance - State_T Map + mutable context.
        In this case of the runtime state _is_ a mainly pointer to STATE in ROM.
*/
typedef const struct StateMachine
{
    /* shorthand passing and container_of */
    void * P_CONTEXT;                           /* Base Struct. Const during handling */
    StateMachine_Active_T * P_ACTIVE;           /* StateMachine "state" runtime data */
    const StateMachine_Machine_T * P_MACHINE;   /* Machine def. Const definition of state transition behaviors, via initial state */

    /*  */
    // const volatile uint32_t * P_TIMER;
    // state_value_t * P_SYNC_INPUTS; /* TRANSITION_TABLE_LENGTH */
    // State_T ** pp_ActiveStatesBuffer; alternative to recursive traverse down
    // const void * P_STATE_BUFFERS[STATE_COUNT]; /* a substate buffer for each top state */
}
StateMachine_T;

#define STATE_MACHINE_ACTIVE_ALLOC() (&(StateMachine_Active_T){0})

#define STATE_MACHINE_INIT(p_Context, p_Machine, p_Active) { .P_CONTEXT = (void *)(p_Context), .P_MACHINE = (p_Machine), .P_ACTIVE = (p_Active) }
#define STATE_MACHINE_INIT_ALLOC(p_Context, p_Machine) STATE_MACHINE_INIT(p_Context, p_Machine, STATE_MACHINE_ACTIVE_ALLOC())


/******************************************************************************/
/*!
    Public Functions
    Defined as static per compilation unit - per [StateMachine_T]
    Let context wrapper optimize away. Pass literals to _StateMachine. [_StateMachine] as common compilation unit
*/
/******************************************************************************/
static void StateMachine_Init(StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;
    _StateMachine_Reset(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
    Critical_ReleaseLock(&p_active->InputSignal);
}

/* Always async without exit */
static void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;
    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_Reset(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/******************************************************************************/
/*
    Top Level StateMachine
*/
/******************************************************************************/
/******************************************************************************/
/*
    Input Functions
    Handles Top level transitions only
        EXIT of current State plus ENTRY of new Top State
    [State_Input_T] must not return a substate.
*/
/******************************************************************************/
/* Transition immediately - Chaining input calls depending on State changes */
static void StateMachine_InputAsyncTransition(StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;
    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, inputId, inputValue);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/* Proc [TRANSITION_TABLE[id]] buffer Transition on next Sync ProcState */
static void StateMachine_InputSyncTransition(StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
{
    _StateMachine_ApplyInputSyncTransition(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT, inputId, inputValue);
}

/* Selection interface */
static inline void StateMachine_ApplyInput(StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
{
// #if defined(STATE_MACHINE_INPUT_ASYNC_TRANSITION)
    StateMachine_InputAsyncTransition(p_stateMachine, inputId, inputValue);
}

/* HSM Common */
/* Set SyncInput Proc InputFunction and Transition on next Sync ProcState */
static void StateMachine_SetInput(StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;

    /* Disables [ProcInput] portion of [Sync_ProcState]. When MultiThreaded + non atomic */
    if (_StateMachine_AcquireSyncInput(p_active) == true)
    {
        _StateMachine_SetSyncInput(p_active, inputId, inputValue);
        _StateMachine_ReleaseSyncInput(p_active);
    }
}

/*

*/
/* Set the State within signal guards. Without check on input. e.g. fault state */
static void StateMachine_ForceTransition(StateMachine_T * p_stateMachine, State_T * p_state)
{
    assert(p_state != NULL); /* compile time known state */

    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncTransition(p_active) == true)
    {
        _StateMachine_Transition(p_active, p_stateMachine->P_CONTEXT, p_state);
        _StateMachine_ReleaseAsyncTransition(p_active);
    }
}

/*
    Invoke a [Transition] or as a Command
*/
// static void StateMachine_InvokeTransition(StateMachine_T * p_stateMachine, State_T * p_start, State_Input_T transition, state_value_t inputValue)
static void StateMachine_InvokeTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_transition, state_value_t inputValue)
{
    assert(p_transition->TRANSITION != NULL);

    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncTransition(p_active) == true)
    {
        if (StateMachine_IsActiveState(p_active, p_transition->P_START) == true)
        {
            _StateMachine_TransitionTo(p_active, p_stateMachine->P_CONTEXT, p_transition->TRANSITION(p_stateMachine->P_CONTEXT, inputValue));
        }
        _StateMachine_ReleaseAsyncTransition(p_active);
    }
}

/* Convenience for inline call [StateMachine_InvokeTransition] */
// #define STATE_MACHINE_CMD(p_start, transition) ((StateMachine_TransitionInput_T) { .P_START = p_start, .TRANSITION = transition });


/******************************************************************************/
/*
    Value Accessor Interface
    Top level state only
*/
/******************************************************************************/
static inline state_value_t StateMachine_Access(StateMachine_T * p_stateMachine, state_input_t id, state_value_t valueK, state_value_t valueV)
{
    return _StateMachine_Access(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT, id, valueK, valueV);
}

static inline void StateMachine_Set(StateMachine_T * p_stateMachine, state_input_t id, state_value_t valueK, state_value_t valueV)
{
    _StateMachine_SetValue(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT, id, valueK, valueV);
}

static inline state_value_t StateMachine_Get(StateMachine_T * p_stateMachine, state_input_t id, state_value_t valueK)
{
    return _StateMachine_GetValue(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT, id, valueK);
}


/******************************************************************************/
/*
    [Hierarchical State Machine]
    [p_ActiveState][p_ActiveSubState]
*/
/******************************************************************************/
static void StateMachine_Branch_InputAsyncTransition(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;
    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcBranchInput(p_active, p_stateMachine->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

static void StateMachine_Branch_InputSyncTransition(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
{
    _StateMachine_ApplyBranchAsyncInput(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT, id, value);
}

/*
    Proc Input with Traversal from Leaf up
*/
static inline void StateMachine_Branch_ApplyInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
{
    StateMachine_Branch_InputAsyncTransition(p_stateMachine, id, value);
}

static inline void StateMachine_Branch_SetInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
{
    StateMachine_SetInput(p_stateMachine, id, value); /* common */
}

/*
    ActiveState is an Descendant of the selected State.

    result the input handle is any of P_START is in the active branch
    Transitions to the State of p_transition->TRANSITION, defined to be valid at compile time
*/
static void StateMachine_Branch_InvokeTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_transition, state_value_t value)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncTransition(p_active) == true)
    {
        if (StateMachine_IsActivePath(p_active, p_transition->P_START) == true) //todo move
        {
            assert(State_IsAncestorOrSelf(p_transition->P_START, p_active->p_ActiveState)); /* ensure substate is in sync with top level state */
            _StateMachine_TransitionBranchTo(p_active, p_stateMachine->P_CONTEXT, p_transition->TRANSITION(p_stateMachine->P_CONTEXT, value));
        }
        _StateMachine_ReleaseAsyncTransition(p_active);
    }
}

/*
    Invoke a Branch Transition when
        ActiveState is an Ancestor of the selected State.
*/
static void StateMachine_Branch_InvokeTransitionFrom(StateMachine_T * p_stateMachine, State_T * p_deepest, State_Input_T transition, state_value_t value)
{
    StateMachine_Active_T * const p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncTransition(p_active) == true)
    {
        if (State_IsAncestorOrSelf(p_deepest, StateMachine_GetActiveSubState(p_active)) == true)
        {
            _StateMachine_TransitionBranchTo(p_active, p_stateMachine->P_CONTEXT, transition(p_stateMachine->P_CONTEXT, value));
        }
        _StateMachine_ReleaseAsyncTransition(p_active);
    }
}


// static void StateMachine_RootFirst_InputAsyncTransition(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
// static void StateMachine_RootFirst_InputSyncTransition(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
// static void StateMachine_RootFirst_ApplyInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
// static void StateMachine_RootFirst_SetInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
// static void StateMachine_RootFirst_InvokeTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_transition, state_value_t value)
// static void StateMachine_RootFirst_InvokeTransitionFrom(StateMachine_T * p_stateMachine, State_T * p_deepest, State_Input_T transition, state_value_t value)

