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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <assert.h>

/******************************************************************************/
/*
    [StateMachine_TransitionCmd]
    Input passed to the StateMachine alternatively to mapped to a State.
    Compile-time defined valid transitions - effectively defines [State] local [state_input_t]

    Functionally InputCmds that map to a single State
    Transition within Signal Lock

    reverse map [Transition]/[Input] to a [State] or set of [State]s
    compare [p_transition->P_START] to [p_this->p_ActiveState]
    Apply transition to State

    [StateMachine_InvokeTransition]
    Directly invoke. defined at compile time as known valid transition
*/
/******************************************************************************/
typedef const struct StateMachine_TransitionCmd
{
    State_T * P_START; /* From/Start. Starting State known to accept this input at compile time. */
    State_Input_T NEXT; /* To/Next. Does not return NULL */
}
StateMachine_TransitionCmd_T;

/******************************************************************************/
/*
    StateMachine_Def / StateMachine_VTable
    Common def for multiple instances sharing the same transition pattern.
    Stateless Machine
    Interface VTable Analogous
*/
/******************************************************************************/
typedef const struct StateMachine_Machine
{
    State_T * P_STATE_INITIAL;
    uint8_t TRANSITION_TABLE_LENGTH;  /* state_input_t count. Shared table length for all states, i.e. all states allocate for all inputs */
    /* Optional */
    // State_T * P_STATE_FAULT; /* Optional. for base fault functions */
    // const State_T STATES[]; /* Root states. auto assign a stateid this way */
    // alternatively, simplified  implementation
    // const State_Input_T * const * const PP_TRANSITION_TABLE; [state_t][state_input_t]
}
StateMachine_Machine_T;


/*
    [StateMachine_T] - as interface instance for P_CONTEXT as subtype
        StateMachine_Machine_T instance - State_T Map + mutable context.
        In this case of the runtime state _is_ a mainly pointer to STATE in ROM.
*/
typedef const struct StateMachine
{
    /* shorthand passing and container_of */
    void * P_CONTEXT;                           /* Base Struct. Const during handling. P_INTERNAL */
    StateMachine_Active_T * P_ACTIVE;           /* StateMachine "state" runtime data */
    const StateMachine_Machine_T * P_MACHINE;   /* Machine def. Const definition of state transition behaviors, via initial state */

    /*  */
    // const volatile uint32_t * P_TIMER;
    // state_value_t * P_SYNC_INPUTS; /* TRANSITION_TABLE_LENGTH */
    // State_T ** pp_ActiveStatesBuffer; alternative to recursive traverse down
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
static void StateMachine_Init(StateMachine_T * p_this)
{
    _StateMachine_Reset(p_this->P_ACTIVE, p_this->P_CONTEXT, p_this->P_MACHINE->P_STATE_INITIAL);
    Critical_ReleaseLock(&p_this->P_ACTIVE->InputSignal);
}

/* Always async without exit */
static void StateMachine_Reset(StateMachine_T * p_this)
{
    if (_StateMachine_AcquireAsyncInput(p_this->P_ACTIVE) == true)
    {
        _StateMachine_Reset(p_this->P_ACTIVE, p_this->P_CONTEXT, p_this->P_MACHINE->P_STATE_INITIAL);
        _StateMachine_ReleaseAsyncInput(p_this->P_ACTIVE);
    }
}

/******************************************************************************/
/*
    [StateMachine]
*/
/******************************************************************************/
/******************************************************************************/
/*
    Input Functions
    Handles Top level transitions only for flat StateMachine.
        EXIT of current State plus ENTRY of new Top State
*/
/******************************************************************************/
/* Transition immediately - Chaining input calls depending on State changes */
static void StateMachine_InputAsyncTransition(StateMachine_T * p_this, state_input_t inputId, state_value_t inputValue)
{
    if (_StateMachine_AcquireAsyncInput(p_this->P_ACTIVE) == true)
    {
        _StateMachine_CallInput(p_this->P_ACTIVE, p_this->P_CONTEXT, inputId, inputValue);
        _StateMachine_ReleaseAsyncInput(p_this->P_ACTIVE);
    }
}

/* Proc [TRANSITION_TABLE[id]] buffer Transition on next Sync ProcState */
static void StateMachine_InputSyncTransition(StateMachine_T * p_this, state_input_t inputId, state_value_t inputValue)
{
    _StateMachine_ApplyInputSyncTransition(p_this->P_ACTIVE, p_this->P_CONTEXT, inputId, inputValue);
}

/* Selection interface */
static inline void StateMachine_ApplyInput(StateMachine_T * p_this, state_input_t inputId, state_value_t inputValue)
{
// #if defined(STATE_MACHINE_INPUT_ASYNC_TRANSITION)
    StateMachine_InputAsyncTransition(p_this, inputId, inputValue);
}

/* HSM Common */
/* Set SyncInput Proc InputFunction and Transition on next Sync ProcState */
static void StateMachine_SetInput(StateMachine_T * p_this, state_input_t inputId, state_value_t inputValue)
{
    /* Disables [ProcInput] portion of [Sync_ProcState]. When MultiThreaded + non atomic */
    if (_StateMachine_AcquireSyncInput(p_this->P_ACTIVE) == true)
    {
        _StateMachine_SetSyncInput(p_this->P_ACTIVE, inputId, inputValue);
        _StateMachine_ReleaseSyncInput(p_this->P_ACTIVE);
    }
}

/*

*/
/* Set the State within signal guards. Without check on input. e.g. fault state */
static void StateMachine_ForceTransition(StateMachine_T * p_this, State_T * p_state)
{
    assert(p_state != NULL); /* compile time known state */

    if (_StateMachine_AcquireAsyncTransition(p_this->P_ACTIVE) == true)
    {
        _StateMachine_TransitionTo(p_this->P_ACTIVE, p_this->P_CONTEXT, p_state);
        _StateMachine_ReleaseAsyncTransition(p_this->P_ACTIVE);
    }
}

/*
    Invoke a [Transition] or as a Command
    Efficiently check transition mapped to a single starting state.
*/
static void StateMachine_InvokeTransition(StateMachine_T * p_this, StateMachine_TransitionCmd_T * p_transition, state_value_t inputValue)
{
    assert(p_transition->NEXT != NULL);

    if (_StateMachine_AcquireAsyncTransition(p_this->P_ACTIVE) == true)
    {
        _StateMachine_InvokeTransition(p_this->P_ACTIVE, p_this->P_CONTEXT, p_transition->P_START, p_transition->NEXT, inputValue);
        _StateMachine_ReleaseAsyncTransition(p_this->P_ACTIVE);
    }
}


/******************************************************************************/
/*
    Value Accessor Interface
*/
/******************************************************************************/
static inline state_value_t StateMachine_Cmd(StateMachine_T * p_this, state_cmd_t id, state_value_t value) { return _StateMachine_Cmd(p_this->P_ACTIVE, p_this->P_CONTEXT, id, value); }

static inline void StateMachine_Set(StateMachine_T * p_this, state_accessor_t id, state_value_t field, state_value_t value) { _StateMachine_SetValue(p_this->P_ACTIVE, p_this->P_CONTEXT, id, field, value); }
static inline state_value_t StateMachine_Get(StateMachine_T * p_this, state_accessor_t id, state_value_t field) { return _StateMachine_GetValue(p_this->P_ACTIVE, p_this->P_CONTEXT, id, field); }


/******************************************************************************/
/*
    [Hierarchical State Machine]
    [p_ActiveState][p_ActiveSubState]
*/
/******************************************************************************/
/******************************************************************************/
/*
    Branch Process - Traversal from Leaf up
*/
/******************************************************************************/
static void StateMachine_Tree_InputAsyncTransition(StateMachine_T * p_this, state_input_t id, state_value_t value)
{
    if (_StateMachine_AcquireAsyncInput(p_this->P_ACTIVE) == true)
    {
        _StateMachine_Branch_CallInput(p_this->P_ACTIVE, p_this->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_this->P_ACTIVE);
    }
}

static void StateMachine_Tree_InputSyncTransition(StateMachine_T * p_this, state_input_t id, state_value_t value)
{
    _StateMachine_Branch_ApplyAsyncInput(p_this->P_ACTIVE, p_this->P_CONTEXT, id, value);
}

/* Default select */
static inline void StateMachine_Tree_Input(StateMachine_T * p_this, state_input_t id, state_value_t value)
{
    StateMachine_Tree_InputAsyncTransition(p_this, id, value);
}

static inline void StateMachine_Tree_SetInput(StateMachine_T * p_this, state_input_t id, state_value_t value)
{
    StateMachine_SetInput(p_this, id, value); /* common */
}

/*
    ActiveState is an Descendant of the selected State.
    Traverse

    result the input handle is any of P_START is in the active branch
    Transitions to the State of p_transition->NEXT, defined to be valid at compile time
*/
static void StateMachine_Tree_InvokeTransition(StateMachine_T * p_this, StateMachine_TransitionCmd_T * p_transition, state_value_t value)
{
    if (_StateMachine_AcquireAsyncTransition(p_this->P_ACTIVE) == true)
    {
        _StateMachine_InvokeTraverseTransition(p_this->P_ACTIVE, p_this->P_CONTEXT, p_transition->P_START, p_transition->NEXT, value);
        _StateMachine_ReleaseAsyncTransition(p_this->P_ACTIVE);
    }
}

/*

*/
static void StateMachine_Tree_InputRootFirst(StateMachine_T * p_this, state_input_t id, state_value_t value)
{
    if (_StateMachine_AcquireAsyncInput(p_this->P_ACTIVE) == true)
    {
        _StateMachine_RootFirst_CallInput(p_this->P_ACTIVE, p_this->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_this->P_ACTIVE);
    }
}

static void StateMachine_Tree_InputRootOnly(StateMachine_T * p_this, state_input_t id, state_value_t value)
{
    if (_StateMachine_AcquireAsyncInput(p_this->P_ACTIVE) == true)
    {
        _StateMachine_RootOnly_CallInput(p_this->P_ACTIVE, p_this->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_this->P_ACTIVE);
    }
}


