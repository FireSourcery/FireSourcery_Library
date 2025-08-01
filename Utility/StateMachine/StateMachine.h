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
    [StateMachine Input]
    Input passed to the StateMachine alternative to mapped to a State.
    effectively defines [State] local [state_input_t]
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
typedef const struct StateMachine_TransitionInput
{
    State_T * P_START; /* From/Source. Starting State known to accept this input at compile time. */
    State_Input_T TRANSITION; /* To/Destination. Does not return NULL */ /* Effectively P_DEST, ON_TRANSITION */
    // State_T * P_START;
    // const State_Input_T TO_NEXT;
}
StateMachine_TransitionInput_T;

/* Convenience for inline call [StateMachine_InvokeTransition] */
// #define STATE_MACHINE_CMD(p_start, transition) ((StateMachine_TransitionInput_T) { .P_START = p_start, .TRANSITION = transition });

// typedef const struct State_TransitionPair
// {
//     State_T * P_START;
//     State_T * P_NEXT;
// }
// State_TransitionPair_T;

// typedef const struct StateMachine_Cmd
// typedef const struct StateMachine_MultiTransitionInput
// {
//     const State_Input_T TRANSITION; /* To/Destination. Does not return NULL */
//     State_T * const * const PP_VALID_LIST; /* From/Source. Starting State known to accept this input at compile time. */
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
    [StateMachine_Context_T] - as Context around mutable state
        In this case of the runtime state _is_ a mainly pointer to STATE in ROM.
    [StateMachine_T] - as interface instance for P_CONTEXT as subtype
        StateMachine_Machine_T instance - State_T Map with mutable context.
*/
typedef const struct StateMachine
{
    /* shorthand passing and container_of */
    void * P_CONTEXT;                           /* Base Struct */
    StateMachine_Active_T * P_ACTIVE;           /* StateMachine "state" runtime data */
    const StateMachine_Machine_T * P_MACHINE;   /* Machine def. Const definition of state transition behaviors, via initial state */

    /*  */
    // state_value_t * P_SYNC_INPUTS; /* TRANSITION_TABLE_LENGTH */
    // State_T ** pp_ActiveStatesBuffer; alternative to recursive traverse down
    // const void * P_STATE_BUFFERS[STATE_COUNT]; /* a substate buffer for each top state */
}
StateMachine_T;

#define STATE_MACHINE_ACTIVE_ALLOC() (&(StateMachine_Active_T){0})

#define STATE_MACHINE_INIT(p_Context, p_Machine, p_Active) { .P_CONTEXT = (void *)(p_Context), .P_MACHINE = (p_Machine), .P_ACTIVE = (p_Active) }
#define STATE_MACHINE_ALLOC(p_Context, p_Machine) STATE_MACHINE_INIT(p_Context, p_Machine, STATE_MACHINE_ACTIVE_ALLOC())


/******************************************************************************/
/*!
    Public Functions
    Let context optimize. Pass literals to _StateMachine as common compilation unit
*/
/******************************************************************************/
static void StateMachine_Init(StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
    p_active->SyncInputMask = 0UL;
    Critical_ReleaseLock(&p_active->InputSignal);
    _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
}

static void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    p_stateMachine->P_ACTIVE->p_SyncNextState = p_stateMachine->P_MACHINE->P_STATE_INITIAL;
    // StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
    // if (_StateMachine_AcquireAsyncInput(p_active) == true)
    // {
    //     _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
    //     _StateMachine_ReleaseAsyncInput(p_active);
    // }
}

/******************************************************************************/
/*
    Top Level StateMachine
*/
/******************************************************************************/
/*
    Input Functions
    Handles Top level transitions only
    [State_Input_T] must not return a substate.
*/
static void StateMachine_ProcInput(StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcAsyncInput(p_active, p_stateMachine->P_CONTEXT, inputId, inputValue);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

static void StateMachine_SetInput(StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    /* Disables [ProcInput] portion of [Sync_ProcState]. */
    if (_StateMachine_AcquireSyncInput(p_active) == true)
    {
        _StateMachine_SetSyncInput(p_active, inputId, inputValue);
        _StateMachine_ReleaseSyncInput(p_active);
    }
}

/* Set the State within signal guards. Without check on input. e.g. fault state */
static void StateMachine_ForceTransition(StateMachine_T * p_stateMachine, State_T * p_state)
{
    assert(p_state != NULL); /* compile time known state */

    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_Transition(p_active, p_stateMachine->P_CONTEXT, p_state);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/*
    Invoke a [Transition] or as a Command
*/
static void StateMachine_InvokeTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_transition, state_value_t inputValue)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (StateMachine_IsActiveState(p_active, p_transition->P_START) == true)
        {
            assert(p_transition->TRANSITION != NULL);
            _StateMachine_TransitionTo(p_active, p_stateMachine->P_CONTEXT, p_transition->TRANSITION(p_stateMachine->P_CONTEXT, inputValue));
        }

        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

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
    [SubState] as fixed 2nd level [p_ActiveSubState]
    independent top level
    Transition/Ouput without traversal

    [p_ActiveSubState] may be == p_ActiveState
    [p_ActiveState] is always a Top level state, always DEPTH == 0

    caller call _StateMachine_EndSubState to end loop
*/
/******************************************************************************/
static void StateMachine_ProcSubStateInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcSubStateInput(p_active, p_stateMachine->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}


/******************************************************************************/
/*
    [Hierarchical State Machine]
    [p_ActiveState][p_ActiveSubState]
*/
/******************************************************************************/
static void StateMachine_ProcBranchInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcBranchAsyncInput(p_active, p_stateMachine->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/*
    ActiveState is an Descendant of the selected State.

    result the input handle is any of P_START is in the active branch
    Transitions to the State of p_transition->TRANSITION, defined to be valid at compile time
*/
static void StateMachine_InvokeBranchTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_transition, state_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (StateMachine_IsActiveBranch(p_active, p_transition->P_START) == true)
        {
            assert(State_IsActiveBranch(p_transition->P_START, p_stateMachine->P_ACTIVE->p_ActiveState)); /* ensure substate is in sync with top level state */

            _StateMachine_TraverseTransitionTo(p_active, p_stateMachine->P_CONTEXT, p_transition->TRANSITION(p_stateMachine->P_CONTEXT, value));
        }
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/*
    Invoke a Branch Transition when
        ActiveState is an Ancestor of the selected State.
*/
static void StateMachine_InvokeBranchTransitionFrom(StateMachine_T * p_stateMachine, State_T * p_deepest, State_Input_T transition, state_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (State_IsActiveBranch(p_deepest, StateMachine_GetActiveSubState(p_active)) == true)
        {
            _StateMachine_TraverseTransitionTo(p_active, p_stateMachine->P_CONTEXT, transition(p_stateMachine->P_CONTEXT, value));
        }
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
// extern void StateMachine_Init(StateMachine_T * p_stateMachine);
// extern void StateMachine_Reset(StateMachine_T * p_stateMachine);

// extern void StateMachine_ForceTransition(StateMachine_T * p_stateMachine, State_T * p_state);
// extern void StateMachine_InvokeTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_input, state_value_t inputValue);

// extern void StateMachine_ProcSubStateInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value);

// extern void StateMachine_ProcBranchInput(StateMachine_T * p_stateMachine, state_input_t id, state_value_t value);
// extern void StateMachine_InvokeBranchTransition(StateMachine_T * p_stateMachine, StateMachine_TransitionInput_T * p_transition, state_value_t value);



