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
    @file   _StateMachine_Tree.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "_StateMachine_Tree.h"

/******************************************************************************/
/*
    [Hierarchical State Machine]
    [p_ActiveState][p_ActiveSubState]
    SubState as a branch
*/
/******************************************************************************/

/*
    Traverse Transition Common

    Implementation defined with known valid [p_state]
    Proc p_state->P_TOP->ENTRY if State_CommonAncestorOf is NULL

    traverse Entry/Exit functions
    sets [LeafState] to [p_state]
    sets [RootState] to [p_state->P_TOP]. no change if [p_state->P_TOP] is the same
*/
void _StateMachine_TraverseTransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    State_TraverseEntryExit(StateMachine_GetLeafState(p_active), p_state, p_context);
    p_active->p_ActiveState = p_state;
}

void _StateMachine_TraverseTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    if (p_state != NULL) { _StateMachine_TraverseTransitionTo(p_active, p_context, p_state); }
}

/*
    For inputs/transition mapped to specific states, rather than all states
    check if p_start is included in the active path
*/
void _StateMachine_InvokeTraverseTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_start, State_Input_T input, state_value_t value)
{
    if (StateMachine_IsActivePath(p_active, p_start) == true) { _StateMachine_TraverseTransition(p_active, p_context, input(p_context, value)); }
}


/******************************************************************************/
/*
    Transition Function Mapping
    Walk up Leaf -> Root
*/
/******************************************************************************/
static inline State_T * OfState(const StateMachine_Active_T * p_active, void * p_context)
{
    return State_TransitionOfOutputUp(StateMachine_GetLeafState(p_active), p_context);
}

static inline State_T * OfInput(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    return State_TransitionOfInputUp(StateMachine_GetLeafState(p_active), p_context, id, value);
}

/*
    Proc Branch State
*/
void _StateMachine_Branch_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TraverseTransition(p_active, p_context, OfState(p_active, p_context));
}

/* Transition immediately */
void _StateMachine_Branch_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_TraverseTransition(p_active, p_context, OfInput(p_active, p_context, id, value));
}

/*
*/
void _StateMachine_Branch_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    // assert(__builtin_clz(p_active->SyncInputMask) < 32 STATE_TRANSITION_TABLE_LENGTH_MAX);
    for (uint32_t inputMask = p_active->SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
    {
        state_input_t input = __builtin_ctz(inputMask);
        // assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
        _StateMachine_Branch_CallInput(p_active, p_context, input, p_active->SyncInputs[input]); /* may update [p_ActiveState] */
    }
    p_active->SyncInputMask = 0UL;
}

// inline void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context, StateMachine_SyncInput_T * p_syncInputs)
// {
//     __StateMachine_ProcSyncInput(p_active, p_context, p_syncInputs, OfInput);
// }


/* Buffered from AsyncInput */
void _StateMachine_Branch_ProcSyncTransition(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TraverseTransition(p_active, p_context, p_active->p_SyncNextState);
    p_active->p_SyncNextState = NULL; /* Clear next state */
}

/*
    Input
    AsyncInputSyncTransition
*/
void _StateMachine_Branch_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
   _StateMachine_SetSyncTransition(p_active, OfInput(p_active, p_context, id, value));
}



/******************************************************************************/
/*
    Proc State
*/
/******************************************************************************/
/* Including Top level state */
void _StateMachine_Branch_Proc(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_Branch_ProcSyncTransition(p_active, p_context); /* check for an buffered transition first */
    _StateMachine_Branch_ProcSyncInput(p_active, p_context);
    _StateMachine_Branch_ProcSyncOutput(p_active, p_context);
}


/******************************************************************************/
/*
    RootFirst once, then Leaf up excluding Root
*/
/******************************************************************************/
/******************************************************************************/
/*
    Root-first:
        1. Check root once (preemptive)
        2. If none, then leaf -> ancestors excluding root

*/
/******************************************************************************/
static inline State_T * RootFirstOutput(StateMachine_Active_T * p_active, void * p_context)
{
    return State_TransitionOfOutput_RootFirst(StateMachine_GetLeafState(p_active), p_context);
}

static inline State_T * RootFirstInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    return State_TransitionOfInput_RootFirst(StateMachine_GetLeafState(p_active), p_context, id, value);
}

/*
    Same as Flat OfState if ActiveState is macro as RootState
*/
static inline State_T * RootOutput(StateMachine_Active_T * p_active, void * p_context)
{
    return State_TransitionOfOutput_AsTop(StateMachine_GetRootState(p_active), p_context);
}

static inline State_T * RootInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    return State_TransitionOfInput_AsTop(StateMachine_GetRootState(p_active), p_context, id, value);
}

/*

*/
void _StateMachine_RootFirst_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TraverseTransition(p_active, p_context, RootFirstOutput(p_active, p_context));
}

void _StateMachine_RootFirst_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_TraverseTransition(p_active, p_context, RootFirstInput(p_active, p_context, id, value));
}

/* Apply the input to Root only. check top state only. traverse on transition compatibility */
void _StateMachine_RootOnly_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_TraverseTransition(p_active, p_context, RootInput(p_active, p_context, id, value));
    // _StateMachine_Transition
}

void _StateMachine_RootFirst_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    for (uint32_t inputMask = p_active->SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
    {
        state_input_t input = __builtin_ctz(inputMask);
        _StateMachine_RootFirst_CallInput(p_active, p_context, input, p_active->SyncInputs[input]);
    }
    p_active->SyncInputMask = 0UL;
}

/*
    Pending transition (buffered async):
    Apply root-level or branch target.
*/
void _StateMachine_RootFirst_ProcSyncTransition(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TraverseTransition(p_active, p_context, p_active->p_SyncNextState);
    p_active->p_SyncNextState = NULL;
}

/*
    Inputs may be selected per call
    Optional
*/
/*
    Async input buffering (root-first selection):
    Store only; application deferred to processing phase.
    Last input wins
*/
void _StateMachine_RootFirst_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_SetSyncTransition(p_active, RootFirstInput(p_active, p_context, id, value));
}

/*
    Full root-first cycle:
        Apply pending
        Output transitions (root-first then branch)
        Sync inputs (root-first per event)
*/
void _StateMachine_RootFirst_Proc(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_RootFirst_ProcSyncTransition(p_active, p_context);
    _StateMachine_RootFirst_ProcSyncOutput(p_active, p_context); /* LOOP first, tighter constrains. results from sync inputs delayed 1 cycle */
    _StateMachine_RootFirst_ProcSyncInput(p_active, p_context);
}




/*
    NestedBranch
    Place in [p_ActiveState->LOOP] selectively.
    Proc traverse up excluding Top State.
    States with ProcBranch_Nested need [p_ActiveSubState] to be cleared accordingly

    Within _StateMachine_ProcState ->
    _StateMachine_ProcSyncNextState
    _StateMachine_ProcSyncInput
    _StateMachine_ProcSyncOutput
        _StateMachine_Branch_Proc_Nested

    _StateMachine_ProcState consumes buffer values or use 2nd buffers

    handles SubState returns from SubStates only
        i.e State DEPTH > 0, are processed with Traverse, and can return SubState

    possibly check pending substate
*/
// void _StateMachine_NestedBranch_Proc(StateMachine_Active_T * p_active, void * p_context)
// {
//     _StateMachine_Branch_ProcSyncOutput(p_active, StateMachine_GetRootState(p_active), p_context);
// }

// void _StateMachine_NestedBranch_Proc(StateMachine_Active_T * p_active, void * p_context)
// {
//     _StateMachine_Branch_ProcSyncOutput(p_active, StateMachine_GetRootState(p_active), p_context);
// }

// top state select passthrough
// void _StateMachine_NestedBranch_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
// {
//     _StateMachine_TraverseTransition(p_active, p_context, OfInput(p_active, p_context, id, value));
//      p_next = State_TransitionOfInputUpTo(StateMachine_GetLeafState(p_active), StateMachine_GetRootState(p_active), p_context, id, value); }
// }

