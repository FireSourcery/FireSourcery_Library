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
// static inline State_T * TransitionFunctionOnBranchState(const StateMachine_Active_T * p_active, void * p_context)
// {
//     return _State_TraverseTransitionOfOutput(StateMachine_GetLeafState(p_active), p_context, 0U);
// }

static inline State_T * TransitionFunctionOnBranchState(const StateMachine_Active_T * p_active, void * p_context, uint8_t stopLevel)
{
    return _State_TraverseTransitionOfOutput(StateMachine_GetLeafState(p_active), p_context, stopLevel);
}

static inline State_T * TransitionFunctionOnBranchInput(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    return State_TraverseTransitionOfInput(StateMachine_GetLeafState(p_active), p_context, id, value);
}

/*
    Branch Transitions

    Implementation defined with known valid [p_state]
    Proc p_state->P_TOP->ENTRY if State_CommonAncestorOf is NULL

    [p_ActiveSubState] == NULL => start from [p_ActiveState]
    traverse Entry/Exit functions
    sets [p_ActiveSubState] to [p_state]
    sets [p_ActiveState] to [p_state->P_TOP]. no change if [p_state->P_TOP] is the same
*/
void _StateMachine_Branch_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    State_TraverseOnTransition(StateMachine_GetLeafState(p_active), p_state, p_context);
    p_active->p_ActiveSubState = p_state;
    p_active->p_ActiveState = _State_GetRoot(p_state);
}

void _StateMachine_Branch_TransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    if (p_state != NULL) { _StateMachine_Branch_Transition(p_active, p_context, p_state); }
}


/*
    Proc Branch State
*/
/* traversing up for now */
void _StateMachine_Branch_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context, uint8_t stopLevel)
{
    _StateMachine_Branch_TransitionTo(p_active, p_context, TransitionFunctionOnBranchState(p_active, p_context, stopLevel));
}

/* Transition immediately */
/* Optional call with AsyncInput */
void _StateMachine_Branch_ProcInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_Branch_TransitionTo(p_active, p_context, TransitionFunctionOnBranchInput(p_active, p_context, id, value));
}

void _StateMachine_Branch_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    // assert(__builtin_clz(p_active->SyncInputMask) < 32 STATE_TRANSITION_TABLE_LENGTH_MAX);
    for (uint32_t inputMask = p_active->SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
    {
        state_input_t input = __builtin_ctz(inputMask);
        // assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
        _StateMachine_Branch_ProcInput(p_active, p_context, input, p_active->SyncInputs[input]); /* may update [p_ActiveState] */
    }
    p_active->SyncInputMask = 0UL;
}

// inline void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context, StateMachine_SyncInput_T * p_syncInputs)
// {
//     __StateMachine_ProcSyncInput(p_active, p_context, p_syncInputs, TransitionFunctionOnBranchInput);
// }


/* Buffered from AsyncInput */
void _StateMachine_Branch_ProcPendingTransition(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_Branch_TransitionTo(p_active, p_context, p_active->p_SyncNextState);
    p_active->p_SyncNextState = NULL; /* Clear next state */
}

/*
    Input
*/
void _StateMachine_Branch_ApplyAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
   _StateMachine_SetSyncTransition(p_active, p_context, TransitionFunctionOnBranchInput(p_active, p_context, id, value));
}

/******************************************************************************/
/*
    Proc State
*/
/******************************************************************************/
/* Including Top level state */
void _StateMachine_Branch_Proc(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_Branch_ProcPendingTransition(p_active, p_context); /* check for an buffered transition first */
    _StateMachine_Branch_ProcSyncInput(p_active, p_context);
    _StateMachine_Branch_ProcSyncOutput(p_active, p_context, 0U);
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
void _StateMachine_Branch_Proc_Nested(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_Branch_ProcSyncOutput(p_active, p_context, 1U);
}


/******************************************************************************/
/*
    RootFirst once, then Leaf up excluding Root
*/
/******************************************************************************/
/******************************************************************************/
/*
    Hybrid usage

    ProcInput function may be selected per [state_input_t] call.
        StateMachine_ApplyInput -   ignores substate exit functions.
        StateMachine_Branch_ApplyInput
        opt StateMachine_ProcRootFirstInput - 0, n to 1, early return on root transition. optional

    ProcRootFirstState requires unique implementation,
        common buffers must be consumed by first proc
        Early return functions slightly more optimal

        _StateMachine_ProcState(p_active, p_context);
        _StateMachine_ProcBranchLeafToRoot(p_active, p_context);

    ProcState
        StateMachine_ProcState
        StateMachine_ProcBranch - n to 0
        StateMachine_ProcRootFirst - 0, n to 1
*/
/******************************************************************************/
// static inline State_T * RootFirstOutput(StateMachine_Active_T * p_active, void * p_context)
// {
//     State_T * p_next = State_TransitionOfOutput_AsTop(StateMachine_GetRootState(p_active), p_context);
//     if (p_next == NULL) { p_next = _State_TraverseTransitionOfOutput(StateMachine_GetLeafState(p_active), p_context, 1U); }
//     return p_next;
// }

// static inline State_T * RootFirstInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
// {
//     State_T * p_next = State_TransitionOfInput_AsTop(StateMachine_GetRootState(p_active), p_context, id, value);
//     if (p_next == NULL) { p_next = _State_TraverseTransitionOfInput(StateMachine_GetLeafState(p_active), p_context, 1U, id, value); }
//     return p_next;
// }

void _StateMachine_RootFirst_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    // // State_TraverseOnTransition(StateMachine_GetLeafState(p_active), p_state, p_context); /* if generalizing multi preemptive level, or per state exit is needed */
    // State_Exit(p_active->p_ActiveState, p_context);
    // State_Entry(p_state, p_context);
    // p_active->p_ActiveSubState = p_state;
    // p_active->p_ActiveState = _State_GetRoot(p_state);

    if (p_state != NULL)
    {
        if (p_state->DEPTH == 0U) { _StateMachine_Transition(p_active, p_context, p_state); } /* Pure top-level transition */
        else { _StateMachine_Branch_Transition(p_active, p_context, p_state); }  /* Treat as hierarchical target – traverse from current leaf */
    }
}


/*
    Root-first output:
    1. Check root once (preemptive)
    2. If none, then leaf -> ancestors excluding root (stopLevel=1)
*/
void _StateMachine_RootFirst_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context)
{
    State_T * p_next = State_TransitionOfOutput_AsTop(StateMachine_GetRootState(p_active), p_context);
    if (p_next == NULL) { p_next = _State_TraverseTransitionOfOutput(StateMachine_GetLeafState(p_active), p_context, 1U); } /* Skiped if Root transitions */
    _StateMachine_RootFirst_Transition(p_active, p_context, p_next);
}

/*
    Root-first input (single event):
    1. Root handler first
    2. If none, traverse leaf -> up (exclude root)
*/
void _StateMachine_RootFirst_ProcInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    State_T * p_next = State_TransitionOfInput_AsTop(StateMachine_GetRootState(p_active), p_context, id, value);
    if (p_next == NULL) { p_next = _State_TraverseTransitionOfInput(StateMachine_GetLeafState(p_active), p_context, 1U, id, value); }
    _StateMachine_RootFirst_Transition(p_active, p_context, p_next);
}

void _StateMachine_RootFirst_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    // assert(__builtin_clz(p_active->SyncInputMask) < STATE_TRANSITION_TABLE_LENGTH_MAX);
    for (uint32_t inputMask = p_active->SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
    {
        state_input_t input = __builtin_ctz(inputMask);
        _StateMachine_RootFirst_ProcInput(p_active, p_context, input, p_active->SyncInputs[input]);
    }
    p_active->SyncInputMask = 0UL;
}

/*
    Pending transition (buffered async):
    Apply root-level or branch target.
*/
void _StateMachine_RootFirst_ProcPendingTransition(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_RootFirst_Transition(p_active, p_context, p_active->p_SyncNextState);
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
    _StateMachine_SetSyncTransition(p_active, p_context, State_TransitionOfInput_AsTop(StateMachine_GetRootState(p_active), p_context, id, value));
    if (p_active->p_SyncNextState == NULL)
    {
        p_active->p_SyncNextState = _State_TraverseTransitionOfInput(StateMachine_GetLeafState(p_active), p_context, 1U, id, value);
    }
}

/*
    Full root-first cycle:
        Apply pending
        Output transitions (root-first then branch)
        Sync inputs (root-first per event)
*/
void _StateMachine_RootFirst_Proc(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_RootFirst_ProcPendingTransition(p_active, p_context);
    _StateMachine_RootFirst_ProcSyncOutput(p_active, p_context); /* LOOP first, tighter constrains. results from sync inputs delayed 1 cycle */
    _StateMachine_RootFirst_ProcSyncInput(p_active, p_context);
}