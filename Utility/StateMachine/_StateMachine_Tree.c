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
static inline State_T * TransitionFunctionOfBranchState(const StateMachine_Active_T * p_active, State_T * p_end, void * p_context)
{
    return State_TraverseTransitionOfOutput(StateMachine_GetLeafState(p_active), p_end, p_context);
}

static inline State_T * TransitionFunctionOfBranchInput(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
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
void _StateMachine_TraverseTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    State_TraverseTransition(StateMachine_GetLeafState(p_active), p_state, p_context);
    p_active->p_ActiveSubState = p_state;
    p_active->p_ActiveState = _State_GetRoot(p_state);
}

void _StateMachine_TraverseTransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    if (p_state != NULL) { _StateMachine_TraverseTransition(p_active, p_context, p_state); }
}

// void _StateMachine_ProcBranchSyncOutputUpTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_end)
// void _StateMachine_ProcBranchSyncOutput(StateMachine_Active_T * p_active, void * p_context)

/* traversing up for now */
void _StateMachine_ProcBranchSyncOutput(StateMachine_Active_T * p_active, void * p_context, State_T * p_end)
{
    _StateMachine_TraverseTransitionTo(p_active, p_context, TransitionFunctionOfBranchState(p_active, p_end, p_context));
}

void _StateMachine_ProcBranchInputTransition(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_TraverseTransitionTo(p_active, p_context, TransitionFunctionOfBranchInput(p_active, p_context, id, value));
}

void _StateMachine_ProcBranchSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    uint32_t inputMask = p_active->SyncInputMask;

    while (inputMask != 0UL)
    {
        state_input_t input = __builtin_ctz(inputMask);
        _StateMachine_ProcBranchInputTransition(p_active, p_context, input, p_active->SyncInputs[input]); /* may update [p_active->p_ActiveState]. optionally return early */
        inputMask &= (inputMask - 1);
    }

    p_active->SyncInputMask = 0UL;
}

void _StateMachine_ProcBranchAsyncInputTransition(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TraverseTransitionTo(p_active, p_context, p_active->p_SyncNextSubState);
    p_active->p_SyncNextSubState = NULL; /* Clear next state */
}


void _StateMachine_ProcBranchAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    p_active->p_SyncNextSubState = TransitionFunctionOfBranchInput(p_active, p_context, id, value);
}


/******************************************************************************/
/*
    Proc State
*/
/******************************************************************************/
/* Including Top level state */
void _StateMachine_ProcBranch(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_ProcBranchAsyncInputTransition(p_active, p_context); /* check for an buffered transition first */
    _StateMachine_ProcBranchSyncInput(p_active, p_context);
    _StateMachine_ProcBranchSyncOutput(p_active, p_context, NULL);
}

/*
    Proc traverse up excluding Top State, for cases where implementation include [_StateMachine_ProcBranch_Nested] in only some [p_ActiveState->LOOP]
    Caller clears SubStates
*/
void _StateMachine_ProcBranch_Nested(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_ProcBranchAsyncInputTransition(p_active, p_context); /* check for an buffered transition first */
    _StateMachine_ProcBranchSyncInput(p_active, p_context);
    _StateMachine_ProcBranchSyncOutput(p_active, p_context, p_active->p_ActiveState);
}

// as Root level first. keep non hsm effectively the same
// void _StateMachine_TraverseTransition_Nested(StateMachine_Active_T * p_active, void * p_context, State_T * p_top, State_T * p_leaf)
// {
//     State_TraverseTransition(StateMachine_GetLeafState(p_active),  , p_context);
//     p_active->p_ActiveSubState = p_leaf;
//     p_active->p_ActiveState = p_top; /* alternatively within subtree only */
// }

/*
    traverse Top Level first. then leaf to top-1
*/
void _StateMachine_ProcBranchRootFirst(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TraverseTransitionTo(p_active, p_context, State_TransitionOfOutput_AsTop(p_active->p_ActiveState, p_context));
    /* If top State transitions. State_TraverseTransitionOfOutput returns NULL  */
    _StateMachine_TraverseTransitionTo(p_active, p_context, State_TraverseTransitionOfOutput(StateMachine_GetLeafState(p_active), p_active->p_ActiveState, p_context));
}


// root first, then leaf to top-1
void _StateMachine_ProcInputRootFirst(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    State_TransitionOfInput_AsTop(StateMachine_GetRootState(p_active), p_context, id, value);

    _StateMachine_TraverseTransitionTo(p_active, p_context, State_TraverseTransitionOfInput(StateMachine_GetLeafState(p_active), p_context, id, value));
}
