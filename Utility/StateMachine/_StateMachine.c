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
    @file   StateMachine.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "_StateMachine.h"

#include "_State.h"
#include "_State_Node.h"


/******************************************************************************/
/*
    [FSM]
    Top Level State [p_ActiveState]
*/
/******************************************************************************/
static inline const State_T * TransitionFunctionSync(const StateMachine_Active_T * p_fields, void * p_context)
{
    return State_TransitionOfContext_AsTop(p_fields->p_ActiveState, p_context); /* Top level LOOP always defined */
}

static inline State_T * TransitionFunctionAsync(const StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value)
{
    return State_TransitionOfInput_AsTop(p_fields->p_ActiveState, p_context, id, value);
}


/******************************************************************************/
/*!
    Mutatate State
*/
/******************************************************************************/
/*
    inline within the compile unit
*/
inline void _StateMachine_Init(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_initialState)
{
    State_Entry(p_initialState, p_context);
    p_fields->p_ActiveState = p_initialState;
}

/*
    Unconditional Transition - Maps active state to new state.
    p_newState assumed to be valid, caller ensure correctness
    call from within [ProcState] high priority thread, or user handle critical
*/
/*
    Async may selectively implement critical. If unprotected:
    Set p_ActiveState after proc ENTRY.
        [p_newState->OUTPUT] will not proc until after [p_newState->ENTRY], correctly
        [p_ActiveState->OUTPUT] (prevState) may proc after [p_newState->ENTRY], overwrite, incorrectly

    Set p_ActiveState before ENTRY.
        [p_newState->ENTRY] will not be overwritten by [p_ActiveState->OUTPUT], correctly
        [p_newState->OUTPUT] may proc before [p_newState->ENTRY], without setup, incorrectly
*/
inline void _StateMachine_Transition(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_newState)
{
    State_Exit(p_fields->p_ActiveState, p_context);
    p_fields->p_ActiveState = p_newState;
    State_Entry(p_newState, p_context);
    /* if used in combination with HSM */
    p_fields->p_ActiveSubState = p_fields->p_ActiveState;
}

inline void _StateMachine_TransitionTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_newState)
{
    assert(p_newState == NULL || p_newState->DEPTH == 0); /* Top level state */
    if (p_newState != NULL) { _StateMachine_Transition(p_fields, p_context, p_newState); }
}

/*
    [ProcInput] must lock, prevent a transition occurring between [AcceptInput] and [ProcTransition]
    Store a local function pointer, reacessing the table by id may return NULL. If an async transition does occur, the previously selected function will run.
*/
// inline void _StateMachine_Input(
// inline void _StateMachine_ApplyInput(
inline void _StateMachine_ProcInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value)
{
    _StateMachine_TransitionTo(p_fields, p_context, TransitionFunctionAsync(p_fields, p_context, id, value));
}

/* Proc State/SyncOutput */
inline void _StateMachine_ProcSyncOutput(StateMachine_Active_T * p_fields, void * p_context)
{
    _StateMachine_TransitionTo(p_fields, p_context, TransitionFunctionSync(p_fields, p_context));
}

inline void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_fields, void * p_context)
{
    if (p_fields->SyncInput != STATE_INPUT_ID_NULL)
    {
        _StateMachine_ProcInput(p_fields, p_context, p_fields->SyncInput, p_fields->SyncInputValue);
        p_fields->SyncInput = STATE_INPUT_ID_NULL;
    }
}

inline void _StateMachine_SetSyncInput(StateMachine_Active_T * p_fields, state_input_t id, state_input_value_t value)
{
    p_fields->SyncInputValue = value;
    p_fields->SyncInput = id;
}

// inline void _StateMachine_ProcAyncInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value)
// {
//     _StateMachine_ProcInput(p_fields, p_context, id, value);
// }

/******************************************************************************/
/*
    comined proc
*/
/******************************************************************************/
// inline void _StateMachine_ProcSyncState(StateMachine_Active_T * p_fields, void * p_context)
inline void _StateMachine_ProcState(StateMachine_Active_T * p_fields, void * p_context)
{
    _StateMachine_ProcSyncInput(p_fields, p_context);
    _StateMachine_ProcSyncOutput(p_fields, p_context);
}

/******************************************************************************/
/*
    Cmds
*/
/******************************************************************************/
  void _StateMachine_SetValueWith(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state, State_Set_T setter, state_input_value_t value)
{
    if (StateMachine_IsActiveState(p_fields, p_state) == true) { setter(p_context, value); }
}


/******************************************************************************/
/*
    [SubState] as fixed 2nd level [p_ActiveSubState]
*/
/******************************************************************************/
const State_T * GetTop(const State_T * p_state)
{
#ifndef NDEBUG
    assert(_State_IterateUp(p_state, p_state->DEPTH) == p_state->P_TOP);
#endif
    // State_T * p_top;

    // p_top = p_state->P_TOP;

    // /* Assume Top is the [P_PARENT] if it is not defined at compile time */
    // // if      (p_state->P_TOP  != NULL)   { p_top = p_state->P_TOP ; }
    // // else if (p_state->P_PARENT != NULL) { p_top = p_state->P_PARENT; }
    // // else                                { p_top = p_state; }
    // // return (p_state->P_TOP != NULL) ? p_state->P_TOP  : ((p_state->P_PARENT != NULL) ? p_state->P_PARENT : p_state);

    // assert(p_top->DEPTH == 0);

    // return p_top;

    /* A top state may define its own P_TOP as NULL or itself */
    /* alternatively iterate if not defined */
    return (p_state->P_TOP == NULL) ? p_state : p_state->P_TOP;
}

/*
    Set SubState. Does not traverse Exit/Entry.
*/
inline void _StateMachine_TransitionSubState(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state)
{
    assert(p_state != NULL);
    if (p_fields->p_ActiveSubState != NULL) { State_Exit(p_fields->p_ActiveSubState, p_context); }
    p_fields->p_ActiveSubState = p_state;
    p_fields->p_ActiveState = GetTop(p_state);
    State_Entry(p_state, p_context);
}

inline void _StateMachine_TransitionSubStateTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state)
{
    if (p_state != NULL) { _StateMachine_TransitionSubState(p_fields, p_context, p_state); }
}


/*
    Proc inside p_fields->p_ActiveState->LOOP
*/
inline void _StateMachine_ProcSubState_Nested(StateMachine_Active_T * p_fields, void * p_context)
{
    // if (p_fields->p_ActiveSubState != NULL)
    if (StateMachine_GetActiveSubState(p_fields) != p_fields->p_ActiveState)
    {
        _StateMachine_TransitionSubStateTo(p_fields, p_context, State_TransitionOfContext(p_fields->p_ActiveSubState, p_context));
    }
}

/*
    Either the select input id, is handled by [ProcSubStateInput] only, or handle with _StateMachine_TransitionSubState within the input handler
*/
inline void _StateMachine_ProcSubStateInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value)
{
    // if (StateMachine_GetActiveSubState(p_fields) != p_fields->p_ActiveState)
    _StateMachine_TransitionSubStateTo(p_fields, p_context, State_TransitionOfInput(StateMachine_GetActiveSubState(p_fields), p_context, id, value));
}


/******************************************************************************/
/*
    [Hierarchical State Machine]
    [p_ActiveState][p_ActiveSubState]
    SubState as a branch
*/
/******************************************************************************/
/*
    Branch Transitions

    Implementation defined with known valid [p_state]
    Proc p_state->P_TOP->ENTRY if State_CommonAncestorOf is NULL

    [p_ActiveSubState] == NULL => start from [p_ActiveState]
    traverse Entry/Exit functions
    sets [p_ActiveSubState] to [p_state]
    sets [p_ActiveState] to [p_state->P_TOP]. no change if [p_state->P_TOP] is the same
*/
void _StateMachine_TraverseTransition(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state)
{
    State_TraverseTransition(StateMachine_GetActiveSubState(p_fields), p_state, p_context);
    p_fields->p_ActiveSubState = p_state;
    p_fields->p_ActiveState = GetTop(p_state); /* alternatively within subtree only */
}

void _StateMachine_TraverseTransitionTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state)
{
    if (p_state != NULL) { _StateMachine_TraverseTransition(p_fields, p_context, p_state); }
}

// void _StateMachine_ProcBranchSyncOutputUpTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_end)
// void _StateMachine_ProcBranchSyncOutput(StateMachine_Active_T * p_fields, void * p_context)

/* traversing up for now */
void _StateMachine_ProcBranchSyncOutput(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_end)
{
    _StateMachine_TraverseTransitionTo(p_fields, p_context, State_TraverseTransitionOfContext(StateMachine_GetActiveSubState(p_fields), p_end, p_context));
}

void _StateMachine_ProcBranchInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value)
{
    _StateMachine_TraverseTransitionTo(p_fields, p_context, State_TraverseTransitionOfInput(StateMachine_GetActiveSubState(p_fields), p_context, id, value));
}

void _StateMachine_ProcBranchSyncInput(StateMachine_Active_T * p_fields, void * p_context)
{
    if (p_fields->SyncInput != STATE_INPUT_ID_NULL)
    {
        _StateMachine_ProcBranchInput(p_fields, p_context, p_fields->SyncInput, p_fields->SyncInputValue);
        p_fields->SyncInput = STATE_INPUT_ID_NULL;
    }
}

/*
    Proc traverse up excluding Top State, for cases where implementation include [_StateMachine_ProcBranch_Nested] in only some [p_ActiveState->LOOP]
    Caller clears SubStates
*/
void _StateMachine_ProcBranch_Nested(StateMachine_Active_T * p_fields, void * p_context)
{
    _StateMachine_ProcBranchSyncOutput(p_fields, p_context, p_fields->p_ActiveState);
}

/*
    Check Top Level first.
*/
void _StateMachine_ProcBranchSyncOutput_(StateMachine_Active_T * p_fields, void * p_context)
{
    _StateMachine_TraverseTransitionTo(p_fields, p_context, State_TransitionOfContext_AsTop(p_fields->p_ActiveState, p_context));
    /* If top State transitions. State_TraverseTransitionOfContext returns NULL  */
    _StateMachine_TraverseTransitionTo(p_fields, p_context, State_TraverseTransitionOfContext(StateMachine_GetActiveSubState(p_fields), p_fields->p_ActiveState, p_context));
}