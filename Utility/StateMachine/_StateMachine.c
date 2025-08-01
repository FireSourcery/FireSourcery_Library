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
/*
    Process user defined handlers output, [State_Input_T] [State_Action_T]
    and return mapped state.

    Caller handles updating p_ActiveState, Entry/Exit
*/
/******************************************************************************/
static inline State_T * TransitionFunctionOfState(const StateMachine_Active_T * p_active, void * p_context)
{
    return State_TransitionOfOutput_AsTop(p_active->p_ActiveState, p_context); /* Top level LOOP always defined */
}

static inline State_T * TransitionFunctionOfInput(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    return State_TransitionOfInput_AsTop(p_active->p_ActiveState, p_context, id, value);
}


/******************************************************************************/
/*!
    Mutatate State
*/
/******************************************************************************/
/*
    inline within the compile unit
*/
inline void _StateMachine_Init(StateMachine_Active_T * p_active, void * p_context, State_T * p_initialState)
{
    State_Entry(p_initialState, p_context);
    p_active->p_ActiveState = p_initialState;
}

/*
    Unconditional Transition - Updates [p_ActiveState] to [p_newState].
    p_newState defined as valid at compile time, caller ensure correctness
    call from within [ProcState] high priority thread, or user handle critical
*/
/*
    Async (non buffered) may selectively implement critical. If unprotected:
    Set p_ActiveState after proc ENTRY.
        [p_newState->OUTPUT] will not proc until after [p_newState->ENTRY], correctly
        [p_ActiveState->OUTPUT] (prevState) may proc after [p_newState->ENTRY], overwrite, incorrectly

    Set p_ActiveState before ENTRY.
        [p_newState->ENTRY] will not be overwritten by [p_ActiveState->OUTPUT], correctly
        [p_newState->OUTPUT] may proc before [p_newState->ENTRY], without setup, incorrectly
*/
inline void _StateMachine_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_newState)
{
    assert(p_newState->DEPTH == 0U); /* Top level state */

    State_Exit(p_active->p_ActiveState, p_context);
    State_Entry(p_newState, p_context);

    p_active->p_ActiveState = p_newState;
// #ifdef CONFIG_STATE_MACHINE_HSM_ENABLE
    /* if used in combination with HSM. Clear the SubState on a Top level State Transition */
    p_active->p_ActiveSubState = p_active->p_ActiveState;
// #endif
}

inline void _StateMachine_TransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_newState)
{
    if (p_newState != NULL) { _StateMachine_Transition(p_active, p_context, p_newState); }
}

/*
    [ProcTransitionInput] must lock if called from a input thread
    prevent a transition occurring between [AcceptInput] and [ProcTransition]
    Store a local function pointer, reacessing the table by id may return NULL.
    If an async transition does occur, the previously selected function will run.
*/
inline void _StateMachine_ProcInputTransition(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_TransitionTo(p_active, p_context, TransitionFunctionOfInput(p_active, p_context, id, value));
}

inline void _StateMachine_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TransitionTo(p_active, p_context, TransitionFunctionOfState(p_active, p_context));
}

/* higher priority proc */
inline void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    uint32_t inputMask = p_active->SyncInputMask;

    while (inputMask != 0UL)
    {
        state_input_t input = __builtin_ctz(inputMask);

        assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */

        _StateMachine_ProcInputTransition(p_active, p_context, input, p_active->SyncInputs[input]); /* may update [p_active->p_ActiveState]. optionally return early */
        inputMask &= (inputMask - 1);
    }

    p_active->SyncInputMask = 0UL;
}

/*  */
inline void _StateMachine_ProcAsyncInputTransition(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_TransitionTo(p_active, p_context, p_active->p_SyncNextState);
    p_active->p_SyncNextState = NULL; /* Clear next state */
}

/*
    Default behavior of Async Input
    Process the use [State_Input_T] handler.
    Buffer the transition, and wait for next [ProcSyncOutput]
    Synchronized sequence, [ProcState] can only interrupt user Input handler
    User can selectively sync on some [state_input_t] inputs - SetSyncInput or lock
        where proc state should not run without input completion

    [State_Input_T P_TRANSITION_TABLE[id]] <- [ProcState] can only interrupt user Input handler
    [State_Exit]
    [State_Entry]
    [State_Action_T LOOP]
*/
/* ProcInputSetTransition */
inline void _StateMachine_ProcAsyncInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    p_active->p_SyncNextState = TransitionFunctionOfInput(p_active, p_context, id, value); /* wait for next transition. transition will run before SYNC_OUTPUT */
}


/******************************************************************************/
/*
    Combined to inline contents within compilation unit
*/
/******************************************************************************/
void _StateMachine_ProcState(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_ProcAsyncInputTransition(p_active, p_context); /* check for an async transition first */
    /* Continue processing [Output] of the new State  */
    /* if a [Transition] occured. SyncInputs are not applied to the new State.  */
    _StateMachine_ProcSyncInput(p_active, p_context);
    _StateMachine_ProcSyncOutput(p_active, p_context);
}


/******************************************************************************/
/*
    [SubState] as fixed 2nd level [p_ActiveSubState]
*/
/******************************************************************************/


/******************************************************************************/
/*
    HSM todo split file
*/
/******************************************************************************/
State_T * GetTop(State_T * p_state)
{
// #ifndef NDEBUG
//     assert(_State_IterateUp(p_state, p_state->DEPTH) == p_state->P_TOP);
// #endif
    // assert(!((p_state->P_TOP == NULL) && (p_state->P_PARENT != NULL))); /*   */
    /* Compile Time define Top State */
    /* A top state may define its own P_TOP as NULL or itself */
    /* alternatively iterate if not defined */
    return (p_state->P_TOP == NULL) ? p_state : p_state->P_TOP;
}


/*
    Set SubState. Does not traverse Exit/Entry.
*/
inline void _StateMachine_TransitionSubState(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    assert(p_state != NULL);
    if (p_active->p_ActiveSubState != NULL) { State_Exit(p_active->p_ActiveSubState, p_context); }
    p_active->p_ActiveSubState = p_state;
    p_active->p_ActiveState = GetTop(p_state);
    // /* Assume Top is the [P_PARENT] if it is not defined at compile time */
    // return (p_state->P_TOP != NULL) ? p_state->P_TOP  : ((p_state->P_PARENT != NULL) ? p_state->P_PARENT : p_state);
    State_Entry(p_state, p_context);
}

inline void _StateMachine_TransitionSubStateTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_state)
{
    if (p_state != NULL) { _StateMachine_TransitionSubState(p_active, p_context, p_state); }
}

/*
    Proc inside p_active->p_ActiveState->LOOP
*/
void _StateMachine_ProcSubState_Nested(StateMachine_Active_T * p_active, void * p_context)
{
    // if (p_active->p_ActiveSubState != NULL)
    if (StateMachine_GetActiveSubState(p_active) != p_active->p_ActiveState)
    {
        _StateMachine_TransitionSubStateTo(p_active, p_context, State_TransitionOfOutput(p_active->p_ActiveSubState, p_context));
    }
}

/*
    Either the select input id, is handled by [ProcSubStateInput] only, or handle with _StateMachine_TransitionSubState within the input handler
*/
void _StateMachine_ProcSubStateInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    // if (StateMachine_GetActiveSubState(p_active) != p_active->p_ActiveState)
    _StateMachine_TransitionSubStateTo(p_active, p_context, State_TransitionOfInput(StateMachine_GetActiveSubState(p_active), p_context, id, value));
}


