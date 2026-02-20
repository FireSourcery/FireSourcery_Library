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
    Process user defined handlers output
    and return mapped state.

    Caller handles updating p_ActiveState, Entry/Exit
*/
/*
    Output/Sync TransitionFunction
    Virtualized calls: [SYNC_OUTPUT] [NEXT]
*/
static inline State_T * TransitionFunctionOfState(const StateMachine_Active_T * p_active, void * p_context)
{
    return State_TransitionOfOutput_AsTop(p_active->p_ActiveState, p_context); /* Top level LOOP always defined */
}

/*
    Input TransitionFunction
    Virtualized calls: [P_TRANSITION_TABLE[id]]
*/
static inline State_T * TransitionFunctionOfInput(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    return State_TransitionOfInput_AsTop(p_active->p_ActiveState, p_context, id, value);
}

// inline void Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_next)

/******************************************************************************/
/*!
    Mutate State
    inline within the compile unit
*/
/******************************************************************************/
inline void _StateMachine_Reset(StateMachine_Active_T * p_active, void * p_context, State_T * p_initialState)
{
    p_active->p_SyncNextState = NULL;
    p_active->SyncInputMask = 0UL;
    State_Entry(p_initialState, p_context);
    p_active->p_ActiveState = p_initialState;
}

/*
    Unconditional Transition - Updates [p_ActiveState] to [p_next].
    p_next defined as valid at compile time, caller ensure correctness
    call from within [ProcState] high priority thread, or user handle critical
*/
/*
    Async (non buffered) may selectively implement critical. If unprotected:
    Set p_ActiveState after proc ENTRY.
        [p_next->OUTPUT] will not proc until after [p_next->ENTRY], correctly
        [p_ActiveState->OUTPUT] (prevState) may proc after [p_next->ENTRY], overwrite, incorrectly

    Set p_ActiveState before ENTRY.
        [p_next->ENTRY] will not be overwritten by [p_ActiveState->OUTPUT], correctly
        [p_next->OUTPUT] may proc before [p_next->ENTRY], without setup, incorrectly
*/
inline void _StateMachine_TransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_next)
{
    assert(p_next->DEPTH == 0U); /* Top level state */
    State_Exit(p_active->p_ActiveState, p_context);
    State_Entry(p_next, p_context);
    p_active->p_ActiveState = p_next;
}

inline void _StateMachine_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_next)
{
    if (p_next != NULL) { _StateMachine_TransitionTo(p_active, p_context, p_next); }
}

/*
    Virtualized calls: [LOOP] [NEXT] [ENTRY] [EXIT]
*/
inline void _StateMachine_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_Transition(p_active, p_context, TransitionFunctionOfState(p_active, p_context));
}

/*
    Virtualized calls: [P_TRANSITION_TABLE[id]] [ENTRY] [EXIT]
    must lock if called from a input thread
    prevent a transition occurring between [AcceptInput] and [ProcTransition]
        Store a local function pointer, re accessing the table by id may return NULL.
        If an async transition does occur, the previously selected function will run.
*/
inline void _StateMachine_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_Transition(p_active, p_context, TransitionFunctionOfInput(p_active, p_context, id, value));
}


/*
    Proc each Transition individually in case of dependencies on transition updates
    Proc entirely at [ProcState]/ISR priority

    Defined inline _StateMachine_SetSyncInput
*/
inline void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context)
{
    for (uint32_t inputMask = p_active->SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
    {
        state_input_t input = __builtin_ctz(inputMask);
        // assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
        _StateMachine_CallInput(p_active, p_context, input, p_active->SyncInputs[input]);
    }
    p_active->SyncInputMask = 0UL;
}


/*
    ProcSyncTransition
*/
inline void _StateMachine_ProcSyncNextState(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_Transition(p_active, p_context, p_active->p_SyncNextState); /* check proc with local variable */
    p_active->p_SyncNextState = NULL;
    // alternatively ignore DEPTH != 0U
}

/*
    Process the use [State_Input_T] handler.
    Buffer the transition, and wait for next [ProcSyncOutput]
    Synchronized sequence, [ProcState] can only interrupt user Input handler
    User can selectively sync on some [state_input_t] inputs - SetSyncInput or lock
        where proc state should not run without input completion

    [P_TRANSITION_TABLE[id]] <- [ProcState] can only interrupt user Input handler
    ...
    Lock
    [EXIT]
    [ENTRY]
    ...
    [LOOP]
    Unlock
*/
inline void _StateMachine_ApplyInputSyncTransition(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _StateMachine_SetSyncTransition(p_active, TransitionFunctionOfInput(p_active, p_context, id, value)); /* transition will run before SYNC_OUTPUT */
}

inline void _StateMachine_InvokeTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_start, State_Input_T next, state_value_t value)
{
    if (StateMachine_IsActiveState(p_active, p_start) == true) { _StateMachine_Transition(p_active, p_context, next(p_context, value)); }
}

/******************************************************************************/
/*
    Combined to inline contents within this compilation unit
*/
/******************************************************************************/
/* check for an async transition first */
/* if a [Transition] occurred.  */
/* Continue processing [Output] of the new State */
void _StateMachine_ProcState(StateMachine_Active_T * p_active, void * p_context)
{
    _StateMachine_ProcSyncNextState(p_active, p_context); //optionally split to include timer
    _StateMachine_ProcSyncOutput(p_active, p_context);
    _StateMachine_ProcSyncInput(p_active, p_context);
}

/* run loop actions first, then transitions. */
// void _StateMachine_ProcState(StateMachine_Active_T * p_active, void * p_context)
// {
//     _StateMachine_ProcSyncOutput(p_active, p_context);
//     _StateMachine_ProcSyncNextState(p_active, p_context);
//     _StateMachine_ProcSyncInput(p_active, p_context);
// }







