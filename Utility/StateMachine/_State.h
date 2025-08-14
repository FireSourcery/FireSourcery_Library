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
    @file   _State.h
    @author FireSourcery
    @brief  Private inline for StateMachine include
*/
/******************************************************************************/
#include "State.h"


/******************************************************************************/
/*!
    [State_T] Common
*/
/******************************************************************************/
static inline void _State_Action(State_Action_T action, void * p_context) { if (action != NULL) { action(p_context); } }
static inline State_T * _State_InputVoid(State_InputVoid_T input, void * p_context) { return (input != NULL) ? input(p_context) : NULL; }

/******************************************************************************/
/* Sync Output */
/******************************************************************************/
/*
    [Transition Of Context/Tick/SyncOutput]
    Map SyncProc(p_context) => newState
*/
static inline State_T * State_TransitionOfOutput(State_T * p_state, void * p_context)
{
    _State_Action(p_state->LOOP, p_context);
    return _State_InputVoid(p_state->NEXT, p_context);
}

/* Top level State does not need to check null */
static inline State_T * State_TransitionOfOutput_AsTop(State_T * p_state, void * p_context)
{
    p_state->LOOP(p_context);
    return _State_InputVoid(p_state->NEXT, p_context); // optionally remove, let top state call Transition
}

/******************************************************************************/
/* Input */
/******************************************************************************/
static inline State_Input_T State_AcceptInputOfTable(State_T * p_state, void * p_context, state_input_t inputId)
{
    (void)p_context;
    return p_state->P_TRANSITION_TABLE[inputId];
}

static inline State_Input_T State_AcceptInputOfMapper(State_T * p_state, void * p_context, state_input_t inputId)
{
    return p_state->TRANSITION_MAPPER(p_context, inputId);
}

/*!
    @return [NULL] indicates not accepted input, transition does not exist, no mapped input function.
            [State_Input_T] indicates accepted input, state may transition, or self transition (with or without entry and exit function).
*/
static inline State_Input_T State_AcceptInput(State_T * p_state, void * p_context, state_input_t inputId)
{
    State_Input_T result = NULL;
    if (p_state->P_TRANSITION_TABLE != NULL) { result = p_state->P_TRANSITION_TABLE[inputId]; }
    else if (p_state->TRANSITION_MAPPER != NULL) { result = p_state->TRANSITION_MAPPER(p_context, inputId); }
    return result;
}

/*
    Resolve on transition action and return new [State]
*/
static inline State_T * _State_CallInput(State_Input_T inputFn, void * p_context, state_value_t inputValue)
{
    return (inputFn != NULL) ? inputFn(p_context, inputValue) : NULL;
}

/*!
    [Transition Of Input/AsyncInput]
    Map (inputId, inputValue) -> p_newState
    Map inputId to [State_Input_T] and call with inputValue
    @return NULL for internal-only-transition without processing ENTRY.
            self for self-transition processing ENTRY
            another State for transition to another State

    @note the result of State_AcceptInput must be stored in a local variable
    without critical section, a mismatched input function may run, however it should not run NULL
*/
// State_TransitionOfInputByMapper
static inline State_T * State_TransitionOfInput(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue)
{
    assert(p_state != NULL);
    return _State_CallInput(State_AcceptInput(p_state, p_context, inputId), p_context, inputValue);
}

static inline State_T * State_TransitionOfInput_AsTop(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue)
{
    return _State_CallInput(State_AcceptInputOfTable(p_state, p_context, inputId), p_context, inputValue);
}

// user handle accept input
// static inline State_T * State_TransitionOfInput_ByHandler(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue)
// {
//     State_TransitionFunction_T function = p_state->TRANSITION_FUNCTION;
//     return (function != NULL) ? function(p_context, inputId, inputValue) : NULL;
// }

// typedef struct State * (*State_TransitionFunction_T)(void * p_context, state_input_t inputId, state_value_t inputValue);
// typedef struct State * (*StateMachine_TransitionFunction_T)(struct State *, void * p_context, state_input_t inputId, state_value_t inputValue);
// static inline State_T * State_TransitionOfInput_Meta(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue)
// {
//     StateMachine_TransitionFunction_T function = State_TransitionOfInput;
//     return function(p_state, p_context, inputId, inputValue);
// }



/******************************************************************************/
/* Transition */
/******************************************************************************/
static inline void State_Entry(State_T * p_state, void * p_context)
{
    _State_Action(p_state->ENTRY, p_context);
}

static inline void State_Exit(State_T * p_state, void * p_context)
{
#ifdef STATE_MACHINE_EXIT_FUNCTION_ENABLE
    _State_Action(p_state->EXIT, p_context);
#else
    (void)p_state;
    (void)p_context;
#endif
}

static inline void State_OnTransition(State_T * p_state, State_T * p_new, void * p_context)
{
    if (p_new != NULL)
    {
        State_Exit(p_state, p_context);
        State_Entry(p_new, p_context);
    }
}


/******************************************************************************/
/* Accessor */
/******************************************************************************/
static inline state_value_t _State_Accessor(State_Accessor_T accessor, void * p_context, state_value_t valueK, state_value_t valueV) { return (accessor != NULL) ? accessor(p_context, valueK, valueV) : 0; }
static inline state_value_t State_Access(State_T * p_state, void * p_context, state_input_t id, state_value_t valueK, state_value_t valueV) { return _State_Accessor(p_state->P_ACCESSOR_TABLE[id], p_context, valueK, valueV); }

static inline state_value_t _State_Getter(State_GetField_T getter, void * p_context, state_value_t valueK) { return (getter != NULL) ? getter(p_context, valueK) : 0; }
static inline state_value_t State_GetValue(State_T * p_state, void * p_context, state_input_t id, state_value_t valueK) { return _State_Getter(p_state->P_GET_FIELD_TABLE[id], p_context, valueK); }

static inline void _State_Setter(State_SetField_T setter, void * p_context, state_value_t valueK, state_value_t valueV) { if (setter != NULL) setter(p_context, valueK, valueV); }
static inline void State_SetValue(State_T * p_state, void * p_context, state_input_t id, state_value_t valueK, state_value_t valueV) { _State_Setter(p_state->P_SET_FIELD_TABLE[id], p_context, valueK, valueV); }
