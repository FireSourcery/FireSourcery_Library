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
/******************************************************************************/
/* Sync Output */
/******************************************************************************/
/*
    [Transition Of Context/Tick/SyncOutput]
    Map SyncProc(p_context) => newState
*/
static inline const State_T * State_TransitionOfContext(const State_T * p_state, void * p_context)
{
    if (p_state->LOOP != NULL) { p_state->LOOP(p_context); }
    return (p_state->NEXT != NULL) ? p_state->NEXT(p_context) : NULL;
}

/* Top level State does not need to check null */
static inline const State_T * State_TransitionOfContext_AsTop(const State_T * p_state, void * p_context)
{
    p_state->LOOP(p_context);
    return (p_state->NEXT != NULL) ? p_state->NEXT(p_context) : NULL;
}

/******************************************************************************/
/* Input */
/******************************************************************************/
static inline State_Input_T State_AcceptInputOfTable(const State_T * p_state, void * p_context, state_input_t inputId)
{
    (void)p_context;
    return p_state->P_TRANSITION_TABLE[inputId];
}

static inline State_Input_T State_AcceptInputOfMapper(const State_T * p_state, void * p_context, state_input_t inputId)
{
    return p_state->TRANSITION_MAPPER(p_context, inputId);
}

/*!
    @return [NULL] indicates not accepted input, transition does not exist, no mapped input function.
            [State_Input_T] indicates accepted input, state may transition, or self transition (with or without entry and exit function).
*/
static inline State_Input_T State_AcceptInput(const State_T * p_state, void * p_context, state_input_t inputId)
{
    State_Input_T result = NULL;
    if (p_state->P_TRANSITION_TABLE != NULL) { result = p_state->P_TRANSITION_TABLE[inputId]; }
    else if (p_state->TRANSITION_MAPPER != NULL) { result = p_state->TRANSITION_MAPPER(p_context, inputId); }
    return result;
}

/*
    Resolve on transition action and return new [State]
*/
static inline State_T * _State_ResolveInputHandler(State_Input_T transition, void * p_context, state_input_value_t inputValue)
{
    return (transition != NULL) ? transition(p_context, inputValue) : NULL;
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
static inline State_T * State_TransitionOfInput(const State_T * p_state, void * p_context, state_input_t inputId, state_input_value_t inputValue)
{
    assert(p_state != NULL);
    return _State_ResolveInputHandler(State_AcceptInput(p_state, p_context, inputId), p_context, inputValue);
}

static inline State_T * State_TransitionOfInput_AsTop(const State_T * p_state, void * p_context, state_input_t inputId, state_input_value_t inputValue)
{
    return _State_ResolveInputHandler(State_AcceptInputOfTable(p_state, p_context, inputId), p_context, inputValue);
}


/******************************************************************************/
/* Transition */
/******************************************************************************/
static inline void State_Entry(const State_T * p_state,  void * p_context)
{
    if (p_state->ENTRY != NULL) { p_state->ENTRY(p_context); }
}

static inline void State_Exit(const State_T * p_state, void * p_context)
{
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    if (p_state->EXIT != NULL) { p_state->EXIT(p_context); }
#else
    (void)p_state;
    (void)p_context;
#endif
}

static inline void State_OnTransition(const State_T * p_state, const State_T * p_new, void * p_context)
{
    if (p_new != NULL)
    {
        State_Exit(p_state, p_context);
        State_Entry(p_new, p_context);
    }
    // else
    // {
    //     State_Entry(p_state, p_context);
    // }
}




