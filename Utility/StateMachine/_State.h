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
static inline State_T * _State_InputVoid(State_Input0_T input, void * p_context) { return (input != NULL) ? input(p_context) : NULL; }

/******************************************************************************/
/* Sync Output */
/******************************************************************************/
// static inline void State_Output(State_T * p_state, void * p_context) { _State_Action(p_state->LOOP, p_context); }
// static inline void State_Output_AsTop(State_T * p_state, void * p_context) { p_state->LOOP(p_context); }

/******************************************************************************/
/* Sync Output with Transition */
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
    assert(p_state->DEPTH == 0U);
    p_state->LOOP(p_context);
    return _State_InputVoid(p_state->NEXT, p_context);
}


/******************************************************************************/
/* Input */
/******************************************************************************/
/*
    Resolve on transition action and return new [State]
*/
static inline State_T * _State_CallInput(State_Input_T input, void * p_context, state_value_t value) { return (input != NULL) ? input(p_context, value) : NULL; }

/*!
    @return [NULL] indicates not accepted input, transition does not exist, no mapped input function.
            [State_Input_T] indicates accepted input, state may transition, or self transition (with or without entry and exit function).
*/
static inline State_Input_T State_AcceptInput(State_T * p_state, state_input_t inputId)
{
    if (inputId < STATE_INPUT_MAPPER_START_ID)
    {
        if (p_state->P_TRANSITION_TABLE != NULL) { return p_state->P_TRANSITION_TABLE[inputId]; }
    }
    else
    {
        if (p_state->TRANSITION_MAPPER != NULL) { return p_state->TRANSITION_MAPPER(inputId); }
    }
    return NULL;
}

/*!
    [Transition Of Input/AsyncInput]
    Map (inputId, inputValue) -> p_next
    Map inputId to [State_Input_T] and call with inputValue
    @return NULL for internal-only-transition without processing ENTRY.
            self for self-transition processing ENTRY
            another State for transition to another State

    @note the result of State_AcceptInput must be stored in a local variable
    without critical section, a mismatched input function may run, however it should not run NULL
*/
static inline State_T * State_TransitionOfInput(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue)
{
    assert(p_state != NULL);
    return _State_CallInput(State_AcceptInput(p_state, inputId), p_context, inputValue);
}


/******************************************************************************/
/* AsTop Without Null Check */
/******************************************************************************/
/* Transition Table defined for every state.. allocate empty for not accepted inputs */
static inline State_Input_T State_AcceptInput_AsTop(State_T * p_state, state_input_t inputId)
{
    assert(p_state->DEPTH == 0U);
    assert(p_state->P_TRANSITION_TABLE != NULL); /* Known at compile time */
    // mapper ids defined with namespaceid << 8 | baseId
    return p_state->P_TRANSITION_TABLE[(uint8_t)inputId];
}

static inline State_T * State_TransitionOfInput_AsTop(State_T * p_state, void * p_context, state_input_t inputId, state_value_t inputValue)
{
    // return _State_CallInput(State_AcceptInput(p_state, inputId), p_context, inputValue);
    return _State_CallInput(State_AcceptInput_AsTop(p_state, inputId), p_context, inputValue);
}


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
    State_Exit(p_state, p_context);
    State_Entry(p_new, p_context);
}



/*
    Minimal handler for orthogonal states
*/
typedef struct State_Active { State_T * p_State; } State_Active_T;

static inline void _State_Reset(State_Active_T * p_active, void * p_context, State_T * p_initial)
{
    if (p_initial != NULL)
    {
        State_Entry(p_initial, p_context);
        p_active->p_State = p_initial;
    }
}


static inline void _State_TransitionTo(State_Active_T * p_active, void * p_context, State_T * p_next)
{
    State_Exit(p_active->p_State, p_context);
    State_Entry(p_next, p_context);
    p_active->p_State = p_next;
}

static inline void _State_Transition(State_Active_T * p_active, void * p_context, State_T * p_next)
{
    if (p_next != NULL) { _State_TransitionTo(p_active, p_context, p_next); }
}

/*
    Async only
*/
static inline void _State_Input(State_Active_T * p_active, void * p_context, state_input_t id, state_value_t value)
{
    _State_Transition(p_active, p_context, State_TransitionOfInput(p_active->p_State, p_context, id, value));
}


/******************************************************************************/
/* Accessor */
/******************************************************************************/
static inline state_value_t _State_Cmd(State_Cmd_T cmd, void * p_context, state_cmd_t id, state_value_t value) { return (cmd != NULL) ? cmd(p_context, value) : 0; }
static inline state_value_t State_Cmd(State_T * p_state, void * p_context, state_cmd_t id, state_value_t value) { return _State_Cmd(p_state->P_CMD_TABLE[id], p_context, id, value); }

static inline state_value_t _State_Getter(State_GetField_T getter, void * p_context, state_value_t field) { return (getter != NULL) ? getter(p_context, field) : 0; }
static inline state_value_t State_GetValue(State_T * p_state, void * p_context, state_accessor_t id, state_value_t field) { return _State_Getter(p_state->P_ACCESSOR_TABLE[id].GET, p_context, field); }

static inline void _State_Setter(State_SetField_T setter, void * p_context, state_value_t field, state_value_t value) { if (setter != NULL) setter(p_context, field, value); }
static inline void State_SetValue(State_T * p_state, void * p_context, state_accessor_t id, state_value_t field, state_value_t value) { _State_Setter(p_state->P_ACCESSOR_TABLE[id].SET, p_context, field, value); }
