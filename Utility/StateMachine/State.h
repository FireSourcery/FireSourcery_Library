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
    @file   State.h
    @author FireSourcery
    @brief  StateMachine Submodule
            expose State_T
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*

*/
struct State;
typedef uint8_t state_t;    /* State ID. User may overwrite with enum */

typedef uint8_t state_input_t;          /* Input/Handler Id. Maps to [State_Input_T]. Index into transition table. Implementation may overwrite with enum. */
typedef intptr_t state_input_value_t;   /* Optional input parameter. User define platform register size */

#define STATE_INPUT_ID_NULL         (0xFFU)
#define STATE_INPUT_VALUE_NULL      (0U)

// id is not passed in a function pointer. will not affect function pointer casting compatibility
// typedef enum state_input_meta { STATE_INPUT_ID_NULL1 = 0xFF } state_input_meta_t;

/* A State Action/Output. On Transition. Mapped per State.*/
typedef void (*State_Action_T)(void * p_context);

/******************************************************************************/
/*!
    @name Input and Transition Handler Function Types
    Forms the Transition Function - defined by user via P_TRANSITION_TABLE

    @return pointer to the next state, if it exists.
    @retval - [NULL] - no transition / "interal-transition", bypass exit and entry, indicates user defined non transition
    @retval - [this] - self-transition, proc exit and entry
    @retval - [State *] - transition, proc exit and entry.

            HSM case - returning SubStates must be called with [ProcBranchInput] or [ProcSubStateInput]
                [ProcInput] will not call SubState transitions

    effectively, destination state + on transition logic [struct { P_STATE, ON_TRANSITION }]
*/
/******************************************************************************/
// typedef union
// {
//     struct State * (*With0)(void * p_context);
//     struct State * (*WithValue)(void * p_context, state_input_value_t inputValue);
// }
// State_Transition_T;
// typedef struct State * (*State_Transition0_T)(void * p_context);
// typedef struct State * (*State_Transition1_T)(void * p_context, state_input_value_t inputValue);

/*!@{*/
/*
    Context Input Handler
    Transition Input with context
    Next_T
*/
typedef struct State * (*State_InputVoid_T)(void * p_context);

/*
    Value Input Handler
    Transition Input with immediate parameter, value/ptr
    Alternative to passing all parameters by context. reduce assigning temporary variables
*/
typedef struct State * (*State_Input_T)(void * p_context, state_input_value_t inputValue);


/*
    implementation handle id look up
    returns NULL as input not accepted. effectively handles case of return status.
*/
typedef State_Input_T(*State_TransitionMapper_T)(void * p_context, state_input_t inputId);


/*
    User define full handler switch
*/
// typedef struct State * (*State_TransitionHandler_T)(void * p_context, state_input_t inputId, state_input_value_t inputValue);

/*!@}*/

/******************************************************************************/
/*
*/
/******************************************************************************/
typedef const struct State
{
    const state_t ID;               /* [State] instances may optionally remain private, by calling via id */
    const State_Action_T ENTRY;     /* Common to all transition to current state, including self transition */
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    const State_Action_T EXIT;
#endif

    /*
        [Synchronous Output] - Sync modes only. Periodic processing.
        Separate parts for SubState regularity.
    */
    /* [Internal Transition] - A "transition" internal to [P_CONTEXT] only. Mutation of P_CONTEXT */
    const State_Action_T LOOP;      /* No null pointer check for TOP level. Implementation supply empty function */
                                    /* Inherited by SubState */
                                    /* SYNC_OUTPUT */

    /* [State Transition of Context/Clock] - Transition to a new State_T determined by [P_CONTEXT] state. no external input. "clock only" Transition. */
    const State_InputVoid_T NEXT;   /* Synchronous Transition Handler.  */
                                    /* Overwritten by SubStates. Separate from LOOP to allow overwrite transition only */
                                    /* SYNC_OUTPUT_TRANSITION */

    /*
        [State Transition of Input] - Sync/Async
    */
    /*!
        Array Implementation - 2D input table
        Map allocates for all possible transitions/inputs for each state, valid and invalid
        Allocates space for fault transition for invalid inputs
        Array index as [state_input_t], eliminates search, only space efficient when inputs are common across many states.
        States belonging to the same [StateMachine] must have same size maps

        Pointer to array of functions [State_Input_T].
        @retval [NULL] - a null function pointer indicates not accepted input

        [State_Input_T] returns a pointer to the next state:
        Not accept input => no process.
        Non-transition/InternalTransition (Output only / Mealy machine style outputs), does not proc entry function => function return NULL
        Self-transition, proc entry function => function return pointer to self

        state_input_t ids are shared across all states in the same StateMachine, including sub states.
    */
    const State_Input_T * const P_TRANSITION_TABLE; /* f(state_input_t) input map. Forms the TransitionFunctionAsync. */

    /*
        Effectively [P_TRANSITION_TABLE] with user implemented switch()
        return a function pointer, instead of directly returning the State.
        [NULL] for not accepted input.
        Unused by Top Level State.
    */
    const State_TransitionMapper_T TRANSITION_MAPPER;

    /*
        [Hierarchical State Machine]
        composite pattern
        SubStates mount onto States
    */
    const struct State * const P_PARENT; /* NULL for top level state */

    /* SubStates define P_TOP and DEPTH for runtime optimization */
    const struct State * const P_TOP; /* SubStates maintain pointer to top level state. */
    const uint8_t DEPTH; /* Depth of state. Depth must be consistent for iteration */
    // const state_t TOP_ID;  /* Top level state ID, can remain private via extern id */

    /*
        pointer to sub state only functions,
        effectively, struct { State_T base, (*Get/Set) } SubState_T;
        P_EXT
    */

    // void * (* const GET_CONTEXT)(void * p_context); /* Retrive State data from p_context */

    /*
    */
    /* Can be generalized as P_TRANSITION_TABLE[MENU](p_context, direction) */
#ifdef CONFIG_STATE_MACHINE_LINKED_STATES_ENABLE
    const struct State * P_LINK_NEXT;
    const struct State * P_LINK_PREV;
#endif

}
State_T;

// #define STATIC_ASSERT_STATE_T(p_state) \


/*
    Public/Private Defs
*/

/******************************************************************************/
/*
    Flex input signature types
    Signatures for inputs passing function pointer
    Per state inputs can be passed at input time,
    Inputs maping to a single state, can simply check State ID

    [StateMachine_SetValueWith]
*/
/******************************************************************************/
typedef void(*State_Set_T)(void * p_context, state_input_value_t inputValue);
typedef state_input_value_t(*State_Get_T)(void * p_context);


/******************************************************************************/
/*
    Transition - or self contained cmd
    InputCmd
    Alternative to mapping Id
    reverse map [Transition]/[Input] to a [State] or set of [State]s
    compare [p_transition->P_START] to [p_stateMachine->p_ActiveState]
    Apply transition to State

    effectively [State] local [state_input_t]

    [StateMachine_InvokeTransition]
    Directly invoke. defined at compile time for valid transition
*/
/******************************************************************************/
typedef const struct State_TransitionInput
{
    const State_T * P_START; /* From/Source. Starting State known to accept this input at compile time. */
    const State_Input_T TRANSITION; /* To/Destination. Does not return NULL */ /* Effectively P_DEST, ON_TRANSITION */
}
State_TransitionInput_T;

/* Convenience for inline call [StateMachine_InvokeTransition] */
// static inline State_TransitionInput_T State_TransitionCmd_Create(const State_T * p_start, const State_Input_T transition)
// {
//     return (State_TransitionInput_T) { .P_START = p_start, .TRANSITION = transition };
// }


// typedef const struct StateMachine_MultiTransitionInput
// {
//     const State_Input_T TRANSITION; /* To/Destination. Does not return NULL */
//     const State_T * const * const PP_VALID_LIST; /* From/Source. Starting State known to accept this input at compile time. */
//     const uint8_t VALID_COUNT;
// }
// StateMachine_MultiTransitionInput_T;

// typedef const struct StateMachine_Cmd
// {
//     uint8_t ID; /* Command ID. User may overwrite with enum */
//     const State_T * const * PP_SOURCE_TABLE; /* From/Source. Starting State known to accept this input at compile time. */
//     uint8_t SOURCE_COUNT;
//     State_Input_T DEST; /* To/Destination. Does not return NULL */
// }
// StateMachine_Cmd_T;

