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

typedef uint8_t state_input_t;      /* Input/Handler Id. Maps to [State_Input_T]. Index into transition table. Implementation may overwrite with enum. */
typedef intptr_t state_value_t;     /* Optional input parameter. User define platform register size */

#define STATE_ID_NULL               (0xFFU)
#define STATE_INPUT_ID_NULL         (0xFFU)
#define STATE_INPUT_VALUE_NULL      (0U)

// id is not passed in a function pointer. will not affect function pointer casting compatibility
// typedef enum state_input_meta { STATE_INPUT_ID_NULL1 = 0xFF } state_input_meta_t;

/* A State Action/Output. On Transition. Mapped per State.*/
typedef void (*State_Action_T)(void * p_context);

/* For Run time Sync buffer only. alternatively, fam or context pointer */
#ifndef STATE_TRANSITION_TABLE_LENGTH_MAX
#define STATE_TRANSITION_TABLE_LENGTH_MAX (16U) /* Max number of inputs per state. */
#endif

/******************************************************************************/
/*!
    @name Input/Output and Transition Handler Function Types

    Map each [state_input_t] id to an Output action and potential new State.

    Forms the Transition Function - defined by user via P_TRANSITION_TABLE
    [StateMachine_TransitionFunction] <= State_TransitionFunction

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
//     struct State * (*WithValue)(void * p_context, state_value_t inputValue);
// }
// State_Transition_T;
// typedef struct State * (*State_Transition_T)(void * p_context);
// typedef struct State * (*State_TransitionWith_T)(void * p_context, state_value_t inputValue);
// typedef struct State * (*State_TransitionFn_T)(void * p_context, state_value_t inputValue);
// #ifndef STATE_INPUT_T
// typedef State_TransitionWith_T State_Input_T;

/*!@{*/
/*
    Context/Clock Input/Output Handler
    Transition Input with context
    Next_T/Output_T
*/
typedef struct State * (*State_InputVoid_T)(void * p_context);

/*
    Value Input/Output Handler
    Transition Input with immediate parameter, value/ptr
    Alternative to passing all parameters by context. reduce assigning temporary variables
*/
typedef struct State * (*State_Input_T)(void * p_context, state_value_t inputExt);

/*
    implementation handle id look up
    returns NULL as input not accepted. effectively handles case of return status.
*/
typedef State_Input_T(*State_TransitionMapper_T)(void * p_context, state_input_t inputId);


/*
    User define full handler switch
    //  hsm input case must return entry to indicate as handled
    // no meta for null
*/
typedef struct State * (*State_TransitionFunction_T)(void * p_context, state_input_t inputId, state_value_t inputValue);

/*!@}*/

/******************************************************************************/
/*
   Generic Value Accessors
   Value access without state transition
   Inputs maping to a single state, can simply check State ID
*/
/******************************************************************************/
typedef state_value_t(*State_ValueAccessor_T)(void * p_context, state_value_t value);
typedef state_value_t(*State_Get_T)(const void * p_context);
typedef void(*State_Set_T)(void * p_context, state_value_t value);

typedef state_value_t(*State_Accessor_T)(void * p_context, state_value_t valueK, state_value_t valueV);
/* Convienient for reusing field ids, and match signature. */
typedef state_value_t(*State_GetField_T)(const void * p_context, state_value_t valueK);
typedef void(*State_SetField_T)(void * p_context, state_value_t valueK, state_value_t valueV);

/******************************************************************************/
/*!
    @brief State Machine State
*/
/******************************************************************************/
typedef const struct State
{
    state_t ID;               /* [State] instances may optionally remain private, by calling via id */
    State_Action_T ENTRY;     /* Common to all transition to current state, including self transition */
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    State_Action_T EXIT;
#endif

    /*
        [Synchronous Output] - Sync modes only. Periodic processing.
        Separate parts for SubState regularity.
    */
    /* [Internal Transition] - A "transition" internal of [P_CONTEXT]. Mutation of P_CONTEXT */
    /* No null pointer check for TOP level. Implementation supply empty function */
    State_Action_T LOOP; /* SYNCHRONOUS */

    /* [State Transition of State/Output/Clock] - Transition to a new State_T determined by [P_CONTEXT] state. no external input. "clock only" Transition. */
    /* Separate from LOOP to allow overwrite transition only */
    State_InputVoid_T NEXT; /* SYNCHRONOUS_TRANSITION */ /* Synchronous Transition Handler. */

    /*
        [State Transition of Input] - Sync/Async
    */
    /*
        Array Implementation - 2D input table
        Map allocates for all possible transitions/inputs for each state, valid and invalid
        Allocates space for fault transition for invalid inputs
        Array index as [state_input_t], eliminates search, only space efficient when inputs are common across many states.
        States belonging to the same [StateMachine] must have same size maps
    */
    /*!
        Pointer to array of functions [State_Input_T].
        @retval [NULL] - a null function pointer indicates not accepted input
        [State_Input_T] returns a pointer to the next state:
            Not accept input => no process.
            Non-transition/InternalTransition (Output only / Mealy machine style outputs), does not proc entry function => function return NULL
            Self-transition, proc entry function => function return pointer to self

        [state_input_t] ids are shared across all states in the same StateMachine, including sub states.
    */
    const State_Input_T * P_TRANSITION_TABLE; /* f(state_input_t) input map. Forms the TransitionFunction. */

    /*
        Effectively extended [P_TRANSITION_TABLE] with user implemented switch()
        return a function pointer, instead of directly returning the State.
        [NULL] for not accepted input.
        Unused by Top Level State.
    */
    State_TransitionMapper_T TRANSITION_MAPPER;

    /*
        Accessor functions
        State controlled value access
        InputMap as user outer call, or unrelated to transitions
        mapped input without transition, no critical section
        If the accessor calls a transition. that function will be wrapped in a critical section.
    */
    const State_Accessor_T * P_ACCESSOR_TABLE;
    const State_GetField_T * P_GET_FIELD_TABLE; /* Get Field Table */
    const State_SetField_T * P_SET_FIELD_TABLE; /* Set Field Table */


    /*
        [Hierarchical State Machine]
        composite pattern
        SubStates mount onto States, maybe defined separately from the StateMachine.

        Defined:
        [ROOT] State - A top state where its [P_PARENT] is [NULL].
        The Common Ancestor of all States may be [NULL] for a multi-root StateMachine.
        [ROOT] States only use [P_TRANSITION_TABLE]
    */
    const struct State * P_PARENT; /* NULL for top level state */

    /* SubStates define P_TOP and DEPTH for runtime optimization */
    uint8_t DEPTH;                  /* Depth of state. Depth must be consistent for iteration */
    const struct State * P_TOP;     /* SubStates maintain pointer to top level state. */


    /* const sub-TYPE context */
    const void * P_EXTENSION;
    // VarAccess_VTable_T optionally include

    /* non const sub-STATE context */
    // void * (* const SUBSTATE_CONTEXT)(void * p_context); /* Retrieve mutable State data from p_context. */

    /* Can be generalized as P_TRANSITION_TABLE[MENU](p_context, direction) */
#ifdef CONFIG_STATE_MACHINE_LINKED_STATES_ENABLE
    struct State * P_LINK_NEXT;
    struct State * P_LINK_PREV;
#endif
}
State_T;


// #define STATE_INIT_WITH_ASSERT(...) {  } \



