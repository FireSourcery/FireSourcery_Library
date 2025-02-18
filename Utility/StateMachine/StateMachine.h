/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @brief     StateMachine module
    @version V0
*/
/******************************************************************************/
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Config.h"

#include "System/Critical/Critical.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <sys/types.h>

#include <assert.h>

#define STATE_MACHINE_INPUT_ID_NULL         (0xFFU)
#define STATE_MACHINE_INPUT_VALUE_NULL      (0U)

typedef uint8_t state_machine_input_id_t;       /* Input ID/Category. Index into transition table. User may overwrite with enum. */
typedef intptr_t state_machine_input_value_t;   /* User define platform register size */

// id is not passed in a function pointer. will not affect function pointer casting compatibility
// typedef enum uint8_t { STATE_MACHINE_INPUT_ID_NULL = 0xFF } state_machine_input_id_t; /* Input ID/Category. Index into transition table. User may overwrite with enum. */

typedef uint8_t state_machine_state_t;          /* State ID. User may overwrite with enum */
struct StateMachine_State;

/*!
    Transition Input Functions
    Forms the Transition Function - defined by user via P_TRANSITION_TABLE
    1 Additional input passed as arguments
    @return returns pointer to new state, if it exists.
            NULL - no transition, bypass exit and entry, indicates user defined non transition
            StateMachine_State * - transition, perform exit and entry.
                User may return same state, for self transition, proc exit and entry
*/
/* Transition Input with context and table id */
typedef struct StateMachine_State * (*StateMachine_Transition_T)(void * p_context);
/* Transition Input with value parameter */
typedef struct StateMachine_State * (*StateMachine_Input_T)(void * p_context, state_machine_input_value_t inputValue); /* Input Handler */

/* A State Function. Depends only on the State and internal values and index id. alternatively as output. */
/* TransitionAction/Output */
typedef void (*StateMachine_Function_T)(void * p_context);

/* Include handle transition, no external input, "clock only" on internal state. alternatively Handler calls _ProcStateTransition */
// typedef struct StateMachine_State * (*StateMachine_Function_T)(void * p_context);

/*  */
// typedef state_machine_state_value_t(*StateMachine_Get_T)(void * p_context);
// typedef void(*StateMachine_Set_T)(void * p_context, state_machine_state_value_t inputValue);
typedef void (*StateMachine_CmdInput_T)(void * p_context, state_machine_input_value_t inputValue); // Non Transition Input Function

/* InnerState/Interface Functions */
// typedef const struct StateMachine_StateInternal
// {
//     const StateMachine_Function_T LOOP;
//     const StateMachine_Function_T ENTRY;
// #ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
//     const StateMachine_Function_T EXIT;
// #endif
// }
// StateMachine_StateInternal_T;

/******************************************************************************/
/******************************************************************************/
// i/o mapping to multiple states, and without transition
/* Keyd for implementation handle */
typedef state_machine_input_value_t(*StateMachine_Get_T)(void * p_context, state_machine_input_value_t inputK);
typedef void (*StateMachine_Set_T)(void * p_context, state_machine_input_value_t inputK, state_machine_input_value_t inputV);

// typedef enum StateMachine_Status
// {
//     STATE_MACHINE_INPUT_ACCEPTED,
//     STATE_MACHINE_INPUT_REJECTED,
//     STATE_MACHINE_ACQUIRE_LOCK_FAILED,
// }
// StateMachine_Status_T;


    /*
        SubState/Cmd
    */
    // Inputs maping to a single state, and without transition, can simply check State ID
    typedef const struct StateMachine_CmdPart
    {
        const struct StateMachine_State * * P_STATE;
        /* LinkedList of StateLogic. Function to determine next State. */
        const StateMachine_Transition_T NEXT; /* overriding each substate */
    }
    StateMachine_CmdPart_T;

    typedef const struct StateMachine_Cmd
    {
        /* require repeat substates, remap function */
        const StateMachine_CmdInput_T CMD;
        const struct StateMachine_State * P_INITIAL; /* reimplement states with NEXT */
        // state_machine_state_t INITIAL_ID;

        // as cmdsubstate need additional invoker logic
        // StateMachine_StateInternal_T * P_INTERNAL; // slightly more reuse
        // StateMachine_Transition_T NEXT;;

        // StateMachine_CmdPart_T * P_CHAIN; // remap next
    }
    StateMachine_Cmd_T;

/*  */
typedef const struct StateMachine_State
{
    /*
        Internal
    */
    // const StateMachine_StateInternal_T INTERNAL;
    const state_machine_state_t ID;             /* Allow States to remain private, by calling via id */
    const StateMachine_Function_T ENTRY;        /* Common to all transition to current state, including self transition */
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    const StateMachine_Function_T EXIT;
#endif
    const StateMachine_Function_T LOOP;      /* Output of the State. Synchronous periodic proc. No null pointer check, user must supply empty function */

    // init with param for substate
    // StateMachine_CmdInput_T CMD;

    /*
        Transition Control
    */

    /* Handle Transition, no external input, "clock only" on internal state. */
    /* alternatively Handler calls _ProcStateTransition */
    /* alternatively include param for external call  */
    const StateMachine_Transition_T NEXT; /* Separate internal input/event transition handler from OUTPUT for SubState regularity */

    // /* input handler without array mapping. for substate. implementation handle id look up */
    // struct StateMachine_State * (*INPUT_HANDLER)(void * p_context, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);

    /*
        Array Implementation - 2D input table
        Map allocates for all possible transitions/inputs for each state, valid and invalid
        Allocates space for fault transition for invalid inputs
        Array index as [state_machine_input_id_t], eliminates search, only space efficient when inputs are common across many states.
        States belonging to the same state machine must have same size maps

        Pointer to array of functions that return a pointer to the next state (No null pointer check, user must supply empty table)
        Not accept input => no process.
        Non-transition (Output only / Mealy machine style outputs), does not proc entry function => function return NULL
        Self-transition, proc entry function => function return pointer to self
    */
    const StateMachine_Input_T * const P_TRANSITION_TABLE; /* f(state_machine_input_id_t) input map. Forms the TransitionFunction.  */

    /*
        for composite pattern, NULL for top level state
        SubStates mount onto States
    */
    const struct StateMachine_State * const P_HOST;
    const state_machine_state_t HOST_ID;
    // const struct StateMachine_State * const P_INITIAL_SUB_STATE;

    // pass through condition logic to implementation
    // StateMachine_Get_T GET;
    // StateMachine_Set_T SET;

    /* Can be generalized as P_TRANSITION_TABLE[MENU](p_context, direction) */
#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
    const struct StateMachine_State * P_LINK_NEXT;
    const struct StateMachine_State * P_LINK_PREV;
#endif
}
StateMachine_State_T;

typedef struct StateMachine_Machine
{
    const StateMachine_State_T * const P_STATE_INITIAL;
    // const StateMachine_Input_T * const * const PP_TRANSITION_TABLE; [state_machine_state_t][state_machine_input_id_t]
    const uint8_t TRANSITION_TABLE_LENGTH;  /* state_machine_input_id_t count. Shared table length for all states, i.e. all states allocate for all inputs */
}
StateMachine_Machine_T;

/*

*/
typedef const struct StateMachine_Const
{
    const StateMachine_Machine_T * const P_MACHINE;         /* Const definition of state transition behaviors */
    // const StateMachine_State_T * const P_STATE_INITIAL;
    // const uint8_t TRANSITION_TABLE_LENGTH;
    void * const P_CONTEXT;                                 /* Mutable state information per state machine. Includes ref to StateMachine */
    const uint32_t * P_TIMER;   /* Auto handle timed state */
}
StateMachine_Const_T;

typedef struct StateMachine
{
    const StateMachine_Const_T CONST;
    const StateMachine_State_T * p_StateActive;

    volatile atomic_flag InputSignal;
    /* Sync machine store result until process */
    volatile state_machine_input_id_t SyncInput;
    volatile state_machine_input_value_t SyncInputValue;
    // volatile state_machine_value_t SyncInputStatus;

    // volatile state_machine_input_value_t InternalInput;

    /* Const State do not contain SubState, instead nested SubStates use array */
    const StateMachine_State_T * p_SubState; /* Active SubState */

    // StateMachine_CmdChain_T CmdChain;
}
StateMachine_T;

#define STATE_MACHINE_INIT(p_Machine, p_Context, ...)           \
{                                                               \
    .CONST =                                                    \
    {                                                           \
        .P_MACHINE = p_Machine,                                 \
        .P_CONTEXT = p_Context,                                 \
    }                                                           \
}

static inline state_machine_state_t StateMachine_GetActiveStateId(const StateMachine_T * p_stateMachine) { return p_stateMachine->p_StateActive->ID; }
static inline bool StateMachine_IsActiveStateId(const StateMachine_T * p_stateMachine, state_machine_state_t stateId) { return (stateId == p_stateMachine->p_StateActive->ID); }

static inline bool StateMachine_IsActiveState(const StateMachine_T * p_stateMachine, const StateMachine_State_T * p_state) { return (p_state == p_stateMachine->p_StateActive); }

/*!
    @return false indicates not accepted input, transition does not exist, no mapped input function.
            true indicates accepted input, state may transition or self transition (with or without entry and exit function).
*/
// static inline bool StateMachine_IsAcceptInput(const StateMachine_T * p_stateMachine, state_machine_input_id_t inputId)
// {
//     return ((inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH) && (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId] != NULL));
// }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState);

extern void _StateMachine_ProcStateOutput(StateMachine_T * p_stateMachine);
extern void _StateMachine_ProcSyncInput(StateMachine_T * p_stateMachine);
extern void _StateMachine_ProcAsyncInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);
extern void _StateMachine_SetSyncInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);

extern void StateMachine_Init(StateMachine_T * p_stateMachine);
extern void StateMachine_Reset(StateMachine_T * p_stateMachine);

extern void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);

extern void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);

extern void _StateMachine_ProcSubState(StateMachine_T * p_stateMachine);
extern void _StateMachine_EndSubState(StateMachine_T * p_stateMachine);

extern void StateMachine_SetSubState(StateMachine_T * p_stateMachine, StateMachine_State_T * p_subState);
extern void StateMachine_StartCmd(StateMachine_T * p_stateMachine, StateMachine_Cmd_T * p_cmd, state_machine_input_value_t inputValue);

extern void StateMachine_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_ProcInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);
extern void StateMachine_SetInput(StateMachine_T * p_stateMachine, state_machine_input_id_t inputId, state_machine_input_value_t inputValue);

#endif


