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

#define STATE_MACHINE_INPUT_ID_NULL         (0xFFU)
#define STATE_MACHINE_INPUT_VALUE_NULL      (0U)

typedef uint8_t  statemachine_input_id_t;       /* Input ID/Category. Index into transition table. User may overwrite with enum. */
typedef uint32_t statemachine_input_value_t;    /* User define platform register size */

// id is not passed in a function pointer. will not affect function pointer casting compatibility
// typedef enum uint8_t { STATE_MACHINE_INPUT_ID_NULL = 0xFF } statemachine_input_id_t; /* Input ID/Category. Index into transition table. User may overwrite with enum. */
// typedef register_t statemachine_input_value_t;  /* User define platform register size */

typedef uint8_t statemachine_state_t;           /* State ID. User may overwrite with enum */
struct StateMachine_State;

/*!
    Input
    Forms the Transition Function - defined by user via P_TRANSITION_TABLE
    1 Additional input passed as arguments
    @return returns pointer to new state, if it exists.
            NULL - no transition, bypass exit and entry, indicates user defined non transition
            StateMachine_State * - transition, perform exit and entry. User may return same state, for self transition, proc exit and entry
*/
// typedef struct StateMachine_State * (*StateMachine_Transition_T)(void * p_context);
typedef struct StateMachine_State * (*StateMachine_Transition_T)(void * p_context, statemachine_input_value_t inputValue);
/* Include handle transition, no external input, "clock only" on internal state. Transition occurs last this way */
typedef struct StateMachine_State * (*StateMachine_StateOutput_T)(void * p_context);
typedef void (*StateMachine_Function_T)(void * p_context); // OnTransition Function

// include additional arg to optimize for r2-r3?
// additional input tables, for inputs that map to many states, without transition
// input only function, check state only, no transition
// Inputs maping to a single state, and without transition, can simply check State ID
// typedef uint32_t(*StateMachine_Get_T)(void * p_context);
// typedef void(*StateMachine_Set_T)(void * p_context, statemachine_input_value_t inputValue);
// typedef void (*StateMachine_SetEntry_T)(void * p_context, statemachine_input_value_t inputK, statemachine_input_value_t inputV); // Input Function

/*
    Array Implementation - 2D input table
    Map allocates for all possible transitions/inputs for each state, valid and invalid
    Allocates space for fault transition for invalid inputs
    Array index as [statemachine_input_id_t], eliminates search, only space efficient when inputs are common across many states.
    States belonging to the same state machine must have same size maps

    Pointer to array of functions that return a pointer to the next state (No null pointer check, user must supply empty table)
    Not accept input => no process.
    Non-transition (Output only / Mealy machine style outputs), does not proc entry function => function return NULL
    Self-transition, proc entry function => function return pointer to self
*/
typedef const struct StateMachine_State
{
    const statemachine_state_t ID;
    const StateMachine_Transition_T * const P_TRANSITION_TABLE;     /* f(statemachine_input_id_t) input map. Forms the TransitionFunction.  */
    const StateMachine_StateOutput_T LOOP;       /* Output of the State. Synchronous periodic proc. No null pointer check, user must supply empty function */
    const StateMachine_Function_T ENTRY;         /* Common to all transition to current state, including self transition */
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    const StateMachine_Function_T EXIT;
#endif

#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
    const struct StateMachine_State * P_LINK_NEXT;
    const struct StateMachine_State * P_LINK_PREV;
#endif
// #ifdef CONFIG_STATE_MACHINE_GETTERS_SETTERS_ENABLE
//     const StateMachine_Get_T * const P_GETTER_TABLE;
//     const StateMachine_Set_T * const P_SETTER_TABLE;
// #endif

    // void * const P_STATE_CONTEXT;        /* SubState Buffer */
}
StateMachine_State_T;

typedef struct StateMachine_Machine
{
    const StateMachine_State_T * const P_STATE_INITIAL;
    const uint8_t TRANSITION_TABLE_LENGTH;     /* statemachine_input_id_t count. Shared table length for all states, i.e. all states allocate for all inputs */
}
StateMachine_Machine_T;

typedef const struct StateMachine_Const
{
    const StateMachine_Machine_T * const P_MACHINE;         /* Const definition of state transition behaviors */
    void * const P_CONTEXT;                                 /* Mutable state information per state machine. Alternatively, in each function */
}
StateMachine_Const_T;

typedef struct StateMachine
{
    const StateMachine_Const_T CONST;
    const StateMachine_State_T * p_StateActive;

    /* Sync machine store result until process */
    volatile statemachine_input_id_t SyncInput;
    volatile statemachine_input_value_t SyncInputValue;

    volatile atomic_flag InputSignal;
}
StateMachine_T;

#define STATE_MACHINE_INIT(p_Machine, p_Context, UseCritical)   \
{                                                               \
    .CONST =                                                    \
    {                                                           \
        .P_MACHINE = p_Machine,                                 \
        .P_CONTEXT = p_Context,                                 \
    }                                                           \
}

static inline statemachine_state_t StateMachine_GetActiveStateId(const StateMachine_T * p_stateMachine) { return p_stateMachine->p_StateActive->ID; }
static inline bool StateMachine_IsActiveState(const StateMachine_T * p_stateMachine, statemachine_state_t stateId) { return (stateId == p_stateMachine->p_StateActive->ID); }

/*!
    @return false indicates not accepted input, transition does not exist, no mapped input function.
            true indicates accepted input, state may transition or self transition (with or without entry and exit function).
*/
static inline bool StateMachine_IsAcceptInput(const StateMachine_T * p_stateMachine, statemachine_input_id_t inputId)
{
    return ((inputId < p_stateMachine->CONST.P_MACHINE->TRANSITION_TABLE_LENGTH) && (p_stateMachine->p_StateActive->P_TRANSITION_TABLE[inputId] != NULL));
}


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState);

extern void _StateMachine_ProcStateOutput(StateMachine_T * p_stateMachine);
extern void _StateMachine_ProcSyncInput(StateMachine_T * p_stateMachine);
extern void _StateMachine_ProcAsyncInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);
extern void _StateMachine_SetSyncInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

extern void StateMachine_Init(StateMachine_T * p_stateMachine);
extern void StateMachine_Reset(StateMachine_T * p_stateMachine);

extern void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

extern void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);
// extern bool StateMachine_Async_ProcInputState(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

extern void StateMachine_ProcState(StateMachine_T * p_stateMachine);
extern void StateMachine_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);
extern void StateMachine_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

#endif


