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

// #ifdef  CONFIG_STATE_MACHINE_LOCAL_CRITICAL_ENABLE
#include "System/Critical/Critical.h"
// #endif

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
    Transition Function - defined by user via P_TRANSITION_TABLE
    1 Additional input passed as arguments
    @return returns pointer to new state, if it exists.
            0 - no transition, bypass exit and entry, indicates user defined non transition
            !0 - transition, perform exist and entry. User may return same state, for self transition, proc exit and entry
*/
typedef struct StateMachine_State * (*StateMachine_Transition_T)(void * p_context, statemachine_input_value_t inputValue); // Input Function
typedef void (*StateMachine_Function_T)(void * p_context); // Output Function

/* Alternatively include handle transition, no external input, "clock only" on internal state. Transition occurs last this way */
// typedef struct StateMachine_State * (*StateMachine_StateOutput_T)(void * p_context);

// include additional arg to optimize for r2-r3?
// input only function, check state only, no transition
// typedef void (*StateMachine_Input_T)(void * p_context, statemachine_input_value_t inputValue0, statemachine_input_value_t inputValue1, statemachine_input_value_t inputValue2); // Input Function

/*
    Array Implementation - 2D input table
    Map allocates for all possible transitions/inputs for each state, valid and invalid
    Allocates space for fault transition for invalid inputs
    Array index is input, eliminates search, only space efficient when inputs are common across many states.
    States belonging to the same state machine must have same size maps

    Pointer to array of functions that return a pointer to the next state (No null pointer check, user must supply empty table)
    Not accept input => define null pointer.
    Non-transition (Output only / Mealy machine style outputs), does not proc entry function => function return 0
    Self-transition, proc entry function => function return pointer to self

    Input directly to output functions, without transition, can simply check State ID
*/
typedef const struct StateMachine_State
{
    const statemachine_state_t ID;
    const StateMachine_Transition_T * const P_TRANSITION_TABLE;     /* f(statemachine_input_id_t) input map. Forms the TransitionFunction.  */
    const StateMachine_Function_T LOOP;       /* Output of the State. Synchronous periodic proc. No null pointer check, user must supply empty function */
    const StateMachine_Function_T ENTRY;      /* Common to all transition to current state, including self transition */
#ifdef CONFIG_STATE_MACHINE_EXIT_FUNCTION_ENABLE
    const StateMachine_Function_T EXIT;
#endif
    // void * const P_STATE_CONTEXT;        /* SubState Buffer */
#ifdef CONFIG_STATE_MACHINE_LINKED_MENU_ENABLE
    const struct StateMachine_State * P_LINK_NEXT;
    const struct StateMachine_State * P_LINK_PREV;
#endif
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

// static inline StateMachine_State_T * TransitionFunction(const StateMachine_State_T * p_active, void * p_context, statemachine_input_id_t inputId, statemachine_input_value_t inputValue)
// {
//     return p_active->P_TRANSITION_TABLE[inputId](p_context, inputValue);
// }

// static inline StateMachine_State_T * StateOuput(const StateMachine_State_T * p_state, void * p_context)
// {
//     return p_state->LOOP(p_context);
// }

// static inline StateMachine_State_T * StateTransition(const StateMachine_State_T * p_active, void * p_context, StateMachine_State_T * p_newState)
// {
//     if (p_active->EXIT != NULL) { p_active->EXIT(p_context); }
//     if (p_newState->ENTRY != NULL) { p_newState->ENTRY(p_context); }
//     return p_newState;
// }

static inline statemachine_state_t StateMachine_GetActiveStateId(const StateMachine_T * p_stateMachine) { return p_stateMachine->p_StateActive->ID; }
static inline bool StateMachine_IsActiveState(const StateMachine_T * p_stateMachine, statemachine_state_t stateId) { return (stateId == p_stateMachine->p_StateActive->ID); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void _StateMachine_ProcStateTransition(StateMachine_T * p_stateMachine, const StateMachine_State_T * p_newState);

extern void StateMachine_Init(StateMachine_T * p_stateMachine);
extern void StateMachine_Reset(StateMachine_T * p_stateMachine);

extern void StateMachine_Sync_ProcState(StateMachine_T * p_stateMachine);
extern bool StateMachine_Sync_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

extern void StateMachine_Async_ProcState(StateMachine_T * p_stateMachine);
extern bool StateMachine_Async_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);
// extern bool StateMachine_Async_ProcInputState(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

extern void StateMachine_ProcState(StateMachine_T * p_stateMachine);
extern bool StateMachine_ProcInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);
extern bool StateMachine_SetInput(StateMachine_T * p_stateMachine, statemachine_input_id_t inputId, statemachine_input_value_t inputValue);

#endif


