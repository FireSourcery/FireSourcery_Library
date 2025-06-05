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
    @file   StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "State.h"
#include "Config.h"

#include "System/Critical/Critical.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <assert.h>


/******************************************************************************/
/*
    StateMachine_Def
*/
/******************************************************************************/
// StateMachine_Less
typedef struct StateMachine_Machine
{
    const State_T * const P_STATE_INITIAL;
    const uint8_t TRANSITION_TABLE_LENGTH;  /* state_input_t count. Shared table length for all states, i.e. all states allocate for all inputs */
    // const State_Input_T * const * const PP_TRANSITION_TABLE; [state_t][state_input_t]
    // const State_T STATES[]; /* auto assign a stateid this way */
    // alternatively
    const State_T * const P_STATE_FAULT; /* Optional, if not defined, no fault state */
}
StateMachine_Machine_T;

// void StateMachine_Init(const StateMachine_Active_T * p_stateMachine, void * p_context, const StateMachine_Machine_T * p_initial)
// {
//     _StateMachine_SetSyncInput(p_active, 0, 0);
//     atomic_flag_clear(&p_active->InputSignal);
//     _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
// }

/* StateMachine StateValues */
typedef struct StateMachine_Active StateMachine_Active_T;

/*
    [StateMachine_Context_T]
    In this case of the runtime state _is_ a pointer to STATE in ROM.
    This const descriptor is only shorthands passing P_CONTEXT for now.
    reserve for mixin/interface side implemented functions
*/
typedef const struct StateMachine
{
    /*
        shorthand passing and container_of
        StateMachine_ProcState([P_CONTEXT], &([P_CONTEXT]->STATE_MACHINE))
            ...
            ([P_CONTEXT]->STATE_MACHINE.P_RUNTIME)->p_ActiveState->LOOP([P_CONTEXT])
            ...

        StateMachine_ProcState(&([P_CONTEXT]->STATE_MACHINE))
            ...
            ([P_CONTEXT]->STATE_MACHINE.P_RUNTIME)->p_ActiveState->LOOP(([P_CONTEXT]->STATE_MACHINE.P_CONTEXT))
            ...
    */
    void * P_CONTEXT; /* Base Struct */
    const StateMachine_Machine_T * P_MACHINE; /* Machine def. Const definition of state transition behaviors, via initial state */
    // const StateMachine_Machine_T MACHINE;    /* Also this could be nested by value, or pass at init. */

    StateMachine_Active_T * P_ACTIVE;         /* StateMachine "state" data */

    /*  */
    // const State_T ** pp_ActiveStatesBuffer; alternative to recursive traverse down
    // const void * P_STATE_BUFFERS[STATE_COUNT]; /* a substate buffer for each top state */
}
StateMachine_T;

#define STATE_MACHINE_ACTIVE_ALLOC() (&(StateMachine_Active_T){0})

#define STATE_MACHINE_INIT(p_Context, p_Machine, p_Active) { .P_CONTEXT = (p_Context), .P_MACHINE = (p_Machine), .P_ACTIVE = (p_Active) }
#define STATE_MACHINE_ALLOC(p_Context, p_Machine) STATE_MACHINE_INIT(p_Context, p_Machine, &(StateMachine_Active_T){ 0 })

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void StateMachine_Init(const StateMachine_T * p_stateMachine);
extern void StateMachine_Reset(const StateMachine_T * p_stateMachine);

extern void StateMachine_Sync_ProcState(const StateMachine_T * p_stateMachine);
extern void StateMachine_Sync_SetInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue);
extern void StateMachine_Async_ProcState(const StateMachine_T * p_stateMachine);
extern void StateMachine_Async_ProcInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue);

extern void StateMachine_ProcState(const StateMachine_T * p_stateMachine);
extern void StateMachine_ProcInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue);
extern void StateMachine_SetInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue);

extern void StateMachine_InvokeTransition(const StateMachine_T * p_stateMachine, const State_TransitionInput_T * p_input, state_input_value_t inputValue);
extern void StateMachine_ForceTransition(const StateMachine_T * p_stateMachine, const State_T * p_state);
extern void StateMachine_SetValueWith(const StateMachine_T * p_stateMachine, const State_T * p_state, State_Set_T setter, state_input_value_t value);

/******************************************************************************/
/*
    SubState
*/
/******************************************************************************/
extern void StateMachine_ProcSubStateInput(const StateMachine_T * p_stateMachine, state_input_t id, state_input_value_t value);

/******************************************************************************/
/*
    HSM
*/
/******************************************************************************/
// extern void _StateMachine_ProcBranch_Nested(const StateMachine_T * p_stateMachine);
extern void StateMachine_ProcBranch(const StateMachine_T * p_stateMachine);
extern void StateMachine_ProcBranchInput(const StateMachine_T * p_stateMachine, state_input_t id, state_input_value_t value);
extern void StateMachine_InvokeBranchTransition(const StateMachine_T * p_stateMachine, const State_TransitionInput_T * p_transition, state_input_value_t value);
extern void StateMachine_SetBranchValueWith(const StateMachine_T * p_stateMachine, const State_T * p_state, State_Set_T setter, state_input_value_t value);
