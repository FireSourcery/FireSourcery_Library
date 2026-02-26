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
    @file   _StateMachine.h
    @author FireSourcery
    @brief  StateMachine
            Selectively inline API functions
            Handle implementations with context as parameter
*/
/******************************************************************************/
#include "_State.h"
#include "_State_Node.h"
#include "State.h"
#include "Config.h"

#include <stdatomic.h>

/******************************************************************************/
/*
    Protected - For Derived StateMachine Use
*/
/******************************************************************************/
typedef struct StateMachine_Active
{
    State_T * p_ActiveState;    /* Flat - The Active Root/Top Level State. */
                                /* HSM - Leaf State, defines full path. */

    /*
        Sync machine store inputs until [ProcState]
        proc the last input of each unique id
        multiple inputs of the same id will overwrite
        lockless operation when [ProcState] priority is higher than [SetInput] and [SetInput] is single threaded.
    */
    volatile uint32_t SyncInputMask; /* Bit mask of inputs that have been set. */
    state_value_t SyncInputs[STATE_TRANSITION_TABLE_LENGTH_MAX]; /* alternatively, context pointer or fam */
    /* mapper version needs separate buffer */

    /*
        Async Input handler, Buffered Sync transition. Restrict transition to during proc state only
        Lock depending on user handler
        In this case user still needs to selectively lock on some inputs, where proc state should not run without input completion
    */
    State_T * volatile p_SyncNextState;

    volatile atomic_flag InputSignal; /* SignalLock */

    // uint32_t FaultFlags;
}
StateMachine_Active_T;

/******************************************************************************/
/*
    Top Level State
*/
/******************************************************************************/

/******************************************************************************/
/*
    Public Getters directly on StateMachine_Active_T
*/
/******************************************************************************/
/*
    HSM use tree functions
    These now return the leaf state.
    otherwire preprocessor alias to top state for direct input to top level.
*/
static inline State_T * StateMachine_GetActiveState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState; }
static inline state_t StateMachine_GetActiveStateId(const StateMachine_Active_T * p_active) { return StateMachine_GetActiveState(p_active)->ID; }
static inline bool StateMachine_IsActiveState(const StateMachine_Active_T * p_active, State_T * p_state) { return (StateMachine_GetActiveState(p_active) == p_state); }
static inline bool StateMachine_IsActiveStateId(const StateMachine_Active_T * p_active, state_t stateId) { return (StateMachine_GetActiveState(p_active)->ID == stateId); }



/******************************************************************************/
/*!
    Protected Inline Functions
*/
/******************************************************************************/
static inline void _StateMachine_SetSyncInput(StateMachine_Active_T * p_active, state_input_t id, state_value_t value)
{
    assert(id < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
    p_active->SyncInputs[(uint8_t)id] = value;
    p_active->SyncInputMask |= (1UL << (uint8_t)id);
}

/* Handle multiple inputs overwrite */
static inline void _StateMachine_SetSyncTransition(StateMachine_Active_T * p_active, State_T * p_next)
{
    if (p_next != NULL) { p_active->p_SyncNextState = p_next; } /* Overwrite with non NULL only */
    // if (p_active->p_SyncNextState == NULL) { p_active->p_SyncNextState = p_next; } /* Keep first */
}

/******************************************************************************/
/*!
    Accessor - Value without Transition
*/
/******************************************************************************/
static inline state_value_t _StateMachine_Cmd(const StateMachine_Active_T * p_active, void * p_context, state_cmd_t id, state_value_t value)
{
    return State_Cmd(StateMachine_GetActiveState(p_active), p_context, id,  value);
}

static inline void _StateMachine_SetValue(const StateMachine_Active_T * p_active, void * p_context, state_accessor_t id, state_value_t field, state_value_t value)
{
    State_SetValue(StateMachine_GetActiveState(p_active), p_context, id, field, value);
}

static inline state_value_t _StateMachine_GetValue(const StateMachine_Active_T * p_active, void * p_context, state_accessor_t id, state_value_t field)
{
    return State_GetValue(StateMachine_GetActiveState(p_active), p_context, id, field);
}

/******************************************************************************/
/*!
    Extern Protected Functions
    Caller implement thread synchronization
    Selectively implement critical in calling layer, if not require for all inputs
*/
/******************************************************************************/
// extern void _StateMachine_Init(StateMachine_Active_T * p_active, void * p_context, State_T * p_initialState);
extern void _StateMachine_Reset(StateMachine_Active_T * p_active, void * p_context, State_T * p_initialState);

extern void _StateMachine_TransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_next);
extern void _StateMachine_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_next);
extern void _StateMachine_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_CallInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);
extern void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context);

extern void _StateMachine_ProcSyncNextState(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_ApplyInputSyncTransition(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_InvokeTransition(StateMachine_Active_T * p_active, void * p_context, State_T * p_start, State_Input_T input, state_value_t value);

extern void _StateMachine_ProcState(StateMachine_Active_T * p_active, void * p_context);

