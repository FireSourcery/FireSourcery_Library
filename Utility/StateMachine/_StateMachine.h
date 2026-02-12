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
    State_T * p_ActiveState;     /* HSM - The Active Top Level State. Keep the top level fast. ActiveBranch */
    State_T * p_ActiveSubState;  /* HSM - Leaf State, defines full path. ActiveLeaf */
    /* todo single pointer p_ActiveState includes P_ROOT for top level. */

    /*
        Buffered Sync transition. For restricting transition to during proc state only
        Lock depending on user handler
        In this case user still needs to selectively lock on some inputs, where proc state should not run without input completion
        or set sync buffer
    */
    State_T * volatile p_SyncNextState;

    /*
        Sync machine store result until process
        proc the last input of each unique id
        multiple inputs of the same id will overwrite
        lockless operation when [ProcState] priority is higher than [SetInput]
    */
    volatile uint32_t SyncInputMask; /* Bit mask of inputs that have been set. */
    state_value_t SyncInputs[STATE_TRANSITION_TABLE_LENGTH_MAX]; /* alternatively, context pointer or fam */
    /* mapper version needs separate buffer */

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
static inline State_T * StateMachine_GetActiveState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState; }
static inline state_t StateMachine_GetActiveStateId(const StateMachine_Active_T * p_active) { return p_active->p_ActiveState->ID; }
static inline bool StateMachine_IsActiveState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == p_active->p_ActiveState); }
// optionally remove
static inline bool StateMachine_IsActiveStateId(const StateMachine_Active_T * p_active, state_t stateId) { return (stateId == p_active->p_ActiveState->ID); }

/******************************************************************************/
/*
    SubState
*/
/******************************************************************************/
static inline State_T * StateMachine_GetActiveSubState(const StateMachine_Active_T * p_active) { return (p_active->p_ActiveSubState == NULL) ? p_active->p_ActiveState : p_active->p_ActiveSubState; }
/* split branch and substate functions */
// static inline State_T * StateMachine_GetActiveSubState(const StateMachine_Active_T * p_active) { return p_active->p_ActiveSubState; }
static inline bool StateMachine_IsActiveSubState(const StateMachine_Active_T * p_active, State_T * p_state) { return (p_state == StateMachine_GetActiveSubState(p_active)); }

// can only guarantee correct substate id when parent state is known, otherwise may be duplicate ids across different branches. Handle with p_parentState
// static inline state_t _StateMachine_GetActiveSubStateId(const StateMachine_Active_T * p_active) { return p_active->p_ActiveSubState->ID; }
static inline state_t _StateMachine_GetActiveSubStateId(const StateMachine_Active_T * p_active)
    { return (p_active->p_ActiveSubState != p_active->p_ActiveState) ? p_active->p_ActiveSubState->ID : STATE_ID_NULL; }


/* Id indicator, for when the Active TOP state is known, and all direct substate ids are unique */
static inline state_t StateMachine_GetActiveSubStateId(const StateMachine_Active_T * p_active, State_T * p_parent)
    { return (StateMachine_GetActiveSubState(p_active)->P_PARENT == p_parent) ? _StateMachine_GetActiveSubStateId(p_active) : STATE_ID_NULL; }

/* Non unique substate id, handle with p_parentState */
static inline bool StateMachine_IsActiveSubStateId(const StateMachine_Active_T * p_active, State_T * p_parent, state_t substateId)
    { return (StateMachine_GetActiveSubStateId(p_active, p_parent) == substateId); }



/******************************************************************************/
/*!
    Protected Inline Functions
*/
/******************************************************************************/
/* optionally keep context in signature */
static inline void _StateMachine_SetSyncInput(StateMachine_Active_T * p_active, state_input_t id, state_value_t value)
{
    p_active->SyncInputs[id] = value;
    p_active->SyncInputMask |= (1UL << id);
}

/* Handle multiple inputs overwrite */
static inline void _StateMachine_SetSyncTransition(StateMachine_Active_T * p_active, State_T * p_newState)
{
    if (p_newState != NULL) { p_active->p_SyncNextState = p_newState; } /* Overwrite with non NULL only */
    // if (p_active->p_SyncNextState == NULL) { p_active->p_SyncNextState = p_newState; } /* Keep first */
}

/******************************************************************************/
/*!
    Accessor - Value without Transition
*/
/******************************************************************************/
static inline state_value_t _StateMachine_Access(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t valueK, state_value_t valueV)
{
    return State_Access(p_active->p_ActiveState, p_context, id, valueK, valueV);
}

static inline void _StateMachine_SetValue(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t valueK, state_value_t valueV)
{
    State_SetValue(p_active->p_ActiveState, p_context, id, valueK, valueV);
}

static inline state_value_t _StateMachine_GetValue(const StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t valueK)
{
    return State_GetValue(p_active->p_ActiveState, p_context, id, valueK);
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

extern void _StateMachine_Transition(StateMachine_Active_T * p_active, void * p_context, State_T * p_newState);
extern void _StateMachine_TransitionTo(StateMachine_Active_T * p_active, void * p_context, State_T * p_newState);
extern void _StateMachine_ProcSyncOutput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_ProcInput(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);
extern void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_active, void * p_context);
extern void _StateMachine_ProcSyncNextState(StateMachine_Active_T * p_active, void * p_context);

// extern void _StateMachine_SetSyncInput(StateMachine_Active_T * p_active, state_input_t id, state_value_t value);
extern void _StateMachine_ApplyInputSyncTransition(StateMachine_Active_T * p_active, void * p_context, state_input_t id, state_value_t value);

extern void _StateMachine_ProcState(StateMachine_Active_T * p_active, void * p_context);

