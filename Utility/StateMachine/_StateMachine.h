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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_State.h"
#include "_State_Node.h"
#include "State.h"
#include "Config.h"

#include "System/Critical/Critical.h"

#include <stdatomic.h>

/******************************************************************************/
/*

*/
/******************************************************************************/
typedef struct StateMachine_Active
{
    const State_T * p_ActiveState;     /* The Active Top Level State. Keep the top level fast. ActiveBranch */
    const State_T * p_ActiveSubState;  /* Leaf State, defines full path. ActiveLeaf */

    volatile atomic_flag InputSignal;  /* SignalLock */

    /*
        Sync machine store result until process
        input id per proc cycle
    */
    volatile state_input_t SyncInput;
    volatile state_input_value_t SyncInputValue;
    // volatile state_input_value_t SyncInputStatus;

    // proc the last input of each unique id
    // effectively becomes polling
    // alternatively SyncInputs[TRANSITION_TABLE_LENGTH_MAX]

    // uint32_t FaultFlags;
}
StateMachine_Active_T;


/******************************************************************************/
/*
    Singal Lock
    export as inline for inline thread handling
*/
/******************************************************************************/
/*
    [Async Machine]
    Synchronization of [Input]s and [SyncOutput]
    Selection between DisableIrq and Signal Lock
    alternatively Inputs spin-wait
*/
static inline bool _StateMachine_AcquireAsyncIsr(StateMachine_Active_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)  /* Case of StateMachine ISR is higher than all Inputs */
    (void)p_stateMachine;
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED  /* Case of Input prioirty higher than ISR */
    _Critical_DisableIrq();
#endif
    return true;
#else
    return Critical_AcquireLock(&p_stateMachine->InputSignal);
#endif
}

static inline void _StateMachine_ReleaseAsyncIsr(StateMachine_Active_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    _Critical_EnableIrq();
#endif
#else
    Critical_ReleaseLock(&p_stateMachine->InputSignal);
#endif
}

static inline bool _StateMachine_AcquireAsyncInput(StateMachine_Active_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
    _Critical_DisableIrq();
    return true;
#else
    return Critical_AcquireLock(&p_stateMachine->InputSignal);
#endif
}

static inline void _StateMachine_ReleaseAsyncInput(StateMachine_Active_T * p_stateMachine)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_stateMachine;
    _Critical_EnableIrq();
#else
    Critical_ReleaseLock(&p_stateMachine->InputSignal);
#endif
}

/*!
    [Sync Machine]
    Special case where [ProcStateOutput] is not blocked by [SetInput]
    Lock [SyncInput] buffer only
*/
static inline bool _StateMachine_AcquireSyncIsr(StateMachine_Active_T * p_stateMachine)
{
    return Critical_AcquireLock(&p_stateMachine->InputSignal);
}

static inline void _StateMachine_ReleaseSyncIsr(StateMachine_Active_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    Critical_ReleaseLock(&p_stateMachine->InputSignal);
#else
    (void)p_stateMachine; /* Single threaded input always overwrite, and state output is not blocked */
#endif
}

static inline bool _StateMachine_AcquireSyncInput(StateMachine_Active_T * p_stateMachine)
{
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    /* Multi-threaded do not proceed, if another thread/Input holds the signal */
    return Critical_AcquireLock(&p_stateMachine->InputSignal);
#else
    /* Single threaded input always overwrite. ISR will run to completion. does not need to block Input */
    Critical_AcquireLock(&p_stateMachine->InputSignal);
    return true;
#endif
}

static inline void _StateMachine_ReleaseSyncInput(StateMachine_Active_T * p_stateMachine)
{
    Critical_ReleaseLock(&p_stateMachine->InputSignal);
}



/******************************************************************************/
/*
    Public Getters directly on StateMachine_Active_T
*/
/******************************************************************************/

/******************************************************************************/
/*
    Top Level State
*/
/******************************************************************************/
static inline const State_T * StateMachine_GetActiveState(const StateMachine_Active_T * p_fields) { return p_fields->p_ActiveState; }
static inline state_t StateMachine_GetActiveStateId(const StateMachine_Active_T * p_fields) { return p_fields->p_ActiveState->ID; }
static inline bool StateMachine_IsActiveState(const StateMachine_Active_T * p_fields, const State_T * p_state) { return (p_state == p_fields->p_ActiveState); }
static inline bool StateMachine_IsActiveStateId(const StateMachine_Active_T * p_fields, state_t stateId) { return (stateId == p_fields->p_ActiveState->ID); }

/******************************************************************************/
/*
    SubState
*/
/******************************************************************************/
/* Checked on get and set */
/* chck handling */
static inline void _StateMachine_EndSubState(StateMachine_Active_T * p_fields) { p_fields->p_ActiveSubState = p_fields->p_ActiveState; }

static inline const State_T * StateMachine_GetActiveSubState(const StateMachine_Active_T * p_fields) { return (p_fields->p_ActiveSubState == NULL) ? p_fields->p_ActiveState : p_fields->p_ActiveSubState; }

/* Id indicator, for when the Active TOP state is known. use for serialization. */
static inline state_t _StateMachine_GetActiveSubStateId(const StateMachine_Active_T * p_fields) { return StateMachine_GetActiveSubState(p_fields)->ID; }

static inline bool StateMachine_IsActiveSubState(const StateMachine_Active_T * p_fields, const State_T * p_state) { return (p_state == StateMachine_GetActiveSubState(p_fields)); }
/* Non unique substate id, handle with p_parentState */
static inline bool StateMachine_IsActiveSubStateId(const StateMachine_Active_T * p_fields, const State_T * p_parent, state_t subStateId)
    { return StateMachine_IsActiveState(p_fields, p_parent) && (subStateId == p_fields->p_ActiveSubState->ID); }

/* Is within the active branch */
/* State is in the active branch. Ancestor of the ActiveSubState */
static inline bool StateMachine_IsActiveBranch(const StateMachine_Active_T * p_fields, const State_T * p_state) { return State_IsActiveBranch(StateMachine_GetActiveSubState(p_fields), p_state); }

/* Ancestor or Descendant */
static inline bool StateMachine_IsDirectBranch(const StateMachine_Active_T * p_fields, const State_T * p_state) { return State_IsDirectBranch(StateMachine_GetActiveSubState(p_fields), p_state); }

/* split branch and substate functions */
// static inline const State_T * StateMachine_GetActiveSubState(const StateMachine_Active_T * p_fields) { return p_fields->p_ActiveSubState; }
// static inline const State_T * StateMachine_GetActiveLeafState(const StateMachine_Active_T * p_fields) { return (p_fields->p_ActiveSubState == NULL) ? p_fields->p_ActiveState : p_fields->p_ActiveSubState; }


/******************************************************************************/
/*!
    Extern Protected Functions
    Caller implement thread synchronization
    Selectively implement critical in calling layer, if not require for all inputs
*/
/******************************************************************************/
/*
    On internal state without critical section lock
*/
extern void _StateMachine_Init(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_initialState);
extern void _StateMachine_Transition(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_newState);
extern void _StateMachine_TransitionTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_newState);
extern void _StateMachine_ProcSyncOutput(StateMachine_Active_T * p_fields, void * p_context);
extern void _StateMachine_ProcInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value);
extern void _StateMachine_ProcSyncInput(StateMachine_Active_T * p_fields, void * p_context);
extern void _StateMachine_SetSyncInput(StateMachine_Active_T * p_fields, state_input_t id, state_input_value_t value);

/*  */
extern void _StateMachine_ProcState(StateMachine_Active_T * p_fields, void * p_context);

extern void _StateMachine_TransitionSubState(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state);
extern void _StateMachine_TransitionSubStateTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state);
extern void _StateMachine_ProcSubStateInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value);
extern void _StateMachine_ProcSubState_Nested(StateMachine_Active_T * p_fields, void * p_context);

extern void _StateMachine_TraverseTransition(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state);
extern void _StateMachine_TraverseTransitionTo(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state);
extern void _StateMachine_ProcBranchSyncOutput(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_limit);
extern void _StateMachine_ProcBranchInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t id, state_input_value_t value);
extern void _StateMachine_ProcBranchSyncInput(StateMachine_Active_T * p_fields, void * p_context);
extern void _StateMachine_ProcBranch_Nested(StateMachine_Active_T * p_fields, void * p_context);

#ifdef STATE_MACHINE_IMPLEMENTATION
/*
    extern implementation
*/
// void _StateMachine_Sync_ProcState(StateMachine_Active_T * p_fields, void * p_context);
// void _StateMachine_Sync_SetInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t inputId, state_input_value_t inputValue);
// void _StateMachine_Async_ProcState(StateMachine_Active_T * p_fields, void * p_context);
// void _StateMachine_Async_ProcInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t inputId, state_input_value_t inputValue);
// void _StateMachine_ProcState(StateMachine_Active_T * p_fields, void * p_context);
// void _StateMachine_ProcInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t inputId, state_input_value_t inputValue);
// void _StateMachine_SetInput(StateMachine_Active_T * p_fields, void * p_context, state_input_t inputId, state_input_value_t inputValue);
// void _StateMachine_InvokeTransition(StateMachine_Active_T * p_fields, void * p_context, const State_TransitionInput_T * p_transition, state_input_value_t inputValue);
// void _StateMachine_ForceTransition(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state);
// void _StateMachine_SetValueWith(StateMachine_Active_T * p_fields, void * p_context, const State_T * p_state, State_Set_T setter, state_input_value_t value);
#endif