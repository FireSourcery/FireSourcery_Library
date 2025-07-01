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
    @file   StateMachine.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "StateMachine.h"
#include "_StateMachine.h"

#include "_State.h"
#include "_State_Node.h"



/******************************************************************************/
/*!
    Public Functions
*/
/******************************************************************************/
/*
    States const strut defined at compile time
*/
void StateMachine_Init(const StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
    /* sync case, no sentenial, case runs dummy input once */
    /* same as case of async unlock with write through dummy input */
    _StateMachine_SetSyncInput(p_active, 0, 0);
    atomic_flag_clear(&p_active->InputSignal);
    _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
}

// void StateMachine_Sync_Init(const StateMachine_T * p_stateMachine)
// {
//     atomic_flag_test_and_set(&p_stateMachine->InputSignal); /* sync single threaded input only case, start locked. */
//     Reset(p_stateMachine);
// }

void StateMachine_Reset(const StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/******************************************************************************/
/*!
    Lock depending on Machine Type
*/
/******************************************************************************/
/******************************************************************************/
/*
    [Synchronous Machine]
    Synchronous Proc State - [ProcState] synchronous to clock/timer
    Synchronous Proc Input - Async user [SetInput] proc synchronously during [ProcState]
        [SetInput] will noy delay periodic [ProcSyncOutput]
        Proc last [SetInput], inputs overwrite. always single threaded proc

    Lock Input buffer only.

    Does not need EnterCritical/DisableIRQ, when:
        [ProcState] thread is higher priority than [SetInput] thread
        [SetInput] calls are on the same thread
        Check accept on [ProcState]

    Multiple [SetInput] calls in between [ProcState] needs to be handled by synchronization mechanism.
        [ProcState] may occur during [SetInput]
        [SyncInput] will not have cleared when a new [SyncInputValue] is set
    E.g.
        p_stateMachine->SyncInputValue = inputValue;
        p_stateMachine->SyncInput = inputId;
        p_stateMachine->SyncInputValue = inputValue;
        StateMachine_Sync_ProcState
        p_stateMachine->SyncInput = inputId;

    Atomic signal skip [ProcState], or spin-wait in low priority input thread

*/
/******************************************************************************/
// void SynchronousMachine_Proc(const StateMachine_T * p_stateMachine)
void StateMachine_Sync_ProcState(const StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireSyncIsr(p_active) == true)  /* set clear/1, lock until next input */
    {
        /*
            Multi-threaded calls to [SetInput] -> use additional sentinel value
            [InputSignal] flag is cleared/1. signal completion for other [SetInput].
                alternatively use semaphore counter, sync overwrite with proc state.

            Single-threaded calls to [SetInput] -> use [InputSignal] as bufferHasData/0, bufferEmpty/1.
            [InputSignal] flag is not cleared/1, always overwritten by [SetInput], bufferHasData/0
            [SyncInput] does not need to clear as additional sentinel value
        */
    #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
        _StateMachine_ProcSyncInput(p_active, p_stateMachine->P_CONTEXT);
    #else
        _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, p_active->SyncInput, p_active->SyncInputValue);
    #endif

        _StateMachine_ReleaseSyncIsr(p_active);
    }

    /* ISR wont interrupt an Input transition in this case.
        All Inputs are [SetSyncInput], all transition are processed Sync, [ProcSyncOutput] will not interrupt a transition  */
    _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);

}

/*!
    [_StateMachine_AcquireSyncInput] blocks [ProcInput] only.
    Multiple calls overwrite the previous, only the last set is processed.

    [StateMachine_IsAcceptInput] may provide an immediate return,
        although may differ when checked again during [ProcState], due to transition or overwrite
*/
void StateMachine_Sync_SetInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    /* Disables [ProcInput] portion of [Sync_ProcState]. */
    if (_StateMachine_AcquireSyncInput(p_active) == true)  /* block ProcState ProcInput */
    {
        _StateMachine_SetSyncInput(p_active, inputId, inputValue);
        _StateMachine_ReleaseSyncInput(p_active);
    }
}

/******************************************************************************/
/*
    [Asynchronous Machine]
    Async input processed on function called. may change state immediately,
    optionally implement periodic [Async_ProcState]
    without SyncInput buffer polling

    Input/State on the same thread:
        Protected functions, no lock, are sufficent. same behavior as synchronous

    Input/State on different threads:
        Critical on input, Disable ProcState when input is processing

*/
/******************************************************************************/
/*
    Proc Common Output
    [ProcState] runs higher priority than [ProcInput], critical is implemented in [ProcInput] only
    when input can be higher priority this must block, prevent transition
*/
// void AsynchronousMachine_ProcSyncOutput(const StateMachine_T * p_stateMachine)
void StateMachine_Async_ProcState(const StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncIsr(p_active) == true) /* Disabled when input is processing, ensure any transition is completed */
    {
        _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);
        _StateMachine_ReleaseAsyncIsr(p_active);
    }
}

/*
    [_StateMachine_AcquireAsyncInput] blocks [ProcState] and other [ProcInput], or disableISR

    Signal Case - blocks [ProcState] and other [ProcInput] without disabling interrupts
        [ProcState] may be skipped
        [ProcInput] must be on the same thread, or a non polling input maybe missed entirely
        [ProcState] at higher prioity will not cause [ProcInput] to be missed
*/
void StateMachine_Async_ProcInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, inputId, inputValue);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}


// /*
//     Implement as a purely Async machine with a common component
// */
// void StateMachine_AsyncInputOutput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue)
// {
//     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

//     if (_StateMachine_AcquireAsyncInput(p_active) == true)
//     {
//         _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, inputId, inputValue);
//         _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);
//         _StateMachine_ReleaseAsyncInput(p_active);
//     }
// }

/******************************************************************************/
/*
    [Combined Semi-Synchronous Machine]
    supports both operation at expense of additional condition check
*/
/******************************************************************************/
// inline void StateMachine_ProcState(const StateMachine_T * p_stateMachine)
// {
//     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

// #if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
//     /* [Async_ProcInput] disables ISR, runs to completion, same as Sync case  */
//     StateMachine_Sync_ProcState(p_active, p_stateMachine->P_CONTEXT);
// #elif defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)
//     /* Checks if an [Async_ProcInput] has the signal, skip until next cycle */
//     /* Proc SyncInput must use sentinel, AsyncInput releases lock without valid SyncInput */
//     if (_StateMachine_AcquireAsyncIsr(p_active) == true)
//     {
//         _StateMachine_ProcState(p_active, p_stateMachine->P_CONTEXT);
//         _StateMachine_ReleaseAsyncIsr(p_active);
//     }
// #endif
// }

/*
    Input Functions
    Handles Top level transitions only
    [State_Input_T] must not return a substate.
*/
inline void StateMachine_ProcInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue)
{
    StateMachine_Async_ProcInput(p_stateMachine, inputId, inputValue);
}

inline void StateMachine_SetInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_input_value_t inputValue)
{
    StateMachine_Sync_SetInput(p_stateMachine, inputId, inputValue);
}


/* Set the State within signal guards. Without check on input. e.g. fault state */
void StateMachine_ForceTransition(const StateMachine_T * p_stateMachine, const State_T * p_state)
{
    assert(p_state != NULL); /* compile time known state */

    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_Transition(p_active, p_stateMachine->P_CONTEXT, p_state);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/*
    Invoke a [Transition] or as a Command
*/
// void StateMachine_InvokeTransitionCmd(const StateMachine_T * p_stateMachine, const State_Cmd_T * p_cmd, state_input_value_t inputValue)
void StateMachine_InvokeTransition(const StateMachine_T * p_stateMachine, const State_TransitionInput_T * p_transition, state_input_value_t inputValue)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (StateMachine_IsActiveState(p_active, p_transition->P_START) == true)
        {
            assert(p_transition->TRANSITION != NULL);
            _StateMachine_TransitionTo(p_active, p_stateMachine->P_CONTEXT, p_transition->TRANSITION(p_stateMachine->P_CONTEXT, inputValue));
        }

        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/*
    Invoke Setter
    SetValueOfStateWith Setter
    State direct inputs. Reject if it is not the active state.
    inputs that only map to 1 state, reduce table size
    per state inputs, only need to check id.
*/
void StateMachine_SetValueWith(const StateMachine_T * p_stateMachine, const State_T * p_state, State_Set_T setter, state_input_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (StateMachine_IsActiveState(p_active, p_state) == true) { setter(p_stateMachine->P_CONTEXT, value); }
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}


/******************************************************************************/
/*
    [SubState] as fixed 2nd level [p_ActiveSubState]
    independent top level
    Transition/Ouput without traversal

    [p_ActiveSubState] may be == p_ActiveState
    [p_ActiveState] is always a Top level state, always DEPTH == 0

    caller call _StateMachine_EndSubState to end loop
*/
/******************************************************************************/
void StateMachine_ProcSubStateInput(const StateMachine_T * p_stateMachine, state_input_t id, state_input_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcSubStateInput(p_active, p_stateMachine->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}
/******************************************************************************/
/* */
/******************************************************************************/


/******************************************************************************/
/*
    [Hierarchical State Machine]
    [p_ActiveState][p_ActiveSubState]
*/
/******************************************************************************/
/* Including Top level state */
void StateMachine_ProcBranch(const StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncIsr(p_active) == true)
    {
        _StateMachine_ProcBranchSyncInput(p_active, p_stateMachine->P_CONTEXT);
        _StateMachine_ProcBranchSyncOutput(p_active, p_stateMachine->P_CONTEXT, NULL);
        _StateMachine_ReleaseAsyncIsr(p_active);
    }
}

void StateMachine_ProcBranchInput(const StateMachine_T * p_stateMachine, state_input_t id, state_input_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_ProcBranchInput(p_active, p_stateMachine->P_CONTEXT, id, value);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

/*
    ActiveState is an Descendant of the selected State.

    result the input handle is any of P_START is in the active branch
    Transitions to the State of p_transition->TRANSITION, defined to be valid at compile time
*/
void StateMachine_InvokeBranchTransition(const StateMachine_T * p_stateMachine, const State_TransitionInput_T * p_transition, state_input_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (StateMachine_IsActiveBranch(p_active, p_transition->P_START) == true)
        {
            assert(State_IsActiveBranch(p_transition->P_START, p_stateMachine->P_ACTIVE->p_ActiveState)); /* ensure substate is in sync with top level state */

            _StateMachine_TraverseTransitionTo(p_active, p_stateMachine->P_CONTEXT, p_transition->TRANSITION(p_stateMachine->P_CONTEXT, value));
        }
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}

// void StateMachine_InvokeBranchTransitionFrom(const StateMachine_T * p_stateMachine, const State_T * p_ , State_Input_T transition, state_input_value_t value)
/*
    Invoke a Branch Transition when
        ActiveState is an Ancestor of the selected State.
*/
void StateMachine_InvokeBranchTransitionFrom(const StateMachine_T * p_stateMachine, const State_T * p_deepest, State_Input_T transition, state_input_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (State_IsActiveBranch(p_deepest, StateMachine_GetActiveSubState(p_active)) == true)
        {
            _StateMachine_TraverseTransitionTo(p_active, p_stateMachine->P_CONTEXT, transition(p_stateMachine->P_CONTEXT, value));
        }
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}


/*
    Effective for all States descending from the selected State
    i.e. the active SubState is below the selected State, in the active branch.
*/
void StateMachine_SetBranchValueWith(const StateMachine_T * p_stateMachine, const State_T * p_state, State_Set_T setter, state_input_value_t value)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        if (StateMachine_IsActiveBranch(p_active, p_state) == true) { setter(p_stateMachine->P_CONTEXT, value); }
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}


