// #pragma once

// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2025 FireSourcery

//     This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
// */
// /******************************************************************************/
// /******************************************************************************/
// /*!
//     @file   StateMachine_Sync.h
//     @author FireSourcery
//     @brief  [Brief description of the file]
// */
// /******************************************************************************/
// #include "../StateMachine.h"

// /******************************************************************************/
// /*!
//     Specialized Version
//     if base case updated to handling transitions as sync,
//     this only provide sync on input value
// */
// /******************************************************************************/

// // extern void StateMachine_Sync_ProcState(const StateMachine_T * p_stateMachine);
// // extern void StateMachine_Sync_SetInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue);
// // extern void StateMachine_Async_ProcState(const StateMachine_T * p_stateMachine);
// // extern void StateMachine_Async_ProcInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue);


// /******************************************************************************/
// /*
//     [Synchronous Machine]
//     Synchronous Proc State - [ProcState] synchronous to clock/timer
//     Synchronous Proc Input - Async user [SetInput] proc synchronously during [ProcState]
//         [SetInput] will noy delay periodic [ProcSyncOutput]
//         Proc last [SetInput], inputs overwrite. always single threaded proc

//     Lock Input buffer only.

//     Does not need EnterCritical/DisableIRQ, when:
//         [ProcState] thread is higher priority than [SetInput] thread
//         [SetInput] calls are on the same thread
//         Check accept on [ProcState]

//     Atomic signal skip [ProcState], or spin-wait in low priority input thread

// */
// /******************************************************************************/
// // void SynchronousMachine_Proc(const StateMachine_T * p_stateMachine)
// static void StateMachine_Sync_ProcState(const StateMachine_T * p_stateMachine)
// {
//     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

//     if (_StateMachine_AcquireSyncIsr(p_active) == true)  /* set clear/1, lock until next input */
//     {
//         /*
//             Multi-threaded calls to [SetInput] -> use additional sentinel value
//             [InputSignal] flag is cleared/1. signal completion for other [SetInput].
//                 alternatively use semaphore counter, sync overwrite with proc state.

//             Single-threaded calls to [SetInput] -> use [InputSignal] as bufferHasData/0, bufferEmpty/1.
//             [InputSignal] flag is not cleared/1, always overwritten by [SetInput], bufferHasData/0
//             [SyncInput] does not need to clear as additional sentinel value
//         */
//     #if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
//         _StateMachine_ProcSyncInput(p_active, p_stateMachine->P_CONTEXT);
//     #else
//         _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, p_active->SyncInput, p_active->SyncInputValue);
//     #endif

//         _StateMachine_ReleaseSyncIsr(p_active);
//     }

//     /* ISR wont interrupt an Input transition in this case.
//         All Inputs are [SetSyncInput], all transition are processed Sync, [ProcSyncOutput] will not interrupt a transition  */
//     _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);

// }



// /*!
//     [_StateMachine_AcquireSyncInput] blocks [ProcInput] only.
//     Multiple calls overwrite the previous, only the last set is processed.

//     [StateMachine_IsAcceptInput] may provide an immediate return,
//         although may differ when checked again during [ProcState], due to transition or overwrite
// */
// static void StateMachine_Sync_SetInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
// {
//     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

//     /* Disables [ProcInput] portion of [Sync_ProcState]. */
//     if (_StateMachine_AcquireSyncInput(p_active) == true)  /* block ProcState ProcInput */
//     {
//         _StateMachine_SetSyncInput(p_active, inputId, inputValue);
//         _StateMachine_ReleaseSyncInput(p_active);
//     }
// }

// // void StateMachine_Sync_Init(const StateMachine_T * p_stateMachine)
// // {
// //     Critical_AcquireLock(&p_active->InputSignal); /* sync single threaded input only case, start locked. */
// //     Reset(p_stateMachine);
// // }

// /******************************************************************************/
// /*
//     [Asynchronous Machine]
//     Async input processed on function called. may change state immediately,
//     optionally implement periodic [Async_ProcState]
//     without SyncInput buffer polling

//     Input/State on the same thread:
//         Protected functions, no lock, are sufficent. same behavior as synchronous

//     Input/State on different threads:
//         Critical on input, Disable ProcState when input is processing

// */
// /******************************************************************************/
// /*
//     [_StateMachine_AcquireAsyncInput] blocks [ProcState] and other [ProcInput], or disableISR

//     Signal Case - blocks [ProcState] and other [ProcInput] without disabling interrupts
//         [ProcState] may be skipped
//         [ProcInput] must be on the same thread, or a non polling input maybe missed entirely
//         [ProcState] at higher prioity will not cause [ProcInput] to be missed
// */
// static void StateMachine_Async_ProcInput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
// {
//     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

//     if (_StateMachine_AcquireAsyncInput(p_active) == true)
//     {
//         _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, inputId, inputValue);
//         _StateMachine_ReleaseAsyncInput(p_active);
//     }
// }


// // /*
// //     Implement as a purely Async machine with a common component
// // */
// // void StateMachine_Async_ProcInputOutput(const StateMachine_T * p_stateMachine, state_input_t inputId, state_value_t inputValue)
// // {
// //     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

// //     if (_StateMachine_AcquireAsyncInput(p_active) == true)
// //     {
// //         _StateMachine_ProcInput(p_active, p_stateMachine->P_CONTEXT, inputId, inputValue);
// //         _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);
// //         _StateMachine_ReleaseAsyncInput(p_active);
// //     }
// // }

// /*
//     Proc Common Output
//     [ProcState] runs higher priority than [ProcInput], critical is implemented in [ProcInput] only
//     when input can be higher priority this must block, prevent transition
// */
// // void AsynchronousMachine_ProcSyncOutput(const StateMachine_T * p_stateMachine)
// // void StateMachine_Async_ProcCommon(const StateMachine_T * p_stateMachine)
// // void StateMachine_Async_ProcState(const StateMachine_T * p_stateMachine)
// // {
// //     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

// //     if (_StateMachine_AcquireAsyncIsr(p_active) == true) /* Disabled when input is processing, ensure any transition is completed */
// //     {
// //         _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);
// //         _StateMachine_ReleaseAsyncIsr(p_active);
// //     }
// // }
