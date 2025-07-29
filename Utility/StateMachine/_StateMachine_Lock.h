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
    @file   _StateMachine_Lock.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_StateMachine.h"

#include "System/Critical/Critical.h"

/******************************************************************************/
/*
    Signal Lock
    export as inline for inline thread handling
    user can directly call _StateMachine_() private functions for lockless operation
*/
/******************************************************************************/
#if     defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL) /* Disable all system IRQs for entire duration of processing Input. */
#elif   defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)   /* Disables ProcState only */
#else
    #define CONFIG_STATE_MACHINE_ASYNC_LOCK_FREE
#endif

/*
    [Async Machine]
    Synchronization of [Input]s and [SyncOutput]
    Selection between DisableIrq and Signal Lock
    alternatively Inputs spin-wait
*/
/* [ProcState] runs higher priority than[ProcInput], critical is implemented in[ProcInput] only */
static inline bool _StateMachine_AcquireAsyncIsr(StateMachine_Active_T * p_active)
{
    /* Case of [Async_ProcInputTransition] on [Async_ProcInput] */
    /* Disabled when input is processing. ensure any transition is completed */
    /* Checks if an [Async_ProcInput] has the signal, skip until next cycle */
    /* [Async_ProcInput] disables ISR, runs to completion => same as SynchronousMachine case */
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_active;
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED  /* Case of Input prioirty higher than ProcState */
    _Critical_DisableIrq();
#endif
    return true;
#elif defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)
    return Critical_AcquireLock(&p_active->InputSignal);
#else
    (void)p_active; return true;
#endif
}

static inline void _StateMachine_ReleaseAsyncIsr(StateMachine_Active_T * p_active)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_active;
#if CONFIG_STATE_MACHINE_INPUT_MULTITHREADED
    _Critical_EnableIrq();
#endif
#elif defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)
    Critical_ReleaseLock(&p_active->InputSignal);
#else
    (void)p_active;
#endif
}

static inline bool _StateMachine_AcquireAsyncInput(StateMachine_Active_T * p_active)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_active;
    _Critical_DisableIrq();
    return true;
#elif defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)
    return Critical_AcquireLock(&p_active->InputSignal);
#else
    (void)p_active; return true;
#endif
}

static inline void _StateMachine_ReleaseAsyncInput(StateMachine_Active_T * p_active)
{
#if defined(CONFIG_STATE_MACHINE_ASYNC_CRITICAL)
    (void)p_active;
    _Critical_EnableIrq();
#elif defined(CONFIG_STATE_MACHINE_ASYNC_SIGNAL)
    Critical_ReleaseLock(&p_active->InputSignal);
#else
    (void)p_active;
#endif
}

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline bool _StateMachine_AcquireSyncInput(StateMachine_Active_T * p_active)
{
#if defined(CONFIG_STATE_MACHINE_SYNC_INPUT_CRITICAL)
    _Critical_DisableIrq();
    return true;
#elif defined(CONFIG_STATE_MACHINE_SYNC_INPUT_SIGNAL)
    return Critical_AcquireLock(&p_active->InputSignal);
#else
    (void)p_active; return true;
#endif
}

static inline void _StateMachine_ReleaseSyncInput(StateMachine_Active_T * p_active)
{
#if defined(CONFIG_STATE_MACHINE_SYNC_INPUT_CRITICAL)
    _Critical_EnableIrq();
#elif defined(CONFIG_STATE_MACHINE_SYNC_INPUT_SIGNAL)
    Critical_ReleaseLock(&p_active->InputSignal);
#else
    (void)p_active;
#endif
}


/******************************************************************************/
/*

*/
/******************************************************************************/
static inline bool _StateMachine_EnterCritical(StateMachine_Active_T * p_active)
{
    _Critical_DisableIrq();
    return true;
    // return Critical_Enter(&p_active->CriticalState);
}

static inline void _StateMachine_ExitCritical(StateMachine_Active_T * p_active)
{
    _Critical_EnableIrq();
    // Critical_Exit(p_active->CriticalState);
}
