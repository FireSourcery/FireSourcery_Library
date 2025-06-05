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
    @file   Critical.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

extern uint32_t _Critical_InterruptDisableCount;

/*
    HAL implement within module.
    Alternatively, implement in parent HAL and include.
*/
#ifdef CONFIG_SYSTEM_MCU_ARM

#include "External/CMSIS/Core/Include/cmsis_compiler.h"

/*
    No nesting Critical within Critical
*/
static inline void _Critical_DisableIrq(void) { __disable_irq(); }
static inline void _Critical_EnableIrq(void) { __enable_irq(); }

static inline void _Critical_Enter(uint32_t * p_state)  { *p_state = __get_PRIMASK(); __disable_irq(); }
static inline void _Critical_Exit(uint32_t state)       { __set_PRIMASK(state); }

#elif defined(CONFIG_CRITICAL_DISABLED)
static inline void _Critical_DisableIrq(void) {}
static inline void _Critical_EnableIrq(void) {}
static inline void _Critical_Enter(uint32_t * p_state);
static inline void _Critical_Exit(uint32_t state);
#endif

static inline void Critical_DisableIrq(void)
{
    _Critical_DisableIrq();
    _Critical_InterruptDisableCount++;
}

static inline void Critical_EnableIrq(void)
{
    if (_Critical_InterruptDisableCount > 0U)
    {
        _Critical_InterruptDisableCount--;
        if (_Critical_InterruptDisableCount <= 0U) { _Critical_EnableIrq(); }
    }
}


/******************************************************************************/
/*
    Lightweight Critical (Mutex Mode)
*/
/******************************************************************************/
typedef volatile atomic_flag critical_lock_t;

static inline void Critical_InitLock(critical_lock_t * p_lock) { atomic_flag_clear(p_lock); }
/* return true if signal is available */
/* atomic_flag_test_and_set returns previous state */
static inline bool Critical_AcquireLock(critical_lock_t * p_signal) { return (atomic_flag_test_and_set(p_signal) == false); }
static inline void Critical_ReleaseLock(critical_lock_t * p_signal) { atomic_flag_clear(p_signal); }

static inline void _Critical_AwaitLock_Blocking(critical_lock_t * p_signal) { while (atomic_flag_test_and_set(p_signal) == true) {} }

/******************************************************************************/
/*
    Unified Critical Section Interface
*/
/******************************************************************************/
// typedef volatile atomic_flag critical_lock_t;
// typedef uint32_t critical_state_t;

// /* Special return value for failed TryEnter */
// #define CRITICAL_STATE_FAILED  (~(critical_state_t)0)

// static inline critical_state_t Critical_AwaitEnter(critical_lock_t * p_lock)
// {
// #ifdef CONFIG_CRITICAL_USE_MUTEX
//     Critical_Lock(p_lock);
//     return 0;
// #else
//     (void)p_lock;
//     critical_state_t state = __get_PRIMASK();
//     __disable_irq();
//     return state;
// #endif
// }

// static inline critical_state_t Critical_TryEnter(critical_lock_t * p_lock)
// {
// #ifdef CONFIG_CRITICAL_USE_MUTEX
//     return Critical_TryLock(p_lock) ? 0 : CRITICAL_STATE_FAILED;
// #else
//     (void)p_lock;
//     critical_state_t state = __get_PRIMASK();
//     __disable_irq();
//     return state;
// #endif
// }

// static inline void Critical_Exit(critical_lock_t * p_lock, critical_state_t saved_state)
// {
// #ifdef CONFIG_CRITICAL_USE_MUTEX
//     if (saved_state != CRITICAL_STATE_FAILED)
//     {
//         Critical_Unlock(p_lock);
//     }
// #else
//     (void)p_lock;
//     __set_PRIMASK(saved_state);
// #endif
// }
