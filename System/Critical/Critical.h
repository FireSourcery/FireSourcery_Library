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
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>


/*
    HAL implementation
*/

typedef uint32_t critical_state_t;

#if defined(__GNUC__) || defined(__CMSIS_COMPILER_H)
static inline void _Critical_DisableIrq(void) { __disable_irq(); }
static inline void _Critical_EnableIrq(void) { __enable_irq(); }

// extern uint32_t _Critical_InterruptDisableCount;

// static inline void Critical_DisableIrq(void)
// {
//     _Critical_DisableIrq();
//     _Critical_InterruptDisableCount++;
// }

// static inline void Critical_EnableIrq(void)
// {
//     if (_Critical_InterruptDisableCount > 0U)
//     {
//         _Critical_InterruptDisableCount--;
//         if (_Critical_InterruptDisableCount <= 0U) { _Critical_EnableIrq(); }
//     }
// }
#endif

// #include "External/CMSIS/Core/Include/cmsis_compiler.h"
#ifdef __CMSIS_COMPILER_H
static inline void _Critical_Enter(critical_state_t * p_state)  { *p_state = __get_PRIMASK(); __disable_irq(); }
static inline void _Critical_Exit(critical_state_t state)       { __set_PRIMASK(state); }
#else
void _Critical_Enter(critical_state_t * p_state);
void _Critical_Exit(critical_state_t state);
#endif

/******************************************************************************/
/*
    Lightweight Critical (Mutex Mode)
*/
/******************************************************************************/
typedef volatile atomic_flag critical_lock_t;

static inline void Critical_InitLock(critical_lock_t * p_lock) { atomic_flag_clear(p_lock); }
/* return true if signal is available - atomic_flag_test_and_set returns previous state */
static inline bool Critical_AcquireLock(critical_lock_t * p_signal) { return (atomic_flag_test_and_set(p_signal) == false); }
static inline void Critical_ReleaseLock(critical_lock_t * p_signal) { atomic_flag_clear(p_signal); }

static inline void Critical_AwaitLock_Blocking(critical_lock_t * p_signal, void (*yield)(void)) { while (atomic_flag_test_and_set(p_signal) == true) { yield(); } }


