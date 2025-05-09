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
    @file   Critical.h
    @author FireSourcery
    @brief  Implements Critical Section
    @version V0
*/
/******************************************************************************/
#ifndef CRITICAL_H
#define CRITICAL_H

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

// static inline void Critical_DisableIrq(void)
// {
//     __disable_irq();
//     _Critical_InterruptDisableCount++;
// }

// static inline void Critical_EnableIrq(void)
// {
//     if (_Critical_InterruptDisableCount > 0U)
//     {
//         _Critical_InterruptDisableCount--;
//         if (_Critical_InterruptDisableCount <= 0U) { __enable_irq(); }
//     }
// }

/*
    process must be polling.
*/
typedef volatile atomic_flag critical_signal_t;

static inline bool Critical_InitSignal(critical_signal_t * p_signal)
{
    atomic_flag_clear(p_signal);
}

static inline bool Critical_AcquireSignal(critical_signal_t * p_signal)
{
    return (atomic_flag_test_and_set(p_signal) == false); // true if signal is available
}

static inline void Critical_ReleaseSignal(critical_signal_t * p_signal)
{
    atomic_flag_clear(p_signal);
}

static inline bool Critical_AwaitSignal_Blocking(critical_signal_t * p_signal)
{
    while (atomic_flag_test_and_set(p_signal) == true) {}
}


static inline bool Critical_AcquireEnter(critical_signal_t * p_signal)
{
#if defined(CONFIG_CRITICAL_USE_MUTEX)
    return Critical_AcquireSignal(p_signal);
#else
    (void)p_signal;
    _Critical_DisableIrq();
    return true;
#endif
// #else
//     _Critical_Enter(p_signal);
// #endif
}

static inline void Critical_ReleaseExit(critical_signal_t * p_signal)
{
#if defined(CONFIG_CRITICAL_USE_MUTEX)
    Critical_ReleaseSignal(p_signal);
#else
    (void)p_signal;
    _Critical_EnableIrq();
#endif
}



#endif
