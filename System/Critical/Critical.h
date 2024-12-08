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
extern uint32_t _Critical_StateOnEnter;

/*
    HAL implement within module.
    Alternatively, implement in parent HAL and include.
*/
#ifdef CONFIG_SYSTEM_MCU_ARM

#include "External/CMSIS/Core/Include/cmsis_compiler.h"

#if defined(__GNUC__)
#define CRITICAL_DISABLE_INTERRUPTS() __disable_irq()
#define CRITICAL_ENABLE_INTERRUPTS() __enable_irq()
#else
#define CRITICAL_DISABLE_INTERRUPTS() __asm("cpsid i")
#define CRITICAL_ENABLE_INTERRUPTS() __asm("cpsie i")
#endif

/*
    No nesting Critical within Critical
*/
static inline void Critical_Enter(void) { __disable_irq(); }
static inline void Critical_Exit(void) { __enable_irq(); }

static inline void _Critical_Enter(uint32_t * p_state)  { *p_state = __get_PRIMASK(); __disable_irq(); }
static inline void _Critical_Exit(uint32_t state)       { __set_PRIMASK(state); }

#elif defined(CONFIG_CRITICAL_DISABLED)
static inline void Critical_Enter(void) {}
static inline void Critical_Exit(void) {}
#endif

static inline void Critical_DisableIrq(void)
{
    __disable_irq();
    _Critical_InterruptDisableCount++;
}

static inline void Critical_EnableIrq(void)
{
    if (_Critical_InterruptDisableCount > 0U)
    {
        _Critical_InterruptDisableCount--;
        if (_Critical_InterruptDisableCount <= 0U) { __enable_irq(); }
    }
}

/*
    process must be polling.
*/
typedef volatile atomic_flag critical_signal_t;

static inline bool Critical_InitSignal(critical_signal_t * p_mutex)
{
    atomic_flag_clear(p_mutex);
}

static inline bool Critical_AcquireSignal(critical_signal_t * p_mutex)
{
    return atomic_flag_test_and_set(p_mutex) ? false : true; // true if mutex is available
}

static inline void Critical_ReleaseSignal(critical_signal_t * p_mutex)
{
    atomic_flag_clear(p_mutex);
}

static inline bool Critical_AwaitSignal_Blocking(critical_signal_t * p_mutex)
{
    while (atomic_flag_test_and_set(p_mutex) == true) {}
}


static inline bool Critical_AcquireEnter(critical_signal_t * p_mutex)
{
#if defined(CONFIG_CRITICAL_USE_MUTEX)
    return Critical_AquireMutex(p_mutex);
#else
    (void)p_mutex;
    Critical_Enter();
    return true;
#endif
}

static inline void Critical_ReleaseExit(critical_signal_t * p_mutex)
{
#if defined(CONFIG_CRITICAL_USE_MUTEX)
    Critical_ReleaseSignal(p_mutex);
#else
    (void)p_mutex;
    Critical_Exit();
#endif
}



#endif
