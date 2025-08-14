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
    @file   Timer.h
    @author FireSourcery
    @brief  Main Header
*/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/*!
    @brief Embedded Template Library Pattern
        "Stratified Static Polymorphism Pattern"

    A 3-layer architecture providing compile-time polymorphism for embedded systems:

    Layer 1 (_TimerT.h): Template Core
    - Static polymorphism through function parameters
    - Maximum performance, minimal overhead
    - Building blocks for higher layers

    Layer 2 (Timer_General.h): Runtime Interface
    - Self-contained objects
    - Familiar C++ style API
    - Good performance for general use

    Layer 3 (TimerT.h): Optimized Context
    - Compile-time constants
    - Zero-cost abstractions
    - Hardware-optimized performance

    @pattern Template Method + Policy-Based Design + Static Polymorphism
    @performance Zero runtime overhead when using Layer 3
    @usability Progressive complexity - choose appropriate layer
*/
/******************************************************************************/
/* Timer Traits/Type */
typedef const struct Timer_Base
{
    const volatile uint32_t * P_TIME; /* Base Register or Counter */
    uint32_t FREQ; /* for conversions */
    // const uint32_t TICKS_PER_MS;            // BaseFreq / 1000
    // const uint32_t TICKS_PER_US;            // BaseFreq / 1000000
    // const uint32_t MS_PER_TICK_SHIFT;       // Log2 optimization for /FREQ
}
Timer_Base_T;

#define TIMER_BASE_INIT(p_BaseTimer, BaseFreq) { .P_TIME = p_BaseTimer, .FREQ = BaseFreq, }

typedef enum Timer_Mode
{
    TIMER_MODE_DISABLED,    /* Disable Timer */
    TIMER_MODE_STOPPED,     /* OneShot/MultiShot Complete */
    TIMER_MODE_PERIODIC,
    TIMER_MODE_ONE_SHOT,
    TIMER_MODE_MULTI_SHOT,  /*  N Repeat */
    // TIMER_MODE_PERIODIC_COUNTER,
    // TIMER_MODE_ONE_SHOT_COUNTER,
}
Timer_Mode_T;

typedef struct Timer_State
{
    uint32_t Period;    /* In Base Freq Ticks, 0 is Disable */
    uint32_t TimeRef;
    uint32_t Counter;   /* Repeat */
    Timer_Mode_T Mode;  /* Mode with disable. */   // bool IsOneShot;
}
Timer_State_T;

// alternatively as a common def
// typedef struct Timer
// {
//     const Timer_Base_T Base;  /* Base Timer. Optionally as entirely RAM contained. allocate unused for Static case */
    // uint32_t Period;    /* In Base Freq Ticks, 0 is Disable */
    // uint32_t TimeRef;
    // uint32_t Counter;   /* Repeat */
    // Timer_Mode_T Mode;  /* Mode with disable. */   // bool IsOneShot;
// }
// Timer_T;

/******************************************************************************/
/*
    Stateless
*/
/******************************************************************************/
static inline uint32_t timer_counter_wrapped(uint32_t wrap, uint32_t time_ref, uint32_t timer) { return (timer < time_ref) ? (wrap - time_ref + timer) : (timer - time_ref); }

/* or move as counter math */
static inline uint32_t timer_elapsed_wrapped(uint32_t timer, uint32_t time_ref) { return (timer < time_ref) ? (UINT32_MAX - time_ref + timer) : (timer - time_ref); }
static inline uint32_t timer_elapsed_direct(uint32_t timer, uint32_t time_ref) { return (timer - time_ref); }

static inline uint32_t timer_elapsed_of(uint32_t timer, uint32_t time_ref)
{
#ifdef TIMER_OVERFLOW_WRAP /* Not necessary if overflow time is in days. e.g using millis */
    return timer_elapsed_wrapped(timer, time_ref);
#else
    return timer_elapsed_direct(timer, time_ref);
#endif
}

static inline bool timer_is_elapsed_of(uint32_t timer, uint32_t time_ref, uint32_t period) { return (timer_elapsed_of(timer, time_ref) >= period); }

static inline bool timer_poll_elapsed(uint32_t * p_time_ref, uint32_t timer, uint32_t time_ref, uint32_t period)
{
    if (timer_elapsed_of(timer, time_ref) >= period) { *p_time_ref = timer; return true; } else { return false; }
}

// static inline uint32_t timer_elapsed(const volatile uint32_t * p_timer, uint32_t time_ref) { return timer_elapsed_of(*p_timer, time_ref); }
// static inline bool timer_is_elapsed(const volatile uint32_t * p_timer, uint32_t time_ref, uint32_t period) { return (timer_is_elapsed_of(*p_timer, time_ref, period)); }
// static inline bool timer_poll_elapsed(const volatile uint32_t * p_timer, uint32_t * p_time_ref, uint32_t period)
// {
//     uint32_t time = *p_timer;
//     if (timer_elapsed_of(time, *p_time_ref) >= period) { *p_time_ref = time; return true; } else { return false; }
// }

static inline bool timer_counter_is_aligned(uint32_t counter, uint32_t mask) { return ((counter & mask) == 0UL); }
static inline bool timer_counter_poll_aligned(uint32_t * p_counter, uint32_t mask) { *p_counter++; return timer_counter_is_aligned(*p_counter, mask); }

// static inline bool timer_counter_tick(uint32_t * p_counter)

/******************************************************************************/
/*
    Common Query
*/
/******************************************************************************/
static inline void Timer_SetPeriod(Timer_State_T * p_state, uint32_t ticks) { p_state->Period = ticks; }
static inline Timer_Mode_T Timer_GetMode(const Timer_State_T * p_state) { return p_state->Mode; }

/* Polling with general mode only */
static inline bool Timer_IsActive(const Timer_State_T * p_timer) { return (p_timer->Mode > TIMER_MODE_STOPPED) && (p_timer->Period > 0U); }
static inline bool Timer_IsStopped(const Timer_State_T * p_state) { return (p_state->Period == 0UL) || (p_state->Mode == TIMER_MODE_STOPPED); }
static inline bool Timer_IsPeriodic(const Timer_State_T * p_timer) { return (p_timer->Mode == TIMER_MODE_PERIODIC); }
static inline bool Timer_IsOneShot(const Timer_State_T * p_timer) { return (p_timer->Mode == TIMER_MODE_ONE_SHOT); }


/******************************************************************************/
/*
    Export
*/
/******************************************************************************/
#include "Timer_General.h"
#include "TimerT.h"



