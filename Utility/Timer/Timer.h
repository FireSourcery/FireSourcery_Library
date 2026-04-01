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
#include "math_counter.h"

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
    TIMER_MODE_PERIODIC_COUNTER, /* Count up. Counter */
    TIMER_MODE_ONE_SHOT_COUNTER, /* Count down. N Repeat */
}
Timer_Mode_T;

typedef struct Timer_State
{
    uint32_t Period;    /* In Base Freq Ticks, 0 is Disable */
    uint32_t TimeRef;
    uint32_t Counter;   /* Repeat */
    Timer_Mode_T Mode;  /* Mode with disable. */
}
Timer_State_T;

// alternatively as a common def
// typedef struct Timer
// {
//     const Timer_Base_T Base;  /* Base Timer. Optionally as entirely RAM contained. allocate unused for template case */
    // uint32_t Period;    /* In Base Freq Ticks, 0 is Disable */
    // uint32_t TimeRef;
    // uint32_t Counter;   /* Repeat */
    // Timer_Mode_T Mode;  /* Mode with disable. */   // bool IsOneShot;
// }
// Timer_T;

/******************************************************************************/
/*
    Common Query
*/
/******************************************************************************/
static inline uint32_t _Timer_Elapsed(uint32_t time_prev, uint32_t time) { return (time - time_prev); }
static inline bool _Timer_IsElapsed(uint32_t period, uint32_t time_prev, uint32_t time) { return (_Timer_Elapsed(time_prev, time) >= period); }

static inline void Timer_SetPeriod(Timer_State_T * p_state, uint32_t ticks) { p_state->Period = ticks; }

/*
    Polling with modal mode only
*/
static inline Timer_Mode_T Timer_GetMode(const Timer_State_T * p_state) { return p_state->Mode; }
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



