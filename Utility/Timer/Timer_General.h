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
    @file   Timer_General.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_TimerT.h"
#include <string.h>

/******************************************************************************/
/*
    Runtime RAM Contiguous Implementation
    - Self-contained Timer_T objects
    - Standard API with good performance
    - General purpose usage
*/
/******************************************************************************/
typedef struct Timer
{
    const Timer_Base_T Base;
    Timer_State_T State;
}
Timer_T;

/******************************************************************************/
/*!
    @brief Timer Initialization
*/
/******************************************************************************/
static inline void Timer_InitFrom(Timer_T * p_timer, const Timer_Base_T * p_base)
{
    /* Cast away const. It is the RAM copy */
    memcpy((void *)&p_timer->Base, p_base, sizeof(Timer_Base_T));
    p_timer->State.Mode = TIMER_MODE_STOPPED;
    p_timer->State.Period = 0U;
    p_timer->State.Counter = 0U;
    p_timer->State.TimeRef = 0U;
}


/******************************************************************************/
/*!
    @brief Timer Common
*/
/******************************************************************************/
static inline uint32_t Timer_GetBase(const Timer_T * p_timer)  { return _TimerT_Ticks(&p_timer->Base); }
/* Only this functions casts away P_BASE const. Call with writable base only */
static inline void Timer_ZeroBase(Timer_T * p_timer)
{
    _TimerT_ZeroBase(&p_timer->Base);
    p_timer->State.TimeRef = _TimerT_Ticks(&p_timer->Base);
}

/*  */
static inline uint32_t Timer_GetElapsed(const Timer_T * p_timer) { return _TimerT_Elapsed(&p_timer->Base, &p_timer->State); }
static inline bool Timer_IsElapsed(Timer_T * p_timer) { return _TimerT_IsElapsed(&p_timer->Base, &p_timer->State); }
static inline void Timer_Restart(Timer_T * p_timer) { _TimerT_Restart(&p_timer->Base, &p_timer->State); }

/* Mode-specific polling */
static inline bool Timer_Periodic_Poll(Timer_T * p_timer) { return _TimerT_Periodic_Poll(&p_timer->Base, &p_timer->State); }
static inline bool Timer_OneShot_Poll(Timer_T * p_timer) { return _TimerT_OneShot_Poll(&p_timer->Base, &p_timer->State); }
static inline bool Timer_CountN_Poll(Timer_T * p_timer) { return _TimerT_CountN_Poll(&p_timer->Base, &p_timer->State); }

/* Counter operations */
static inline bool Timer_Counter_Poll(Timer_T * p_timer) { return _TimerT_Counter_Poll(&p_timer->Base, &p_timer->State); }
static inline bool Timer_Counter_IsAligned(const Timer_T * p_timer, uint32_t align) { return _TimerT_Counter_IsAligned(&p_timer->State, align); }

/* General polling */
static inline bool Timer_Poll(Timer_T * p_timer) { return _TimerT_Poll(&p_timer->Base, &p_timer->State); }
static inline bool Timer_Modal_Poll(Timer_T * p_timer) { return _TimerT_Modal_Poll(&p_timer->Base, &p_timer->State); }

static inline void Timer_Start(Timer_T * p_timer, uint32_t period) { _TimerT_Start(&p_timer->Base, &p_timer->State, period); }
static inline void Timer_Stop(Timer_T * p_timer) { _TimerT_Stop(&p_timer->State); }

// /*
//     Unit Conversions
// */
// static inline uint32_t Timer_GetElapsed_Seconds(Timer_T * p_timer)  { return Timer_GetElapsed(p_timer) / p_timer->BaseFreq; }
// /* overflow at 1 hour for millis, 3 min for 20khz */
// static inline uint32_t Timer_GetElapsed_Millis(Timer_T * p_timer)   { return Timer_GetElapsed(p_timer) * 1000U / p_timer->BaseFreq; }

// static inline uint32_t Timer_GetElapsed_Micros(Timer_T * p_timer)
// {
//     uint32_t ticks = Timer_GetElapsed(p_timer);
//     return (ticks > UINT32_MAX / 1000000U) ? (ticks / p_timer->BaseFreq * 1000000U) : (ticks * 1000000U / p_timer->BaseFreq);
// }

// /* freq != 0U, freq < BaseFreq */
// static inline void Timer_SetFreq(Timer_T * p_timer, uint16_t freq)              { p_timer->Period = p_timer->BaseFreq / freq; }
// static inline void Timer_SetPeriod_Millis(Timer_T * p_timer, uint32_t millis)   { p_timer->Period = p_timer->BaseFreq * millis / 1000U; }
// static inline void Timer_StartPeriod_Millis(Timer_T * p_timer, uint32_t millis) { Timer_SetPeriod_Millis(p_timer, millis); Timer_Restart(p_timer); }
