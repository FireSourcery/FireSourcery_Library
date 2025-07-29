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
    @file   TimerT.h
    @author FireSourcery
    @brief  Timer Template
*/
/******************************************************************************/
#include "Timer.h"

/******************************************************************************/
/*!
    Core Implementation (Template-like behavior)
    "Static Polymorphism" - compile-time polymorphism, macro-like behavior
    - Maximum performance through compile-time optimization
    - Pass configuration by value/pointer
    - Internal implementation details
*/
/******************************************************************************/
static inline uint32_t _TimerT_Ticks(const Timer_Base_T * p_base) { return *p_base->P_TIME; }
/* Only this functions casts away P_BASE const. Call with writable base only */
static inline void _TimerT_ZeroBase(const Timer_Base_T * p_base) { *((volatile uint32_t *)p_base->P_TIME) = 0UL; }

/* Polling */
static inline uint32_t _TimerT_Elapsed(const Timer_Base_T * p_base, const Timer_State_T * p_state) { return timer_elapsed_of(*p_base->P_TIME, p_state->TimeRef); }
static inline bool _TimerT_IsElapsed(const Timer_Base_T * p_base, const Timer_State_T * p_state) { return timer_is_elapsed_of(*p_base->P_TIME, p_state->TimeRef, p_state->Period); }
static inline void _TimerT_Restart(const Timer_Base_T * p_base, Timer_State_T * p_state) { p_state->TimeRef = *p_base->P_TIME; }

/******************************************************************************/
/*
    Polling Logic
*/
/******************************************************************************/
static inline bool _TimerT_PollElapsed(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    return timer_poll_elapsed(&p_state->TimeRef, *p_base->P_TIME, p_state->TimeRef, p_state->Period);
}

static inline bool _TimerT_PollElapsedCount(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    if (_TimerT_PollElapsed(p_base, p_state)) { p_state->Counter++; return true; } else { return false; }
}

/* Async processing */

/*
    Common
*/
static inline void __TimerT_Init(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t ticks) { p_state->Period = ticks; p_state->Counter = 0U; _TimerT_Restart(p_base, p_state); }
static inline void __TimerT_Start(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t ticks) { p_state->Period = ticks; _TimerT_Restart(p_base, p_state); }
// static inline void __TimerT_Stop(const Timer_Base_T * p_base, Timer_State_T * p_state) {

/* Async Restart */
// static inline bool _TimerT_Async_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state) { return _TimerT_IsElapsed(p_base, p_state); }
// static inline void _TimerT_Async_Restart(const Timer_Base_T * p_base, Timer_State_T * p_state) { return _TimerT_PollElapsed(p_base, p_state); }

/******************************************************************************/
/*
    Specialized functions for each mode (fastest approach)
    Does not involve the disabled State. Caller handle.

    MISRA violation: Early return - Better Code readability. Less intermediate variables.
*/
/******************************************************************************/
/* Periodic - no Stop or Disable */
static inline bool _TimerT_Periodic_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state) { return _TimerT_PollElapsed(p_base, p_state); }
static inline void _TimerT_Periodic_Init(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { __TimerT_Init(p_base, p_state, period); }
static inline void _TimerT_Periodic_Set(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { p_state->Period = period; }
static inline void _TimerT_Periodic_Stop(const Timer_Base_T * p_base, Timer_State_T * p_state ) { p_state->Period = UINT32_MAX; }


/*
    Periodic Counter
    Poll Elapsed Count
*/
static inline bool _TimerT_Counter_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state) { return _TimerT_PollElapsedCount(p_base, p_state); }
static inline void _TimerT_Counter_Init(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { __TimerT_Init(p_base, p_state, period); }
static inline void _TimerT_Counter_Set(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { p_state->Period = period; }
static inline bool _TimerT_Counter_IsAligned(const Timer_State_T * p_state, uint32_t mask) { return timer_counter_is_aligned(p_state->Counter, mask); }

// static inline bool _TimerT_Counter_Tick(const Timer_Base_T * p_base, Timer_State_T * p_state)
// {
//     p_state->Counter++;
//  if DIVIDER
// }

/*
    Counter with N repeat
    or as OneShotCounter
*/
static inline bool _TimerT_CountN_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    if ((p_state->Counter > 0U) && _TimerT_PollElapsed(p_base, p_state)) { p_state->Counter--; return true; } else { return false; }
}

/* Inits Stopped */
/* one shot counter? */
static inline void _TimerT_CountN_Init(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { p_state->Counter = 0U; p_state->Period = period; }
static inline void _TimerT_CountN_Restart(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t count) { p_state->Counter = count; _TimerT_Restart(p_base, p_state); }
static inline void _TimerT_CountN_Start(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period, uint32_t count) { p_state->Counter = count; __TimerT_Start(p_base, p_state, period); }


/* OneShot as a special case of Counter */
static inline bool _TimerT_OneShot_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    if ((p_state->Counter != 0U) && (_TimerT_IsElapsed(p_base, p_state))) { p_state->Counter = 0U; return true; } else { return false; }
}

static inline void _TimerT_OneShot_Init(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t ticks) { p_state->Counter = 0U; p_state->Period = ticks; }
static inline void _TimerT_OneShot_Restart(const Timer_Base_T * p_base, Timer_State_T * p_state) { p_state->Counter = 1U; _TimerT_Restart(p_base, p_state); }
static inline void _TimerT_OneShot_Start(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t ticks) { p_state->Counter = 1U; __TimerT_Start(p_base, p_state, ticks); }


/******************************************************************************/
/*!
    General Polling Function - Flexible mode
*/
/******************************************************************************/
/******************************************************************************/
/*!
    State Representation
    Timer Type  |   _TimerT_Poll Logic           |   _TimerT_Modal_Poll Logic
    Stopped     |   Period == 0                 |   Mode == TIMER_MODE_STOPPED
    Periodic    |   Period > 0 && Counter == 0  |   Mode == TIMER_MODE_PERIODIC
    OneShot     |   Period > 0 && Counter == 1  |   Mode == TIMER_MODE_ONE_SHOT
    MultiShot   |   Period > 0 && Counter > 1   |   Mode == TIMER_MODE_MULTI_SHOT
    Disabled    |   Not handled                 |   Mode == TIMER_MODE_DISABLED
*/
/******************************************************************************/
/*
    Modal without disabled state
*/
static inline bool _TimerT_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    if ((p_state->Period > 0U) && _TimerT_PollElapsed(p_base, p_state))
    {
        if (p_state->Counter > 0U) /* Handle Periodic Mode, (p_state->Counter == 0U), First */
        {
            if (p_state->Counter > 1U)  /* Counter/MultiShot Mode */
            {
                p_state->Counter--;
            }
            else /* (p_state->Counter == 1U) */ /* Counter/OneShot Mode */
            {
                p_state->Period = 0U; /* Stop the timer on the last tick to distinguish from Periodic Mode */
                p_state->Counter = 0U; /* Or leave at 1 */
            }
        }
        return true;
    }
    return false;
}

// static inline bool _TimerT_Modal_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state)
// {
//     bool isElapsed;
//     switch (p_state->Mode)
//     {
//         case TIMER_MODE_DISABLED:
//         case TIMER_MODE_STOPPED:
//             isElapsed = false;
//             break;
//         case TIMER_MODE_PERIODIC:
//         case TIMER_MODE_ONE_SHOT:
//         case TIMER_MODE_MULTI_SHOT:
//             isElapsed = _TimerT_Poll(p_base, p_state);
//             if (p_state->Period == 0U) { p_state->Mode = TIMER_MODE_STOPPED; };
//             break;
//         default: isElapsed = false; break;
//     }
// }

/* Common implementation for Specialized modes checking for Disabled State */
static inline bool _TimerT_Modal_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    bool isElapsed;

    switch (p_state->Mode)
    {
        case TIMER_MODE_DISABLED:
        case TIMER_MODE_STOPPED:
            isElapsed = false;
            break;
        case TIMER_MODE_PERIODIC:
        case TIMER_MODE_ONE_SHOT:
        case TIMER_MODE_MULTI_SHOT:
            isElapsed = _TimerT_PollElapsed(p_base, p_state);
            break;
        default: isElapsed = false; break;
    }

    if (isElapsed == true)
    {
        switch (p_state->Mode)
        {
            case TIMER_MODE_DISABLED: break;
            case TIMER_MODE_STOPPED: break;
            case TIMER_MODE_PERIODIC: break;
            case TIMER_MODE_ONE_SHOT:
                p_state->Mode = TIMER_MODE_STOPPED;
                break;
            case TIMER_MODE_MULTI_SHOT:
                if (p_state->Counter > 0UL) { p_state->Counter--; }
                else { p_state->Mode = TIMER_MODE_STOPPED; }
                break;
            default: break;
        }
    }
    return isElapsed;
}

/******************************************************************************/
/*
    General/Modal
*/
/******************************************************************************/
/*
    Init exits Disabled
*/
static inline void _TimerT_InitMode(Timer_State_T * p_state, Timer_Mode_T mode)
{
    p_state->Mode = mode;
    p_state->Period = 0U;
    p_state->Counter = 0U;
    p_state->TimeRef = 0U;
}

static inline void _TimerT_InitPeriodic(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { p_state->Mode = TIMER_MODE_PERIODIC; _TimerT_Periodic_Init(p_base, p_state, period); }
static inline void _TimerT_InitOneShot(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t ticks) { p_state->Mode = TIMER_MODE_ONE_SHOT; _TimerT_OneShot_Init(p_base, p_state, ticks); }
static inline void _TimerT_InitCounter(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { p_state->Mode = TIMER_MODE_MULTI_SHOT; _TimerT_Counter_Init(p_base, p_state, period); }
static inline void _TimerT_InitCounterN(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { p_state->Mode = TIMER_MODE_MULTI_SHOT; _TimerT_CountN_Init(p_base, p_state, period); }

static inline void _TimerT_StartMode(const Timer_Base_T * p_base, Timer_State_T * p_state, Timer_Mode_T mode, uint32_t period)
{
    if (p_state->Mode != TIMER_MODE_DISABLED)
    {
        p_state->Mode = mode;
        __TimerT_Start(p_base, p_state, period);
    }
}

static inline void _TimerT_Start(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period) { if (p_state->Mode != TIMER_MODE_DISABLED) { __TimerT_Start(p_base, p_state, period); } }
static inline void _TimerT_Stop(Timer_State_T * p_state) { if (p_state->Mode != TIMER_MODE_DISABLED) { p_state->Mode = TIMER_MODE_STOPPED; p_state->Period = 0U; } }
/* useful for updating period after elapsed */
static inline void _TimerT_SetPeriod(Timer_State_T * p_state, uint32_t ticks) { p_state->Period = ticks; }

/* alternatively move to thread */
static inline void _TimerT_Disable(Timer_State_T * p_state) { p_state->Mode = TIMER_MODE_DISABLED; }
static inline void _TimerT_Enable(Timer_State_T * p_state) { p_state->Mode = TIMER_MODE_STOPPED; }



/* Start/Stop operations */
/* check for disabled state  */
static inline void _TimerT_StartPeriodic(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period)
{
    _TimerT_StartMode(p_base, p_state, TIMER_MODE_PERIODIC, period);
    p_state->Counter = 0;
}

static inline void _TimerT_StartOneShot(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period)
{
    _TimerT_StartMode(p_base, p_state, TIMER_MODE_ONE_SHOT, period);
    p_state->Counter = 1;
}

static inline void _TimerT_RestartOneShot(const Timer_Base_T * p_base, Timer_State_T * p_state)
{
    _TimerT_StartMode(p_base, p_state, TIMER_MODE_ONE_SHOT, p_state->Period);
    p_state->Counter = 1;
}

static inline void _TimerT_StartCounterN(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t period, uint32_t shots)
{
    _TimerT_StartMode(p_base, p_state, TIMER_MODE_MULTI_SHOT, period);
    p_state->Counter = shots;
}


/*
    Unit Conversions
*/
// static inline uint32_t TimerT_GetElapsed_Seconds(const Timer_Base_T * p_base, Timer_State_T * p_state) { return _TimerT_Elapsed(p_base, p_timer) / p_base->FREQ; }
// /* overflow at 1 hour for millis, 3 min for 20khz */
// static inline uint32_t TimerT_GetElapsed_Millis(const Timer_Base_T * p_base, Timer_State_T * p_state) { return _TimerT_Elapsed(p_base, p_timer) * 1000U / p_base->FREQ; }

// static inline uint32_t TimerT_GetElapsed_Micros(const Timer_Base_T * p_base, Timer_State_T * p_state)
// {
//     uint32_t ticks = TimerT_GetElapsed(p_timer);
//     return (ticks > UINT32_MAX / 1000000U) ? (ticks / p_timer->BaseFreq * 1000000U) : (ticks * 1000000U / p_timer->BaseFreq);
// }

// /* freq != 0U, freq < BaseFreq */
// static inline void Timer_SetFreq(const Timer_Base_T * p_base, Timer_State_T * p_state, uint16_t freq)              { p_timer->Period = p_base->FREQ / freq; }
// static inline void Timer_SetPeriod_Millis(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t millis)   { p_timer->Period = p_base->FREQ * millis / 1000U; }
// static inline void Timer_StartPeriod_Millis(const Timer_Base_T * p_base, Timer_State_T * p_state, uint32_t millis) { Timer_SetPeriod_Millis(p_base, p_timer, millis); Timer_Restart(p_base, p_timer); }



// /* Context unit conversions (compile-time optimized) */
// static inline uint32_t TimerT_GetElapsed_Millis(const TimerT_T * p_timer)
// {
//     return (TimerT_GetElapsed(p_timer) * 1000U) / p_timer->BASE.FREQ;
// }

// static inline uint32_t TimerT_GetElapsed_Micros(const TimerT_T * p_timer)
// {
//     uint32_t elapsed = TimerT_GetElapsed(p_timer);
//     return (elapsed > UINT32_MAX / 1000000U) ?
//         (elapsed / p_timer->BASE.FREQ * 1000000U) :
//         (elapsed * 1000000U / p_timer->BASE.FREQ);
// }

// static inline void TimerT_SetPeriod_Millis(const TimerT_T * p_timer, uint32_t millis)
// {
//     p_timer->P_STATE->Period = (p_timer->BASE.FREQ * millis) / 1000U;
// }

// static inline void TimerT_StartPeriod_Millis(const TimerT_T * p_timer, uint32_t millis)
// {
//     TimerT_SetPeriod_Millis(p_timer, millis);
//     TimerT_Restart(p_timer);
// }

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


