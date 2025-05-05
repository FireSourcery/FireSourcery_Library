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
    @file   Timer.h
    @author FireSourcery
    @brief  Elapsed time
    @version V0
*/
/******************************************************************************/
#ifndef TIMER_UTILITY_H
#define TIMER_UTILITY_H

#include "Config.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct Timer_Const
{
    const volatile uint32_t * const P_BASE; /* Base Timer */
    const uint32_t BASE_FREQ;
}
Timer_Const_T;

typedef struct Timer
{
    const Timer_Const_T CONST;
    uint32_t TimeRef;
    uint32_t Period;                /* In Base Freq Ticks, 0 is Disable */
    bool IsOneShot;
}
Timer_T;

/* Repeat Timer */
// typedef struct TimerN
// {
//     Timer_T Timer;
//     uint32_t Repeat;
//     uint32_t Period;            /* In Base Freq Ticks, 0 is Loop infinite */
//     uint32_t Counter;
// }
// TimerN_T;

#define TIMER_INIT(p_Base, BaseFreq) { .CONST = { .P_BASE = p_Base, .BASE_FREQ = BaseFreq, }, }

/******************************************************************************/
/*!
    @brief    Timer Common
*/
/******************************************************************************/
static inline uint32_t Timer_GetBase(Timer_T * p_timer) { return *p_timer->CONST.P_BASE; }

/* Only this functions casts away P_BASE const. Call with writable base only */
static inline void Timer_ZeroBase(Timer_T * p_timer)
{
    *((volatile uint32_t *)p_timer->CONST.P_BASE) = 0U;
    p_timer->TimeRef = *p_timer->CONST.P_BASE;
}

static inline void Timer_Disable(Timer_T * p_timer) { p_timer->Period = 0U; }
static inline void Timer_Restart(Timer_T * p_timer) { p_timer->TimeRef = *p_timer->CONST.P_BASE; }

static inline uint32_t Timer_GetElapsed(Timer_T * p_timer)
{
#ifdef CONFIG_TIMER_OVERFLOW_WRAP /* Not necessarily needed if overflow time is in days. e.g using millis */
    return (*p_timer->CONST.P_BASE < p_timer->TimeRef) ? (UINT32_MAX - p_timer->TimeRef + *p_timer->CONST.P_BASE) : (*p_timer->CONST.P_BASE - p_timer->TimeRef);
#else
    return (*p_timer->CONST.P_BASE - p_timer->TimeRef);
#endif
}

static inline uint32_t Timer_GetElapsed_Seconds(Timer_T * p_timer)  { return Timer_GetElapsed(p_timer) / p_timer->CONST.BASE_FREQ; }
/* 1 hour for overflow if timer is millis, 3 min for 20khz */
static inline uint32_t Timer_GetElapsed_Millis(Timer_T * p_timer)   { return Timer_GetElapsed(p_timer) * 1000U / p_timer->CONST.BASE_FREQ; }

static inline uint32_t Timer_GetElapsed_Micros(Timer_T * p_timer)
{
    uint32_t ticks = Timer_GetElapsed(p_timer);
    return (ticks > UINT32_MAX / 1000000U) ? (ticks / p_timer->CONST.BASE_FREQ * 1000000U) : (ticks * 1000000U / p_timer->CONST.BASE_FREQ);
}

/* has elapsed */
static inline bool Timer_IsElapsed(Timer_T * p_timer) { return (Timer_GetElapsed(p_timer) >= p_timer->Period); }

/* freq != 0U, freq < Base Freq */
static inline void Timer_SetFreq(Timer_T * p_timer, uint16_t freq)              { p_timer->Period = p_timer->CONST.BASE_FREQ / freq; }

static inline void Timer_SetPeriod(Timer_T * p_timer, uint32_t ticks)           { p_timer->Period = ticks; }
static inline void Timer_SetPeriod_Millis(Timer_T * p_timer, uint32_t millis)   { p_timer->Period = p_timer->CONST.BASE_FREQ * millis / 1000U; }

static inline void Timer_StartPeriod(Timer_T * p_timer, uint32_t ticks)         { Timer_SetPeriod(p_timer, ticks); Timer_Restart(p_timer); }
static inline void Timer_StartPeriod_Millis(Timer_T * p_timer, uint32_t millis) { Timer_SetPeriod_Millis(p_timer, millis); Timer_Restart(p_timer); }

/******************************************************************************/
/*!
    Timer Both Modes
*/
/******************************************************************************/
static inline bool Timer_Poll(Timer_T * p_timer)
{
    bool isElapsed = (p_timer->Period > 0U) && (Timer_IsElapsed(p_timer) == true);
    if (isElapsed == true)
    {
        if (p_timer->IsOneShot == true) { p_timer->Period = 0U; }
        else { Timer_Restart(p_timer); }
    }
    return isElapsed;
}

static inline void Timer_Init(Timer_T * p_timer)                            { p_timer->IsOneShot = false; p_timer->Period = 0U; }
static inline void Timer_InitPeriodic(Timer_T * p_timer, uint32_t ticks)    { p_timer->IsOneShot = false; p_timer->Period = ticks;}
static inline void Timer_InitOneShot(Timer_T * p_timer)                     { p_timer->IsOneShot = true; p_timer->Period = 0U; }
static inline void Timer_SetPeriodic(Timer_T * p_timer)                     { p_timer->IsOneShot = false; }
static inline void Timer_SetOneShot(Timer_T * p_timer)                      { p_timer->IsOneShot = true; }
static inline void Timer_StartOneShot(Timer_T * p_timer, uint32_t ticks)    { p_timer->IsOneShot = true; Timer_StartPeriod(p_timer, ticks); }
static inline void Timer_StartPeriodic(Timer_T * p_timer, uint32_t ticks)   { p_timer->IsOneShot = false; Timer_StartPeriod(p_timer, ticks); }
static inline bool Timer_IsOneShot(Timer_T * p_timer)                       { return p_timer->IsOneShot; }
static inline bool Timer_IsPeriodic(Timer_T * p_timer)                      { return !p_timer->IsOneShot; }

/******************************************************************************/
/*!
    Periodic Only Timer
*/
/******************************************* ***********************************/
static inline bool Timer_Periodic_Poll(Timer_T * p_timer)
{
    // bool isElapsed = Timer_IsElapsed(p_timer);
    // if (isElapsed == true) { Timer_Restart(p_timer); }
    // return isElapsed;
    return Timer_IsElapsed(p_timer) ? ({ Timer_Restart(p_timer); true; }) : false;
}

static inline void Timer_Periodic_Init(Timer_T * p_timer, uint32_t ticks) { p_timer->IsOneShot = false; p_timer->Period = ticks; }
// static inline void Timer_Periodic_Disable(Timer_T * p_timer) { p_timer->Period = UINT32_MAX; }

/******************************************************************************/
/*!
    OneShot Only Timer
*/
/******************************************************************************/
static inline bool Timer_OneShot_Poll(Timer_T * p_timer)
{
    bool isElapsed = (p_timer->Period > 0U) && (Timer_IsElapsed(p_timer) == true);
    if (isElapsed == true) { p_timer->Period = 0U; }
    return isElapsed;
}

static inline void Timer_OneShot_Init(Timer_T * p_timer) { p_timer->IsOneShot = true; p_timer->Period = 0U; }

// extern bool Timer_Poll(Timer_T * p_timer);
// extern bool Timer_Periodic_Poll(Timer_T * p_timer);
// extern bool Timer_OneShot_Poll(Timer_T * p_timer);

#endif /* TIMER_H */
