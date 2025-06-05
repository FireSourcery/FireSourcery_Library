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

*/
/******************************************************************************/
#ifndef TIMER_UTILITY_H
#define TIMER_UTILITY_H

#include "Config.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TIMER_MODE_DISABLED,      /* Disable Timer */
    // TIMER_MODE_STOPPED,
    TIMER_MODE_PERIODIC,
    TIMER_MODE_ONE_SHOT,
    TIMER_MODE_MULTI_SHOT,
}
Timer_Mode_T;

typedef struct Timer
{
    struct
    {
        const volatile uint32_t * const p_Base; /* Base Timer */
        const uint32_t BaseFreq;
    };
    uint32_t TimeRef;
    uint32_t Period;                /* In Base Freq Ticks, 0 is Disable */
    bool IsOneShot;
    // uint32_t Repeat;
    uint32_t Counter;
    Timer_Mode_T Mode;
}
Timer_T;


#define TIMER_INIT(p_BaseValue, BaseFreqValue) { { .p_Base = p_BaseValue, .BaseFreq = BaseFreqValue, }, }

/******************************************************************************/
/*!
    @brief    Timer Common
*/
/******************************************************************************/
static inline uint32_t Timer_GetBase(Timer_T * p_timer) { return *p_timer->p_Base; }

/* Only this functions casts away P_BASE const. Call with writable base only */
static inline void Timer_ZeroBase(Timer_T * p_timer)
{
    *((volatile uint32_t *)p_timer->p_Base) = 0U;
    p_timer->TimeRef = *p_timer->p_Base;
}

static inline void Timer_Restart(Timer_T * p_timer) { p_timer->TimeRef = *p_timer->p_Base; }

static inline uint32_t Timer_GetElapsed(Timer_T * p_timer)
{
#ifdef CONFIG_TIMER_OVERFLOW_WRAP /* Not necessarily needed if overflow time is in days. e.g using millis */
    return (*p_timer->p_Base < p_timer->TimeRef) ? (UINT32_MAX - p_timer->TimeRef + *p_timer->p_Base) : (*p_timer->p_Base - p_timer->TimeRef);
#else
    return (*p_timer->p_Base - p_timer->TimeRef);
#endif
}

/* has elapsed */
static inline bool Timer_IsElapsed(Timer_T * p_timer) { return (Timer_GetElapsed(p_timer) >= p_timer->Period); }

static inline void Timer_SetPeriod(Timer_T * p_timer, uint32_t ticks) { p_timer->Period = ticks; }

/*

*/
static inline uint32_t Timer_GetElapsed_Seconds(Timer_T * p_timer)  { return Timer_GetElapsed(p_timer) / p_timer->BaseFreq; }
/* 1 hour for overflow if timer is millis, 3 min for 20khz */
static inline uint32_t Timer_GetElapsed_Millis(Timer_T * p_timer)   { return Timer_GetElapsed(p_timer) * 1000U / p_timer->BaseFreq; }

static inline uint32_t Timer_GetElapsed_Micros(Timer_T * p_timer)
{
    uint32_t ticks = Timer_GetElapsed(p_timer);
    return (ticks > UINT32_MAX / 1000000U) ? (ticks / p_timer->BaseFreq * 1000000U) : (ticks * 1000000U / p_timer->BaseFreq);
}

/* freq != 0U, freq < Base Freq */
static inline void Timer_SetFreq(Timer_T * p_timer, uint16_t freq)              { p_timer->Period = p_timer->BaseFreq / freq; }

static inline void Timer_SetPeriod_Millis(Timer_T * p_timer, uint32_t millis)   { p_timer->Period = p_timer->BaseFreq * millis / 1000U; }

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

static inline void _Timer_StartPeriod(Timer_T * p_timer, uint32_t ticks)    { Timer_SetPeriod(p_timer, ticks); Timer_Restart(p_timer); }
static inline void Timer_StartOneShot(Timer_T * p_timer, uint32_t ticks)    { p_timer->IsOneShot = true; Timer_StartPeriod(p_timer, ticks); }
static inline void Timer_StartPeriodic(Timer_T * p_timer, uint32_t ticks)   { p_timer->IsOneShot = false; Timer_StartPeriod(p_timer, ticks); }
static inline bool Timer_IsOneShot(Timer_T * p_timer)                       { return p_timer->IsOneShot; }
static inline bool Timer_IsPeriodic(Timer_T * p_timer)                      { return !p_timer->IsOneShot; }

static inline void Timer_Stop(Timer_T * p_timer)                            { p_timer->IsOneShot = true; p_timer->Period = 0U; }

// static inline Timer_Mode_T Timer_GetMode(Timer_T * p_timer) { return p_timer->Mode; }
// static inline void Timer_Disable(Timer_T * p_timer) { p_timer->Mode = TIMER_MODE_DISABLED; }

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

/******************************************************************************/
/*!
    OneShot Only Timer
*/
/******************************************************************************/
static inline bool Timer_OneShot_Poll(Timer_T * p_timer)
{
    // bool isElapsed = (p_timer->Period > 0U) && (Timer_IsElapsed(p_timer) == true);
    // if (isElapsed == true) { p_timer->Period = 0U; }
    // return isElapsed;
    return ((p_timer->Period > 0U) && (Timer_IsElapsed(p_timer) == true)) ? ({ p_timer->Period = 0U; true; }) : false;
}

static inline void Timer_OneShot_Init(Timer_T * p_timer) { p_timer->IsOneShot = true; p_timer->Period = 0U; }

// extern bool Timer_Poll(Timer_T * p_timer);
// extern bool Timer_Periodic_Poll(Timer_T * p_timer);
// extern bool Timer_OneShot_Poll(Timer_T * p_timer);

#endif /* TIMER_H */
