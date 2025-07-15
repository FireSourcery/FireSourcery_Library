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
    TIMER_MODE_DISABLED,    /* Disable Timer */
    TIMER_MODE_STOPPED,     /* OneShot/MultiShot Complete */
    TIMER_MODE_PERIODIC,
    TIMER_MODE_ONE_SHOT,
    TIMER_MODE_MULTI_SHOT,
}
Timer_Mode_T;

// typedef const struct Timer_Base
// {
//      const volatile uint32_t * P_BASE; /* Base Timer */
//      uint32_t BASE_FREQ; /* for conversions */
// }
// Timer_Base_T;

typedef struct Timer
{
    struct
    {
        const volatile uint32_t * const p_Base; /* Base Timer */
        const uint32_t BaseFreq; /* for conversions */
    };
    uint32_t TimeRef;
    uint32_t Period;                /* In Base Freq Ticks, 0 is Disable */

    /* mode with disable */
    bool IsOneShot;
    Timer_Mode_T Mode;
    uint32_t Counter; /* Repeat */
}
Timer_T;

// typedef const struct Timer_Context
// {
//      const volatile uint32_t * P_BASE; /* Base Timer */
//      uint32_t BASE_FREQ; /* for conversions */
//      uint32_t DIVIDER;
//      Timer_T * P_STATE; /* Pointer to Timer State */// Optional: can be NULL for dynamic binding
// }
// Timer_Context_T;

#define TIMER_INIT(p_BaseValue, BaseFreqValue) { { .p_Base = p_BaseValue, .BaseFreq = BaseFreqValue, }, }

// static inline uint32_t timer_elapsed_wrapped(const volatile uint32_t * p_timer, uint32_t timeRef)
// {
//     uint32_t time = *p_timer;
//     return (time < timeRef) ? (UINT32_MAX - timeRef + time) : (time - timeRef);
// }

// static inline uint32_t timer_elapsed_direct(const volatile uint32_t * p_timer, uint32_t timeRef)
// {
//     return (*p_timer - timeRef);
// }

// bool Timer_Base_Poll(const Timer_Base_T * p_base, Timer_State_T * p_state);
// bool Timer_Poll(const Timer_Context_T * p_context);

/******************************************************************************/
/*!
    @brief    Timer Common
*/
/******************************************************************************/
static inline uint32_t Timer_GetBase(const Timer_T * p_timer) { return *p_timer->p_Base; }

/* Only this functions casts away P_BASE const. Call with writable base only */
static inline void Timer_ZeroBase(Timer_T * p_timer)
{
    *((volatile uint32_t *)p_timer->p_Base) = 0U;
    p_timer->TimeRef = *p_timer->p_Base;
}

static inline uint32_t _Timer_GetElapsed_Wrapped(const Timer_T * p_timer)
{
    uint32_t time = *p_timer->p_Base;
    return (time < p_timer->TimeRef) ? (UINT32_MAX - p_timer->TimeRef + time) : (time - p_timer->TimeRef);
}

/*  */
static inline uint32_t Timer_GetElapsed(const Timer_T * p_timer)
{
#ifdef CONFIG_TIMER_OVERFLOW_WRAP /* Not necessary if overflow time is in days. e.g using millis */
    return _Timer_GetElapsed_Wrapped(p_timer);
#else
    return (*p_timer->p_Base - p_timer->TimeRef);
#endif
}

/* has elapsed */
static inline bool Timer_IsElapsed(Timer_T * p_timer) { return (Timer_GetElapsed(p_timer) >= p_timer->Period); }

/*  */
static inline void Timer_Restart(Timer_T * p_timer) { p_timer->TimeRef = *p_timer->p_Base; }

/* Async processing */
static inline void Timer_RestartIfElapsed(Timer_T * p_timer) { if (Timer_IsElapsed(p_timer) == true) { Timer_Restart(p_timer); } }

/*

*/
static inline void Timer_SetPeriod(Timer_T * p_timer, uint32_t ticks) { p_timer->Period = ticks; }

static inline void Timer_StartPeriod(Timer_T * p_timer, uint32_t ticks) { Timer_SetPeriod(p_timer, ticks); Timer_Restart(p_timer); }

/*
    Unit Conversions
*/
static inline uint32_t Timer_GetElapsed_Seconds(Timer_T * p_timer)  { return Timer_GetElapsed(p_timer) / p_timer->BaseFreq; }
/* overflow at 1 hour for millis, 3 min for 20khz */
static inline uint32_t Timer_GetElapsed_Millis(Timer_T * p_timer)   { return Timer_GetElapsed(p_timer) * 1000U / p_timer->BaseFreq; }

static inline uint32_t Timer_GetElapsed_Micros(Timer_T * p_timer)
{
    uint32_t ticks = Timer_GetElapsed(p_timer);
    return (ticks > UINT32_MAX / 1000000U) ? (ticks / p_timer->BaseFreq * 1000000U) : (ticks * 1000000U / p_timer->BaseFreq);
}

/* freq != 0U, freq < BaseFreq */
static inline void Timer_SetFreq(Timer_T * p_timer, uint16_t freq)              { p_timer->Period = p_timer->BaseFreq / freq; }
static inline void Timer_SetPeriod_Millis(Timer_T * p_timer, uint32_t millis)   { p_timer->Period = p_timer->BaseFreq * millis / 1000U; }
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
static inline void Timer_Stop(Timer_T * p_timer)                            { p_timer->IsOneShot = true; p_timer->Period = 0U; }

static inline bool Timer_IsOneShot(const Timer_T * p_timer)                 { return p_timer->IsOneShot; }
static inline bool Timer_IsPeriodic(const Timer_T * p_timer)                { return !p_timer->IsOneShot; }

/******************************************************************************/
/*!
    With Mode + Disable
*/
/******************************************************************************/
// static inline bool Timer_Generic_Poll(Timer_T * p_timer)
// {
//     bool isElapsed;
//     // switch (p_timer->Mode)
//     // {
//     //     case TIMER_MODE_DISABLED: break;
//     //     case TIMER_MODE_STOPPED: break;
//     //     case TIMER_MODE_PERIODIC:
//     //         isElapsed = Timer_IsElapsed(p_timer);
//     //         if (isElapsed == true) { Timer_Restart(p_timer); }
//     //         break;
//     //     case TIMER_MODE_ONE_SHOT:
//     //         isElapsed = Timer_IsElapsed(p_timer);
//     //         if (isElapsed == true) { p_timer->Mode = TIMER_MODE_STOPPED; }
//     //         break;
//     //     case TIMER_MODE_MULTI_SHOT:
//     //         isElapsed = Timer_IsElapsed(p_timer);
//     //         if (isElapsed == true)
//     //         {
//     //             if (p_timer->Counter > 0U) { p_timer->Counter--; }
//     //             else { p_timer->Mode = TIMER_MODE_STOPPED; }
//     //         }
//     //         break;
//     //     default: break;
//     // }

//     switch (p_timer->Mode)
//     {
//         case TIMER_MODE_DISABLED: break;
//         case TIMER_MODE_STOPPED: break;
//         default: isElapsed = Timer_IsElapsed(p_timer); /* intentional fallthrough */
//         case TIMER_MODE_ONE_SHOT: /* p_timer->Counter = 1 */
//         case TIMER_MODE_MULTI_SHOT:
//             if (isElapsed == true)
//             {
//                 if (p_timer->Counter > 0U) { p_timer->Counter--; }
//                 else { p_timer->Mode = TIMER_MODE_STOPPED; }
//             }
//         case TIMER_MODE_PERIODIC: if (isElapsed == true) { Timer_Restart(p_timer); }
//     }

//     return isElapsed;
// }

/* module handle disable */
// static inline void Timer_StartPeriodic(Timer_T * p_timer, uint32_t ticks)
// {
//     if (p_timer->Mode != TIMER_MODE_DISABLED) { p_timer->Mode = TIMER_MODE_PERIODIC; Timer_StartPeriod(p_timer, ticks); } /* set mode */
// }

/* Disables Start */
// static inline void Timer_Disable(Timer_T * p_timer) { p_timer->Mode = TIMER_MODE_DISABLED; }
// static inline Timer_Mode_T Timer_GetMode(Timer_T * p_timer) { return p_timer->Mode; }

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
    // return ((p_timer->Mode == TIMER_MODE_ONE_SHOT) && Timer_IsElapsed(p_timer)) ? ({ p_timer->Mode == TIMER_MODE_STOPPED; true; }) : false;
}

static inline void Timer_OneShot_Init(Timer_T * p_timer) { p_timer->IsOneShot = true; p_timer->Period = 0U; }

/******************************************************************************/
/*!
    MultiShot Timer
*/
/******************************************************************************/
static inline bool Timer_MultiShot_Poll(Timer_T * p_timer)
{
    bool isElapsed = ((p_timer->Period > 0U) && Timer_Periodic_Poll(p_timer));
    if (isElapsed == true)
    {
        if (p_timer->Counter > 0U) { p_timer->Counter--; }
        else { p_timer->Period = 0U; }
    }
    return isElapsed;

    // bool isElapsed = false;
    // switch (p_timer->Mode)
    // {
    //     case TIMER_MODE_DISABLED: break;
    //     case TIMER_MODE_STOPPED: break;

    //     case TIMER_MODE_ONE_SHOT:
    //     case TIMER_MODE_MULTI_SHOT:
    //         if (Timer_IsElapsed(p_timer) == true)
    //         {
    //             if (p_timer->Counter > 0U) { p_timer->Counter--; }
    //             else { p_timer->Mode = TIMER_MODE_STOPPED; }
    //             Timer_Restart(p_timer);
    //             isElapsed = true;/* true per tick */
    //         }
    //         break;
    //     default: break;
    // }
    // return isElapsed;
}

// extern bool Timer_Poll(Timer_T * p_timer);
// extern bool Timer_Periodic_Poll(Timer_T * p_timer);
// extern bool Timer_OneShot_Poll(Timer_T * p_timer);

#endif /* TIMER_H */
