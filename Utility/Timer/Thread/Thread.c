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
    @file    Thread.c
    @author FireSourcery
    @brief     Simple thread manager.
    @version V0
 */
/******************************************************************************/
#include "Thread.h"
#include "Config.h"
#include "Utility/Timer/Timer.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/*!
    @brief    Thread Module
*/
/******************************************************************************/
static inline void ProcThread(Thread_T * p_thread)
{
    if (p_thread->Function != 0U) // error checking only, no status report
    {
#ifdef CONFIG_THREAD_FUNCTION_CONTEXT_ENABLED
        p_thread->Function(p_thread->p_Context);
#elif defined (CONFIG_THREAD_FUNCTION_CONTEXT_DISABLED)
        p_thread->Function();
#endif
    }
}

static inline void ProcThreadOneShot(Thread_T * p_thread)
{
    if (p_thread->OneShotProcsRemaining > 0U) // last update on 1 procCounts remaining
    {
        p_thread->OneShotProcsRemaining--;
        ProcThread(p_thread);
    }
    else
    {
        p_thread->IsEnabled = false;

        if (p_thread->OnComplete != 0U)
        {
#ifdef CONFIG_THREAD_FUNCTION_CONTEXT_ENABLED
            p_thread->OnComplete(p_thread->p_Context);
#elif defined (CONFIG_THREAD_FUNCTION_CONTEXT_DISABLED)
            p_thread->OnComplete();
#endif
        }
    }
}

void Thread_Proc(Thread_T * p_thread)
{
    (p_thread->IsOneShot == true) ? ProcThreadOneShot(p_thread) : ProcThread(p_thread);
}

/*!
    @brief    Public function for processing threads.
            Run this function inside the main loop.

    @param    thread pointer to the thread to be processed
    @return True if the function ran, otherwise false.
*/
//todo timer handle oneshot
bool Thread_Poll(Thread_T * p_thread)
{
    bool proc;

    if ((p_thread->IsEnabled == true) && (Timer_Poll(&p_thread->Timer) == true))
    {
        Thread_ProcThread(p_thread);
        proc = true;
    }
    else
    {
        proc = false;
    }

    return proc;
}

uint32_t Thread_ProcN(Thread_T * p_thread, uint32_t nProcs)
{
    uint32_t count;

    if (p_thread->IsOneShot == true)
    {
        (nProcs > p_thread->OneShotProcsRemaining) ? (count = p_thread->OneShotProcsRemaining) : (count = nProcs + 1U);

        for (uint32_t iProc = 0U; iProc < count; iProc++)
        {
            ProcThreadOneShot(p_thread);
        }
    }
    else
    {
         for (uint32_t iProc = 0U; iProc < nProcs; iProc++)
        {
            ProcThread(p_thread);
        }
    }

    return count;
}

/******************************************************************************/
/*!
    @brief     Thread Init functions - assigns all parameters necessary to run
 */
/*! @{ */
/******************************************************************************/
void Thread_InitPeriodic_Period(Thread_T * p_thread, uint32_t period_Ticks, void (*function)(void*), void * p_context)
{
    Thread_SetFunction(p_thread, function, p_context);
    Thread_SetPeriodic(p_thread);
    Timer_SetPeriod(&p_thread->Timer, period_Ticks);
    Timer_Restart(&p_thread->Timer);
}

void Thread_InitPeriodic_Freq(Thread_T * p_thread, uint32_t freq, void (*function)(void*), void * p_context)
{
    Thread_SetFunction(p_thread, function, p_context);
    Thread_SetPeriodic(p_thread);
    Timer_SetFreq(&p_thread->Timer, freq);
    Timer_Restart(&p_thread->Timer);
}

/*
    e.g. wipe 2 leds at the rate of 500ms each
 */
void Thread_InitOneShot_Period(Thread_T * p_thread, uint32_t period_Ticks, uint32_t procCounts, void (*function)(void*), void (*onComplete)(void*), void * p_context)
{
    Thread_SetFunction(p_thread, function, p_context);
    Thread_SetOneShot(p_thread, onComplete);
    Timer_SetPeriod(&p_thread->Timer, period_Ticks);
}

/*
    e.g. wipe 2 leds at the rate of 5 fps
 */
void Thread_InitOneShot_Freq(Thread_T * p_thread, uint32_t freq, uint32_t procCounts, void (*function)(void*), void (*onComplete)(void*), void * p_context)
{
    Thread_SetFunction(p_thread, function, p_context);
    Thread_SetOneShot(p_thread, onComplete);
    Timer_SetFreq(&p_thread->Timer, freq);
}

/*
    e.g. total time of 1000ms for 100 procCounts
 */
void Thread_InitOneShot_Millis(Thread_T * p_thread, uint32_t millis, uint32_t procCounts, void (*function)(void*), void (*onComplete)(void*), void * p_context)
{
    Thread_SetFunction(p_thread, function, p_context);
    Thread_SetOneShot(p_thread,  onComplete);
    Thread_SetOneShotTime_MillisProcs(p_thread, millis, procCounts);
}

/*
    e.g. 1000ms at 500ms per update
 */
void Thread_InitOneShot_MillisPeriod(Thread_T * p_thread, uint32_t millis, uint32_t period, void (*function)(void*), void (*onComplete)(void*), void * p_context)
{

}

/*
    e.g. 1000ms at 60fps
 */
void Thread_InitOneShot_MillisFreq(Thread_T * p_thread, uint32_t millis, uint32_t freq, void (*function)(void*), void (*onComplete)(void*), void * p_context)
{
    Thread_SetFunction(p_thread, function, p_context);
    Thread_SetOneShot(p_thread, onComplete);
    Thread_SetOneShotTime_MillisPeriod(p_thread, millis, freq);
}

void Thread_InitAll
(
    Thread_T * p_thread,
    uint32_t period_Ticks,
    void (*function)(void *),
    void * p_context,
    bool oneShot,
    uint32_t procCounts,
    void(*onComplete)(void *)
)
{
    Timer_SetPeriod(&p_thread->Timer, period_Ticks);
    Thread_SetFunction(p_thread, function, p_context);
    if (oneShot == true)
    {
        Thread_SetOneShot(p_thread, onComplete);
    }
    else
    {
        Thread_SetPeriodic(p_thread);
        p_thread->IsEnabled = true;
    }

    Timer_Restart(&p_thread->Timer);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
    @brief  Thread Set functions
 */
/*! @{ */
/******************************************************************************/
void Thread_Start(Thread_T * p_thread)
{
    p_thread->IsEnabled = true;
    if (p_thread->IsOneShot) {p_thread->OneShotProcsRemaining = p_thread->OneShotProcs;}
    Timer_Restart(&p_thread->Timer);
}

void Thread_Stop(Thread_T * p_thread)
{
    p_thread->IsEnabled = false;
}

void Thread_SetFunction(Thread_T *p_thread, void (*function)(void*), void *p_context)
{
    p_thread->Function = function;
    p_thread->p_Context = p_context;
}

void Thread_SetPeriod(Thread_T * p_thread, uint16_t period)
{
    Timer_SetPeriod(&p_thread->Timer, period);
//    if (period < p_thread->Timer.PeriodMin)    p_thread->Timer.Period = p_thread->Timer.PeriodMin;
//    if (period > p_thread->Timer.PeriodMax)    p_thread->Timer.Period = p_thread->Timer.PeriodMax;
//    else                                p_thread->Timer.Period = period;
}

void Thread_SetFreq(Thread_T * p_thread, uint16_t freq)
{
    Timer_SetFreq(&p_thread->Timer, freq);
//    if         (freq > p_thread->Timer.CONFIG.COUNTER_FREQ/p_thread->Timer.PeriodMin)    p_thread->Timer.Period = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.PeriodMin;
//    else if (freq < p_thread->Timer.CONFIG.COUNTER_FREQ/p_thread->Timer.PeriodMax)    p_thread->Timer.Period = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.PeriodMax;
//    else                                                        p_thread->Timer.Period = p_thread->Timer.CONFIG.COUNTER_FREQ / freq;
}

void Thread_SetPeriodic(Thread_T * p_thread)
{
    p_thread->IsOneShot = false;
}

void Thread_SetOneShot(Thread_T * p_thread, void (*onComplete)(void*))
{
    p_thread->IsOneShot = true;
    p_thread->OnComplete = onComplete;
    p_thread->IsEnabled = false;
}

void Thread_SetOneShotProcs(Thread_T * p_thread, uint32_t procCounts, void (*onComplete)(void*))
{
    p_thread->OneShotProcs = procCounts;
    p_thread->OneShotProcsRemaining = procCounts;
}

void Thread_SetOneShotTime_MillisProcs(Thread_T * p_thread, uint32_t millis, uint32_t procCounts)
{
    Timer_SetPeriod(&p_thread->Timer, p_thread->Timer.CONFIG.BASE_FREQ * millis / procCounts / 1000U);
    p_thread->OneShotProcs = procCounts;
    p_thread->OneShotProcsRemaining = p_thread->OneShotProcs;
}

void Thread_SetOneShotTime_MillisPeriod(Thread_T * p_thread, uint32_t millis, uint32_t period)
{

}

void Thread_SetOneShotTime_MillisFreq(Thread_T * p_thread, uint32_t millis, uint32_t freq)
{
    Timer_SetFreq(&p_thread->Timer, freq);
    p_thread->OneShotProcs = freq * millis / 1000U;
    p_thread->OneShotProcsRemaining = p_thread->OneShotProcs;
}

static uint32_t CalcFaster(uint32_t timerFreq, uint16_t freqInc, uint16_t freqCurrent, uint16_t freqMin, uint16_t freqMax, bool useLoopBoundary)
{
    uint32_t freqNew;

    if(freqCurrent + freqInc > freqMax)
    {
        freqNew = (useLoopBoundary == true) ? freqMin : freqMax;
    }
    else
    {
        freqNew = freqCurrent + freqInc;
    }

    return freqNew;
}

static uint32_t CalcSlower(uint32_t timerFreq, uint16_t freqDec, uint16_t freqCurrent, uint16_t freqMin, uint16_t freqMax, bool useLoopBoundary)
{
    uint32_t freqNew;

    if(freqCurrent - freqDec < freqMin)
    {
        freqNew = (useLoopBoundary == true) ? freqMax : freqMin;
    }
    else
    {
        freqNew = freqCurrent - freqDec;
    }

    return freqNew;
}

//void Thread_SetFaster(Thread_T * p_thread, uint8_t freqInc, bool useLoopBoundary)
//{
//    uint32_t freqCurrent = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.Period;
//    uint32_t freqMin = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.PeriodMax;
//    uint32_t freqMax = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.PeriodMin;
//
//    p_thread->Timer.Period = BoundedFaster(freqInc, freqMin, freqCurrent, freqMax, useLoopBoundary);
//}
//
//void Thread_SetSlower(Thread_T * p_thread, uint8_t freqDec, bool useLoopBoundary)
//{
//    uint32_t freqCurrent = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.Period;
//    uint32_t freqMin = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.PeriodMax;
//    uint32_t freqMax = p_thread->Timer.CONFIG.COUNTER_FREQ / p_thread->Timer.PeriodMin;
//
//    p_thread->Timer.Period = CalcSlower(freqDec, freqMin, freqCurrent, freqMax, useLoopBoundary);
//}

void Thread_SetFaster_MinMax(Thread_T * p_thread, uint16_t freqInc, uint16_t freqMin, uint16_t freqMax, bool useLoopBoundary)
{
    uint32_t freqCurrent = Timer_GetFreq(&p_thread->Timer);
    uint32_t freqNew = CalcFaster(Timer_GetFreq(&p_thread->Timer), freqInc, freqMin, freqCurrent, freqMax, useLoopBoundary);
    Timer_SetFreq(&p_thread->Timer, freqNew);
}

void Thread_SetSlower_MinMax(Thread_T * p_thread, uint16_t freqDec, uint16_t freqMin, uint16_t freqMax, bool useLoopBoundary)
{
    uint32_t freqCurrent = Timer_GetFreq(&p_thread->Timer);
    uint32_t freqNew = CalcSlower(Timer_GetFreq(&p_thread->Timer), freqDec, freqMin, freqCurrent, freqMax, useLoopBoundary);
    Timer_SetFreq(&p_thread->Timer, freqNew);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*! @} */
/******************************************************************************/
