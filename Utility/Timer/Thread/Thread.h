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
    @file      Thread.h
    @author FireSourcery
    @brief     Simple thread manager.

    Limitations:
    Not a real context switch, no saving/restoring registers using the stack.
    Hence, functions must be non blocking. Main loop cannot block.
    Cannot time slice i.e cannot switch threads in the middle of execution
    No thread scheduler, all threads are equal priority and must run to completion in sequence.

    @version V0
 */
/******************************************************************************/
#ifndef THREAD_H
#define THREAD_H

#include "Config.h"

#include "Utility/Timer/Timer.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    Timer_T Timer;

    volatile bool IsEnabled;        // Enable or disable

    bool IsOneShot;
    uint32_t OneShotProcs;
    volatile uint32_t OneShotProcsRemaining;

#ifdef CONFIG_THREAD_FUNCTION_CONTEXT_ENABLED
    void (*Function)(void * p_context);
    void (*OnComplete)(void * p_context);
    void * p_Context;
//    void * p_OnCompleteContext;
#elif defined (CONFIG_THREAD_FUNCTION_CONTEXT_DISABLED)
    void (*Function)(void);
    void (*OnComplete)(void);
#endif
}
Thread_T;

extern void Thread_Start(Thread_T * p_thread);
extern void Thread_Stop(Thread_T * p_thread);
extern void Thread_SetFunction(Thread_T *p_thread, void (*function)(void*), void *p_context);
extern void Thread_SetPeriod(Thread_T * p_thread, uint16_t period);
extern void ThreadThread_SetFreq(Thread_T * p_thread, uint16_t freq);
extern void Thread_SetPeriodic(Thread_T * p_thread);
extern void Thread_SetOneShot(Thread_T * p_thread, void (*onComplete)(void*));
extern void Thread_SetOneShotProcs(Thread_T * p_thread, uint32_t procCounts, void (*onComplete)(void*));
extern void Thread_SetOneShotTime_MillisProcs(Thread_T * p_thread, uint32_t millis, uint32_t procCounts);
extern void Thread_SetOneShotTime_MillisPeriod(Thread_T * p_thread, uint32_t millis, uint32_t period);
extern void Thread_SetOneShotTime_MillisFreq(Thread_T * p_thread, uint32_t millis, uint32_t freq);
#endif /* THREAD_H_ */
