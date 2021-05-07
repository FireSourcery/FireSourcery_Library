/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@file  	Thread.h
	@author FireSourcery
	@brief 	Simple thread manager.

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

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	const volatile uint32_t *p_Timer; 	// User app must provide millis timer. Shared between all thread instances
	uint32_t TimerFreq;					// 1000 ticks per second using millis

//	uint32_t PeriodMax;
//	uint32_t PeriodMin;

	volatile uint32_t Period;		// Number of timer ticks between updates
	volatile uint32_t TimePrev; 	// Last update time
	volatile bool IsEnabled;		// Enable or disable

#ifdef CONFIG_THREAD_FUNCTION_CONTEXT_ENABLED
	void (*Function)(volatile void * p_context); /* Current function to run */
	volatile void * p_Context;						// Option to load context before running
#elif defined (CONFIG_THREAD_FUNCTION_CONTEXT_DISABLED)
	void (*Function)(void);
#endif

	volatile bool IsOneShot;			// One-time or periodic
	volatile uint32_t Ticks;			// One shot procs count, not timer ticks
	volatile uint32_t TicksRemaining;	// One shot procs count, not timer timer

#ifdef CONFIG_THREAD_FUNCTION_CONTEXT_ENABLED
	void (*OnComplete)(volatile void * p_context); /* Current function to run */
	volatile void * p_OnCompleteContext;						// Option to load context before running
#elif defined (CONFIG_THREAD_FUNCTION_CONTEXT_DISABLED)
	void (*OnComplete)(void);
#endif
} Thread_T;



/**************************************************************************/
/*!
    @brief	Timer SubModule Functions
*/
/**************************************************************************/
static inline bool Thread_PollTimer(Thread_T * p_thread)
{
	bool proc;

	if (*p_thread->p_Timer < p_thread->TimePrev + p_thread->Period)
	{
		proc = false;
	}
	else
	{
		proc = true;
	}

	return proc;
}

static inline uint32_t Thread_GetTimer(Thread_T *p_thread)
{
	return *p_thread->p_Timer - p_thread->TimePrev;
}

static inline uint32_t Thread_GetTimer_Millis(Thread_T *p_thread)
{
	return p_thread->TimerFreq / ((*p_thread->p_Timer - p_thread->TimePrev) * 1000U);
}

static inline uint32_t Thread_ConvertMillisToTimer(Thread_T *p_thread, uint32_t ms)
{
	return p_thread->TimerFreq / (ms * 1000U);
}

static inline void Thread_SetTimer(Thread_T *p_thread, uint32_t ticks)
{
	p_thread->Period = ticks;
	p_thread->TimePrev = *p_thread->p_Timer;
}

static inline void Thread_SetTimer_Millis(Thread_T *p_thread, uint32_t ms)
{
	p_thread->Period = p_thread->TimerFreq / (1000U * ms);
	p_thread->TimePrev = *p_thread->p_Timer;
}

static inline void Thread_RestartTimer(Thread_T *p_thread)
{
	p_thread->TimePrev = *p_thread->p_Timer;
}

#endif /* THREAD_H_ */
