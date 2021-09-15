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
	const volatile uint32_t * p_Timer; 	// User app must provide timercounter. per thread instance allows different timers bases

	uint32_t TimerFreq;					// 1000 ticks per second using millis
//	uint32_t PeriodMax;
//	uint32_t PeriodMin;
	uint32_t Period;				// Number of timer ticks between updates
	volatile uint32_t TimerPrev; 	// Last update time

	bool IsOneShot;					// One-time or periodic
	uint32_t ProcCounts;			// One shot procs count, not timer ticks
	volatile uint32_t ProcCountsRemaining;	// One shot procs count, not timer timer

	volatile bool IsEnabled;		// Enable or disable

#ifdef CONFIG_THREAD_FUNCTION_CONTEXT_ENABLED
	void (*Function)(volatile void * p_context); /* Current function to run */
	volatile void * p_Context;						// Option to load context before running
	void (*OnComplete)(volatile void * p_context);
	volatile void * p_OnCompleteContext;
#elif defined (CONFIG_THREAD_FUNCTION_CONTEXT_DISABLED)
	void (*Function)(void);
	void (*OnComplete)(void);
#endif
} Thread_T;



/**************************************************************************/
/*!
    @brief	Timer SubModule Functions
*/
/**************************************************************************/
static inline uint32_t Thread_GetTimerElapsed(Thread_T *p_thread)
{
	uint32_t ticks;

#ifdef CONFIG_THREAD_TIMER_OVERFLOW_WRAP
	//Not necessarily needed is overflow time is in days
	if (*p_thread->p_Timer < p_thread->TimerPrev)
	{
		ticks = UINT32_MAX - p_thread->TimePrev + *p_thread->p_Timer
	}
	else
#else
	{
		ticks = *p_thread->p_Timer - p_thread->TimerPrev;
	}
#endif

	return ticks;
}

static inline uint32_t Thread_GetTimerElapsedTicks(Thread_T * p_thread)
{
	return Thread_GetTimerElapsed(p_thread);
}

static inline uint32_t Thread_GetTimerElapsedSeconds(Thread_T * p_thread)
{
	return Thread_GetTimerElapsedTicks(p_thread) / p_thread->TimerFreq;
}

static inline uint32_t Thread_GetTimerElapsedMillis(Thread_T * p_thread)
{
	return Thread_GetTimerElapsedTicks(p_thread) * 1000U / p_thread->TimerFreq;
}

static inline uint32_t Thread_GetTimerElapsedMicros(Thread_T *p_thread)
{
	uint32_t ticks = Thread_GetTimerElapsedTicks(p_thread);
	uint32_t micros;

	if(ticks > UINT32_MAX / 1000000U)
	{
		micros = ticks / p_thread->TimerFreq * 1000000U;
	}
	else
	{
		micros = ticks * 1000000U / p_thread->TimerFreq;
	}

	return  micros;
}

static inline uint32_t Thread_ConvertTimerMillisToTicks(Thread_T *p_thread, uint32_t ms)
{
	return (p_thread->TimerFreq * ms / 1000U);
}

static inline bool Thread_PollTimerComplete(Thread_T * p_thread)
{
	bool proc;

	//will return erroneous true if timer wraps + period
	if (Thread_GetTimerElapsedTicks(p_thread) >= p_thread->Period)
	{
		proc = true;
	}
	else
	{
		proc = false;
	}

	return proc;
}

static inline bool Thread_PollTimerCompleteOnce(Thread_T * p_thread)
{
 	return Thread_PollTimerComplete(p_thread) ;
}

static inline bool Thread_PollTimerCompletePeriodic(Thread_T * p_thread)
{
	bool proc;

	if (Thread_PollTimerComplete(p_thread) == true)
	{
		p_thread->TimerPrev = *p_thread->p_Timer;
		proc = true;
	}
	else
	{
		proc = false;
	}

	return proc;
}

//static inline bool Thread_PollTimerCompleteThread(Thread_T * p_thread)
//{
//	bool proc;
//
//	if ((Thread_GetTimerElapsedTicks(p_thread) >= p_thread->Period) && p_thread->IsEnabled == true)
//	{
//		proc = true;
//		if (p_thread->IsOneShot == true)
//		{
//			p_thread->IsEnabled = false;
//		}
//		else
//		{
//			p_thread->TimerPrev = *p_thread->p_Timer;
//		}
//	}
//	else
//	{
//		proc = false;
//	}
//
//	return proc;
//}

static inline void Thread_SetTimerPeriod(Thread_T *p_thread, uint32_t ticks)
{
	p_thread->Period = ticks;
	p_thread->TimerPrev = *p_thread->p_Timer;
}

static inline void Thread_SetTimerPeriodTicks(Thread_T *p_thread, uint32_t ticks)
{
	Thread_SetTimerPeriod(p_thread, ticks);
}

static inline void Thread_SetTimerPeriodMillis(Thread_T *p_thread, uint32_t ms)
{
	p_thread->Period = p_thread->TimerFreq * ms / 1000U;
	p_thread->TimerPrev = *p_thread->p_Timer;
}

static inline void Thread_RestartTimer(Thread_T *p_thread)
{
	p_thread->TimerPrev = *p_thread->p_Timer;
}

#endif /* THREAD_H_ */
