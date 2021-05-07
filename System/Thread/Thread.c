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
	@file  	Thread.c
	@author FireSourcery
	@brief 	Simple thread manager.
	@version V0
 */
/******************************************************************************/
#include "Thread.h"

#include <stdbool.h>
#include <stdint.h>

/**************************************************************************/
/*!
    @brief	Thread Module
*/
/**************************************************************************/
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

static inline bool ProcThreadOneShot(Thread_T * p_thread)
{
	bool proc;

	if (p_thread->TicksRemaining > 0) // last update on 1 ticks remaining
	{
		p_thread->TicksRemaining--;
		ProcThread(p_thread);
		proc = true;
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
		proc = false;
	}

	return proc;
}

/*!
    @brief	Public function for processing threads.
    		Run this function inside the main loop.

    @param	thread pointer to the thread to be processed
    @return True if the function ran, otherwise false.
*/
bool Thread_ProcThread(Thread_T * p_thread)
{
	bool proc;

	if (!p_thread->IsEnabled)
	{
		proc = false;
	}
	else
	{
		p_thread->TimePrev = *p_thread->p_Timer;

		if (p_thread->IsOneShot)
		{
			proc = ProcThreadOneShot(p_thread);
		}
		else
		{
			ProcThread(p_thread);
			proc = true;
		}
	}

	return proc;
}

bool Thread_PollThread(Thread_T * p_thread)
{
	bool proc;

	if (Thread_PollTimer(p_thread) == false)
	{
		proc = false;
	}
	else
	{
		proc = Thread_ProcThread(p_thread);
	}

	return proc;
}

uint32_t Thread_ProcThreadNRepeat(Thread_T * p_thread, uint32_t n)
{
	uint8_t count = 0;

	for (uint32_t t = 0; t < n; t++)
	{
		if (ProcThreadOneShot(p_thread)) {count++;}
	}

	return count;
}

/******************************************************************************/
/*!
    @brief	 Thread Init functions - assigns all parameters necessary to run
 */
/*! @{ */
/******************************************************************************/
void Thread_InitThread
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t timerfreq,
	uint32_t period,
	void (*function)(volatile void *),
	volatile void * p_context,
	bool oneShot,
	uint32_t ticks,
	void(*onComplete)(volatile void *),
	volatile void * p_onCompleteContext
)
{
	p_thread->p_Timer 	= p_timer;
	p_thread->TimerFreq = timerfreq;
	p_thread->Period 	= period;

	p_thread->Function 	= function;
	p_thread->p_Context = p_context;

	p_thread->IsOneShot				= oneShot;
	p_thread->Ticks		 			= ticks;
	p_thread->TicksRemaining 		= ticks;
	p_thread->OnComplete 			= onComplete;
	p_thread->p_OnCompleteContext	= p_onCompleteContext;

	p_thread->IsEnabled = false;
	p_thread->TimePrev 	= *p_thread->p_Timer - period;
}

void Thread_InitThreadPeriodic_Period
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t period,
	uint32_t timerfreq,
	void (*function)(volatile void *),
	volatile void * p_context
)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, period, function, p_context, 0, 0, 0, 0);
}

void Thread_InitThreadPeriodic_Freq
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t freq,
	uint32_t timerfreq,
	void (*function)(volatile void *),
	volatile void * p_context
)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, timerfreq / freq, function, p_context, 0, 0, 0, 0);
}

/*
	e.g. wipe 2 leds at the rate of 500ms each
 */
void Thread_InitThreadOneShot_Period
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t timerfreq,
	uint32_t period,
	uint32_t ticks,
	void (*function)(volatile void *),
	volatile void * p_context,
	void(*onComplete)(volatile void *),
	volatile void * p_onCompleteContext
)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, period, function, p_context, true, ticks, onComplete, p_onCompleteContext);
}

/*
	e.g. wipe 2 leds at the rate of 5 fps
 */
void Thread_InitThreadOneShot_Freq
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t timerfreq,
	uint32_t freq,
	uint32_t ticks,
	void (*function)(volatile void *),
	volatile void * p_context,
	void(*onComplete)(volatile void *),
	volatile void * p_onCompleteContext
)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, timerfreq / freq, function, p_context, true, ticks, onComplete, p_onCompleteContext);
}

/*
	e.g. wipe 2 leds at the total time of 1200ms, fader of 100 ticks in 1200ms
 */
void Thread_InitThreadOneShot_Millis
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t timerfreq,
	uint32_t millis,
	uint32_t ticks,
	void (*function)(volatile void *),
	volatile void * p_context,
	void(*onComplete)(volatile void *),
	volatile void * p_onCompleteContext
)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, timerfreq / (millis * 1000U) / ticks, function, p_context, true, ticks, onComplete, p_onCompleteContext);
}

/*
	rainbow 1000ms at 60fps
 */
void Thread_InitThreadOneShot_MillisFreq
(
	Thread_T * p_thread,
	const volatile uint32_t *p_timer,
	uint32_t timerfreq,
	uint32_t millis,
	uint32_t freq,
	void (*function)(volatile void *),
	volatile void * p_context,
	void(*onComplete)(volatile void *),
	volatile void * p_onCompleteContext
)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, timerfreq / freq, function, p_context, true, freq / (millis * 1000U), onComplete, p_onCompleteContext);
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
void Thread_StartThread(Thread_T * p_thread)
{
	p_thread->IsEnabled = true;
	if (p_thread->IsOneShot) {p_thread->TicksRemaining = p_thread->Ticks;}
}

void Thread_StopThread(Thread_T * p_thread)
{
	p_thread->IsEnabled = false;
}

void Thread_SetFunction(Thread_T *p_thread, void (*function)(volatile void*), volatile void *p_context)
{
	p_thread->Function = function;
	p_thread->p_Context = p_context;
}

void Thread_SetPeriod(Thread_T * p_thread, uint16_t period)
{
	p_thread->Period = period;
//	if (period < p_thread->PeriodMin)	p_thread->Period = p_thread->PeriodMin;
//	if (period > p_thread->PeriodMax)	p_thread->Period = p_thread->PeriodMax;
//	else								p_thread->Period = period;
}

void Thread_SetFreq(Thread_T * p_thread, uint16_t freq)
{
	p_thread->Period = p_thread->TimerFreq / freq;
//	if 		(freq > p_thread->TimerFreq/p_thread->PeriodMin)	p_thread->Period = p_thread->TimerFreq / p_thread->PeriodMin;
//	else if (freq < p_thread->TimerFreq/p_thread->PeriodMax)	p_thread->Period = p_thread->TimerFreq / p_thread->PeriodMax;
//	else														p_thread->Period = p_thread->TimerFreq / freq;
}

void Thread_SetOneShot(Thread_T *p_thread, uint32_t ticks)
{
	p_thread->IsOneShot = true;
	p_thread->Ticks = ticks;
	p_thread->TicksRemaining = ticks;
}

void Thread_SetOneShot_Period(Thread_T *p_thread, uint32_t ticks, uint32_t period)
{
	p_thread->IsOneShot = true;
	p_thread->Ticks = ticks;
	p_thread->TicksRemaining = ticks;
//		Thread_SetPeriod(thread, period);
	p_thread->Period = period;
}

void Thread_SetOneShot_Freq(Thread_T *p_thread, uint32_t ticks, uint32_t freq)
{
	p_thread->IsOneShot = true;
	p_thread->Ticks = ticks;
	p_thread->TicksRemaining = ticks;
//	Thread_SetPeriod(thread, TimerFreq / freq);
	p_thread->Period = p_thread->TimerFreq / freq;
}

void Thread_SetOneShot_Millis(Thread_T *p_thread, uint32_t millis, uint32_t ticks)
{
	p_thread->IsOneShot = true;
	p_thread->Ticks = ticks;
	p_thread->TicksRemaining = ticks;
//	Thread_SetPeriod(thread, period / ticks);
	p_thread->Period = p_thread->TimerFreq / (millis * 1000U) / ticks; //check error
}

void Thread_SetOneShot_MillisFreq(Thread_T *p_thread, uint32_t millis, uint32_t freq)
{
	p_thread->IsOneShot = true;
//	Thread_SetPeriod(thread, TimerFreq / freq);
	p_thread->Period = p_thread->TimerFreq / freq;
	p_thread->Ticks = freq / (millis * 1000U); //check error //round up or down?
	p_thread->TicksRemaining = p_thread->Ticks;
}

	static uint32_t BoundedFaster(uint32_t timerFreq, uint16_t freqInc, uint16_t freqCurrent, uint16_t freqMin, uint16_t freqMax, bool useLoopBoundary)
	{
		if(freqCurrent + freqInc > freqMax)
		{
			if (useLoopBoundary)	freqCurrent = freqMin;
			else					freqCurrent = freqMax;
		}
		else
		{
			freqCurrent += freqInc;
		}

		return timerFreq / freqCurrent;
	}

	static uint32_t BoundedSlower(uint32_t timerFreq, uint16_t freqDec, uint16_t freqCurrent, uint16_t freqMin, uint16_t freqMax, bool useLoopBoundary)
	{
		if(freqCurrent - freqDec < freqMin)
		{
			if (useLoopBoundary)		freqCurrent = freqMax;
			else						freqCurrent = freqMin;
		}
		else
		{
			freqCurrent -= freqDec;
		}

		return timerFreq / freqCurrent;
	}

//void Thread_SetFaster(Thread_T * p_thread, uint8_t freqInc, bool useLoopBoundary)
//{
//	uint32_t freqCurrent = p_thread->TimerFreq / p_thread->Period;
//	uint32_t freqMin = p_thread->TimerFreq / p_thread->PeriodMax;
//	uint32_t freqMax = p_thread->TimerFreq / p_thread->PeriodMin;
//
//	p_thread->Period = BoundedFaster(freqInc, freqMin, freqCurrent, freqMax, useLoopBoundary);
//}
//
//void Thread_SetSlower(Thread_T * p_thread, uint8_t freqDec, bool useLoopBoundary)
//{
//	uint32_t freqCurrent = p_thread->TimerFreq / p_thread->Period;
//	uint32_t freqMin = p_thread->TimerFreq / p_thread->PeriodMax;
//	uint32_t freqMax = p_thread->TimerFreq / p_thread->PeriodMin;
//
//	p_thread->Period = BoundedSlower(freqDec, freqMin, freqCurrent, freqMax, useLoopBoundary);
//}

void Thread_SetFaster_MinMax(Thread_T * p_thread, uint8_t freqInc, uint8_t freqMin, uint8_t freqMax, bool useLoopBoundary)
{
	uint32_t freqCurrent = p_thread->TimerFreq / p_thread->Period;

	p_thread->Period = BoundedFaster(p_thread->TimerFreq, freqInc, freqMin, freqCurrent, freqMax, useLoopBoundary);
}

void Thread_SetSlower_MinMax(Thread_T * p_thread, uint8_t freqDec, uint8_t freqMin, uint8_t freqMax, bool useLoopBoundary)
{
	uint32_t freqCurrent = p_thread->TimerFreq / p_thread->Period;

	p_thread->Period = BoundedSlower(p_thread->TimerFreq, freqDec, freqMin, freqCurrent, freqMax, useLoopBoundary);
}
/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
    @brief	Timer Mode Init
 */
/*! @{ */
/******************************************************************************/
void Thread_InitTimer(Thread_T *p_thread, const volatile uint32_t *p_timer, uint32_t timerfreq, uint32_t period)
{
	Thread_InitThread(p_thread, p_timer, timerfreq, period, 0U, 0U, 0U, 0U, 0U, 0U);
	p_thread->TimePrev = *p_thread->p_Timer;
}

void Thread_InitTimer_Millis(Thread_T *p_thread, const volatile uint32_t *p_timer, uint32_t timerfreq, uint32_t ms)
{
	Thread_InitTimer(p_thread, p_timer, timerfreq, p_thread->TimerFreq / (1000 * ms));
}
/******************************************************************************/
/*! @} */
/******************************************************************************/