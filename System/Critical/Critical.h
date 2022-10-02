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
	@file 	Critical.h
	@author FireSoucery
	@brief  Implements Critical Section
	@version V0
*/
/******************************************************************************/
#ifndef CRITICAL_H
#define CRITICAL_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

extern uint32_t _Critical_InterruptDisableCount;
extern uint32_t _Critical_RegPrimask;

/*
	HAL implement within module.
	Alternatively, implement in parent HAL and include.
*/
#ifdef CONFIG_SYSTEM_MCU_ARM

#include "External/CMSIS/Core/Include/cmsis_compiler.h"

#if defined(__GNUC__)
#define CRITICAL_DISABLE_INTERRUPTS() __asm volatile ("cpsid i" : : : "memory");
#define CRITICAL_ENABLE_INTERRUPTS() __asm volatile ("cpsie i" : : : "memory");
#else
#define CRITICAL_DISABLE_INTERRUPTS() __asm("cpsid i")
#define CRITICAL_ENABLE_INTERRUPTS() __asm("cpsie i")
#endif

/*
	No nesting Critical within Critical
*/
static inline void Critical_Enter(void) { _Critical_RegPrimask = __get_PRIMASK(); CRITICAL_DISABLE_INTERRUPTS(); }
static inline void Critical_Exit(void) 	{ __set_PRIMASK(_Critical_RegPrimask); }

#elif defined(CONFIG_CRITICAL_USER_DEFINED)
/*
	user provider
	#define DISABLE_INTERRUPTS() {...}
	#define ENABLE_INTERRUPTS() {...}
*/
#elif defined(CONFIG_CRITICAL_DISABLED)
#define CRITICAL_DISABLE_INTERRUPTS()
#define CRITICAL_ENABLE_INTERRUPTS()
static inline void Critical_Enter(void) {}
static inline void Critical_Exit(void) {}
#endif



static inline void Critical_DisableIrq(void)
{
	CRITICAL_DISABLE_INTERRUPTS();
	_Critical_InterruptDisableCount++;
}

static inline void Critical_EnableIrq(void)
{
	if(_Critical_InterruptDisableCount > 0U)
	{
		_Critical_InterruptDisableCount--;
		if(_Critical_InterruptDisableCount <= 0U)
		{
			CRITICAL_ENABLE_INTERRUPTS();
		}
	}
}

/*
	Non blocking mutex. must check if process completed
*/
typedef volatile uint8_t critical_mutex_t;

static inline bool Critical_AquireMutex(critical_mutex_t * p_mutex)
{
	bool status = false;

	Critical_Enter();
	if(*p_mutex == 1U)
	{
		*p_mutex = 0U;
		status = true;
	}
	Critical_Exit();

	return status;
}

static inline void Critical_ReleaseMutex(critical_mutex_t * p_mutex)
{
	Critical_Enter();
	if(*p_mutex == 0U)
	{
		*p_mutex = 1U;
	}
	Critical_Exit();
}

/* When thread sleep is not support, configure to global interrupt disable */

static inline bool Critical_AquireEnter(critical_mutex_t * p_mutex)
{
#if defined(CONFIG_CRITICAL_USE_MUTEX)
	return Critical_AquireMutex(p_mutex) ? true : false;
#else
	(void)p_mutex;
	Critical_Enter();
	return true;
#endif
}

static inline void Critical_ReleaseExit(critical_mutex_t * p_mutex)
{
#if defined(CONFIG_CRITICAL_USE_MUTEX)
	Critical_ReleaseMutex(p_mutex);
#else
	(void)p_mutex;
	Critical_Exit();
#endif
}


/*
	todo static inline void Critical_Enter(void * stateData) for other/semaphore implementation
*/
 //typedef volatile uint32_t critical_semaphore_t;
 //
 //static inline void Critical_SemaphorePost(critical_semaphore_t * p_semaphore)
 //{
 //	CRITICAL_DISABLE_INTERRUPTS();
 //	if (*p_semaphore > 0U)
 //	{
 //		*p_semaphore--;
 //	}
 //	CRITICAL_ENABLE_INTERRUPTS();
 //
 //}
 //
 //static inline void Critical_SemaphoreSignal(critical_semaphore_t *p_semaphore)
 //{
 //	CRITICAL_DISABLE_INTERRUPTS();
 //	if (*p_semaphore < 1U)
 //	{
 //		*p_semaphore++;
 //	}
 //	CRITICAL_ENABLE_INTERRUPTS();
 //}

#endif
