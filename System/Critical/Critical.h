/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Critical.h
    @author FireSoucery
    @brief  Implements Critical Section
    @version V0
*/
/**************************************************************************/
#ifndef CRITICAL_H
#define CRITICAL_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Implement per submodule HAL for now
 * 	or implement in parent HAL and include
 */
#ifdef CONFIG_CRITICAL_MCU_ARM
	#if defined (__GNUC__)
		#define CRITICAL_DISABLE_INTERRUPTS() __asm volatile ("cpsid i" : : : "memory");
		#define CRITICAL_ENABLE_INTERRUPTS() __asm volatile ("cpsie i" : : : "memory");
	#else
		#define CRITICAL_DISABLE_INTERRUPTS() __asm("cpsid i")
		#define CRITICAL_ENABLE_INTERRUPTS() __asm("cpsie i")
	#endif
#elif defined(CONFIG_CRITICAL_USER_DEFINED)
/*
 * user provide
 * #define DISABLE_INTERRUPTS() {...}
 * #define ENABLE_INTERRUPTS() {...}
 */
#elif defined(CONFIG_CRITICAL_DISABLED)
	#define CRITICAL_DISABLE_INTERRUPTS() {}
	#define CRITICAL_ENABLE_INTERRUPTS() {}
#endif

static int32_t InterruptDisableCount = 0;

static inline void Critical_Enter(void)
{
	CRITICAL_DISABLE_INTERRUPTS();
	InterruptDisableCount++;
}

static inline void Critical_Exit(void)
{
	if (InterruptDisableCount > 0)
	{
		InterruptDisableCount--;
		if (InterruptDisableCount <= 0)
		{
			CRITICAL_ENABLE_INTERRUPTS();
		}
	}
}



typedef volatile uint8_t critical_mutex_t;

static inline bool Critical_MutexAquire(critical_mutex_t * p_mutex)
{
	bool status = false;

	CRITICAL_DISABLE_INTERRUPTS();
	if (*p_mutex == 1U)
	{
		*p_mutex = 0U;
		status = true;
	}
	CRITICAL_ENABLE_INTERRUPTS();

	return status;
}

static inline void Critical_MutexRelease(critical_mutex_t * p_mutex)
{
	CRITICAL_DISABLE_INTERRUPTS();
	if (*p_mutex == 0U)
	{
		*p_mutex = 1U;
	}
	CRITICAL_ENABLE_INTERRUPTS();
}

/*
 * Todo static inline void Critical_Enter(void * stateData) for other/semaphore implementation
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
