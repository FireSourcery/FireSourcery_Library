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
	@file 	Queue.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef QUEUE_H
#define QUEUE_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef const struct
{
	void * const P_BUFFER;
	const size_t LENGTH; 	/* in unit counts */
	const size_t UNIT_SIZE;
#if defined(CONFIG_QUEUE_LENGTH_POW2) || defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	const uint32_t POW2_MASK;
#endif
#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	const bool IS_MULTITHREADED;
#endif
}
Queue_Config_T;

/*
 * Empty space detection method. Head always points to empty space. Max usable capacity is length - 1
 */
typedef struct
{
	const Queue_Config_T CONFIG;
//	bool IsOverwritable;
	volatile size_t Head;	//write to head
	volatile size_t Tail;	//read from tail
#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	volatile critical_mutext_t Mutex;
#endif
}
Queue_T;

#if defined(CONFIG_QUEUE_LENGTH_POW2) || defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	#define QUEUE_CONFIG_POW2(Pow2Mask) .POW2_MASK = Pow2Mask,
#else
	#define QUEUE_CONFIG_POW2(Pow2Mask)
#endif

#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	#define QUEUE_CONFIG_MULTITHREADED(IsMultithreaded) .IS_MULTITHREADED = IsMultithreaded,
#else
	#define QUEUE_CONFIG_MULTITHREADED(IsMultithreaded)
#endif

#define QUEUE_CONFIG(p_Buffer, Length, UnitSize, IsMultithreaded)	\
{														\
	.CONFIG =											\
	{                                               	\
		.P_BUFFER			= p_Buffer,					\
		.LENGTH				= Length,					\
		.UNIT_SIZE			= UnitSize,					\
		QUEUE_CONFIG_POW2(Length - 1U)					\
		QUEUE_CONFIG_MULTITHREADED(IsMultithreaded)		\
	},													\
}

static inline size_t CalcQueueIndexMasked(const Queue_T * p_queue, size_t index)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED) || defined(CONFIG_QUEUE_LENGTH_POW2)
	return (index & p_queue->CONFIG.POW2_MASK);
#else
	return (index % p_queue->CONFIG.LENGTH);
#endif
}

static inline size_t CalcQueueIndexInc(const Queue_T * p_queue, size_t index, size_t inc)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	return index + inc;
#else
	return CalcQueueIndexMasked(p_queue, index + inc);
#endif
}


static inline size_t CalcQueueIndexDec(const Queue_T * p_queue, size_t index, size_t dec)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2) || defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	return CalcQueueIndexMasked(p_queue, index - dec);
#else
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + index - dec);
	//(index < 1U) ? index = p_unit, p_queue->CONFIG.LENGTH - 1U : index--;
#endif
}

/*
 * later 2 cases: returns max of buffer length - 1, to account for 1 space used for detection
 */
static inline size_t Queue_GetFullCount(const Queue_T * p_queue)
{
#ifdef CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED
	return (p_queue->Head - p_queue->Tail);
#elif defined(CONFIG_QUEUE_LENGTH_POW2)
	return CalcQueueIndexMasked(p_queue, p_queue->Head - p_queue->Tail);
#else
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + p_queue->Head - p_queue->Tail);
#endif
}

/*
 * later 2 cases: -1 to account for space used for detection
 */
static inline size_t Queue_GetEmptyCount(const Queue_T * p_queue)
{
#ifdef CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED
	return p_queue->CONFIG.LENGTH + p_queue->Tail - p_queue->Head;
#elif defined(CONFIG_QUEUE_LENGTH_POW2)
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + p_queue->Tail - p_queue->Head - 1U);
#else
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + p_queue->Tail - p_queue->Head - 1U);
	//	return (p_queue->Tail > p_queue->Head) ? (p_queue->Tail - p_queue->Head - 1U) : (p_queue->CONFIG.LENGTH - p_queue->Head + p_queue->Tail - 1U);
#endif
}

static inline bool Queue_GetIsEmpty(const Queue_T *p_queue)
{
	return (p_queue->Head == p_queue->Tail);
}

static inline bool Queue_GetIsFull(const Queue_T * p_queue)
{
#ifdef CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED
	return (Queue_GetFullCount(p_queue) == p_queue->CONFIG.LENGTH);
#else
	return (CalcQueueIndexInc(p_queue, p_queue->Head, 1U) == p_queue->Tail);
#endif
}

extern void Queue_Init(Queue_T * p_queue);
#ifdef CONFIG_QUEUE_DYNAMIC_MEMORY_ALLOCATION
extern Queue_T * Queue_New(size_t unitCount, size_t unitSize);
#endif
extern void Queue_Clear(Queue_T * p_queue);
extern bool Queue_Enqueue	(Queue_T * p_queue, const void * p_unit);
extern bool Queue_Dequeue	(Queue_T * p_queue, void * p_dest);
extern bool Queue_EnqueueN	(Queue_T * p_queue, const void * p_unit, size_t nUnits);
extern bool Queue_DequeueN	(Queue_T * p_queue, void * p_dest, size_t nUnits);
extern size_t Queue_EnqueueMax(Queue_T * p_queue, const void * p_units, size_t nUnits);
extern size_t Queue_DequeueMax(Queue_T * p_queue, void * p_dest, size_t p_destLength);
extern bool Queue_PushFront	(Queue_T * p_queue, const void * p_unit);
extern bool Queue_PopBack	(Queue_T * p_queue, void * p_dest);
extern bool Queue_PeekFront	(Queue_T * p_queue, void * p_dest);
extern bool Queue_PeekBack	(Queue_T * p_queue, void * p_dest);
extern bool Queue_PeekIndex	(Queue_T * p_queue, void * p_dest, size_t index);

extern void * Queue_PeekPtrFront(Queue_T * p_queue);
extern void * Queue_PeekPtrIndex(Queue_T * p_queue, size_t index);
extern void * Queue_DequeuePtr(Queue_T * p_queue);

#endif
