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
	@brief	Improved efficiency using POW2_MASK over modulus
	@version V0
*/
/******************************************************************************/
#ifndef QUEUE_FIRE_SOURCERY_H
#define QUEUE_FIRE_SOURCERY_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef const struct Queue_Config_Tag
{
	void * const P_BUFFER;
	const size_t UNIT_SIZE;
	const size_t LENGTH; 	/* In UNIT_SIZE counts (NOT bytes) */
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED) || defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	const uint32_t POW2_MASK;
#endif
#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_EXTERN_DEFINED)
	const bool USE_CRITICAL;
#endif
}
Queue_Config_T;

/*
	CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED
	Max usable capacity is length

	CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED, CONFIG_QUEUE_LENGTH_ANY
	Empty space detection method. Head always points to empty space. Max usable capacity is length - 1
*/
typedef struct Queue_Tag
{
	const Queue_Config_T CONFIG;
	volatile size_t Head;	/* Write to head  */
	volatile size_t Tail;	/* Read from tail */
#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_EXTERN_DEFINED)
	volatile critical_mutext_t Mutex;
#endif
}
Queue_T;

#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED) || defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
#define QUEUE_DEFINE_POW2(Pow2Mask) .POW2_MASK = Pow2Mask,
#else
#define QUEUE_DEFINE_POW2(Pow2Mask)
#endif

#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_EXTERN_DEFINED)
#define QUEUE_DEFINE_CRITICAL(UseCritical) .USE_CRITICAL = UseCritical,
#else
#define QUEUE_DEFINE_CRITICAL(UseCritical)
#endif

#define QUEUE_DEFINE(p_Buffer, Length, UnitSize, UseCritical)	\
{																\
	.CONFIG =													\
	{                                               			\
		.P_BUFFER			= p_Buffer,							\
		.LENGTH				= Length,							\
		.UNIT_SIZE			= UnitSize,							\
		QUEUE_DEFINE_POW2(Length - 1U)							\
		QUEUE_DEFINE_CRITICAL(UseCritical)						\
	},															\
}

static inline size_t CalcQueueIndexMasked(const Queue_T * p_queue, size_t index)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED) || defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED)
	return (index & p_queue->CONFIG.POW2_MASK);
#else
	return (index % p_queue->CONFIG.LENGTH);
#endif
}

static inline size_t CalcQueueIndexInc(const Queue_T * p_queue, size_t index, size_t inc)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	(void)p_queue;
	return index + inc;
#else
	return CalcQueueIndexMasked(p_queue, index + inc);
#endif
}

static inline size_t CalcQueueIndexDec(const Queue_T * p_queue, size_t index, size_t dec)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	(void)p_queue;
	return index - dec;
#elif defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED)
	return CalcQueueIndexMasked(p_queue, index - dec);
#else
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + index - dec);
	//(index < 1U) ? index = p_unit, p_queue->CONFIG.LENGTH - 1U : index--;
#endif
}

/*
	later 2 cases: returns max of buffer length - 1, to account for 1 space used for detection
*/
static inline size_t Queue_GetFullCount(const Queue_T * p_queue)
{
#ifdef CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED
	return (p_queue->Head - p_queue->Tail);
#elif defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED)
	return CalcQueueIndexMasked(p_queue, p_queue->Head - p_queue->Tail);
#else
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + p_queue->Head - p_queue->Tail);
#endif
}

/*
	later 2 cases: -1 to account for space used for detection
*/
static inline size_t Queue_GetEmptyCount(const Queue_T * p_queue)
{
#ifdef CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED
	return p_queue->CONFIG.LENGTH + p_queue->Tail - p_queue->Head;
#elif defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_WRAPPED)
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + p_queue->Tail - p_queue->Head - 1U);
#else
	return CalcQueueIndexMasked(p_queue, p_queue->CONFIG.LENGTH + p_queue->Tail - p_queue->Head - 1U);
	//	return (p_queue->Tail > p_queue->Head) ? (p_queue->Tail - p_queue->Head - 1U) : (p_queue->CONFIG.LENGTH - p_queue->Head + p_queue->Tail - 1U);
#endif
}

static inline bool Queue_GetIsEmpty(const Queue_T * p_queue)
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
extern bool Queue_Enqueue(Queue_T * p_queue, const void * p_unit);
extern bool Queue_Dequeue(Queue_T * p_queue, void * p_dest);
extern bool Queue_EnqueueN(Queue_T * p_queue, const void * p_unit, size_t nUnits);
extern bool Queue_DequeueN(Queue_T * p_queue, void * p_dest, size_t nUnits);
extern size_t Queue_EnqueueMax(Queue_T * p_queue, const void * p_units, size_t nUnits);
extern size_t Queue_DequeueMax(Queue_T * p_queue, void * p_dest, size_t destLength);
extern bool Queue_PushFront(Queue_T * p_queue, const void * p_unit);
extern bool Queue_PopBack(Queue_T * p_queue, void * p_dest);
extern bool Queue_PeekFront(Queue_T * p_queue, void * p_dest);
extern bool Queue_PeekBack(Queue_T * p_queue, void * p_dest);
extern bool Queue_PeekIndex(Queue_T * p_queue, void * p_dest, size_t index);
extern bool Queue_RemoveFront(Queue_T * p_queue, size_t nUnits);
extern bool Queue_RemoveBack(Queue_T * p_queue, size_t nUnits);
extern bool Queue_Seek(Queue_T * p_queue, void * p_dest, size_t index);
extern void * Queue_PeekPtrFront(Queue_T * p_queue);
extern void * Queue_PeekPtrBack(Queue_T * p_queue);
extern void * Queue_PeekPtrIndex(Queue_T * p_queue, size_t index);
extern void * Queue_SeekPtr(Queue_T * p_queue, size_t index);
extern uint8_t * Queue_AcquireBuffer(Queue_T * p_queue);
extern void Queue_ReleaseBuffer(Queue_T * p_queue, size_t writeSize);

#endif
