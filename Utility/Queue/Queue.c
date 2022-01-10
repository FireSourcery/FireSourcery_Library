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
	@file 	Queue.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Queue.h"
#include "Config.h"

#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED)
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_QUEUE_CRITICAL_USER_DEFINED)
	extern inline void Critical_Enter();
	extern inline void Critical_Exit();
	extern inline void Critical_AquireMutex(void * p_mutex);
	extern inline void Critical_AquireMutex(void * p_mutex);
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/******************************************************************************/
/*!
 * Private
 */
/******************************************************************************/
static inline void EnterCritical(void)
{
#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_USER_DEFINED)
	if (p_queue.CONFIG.USE_CRITICAL == true) { Critical_Enter(); }
#endif
}

static inline void ExitCritical(void)
{
#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_USER_DEFINED)
	if (p_queue.CONFIG.USE_CRITICAL == true) { Critical_Exit(); }
#endif
}

static inline bool AquireCritical(Queue_T * p_queue)
{
#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_USER_DEFINED)
		return (p_queue.CONFIG.USE_CRITICAL == true) ? Critical_Enter_Common(&p_queue->Mutex) : true;
#else
		return true;
#endif
}

static inline void ReleaseCritical(Queue_T * p_queue)
{
#if defined(CONFIG_QUEUE_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_CRITICAL_USER_DEFINED)
		if (p_queue.CONFIG.USE_CRITICAL == true) { Critical_Exit_Common(&p_queue->Mutex) };
#endif
}

static inline size_t CalcIndexWrap(Queue_T * p_queue, size_t index)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	return CalcQueueIndexMasked(p_queue, index);
#else
	return index;
#endif
}

static inline size_t CalcBufferOffset(Queue_T * p_queue, size_t index)
{
	return CalcIndexWrap(p_queue, index) * p_queue->CONFIG.UNIT_SIZE;
}

static inline void * GetPtrFront(Queue_T * p_queue)								{return ((uint8_t *)p_queue->CONFIG.P_BUFFER + CalcBufferOffset(p_queue, p_queue->Tail));}
static inline void * GetPtrBack(Queue_T * p_queue)								{return ((uint8_t *)p_queue->CONFIG.P_BUFFER + CalcBufferOffset(p_queue, p_queue->Head));}
static inline void * GetPtrIndex(Queue_T * p_queue, size_t index)				{return ((uint8_t *)p_queue->CONFIG.P_BUFFER + CalcBufferOffset(p_queue, p_queue->Tail + index));}
static inline void PeekFront(Queue_T * p_queue, void * p_dest)					{memcpy(p_dest, GetPtrFront(p_queue), p_queue->CONFIG.UNIT_SIZE);}
static inline void PeekBack(Queue_T * p_queue, void * p_dest)					{memcpy(p_dest, GetPtrBack(p_queue), p_queue->CONFIG.UNIT_SIZE);}
static inline void PeekIndex(Queue_T * p_queue, void * p_dest, size_t index)	{memcpy(p_dest, GetPtrIndex(p_queue, index), p_queue->CONFIG.UNIT_SIZE);}
static inline void PlaceFront(Queue_T * p_queue, const void * p_unit)			{memcpy(GetPtrFront(p_queue), p_unit, p_queue->CONFIG.UNIT_SIZE);}
static inline void PlaceBack(Queue_T * p_queue, const void * p_unit)			{memcpy(GetPtrBack(p_queue), p_unit, p_queue->CONFIG.UNIT_SIZE);}
static inline void RemoveFront(Queue_T * p_queue, size_t nUnits)				{p_queue->Tail = CalcQueueIndexInc(p_queue, p_queue->Tail, nUnits);}
static inline void RemoveBack(Queue_T * p_queue, size_t nUnits)					{p_queue->Head = CalcQueueIndexDec(p_queue, p_queue->Head, nUnits);}
static inline void AddFront(Queue_T * p_queue, size_t nUnits)					{p_queue->Tail = CalcQueueIndexDec(p_queue, p_queue->Tail, nUnits);}
static inline void AddBack(Queue_T * p_queue, size_t nUnits)					{p_queue->Head = CalcQueueIndexInc(p_queue, p_queue->Head, nUnits);}
static inline void Enqueue(Queue_T * p_queue, const void * p_unit)				{PlaceBack(p_queue, p_unit); AddBack(p_queue, 1U);}
static inline void Dequeue(Queue_T * p_queue, void * p_dest)					{PeekFront(p_queue, p_dest); RemoveFront(p_queue, 1U);}
static inline void PushFront(Queue_T * p_queue, const void * p_unit) 			{AddFront(p_queue, 1U); PlaceFront(p_queue, p_unit);}
static inline void PopBack(Queue_T * p_queue, void * p_dest)					{RemoveBack(p_queue, 1U); PeekBack(p_queue, p_dest);}
static inline void * SeekPtr(Queue_T * p_queue, size_t index) 					{RemoveFront(p_queue, index); return GetPtrFront(p_queue);}
static inline void Seek(Queue_T * p_queue, void * p_dest, size_t index)			{RemoveFront(p_queue, index); PeekFront(p_queue, p_dest);}

/******************************************************************************/
/*!
 * Public
 */
/******************************************************************************/
void Queue_Init(Queue_T * p_queue)
{
	Queue_Clear(p_queue);
}

#ifdef CONFIG_QUEUE_DYNAMIC_MEMORY_ALLOCATION
Queue_T * Queue_New(size_t unitCount, size_t unitSize)
{
	Queue_T * p_queue = malloc(sizeof(Queue_T));
	void * p_buffer = malloc(unitCount * unitSize);

	if (p_queue != 0U && p_buffer != 0U)
	{
		*(void *)&p_queue->CONFIG.P_BUFFER 		= p_buffer;
		*(size_t)&p_queue->CONFIG.LENGTH 		= unitCount;
		*(size_t)&p_queue->CONFIG.UNIT_SIZE 	= unitSize;

		Queue_Init(p_queue);
	}

	return p_queue;
}
#endif

void Queue_Clear(Queue_T * p_queue)
{
	p_queue->Head = 0U;
	p_queue->Tail = 0U;
}

bool Queue_Enqueue(Queue_T * p_queue, const void * p_unit)
{
	bool isSuccess = false;
	EnterCritical();
	if (Queue_GetIsFull(p_queue) == false) //ideally compiler stores value from inline function
	{
		Enqueue(p_queue, p_unit);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_Dequeue(Queue_T * p_queue, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if (Queue_GetIsEmpty(p_queue) == false)
	{
		Dequeue(p_queue, p_dest);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_EnqueueN(Queue_T * p_queue, const void * p_units, size_t nUnits)
{
	bool isSuccess = false;

	EnterCritical();
	if (nUnits <= Queue_GetEmptyCount(p_queue))
	{
		for(size_t iUnit = 0U; iUnit < nUnits; iUnit++)
		{
			Enqueue(p_queue, p_units + (iUnit * p_queue->CONFIG.UNIT_SIZE));
		}
		isSuccess = true;
	}
	ExitCritical();

	return isSuccess;
}

bool Queue_DequeueN(Queue_T * p_queue, void * p_dest, size_t nUnits)
{
	bool isSuccess = false;

	EnterCritical();
	if(nUnits <= Queue_GetFullCount(p_queue))
	{
		for(size_t iUnit = 0U; iUnit < nUnits; iUnit++)
		{
			Dequeue(p_queue, p_dest + (iUnit * p_queue->CONFIG.UNIT_SIZE));
		}
		isSuccess = true;
	}
	ExitCritical();

	return isSuccess;
}

size_t Queue_EnqueueMax(Queue_T * p_queue, const void * p_units, size_t nUnits)
{
	size_t count;

	EnterCritical();
	for(count = 0; count < nUnits; count++)
	{
		if(Queue_GetIsFull(p_queue) == false)
		{
			Enqueue(p_queue, p_units + (count * p_queue->CONFIG.UNIT_SIZE));
		}
		else
		{
			break;
		}
	}
	ExitCritical();

	return count;
}

size_t Queue_DequeueMax(Queue_T * p_queue, void * p_dest, size_t p_destLength)
{
	size_t count;

	EnterCritical();
	for(count = 0; count < p_destLength; count++)
	{
		if(Queue_GetIsEmpty(p_queue) == false)
		{
			Dequeue(p_queue, p_dest + (count * p_queue->CONFIG.UNIT_SIZE));
		}
		else
		{
			break;
		}
	}
	ExitCritical();

	return count;
}

bool Queue_PushFront(Queue_T * p_queue, const void * p_unit)
{
	bool isSuccess = false;
	EnterCritical();
	if(Queue_GetIsFull(p_queue) == false)
	{
		PushFront(p_queue, p_unit);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_PopBack(Queue_T * p_queue, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Queue_GetIsEmpty(p_queue) == false)
	{
		PopBack(p_queue, p_dest);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_PeekFront(Queue_T * p_queue, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Queue_GetIsEmpty(p_queue) == false)
	{
		PeekFront(p_queue, p_dest);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_PeekBack(Queue_T * p_queue, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Queue_GetIsEmpty(p_queue) == false)
	{
		PeekBack(p_queue, p_dest);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_PeekIndex(Queue_T * p_queue, void * p_dest, size_t index)
{
	bool isSuccess = false;
	EnterCritical();
	if(index < Queue_GetFullCount(p_queue))
	{
		PeekIndex(p_queue, p_dest, index);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_RemoveFront(Queue_T * p_queue, size_t nUnits)
{
	bool isSuccess = false;
	EnterCritical();
	if(nUnits <= Queue_GetFullCount(p_queue))
	{
		RemoveFront(p_queue, nUnits);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_RemoveBack(Queue_T * p_queue, size_t nUnits)
{
	bool isSuccess = false;
	EnterCritical();
	if(nUnits <= Queue_GetFullCount(p_queue))
	{
		RemoveBack(p_queue, nUnits);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

bool Queue_Seek(Queue_T * p_queue, void * p_dest, size_t index)
{
	bool isSuccess = false;
	EnterCritical();
	if(index < Queue_GetFullCount(p_queue))
	{
		Seek(p_queue, p_dest, index);
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

/*
	single threaded use only, buffer may be overwritten
	result in return value, pointer 0 indicate invalid
 */
void * Queue_PeekPtrFront(Queue_T * p_queue) 				{return (Queue_GetIsEmpty(p_queue) == false) ? GetPtrFront(p_queue) : 0U;}
void * Queue_PeekPtrBack(Queue_T * p_queue)					{return (Queue_GetIsEmpty(p_queue) == false) ? GetPtrBack(p_queue) : 0U;}
void * Queue_PeekPtrIndex(Queue_T * p_queue, size_t index)	{return (index < Queue_GetFullCount(p_queue)) ? GetPtrIndex(p_queue, index) : 0U;}
void * Queue_SeekPtr(Queue_T * p_queue, size_t index) 		{return (index < Queue_GetFullCount(p_queue)) ? SeekPtr(p_queue, index) : 0U;}
