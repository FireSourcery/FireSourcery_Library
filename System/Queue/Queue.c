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
#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	Critical_Enter();
#endif
}

static inline void ExitCritical(void)
{
#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	Critical_Exit();
#endif
}

static inline bool AquireMutex(Queue_T * p_queue)
{
#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	return Critical_AquireMutex(&p_queue->Mutex);
#else
	return true;
#endif
}

static inline void ReleaseMutex(Queue_T * p_queue)
{
#if defined(CONFIG_QUEUE_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_QUEUE_MULTITHREADED_USER_DEFINED)
	Critical_ReleaseMutex(&p_queue->Mutex);
#endif
}

static inline size_t WrapArrayIndex(Queue_T * p_queue, size_t index)
{
#if defined(CONFIG_QUEUE_LENGTH_POW2_INDEX_UNBOUNDED)
	return CalcQueueIndexMasked(p_queue, index);
#else
	return index;
#endif
}

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

	if(p_queue != 0U && p_buffer != 0U)
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
	//	queue->Count 		= 0;
	p_queue->Head = 0U;
	p_queue->Tail = 0U;
}

bool Queue_Enqueue(Queue_T * p_queue, const void * p_unit)
{
	bool isSucess = false;

	EnterCritical();

	if (Queue_GetIsFull(p_queue) == false) //ideally compiler stores value from inline function
	{
		memcpy(&p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Head)], p_unit, p_queue->CONFIG.UNIT_SIZE);
		p_queue->Head = CalcQueueIndexInc(p_queue, p_queue->Head, 1U);
		isSucess = true;
	}

	ExitCritical();

	return isSucess;
}

bool Queue_Dequeue(Queue_T * p_queue, void * p_dest)
{
	bool isSucess = false;

	EnterCritical();

	if (Queue_GetIsEmpty(p_queue) == false)
	{
		memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Tail)], p_queue->CONFIG.UNIT_SIZE);
		p_queue->Tail = CalcQueueIndexInc(p_queue, p_queue->Tail, 1U);
		isSucess = true;
	}

	ExitCritical();

	return isSucess;
}

bool Queue_EnqueueN(Queue_T * p_queue, const void * p_units, size_t nUnits)
{
	bool isSucess = false;

	if(AquireMutex(p_queue))
	{
		if (nUnits < Queue_GetEmptyCount(p_queue))
		{
			memcpy(&p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Head)], p_units, p_queue->CONFIG.UNIT_SIZE * nUnits);
			p_queue->Head = CalcQueueIndexInc(p_queue, p_queue->Head, nUnits);
			isSucess = true;
		}

		ReleaseMutex(p_queue);
	}

	return isSucess;
}

bool Queue_DequeueN(Queue_T * p_queue, void * p_dest, size_t nUnits)
{
	bool isSucess = false;

	if(AquireMutex(p_queue))
	{
		if (nUnits < Queue_GetEmptyCount(p_queue))
		{
			memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Tail)], p_queue->CONFIG.UNIT_SIZE * nUnits);
			p_queue->Tail = CalcQueueIndexInc(p_queue, p_queue->Tail, nUnits);
			isSucess = true;
		}

		ReleaseMutex(p_queue);
	}

	return isSucess;
}

size_t Queue_EnqueueMax(Queue_T * p_queue, const void * p_units, size_t nUnits)
{
	size_t count;

	if(AquireMutex(p_queue))
	{
		for (count = 0; count < nUnits; count++)
		{
			if (Queue_GetIsFull(p_queue) == false)
			{
				break;
			}
			else
			{
				memcpy(&p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Head)], p_units, p_queue->CONFIG.UNIT_SIZE);
				p_queue->Head = CalcQueueIndexInc(p_queue, p_queue->Head, 1U);
			}
		}

		ReleaseMutex(p_queue);
	}

	return count;
}

size_t Queue_DequeueMax(Queue_T * p_queue, const void * p_dest, size_t p_destLength)
{
	size_t count;

	if(AquireMutex(p_queue))
	{
		for (count = 0; count < p_destLength; count++)
		{
			if (Queue_GetIsEmpty(p_queue) == false)
			{
				break;
			}
			else
			{
				memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Tail)], p_queue->CONFIG.UNIT_SIZE);
				p_queue->Tail = CalcQueueIndexInc(p_queue, p_queue->Tail, 1U);
			}
		}

		ReleaseMutex(p_queue);
	}

	return count;
}

bool Queue_PushFront(Queue_T * p_queue, const void * p_unit)
{
	bool isSucess = false;

	EnterCritical();

	if (Queue_GetIsFull(p_queue) == false)
	{
		p_queue->Tail = CalcQueueIndexDec(p_queue, p_queue->Tail, 1U);
		memcpy(&p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Tail)], p_unit, p_queue->CONFIG.UNIT_SIZE);
		isSucess = true;
	}

	ExitCritical();

	return isSucess;
}

bool Queue_PopBack(Queue_T * p_queue, void * p_dest)
{
	bool isSucess = false;

	EnterCritical();

	if (Queue_GetIsFull(p_queue) == false)
	{
		p_queue->Head = CalcQueueIndexDec(p_queue, p_queue->Head, 1U);
		memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Head)], p_queue->CONFIG.UNIT_SIZE);
		isSucess = true;
	}

	ExitCritical();

	return isSucess;
}

bool Queue_PeekFront(Queue_T * p_queue, void * p_dest)
{
	bool isSucess = false;

	if (Queue_GetIsEmpty(p_queue) == false)
	{
		memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Tail)], p_queue->CONFIG.UNIT_SIZE);
		isSucess = true;
	}

	return isSucess;
}

bool Queue_PeekIndex(Queue_T * p_queue, void * p_dest, size_t index)
{
	bool isSucess = false;

	if (index < Queue_GetEmptyCount(p_queue))
	{
		memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, index)], p_queue->CONFIG.UNIT_SIZE);
		isSucess = true;
	}

	return isSucess;
}

bool Queue_PeekBack(Queue_T * p_queue, void * p_dest)
{
	bool isSucess = false;

	if (Queue_GetIsEmpty(p_queue) == false)
	{
		memcpy(p_dest, &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Head)], p_queue->CONFIG.UNIT_SIZE);
		isSucess = true;
	}

	return isSucess;
}

bool Queue_Remove(Queue_T * p_queue, size_t nUnits)
{
	bool isSucess = false;

	EnterCritical();

	if (nUnits < Queue_GetEmptyCount(p_queue))
	{
		p_queue->Tail = CalcQueueIndexInc(p_queue, p_queue->Tail, nUnits);
		isSucess = true;
	}

	ExitCritical();

	return isSucess;
}

bool Queue_Seek(Queue_T * p_queue, size_t nUnits)
{
	return Queue_Remove(p_queue, nUnits);
}

/*
	single threaded use only, buffer may be overwritten
	result in return value, pointer cannot be 0
 */
void * Queue_PeekPtrFront(Queue_T * p_queue)
{
	void * p_peek = 0U;

	if (Queue_GetIsEmpty(p_queue) == false)
	{
		p_peek = &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, p_queue->Tail)];
	}

	return p_peek;
}

void * Queue_PeekPtrIndex(Queue_T * p_queue, size_t index)
{
	void * p_peek = 0U;

	if (index < Queue_GetFullCount(p_queue))
	{
		p_peek = &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue, index)];
	}

	return p_peek;
}

void * Queue_DequeuePtr(Queue_T * p_queue)
{
	void * p_unit;

	EnterCritical();

	if (Queue_GetIsEmpty(p_queue) == false)
	{
		p_unit = &p_queue->CONFIG.P_BUFFER[WrapArrayIndex(p_queue,  p_queue->Tail)];
		p_queue->Tail = CalcQueueIndexInc(p_queue, p_queue->Tail, 1U);
	}

	ExitCritical();

	return p_unit;
}
