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
	@file 	Ring.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Ring.h"

#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
#include "System/Critical/Critical.h"
#endif

#include <string.h>

/******************************************************************************/
/*!
	Private
*/
/******************************************************************************/

/******************************************************************************/
/*!
	Optionally implement Critical at Buffer isntace level.
*/
/******************************************************************************/
static inline void EnterCritical(void)
{
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	if(p_ring.CONFIG.USE_CRITICAL == true) { Critical_Enter(); }
#endif
}

static inline void ExitCritical(void)
{
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	if(p_ring.CONFIG.USE_CRITICAL == true) { Critical_Exit(); }
#endif
}

static inline bool AcquireMutex(Ring_T * p_ring)
{
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	return (p_ring.CONFIG.USE_CRITICAL == true) ? Critical_AcquireMutex(&p_ring->Mutex) : true;
#else
	(void)p_ring;
	return true;
#endif
}

static inline void ReleaseMutex(Ring_T * p_ring)
{
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	if(p_ring.CONFIG.USE_CRITICAL == true) { Critical_ReleaseMutex(&p_ring->Mutex) };
#else
	(void)p_ring;
#endif
}

static inline bool AcquireCritical(Ring_T * p_ring)
{
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	return (p_ring.CONFIG.USE_CRITICAL == true) ? Critical_AcquireEnter(&p_ring->Mutex) : true;
#else
	(void)p_ring;
	return true;
#endif
}

static inline void ReleaseCritical(Ring_T * p_ring)
{
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	if(p_ring.CONFIG.USE_CRITICAL == true) { Critical_ReleaseExit(&p_ring->Mutex) };
#else
	(void)p_ring;
#endif
}
/******************************************************************************/
/*!
*/
/******************************************************************************/

static inline size_t CalcIndexWrap(Ring_T * p_ring, size_t index)
{
#if defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)
	return _Ring_CalcIndexMasked(p_ring, index);
#else
	(void)p_ring;
	return index;
#endif
}

static inline size_t CalcIndexOffset(Ring_T * p_ring, size_t index)
{
	return CalcIndexWrap(p_ring, index) * p_ring->CONFIG.UNIT_SIZE;
}

static inline void * GetPtrFront(Ring_T * p_ring) 								{ return ((uint8_t *)p_ring->CONFIG.P_BUFFER + CalcIndexOffset(p_ring, p_ring->Tail)); }
static inline void * GetPtrBack(Ring_T * p_ring) 								{ return ((uint8_t *)p_ring->CONFIG.P_BUFFER + CalcIndexOffset(p_ring, p_ring->Head)); }
static inline void * GetPtrIndex(Ring_T * p_ring, size_t index) 				{ return ((uint8_t *)p_ring->CONFIG.P_BUFFER + CalcIndexOffset(p_ring, p_ring->Tail + index)); }
//todo compare switch(size) functions in addition to memcpy
static inline void PeekFront(Ring_T * p_ring, void * p_dest) 					{ memcpy(p_dest, GetPtrFront(p_ring), p_ring->CONFIG.UNIT_SIZE); }
static inline void PeekBack(Ring_T * p_ring, void * p_dest) 					{ memcpy(p_dest, GetPtrBack(p_ring), p_ring->CONFIG.UNIT_SIZE); }
static inline void PeekIndex(Ring_T * p_ring, void * p_dest, size_t index) 		{ memcpy(p_dest, GetPtrIndex(p_ring, index), p_ring->CONFIG.UNIT_SIZE); }
static inline void PlaceFront(Ring_T * p_ring, const void * p_unit) 			{ memcpy(GetPtrFront(p_ring), p_unit, p_ring->CONFIG.UNIT_SIZE); }
static inline void PlaceBack(Ring_T * p_ring, const void * p_unit) 				{ memcpy(GetPtrBack(p_ring), p_unit, p_ring->CONFIG.UNIT_SIZE); }
static inline void RemoveFront(Ring_T * p_ring, size_t nUnits) 					{ p_ring->Tail = _Ring_CalcIndexInc(p_ring, p_ring->Tail, nUnits); }
static inline void RemoveBack(Ring_T * p_ring, size_t nUnits) 					{ p_ring->Head = _Ring_CalcIndexDec(p_ring, p_ring->Head, nUnits); }
static inline void AddFront(Ring_T * p_ring, size_t nUnits) 					{ p_ring->Tail = _Ring_CalcIndexDec(p_ring, p_ring->Tail, nUnits); }
static inline void AddBack(Ring_T * p_ring, size_t nUnits) 						{ p_ring->Head = _Ring_CalcIndexInc(p_ring, p_ring->Head, nUnits); }
static inline void Enqueue(Ring_T * p_ring, const void * p_unit) 				{ PlaceBack(p_ring, p_unit); AddBack(p_ring, 1U); }
static inline void Dequeue(Ring_T * p_ring, void * p_dest) 						{ PeekFront(p_ring, p_dest); RemoveFront(p_ring, 1U); }
static inline void PushFront(Ring_T * p_ring, const void * p_unit) 				{ AddFront(p_ring, 1U); PlaceFront(p_ring, p_unit); }
static inline void PopBack(Ring_T * p_ring, void * p_dest) 						{ RemoveBack(p_ring, 1U); PeekBack(p_ring, p_dest); }
static inline void * SeekPtr(Ring_T * p_ring, size_t index) 					{ RemoveFront(p_ring, index); return GetPtrFront(p_ring); }
static inline void Seek(Ring_T * p_ring, void * p_dest, size_t index) 			{ RemoveFront(p_ring, index); PeekFront(p_ring, p_dest); }

/******************************************************************************/
/*!
	Public
*/
/******************************************************************************/
void Ring_Init(Ring_T * p_ring)
{
	Ring_Clear(p_ring);
}

#ifdef CONFIG_RING_DYNAMIC_MEMORY_ALLOCATION
Ring_T * Ring_New(size_t unitCount, size_t unitSize)
{
	Ring_T * p_ring = malloc(sizeof(Ring_T));
	void * p_buffer = malloc(unitCount * unitSize);

	if(p_ring != 0U && p_buffer != 0U)
	{
		*(void *)&p_ring->CONFIG.P_BUFFER = p_buffer;
		*(size_t)&p_ring->CONFIG.LENGTH = unitCount;
		*(size_t)&p_ring->CONFIG.UNIT_SIZE = unitSize;

		Ring_Init(p_ring);
	}

	return p_ring;
}
#endif

void Ring_Clear(Ring_T * p_ring)
{
	p_ring->Head = 0U;
	p_ring->Tail = 0U;
}

bool Ring_Enqueue(Ring_T * p_ring, const void * p_unit)
{
	bool isSuccess = false;
	EnterCritical();
	if(Ring_GetIsFull(p_ring) == false) { Enqueue(p_ring, p_unit); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_Dequeue(Ring_T * p_ring, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Ring_GetIsEmpty(p_ring) == false) {	Dequeue(p_ring, p_dest); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_EnqueueN(Ring_T * p_ring, const void * p_units, size_t nUnits)
{
	bool isSuccess = false;
	EnterCritical(); //if(AcquireCritical(p_ring) == true) {
	if(nUnits <= Ring_GetEmptyCount(p_ring))
	{
		for(size_t iUnit = 0U; iUnit < nUnits; iUnit++) { Enqueue(p_ring, p_units + (iUnit * p_ring->CONFIG.UNIT_SIZE)); }
		isSuccess = true;
	}
	ExitCritical();
	// ReleaseCritical(p_ring); }
	return isSuccess;
}

bool Ring_DequeueN(Ring_T * p_ring, void * p_dest, size_t nUnits)
{
	bool isSuccess = false;
	EnterCritical();
	if(nUnits <= Ring_GetFullCount(p_ring))
	{
		for(size_t iUnit = 0U; iUnit < nUnits; iUnit++) { Dequeue(p_ring, p_dest + (iUnit * p_ring->CONFIG.UNIT_SIZE)); }
		isSuccess = true;
	}
	ExitCritical();
	return isSuccess;
}

size_t Ring_EnqueueMax(Ring_T * p_ring, const void * p_units, size_t nUnits)
{
	// size_t count;
	// EnterCritical();
	// count = Ring_GetEmptyCount(p_ring);
	// if(count > nUnits) { count = nUnits; }
	// for(size_t iCount = 0U; iCount < count; iCount++) { Enqueue(p_ring, p_units + (iCount * p_ring->CONFIG.UNIT_SIZE)); }
	// ExitCritical();
	// return count;

	size_t count;
	EnterCritical();
	for(count = 0; count < nUnits; count++)
	{
		if(Ring_GetIsFull(p_ring) == false)
		{
			Enqueue(p_ring, p_units + (count * p_ring->CONFIG.UNIT_SIZE));
		}
		else
		{
			break;
		}
	}
	ExitCritical();
	return count;
}

size_t Ring_DequeueMax(Ring_T * p_ring, void * p_dest, size_t nUnits)
{
	// size_t count;
	// EnterCritical();
	// count = Ring_GetFullCount(p_ring);
	// if(count > nUnits) { count = nUnits; }
	// for(size_t iCount = 0U; iCount < count; iCount++) { Dequeue(p_ring, p_units + (iCount * p_ring->CONFIG.UNIT_SIZE)); }
	// ExitCritical();
	// return count;

	size_t count;
	EnterCritical();
	for(count = 0; count < nUnits; count++)
	{
		if(Ring_GetIsEmpty(p_ring) == false)
		{
			Dequeue(p_ring, p_dest + (count * p_ring->CONFIG.UNIT_SIZE));
		}
		else
		{
			break;
		}
	}
	ExitCritical();
	return count;
}

bool Ring_PushFront(Ring_T * p_ring, const void * p_unit)
{
	bool isSuccess = false;
	EnterCritical();
	if(Ring_GetIsFull(p_ring) == false) { PushFront(p_ring, p_unit); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_PopBack(Ring_T * p_ring, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Ring_GetIsEmpty(p_ring) == false) { PopBack(p_ring, p_dest); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_PeekFront(Ring_T * p_ring, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Ring_GetIsEmpty(p_ring) == false) { PeekFront(p_ring, p_dest); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_PeekBack(Ring_T * p_ring, void * p_dest)
{
	bool isSuccess = false;
	EnterCritical();
	if(Ring_GetIsEmpty(p_ring) == false) { PeekBack(p_ring, p_dest); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_PeekIndex(Ring_T * p_ring, void * p_dest, size_t index)
{
	bool isSuccess = false;
	EnterCritical();
	if(index < Ring_GetFullCount(p_ring)) { PeekIndex(p_ring, p_dest, index); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_RemoveFront(Ring_T * p_ring, size_t nUnits)
{
	bool isSuccess = false;
	EnterCritical();
	if(nUnits <= Ring_GetFullCount(p_ring)) { RemoveFront(p_ring, nUnits); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_RemoveBack(Ring_T * p_ring, size_t nUnits)
{
	bool isSuccess = false;
	EnterCritical();
	if(nUnits <= Ring_GetFullCount(p_ring)) { RemoveBack(p_ring, nUnits); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

bool Ring_Seek(Ring_T * p_ring, void * p_dest, size_t index)
{
	bool isSuccess = false;
	EnterCritical();
	if(index < Ring_GetFullCount(p_ring)) { Seek(p_ring, p_dest, index); isSuccess = true; }
	ExitCritical();
	return isSuccess;
}

/*
	single threaded use only, buffer may be overwritten, alternatively use mutex
	result in return value, pointer 0 indicate invalid
*/
void * Ring_PeekPtrFront(Ring_T * p_ring) 					{ return (Ring_GetIsEmpty(p_ring) == false) ? GetPtrFront(p_ring) : 0U; }
void * Ring_PeekPtrBack(Ring_T * p_ring) 					{ return (Ring_GetIsEmpty(p_ring) == false) ? GetPtrBack(p_ring) : 0U; }
void * Ring_PeekPtrIndex(Ring_T * p_ring, size_t index) 	{ return (index < Ring_GetFullCount(p_ring)) ? GetPtrIndex(p_ring, index) : 0U; }
void * Ring_SeekPtr(Ring_T * p_ring, size_t index) 			{ return (index < Ring_GetFullCount(p_ring)) ? SeekPtr(p_ring, index) : 0U; }

/*
	Give caller exclusive buffer write. Will flush buffer to start from index 0
*/
uint8_t * Ring_AcquireBuffer(Ring_T * p_ring)
{
	uint8_t * p_buffer = 0U;

	if(AcquireMutex(p_ring) == true)
	{
		Ring_Clear(p_ring);
		p_buffer = p_ring->CONFIG.P_BUFFER;
	}

	return p_buffer;
}

void Ring_ReleaseBuffer(Ring_T * p_ring, size_t writeSize)
{
	p_ring->Head = writeSize;
	ReleaseMutex(p_ring);
}