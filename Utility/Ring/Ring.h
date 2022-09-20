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
	@file 	Ring.h
	@author FireSoucery
	@brief	Ring Buffer. Improved efficiency using POW2_MASK over modulus
	@version V0
*/
/******************************************************************************/
#ifndef RING_UTILITY_H
#define RING_UTILITY_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef const struct Ring_Config_Tag
{
	void * const P_BUFFER;
	const size_t UNIT_SIZE;
	const size_t LENGTH; 	/* In UNIT_SIZE counts (NOT bytes) */
#if defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED) || defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)
	const uint32_t POW2_MASK;
#endif
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	const bool USE_CRITICAL;
#endif
}
Ring_Config_T;

/*
	CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED
	Max usable capacity is length

	CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED, CONFIG_RING_LENGTH_ANY
	Empty space detection method. Head always points to empty space. Max usable capacity is length - 1
*/
typedef struct Ring_Tag
{
	const Ring_Config_T CONFIG;
	volatile size_t Head;	/* Write to head  */
	volatile size_t Tail;	/* Read from tail */
#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
	volatile critical_mutext_t Mutex;
#endif
}
Ring_T;

#if defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED) || defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)
#define _RING_INIT_POW2(Pow2Mask) .POW2_MASK = Pow2Mask,
#else
#define _RING_INIT_POW2(Pow2Mask)
#endif

#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
#define _RING_INIT_CRITICAL(UseCritical) .USE_CRITICAL = UseCritical,
#else
#define _RING_INIT_CRITICAL(UseCritical)
#endif

#define RING_INIT(p_Buffer, Length, UnitSize, UseCritical) 		\
{																\
	.CONFIG =													\
	{                                               			\
		.P_BUFFER 		= p_Buffer,								\
		.LENGTH 		= Length,								\
		.UNIT_SIZE 		= UnitSize,								\
		_RING_INIT_POW2(Length - 1U)							\
		_RING_INIT_CRITICAL(UseCritical)						\
	},															\
}

// #define RING_INIT(p_Buffer, Length, UnitSize)
// #define RING_INIT_MULTITHREADED(p_Buffer, Length, UnitSize, UseCritical)

static inline size_t _Ring_CalcIndexMasked(const Ring_T * p_ring, size_t index)
{
#if defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED) || defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED)
	return (index & p_ring->CONFIG.POW2_MASK);
#else
	return (index % p_ring->CONFIG.LENGTH);
#endif
}

static inline size_t _Ring_CalcIndexInc(const Ring_T * p_ring, size_t index, size_t inc)
{
#if defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)
	(void)p_ring;
	return index + inc;
#else
	return _Ring_CalcIndexMasked(p_ring, index + inc);
#endif
}

static inline size_t _Ring_CalcIndexDec(const Ring_T * p_ring, size_t index, size_t dec)
{
#if defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)
	(void)p_ring;
	return index - dec;
#elif defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED)
	return _Ring_CalcIndexMasked(p_ring, index - dec);
#else
	return _Ring_CalcIndexMasked(p_ring, p_ring->CONFIG.LENGTH + index - dec);
	//(index < 1U) ? index = p_unit, p_ring->CONFIG.LENGTH - 1U : index--;
#endif
}

/*
	later 2 cases: returns max of buffer length - 1, to account for 1 space used for detection
*/
static inline size_t Ring_GetFullCount(const Ring_T * p_ring)
{
#ifdef CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED
	return (p_ring->Head - p_ring->Tail);
#elif defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED)
	return _Ring_CalcIndexMasked(p_ring, p_ring->Head - p_ring->Tail);
#else
	return _Ring_CalcIndexMasked(p_ring, p_ring->CONFIG.LENGTH + p_ring->Head - p_ring->Tail);
#endif
}

/*
	later 2 cases: -1 to account for space used for detection
*/
static inline size_t Ring_GetEmptyCount(const Ring_T * p_ring)
{
#ifdef CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED
	return p_ring->CONFIG.LENGTH + p_ring->Tail - p_ring->Head;
#elif defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED)
	return _Ring_CalcIndexMasked(p_ring, p_ring->CONFIG.LENGTH + p_ring->Tail - p_ring->Head - 1U);
#else
	return _Ring_CalcIndexMasked(p_ring, p_ring->CONFIG.LENGTH + p_ring->Tail - p_ring->Head - 1U);
	//	return (p_ring->Tail > p_ring->Head) ? (p_ring->Tail - p_ring->Head - 1U) : (p_ring->CONFIG.LENGTH - p_ring->Head + p_ring->Tail - 1U);
#endif
}

static inline bool Ring_GetIsEmpty(const Ring_T * p_ring)
{
	return (p_ring->Head == p_ring->Tail);
}

static inline bool Ring_GetIsFull(const Ring_T * p_ring)
{
#ifdef CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED
	return (Ring_GetFullCount(p_ring) == p_ring->CONFIG.LENGTH);
#else
	return (_Ring_CalcIndexInc(p_ring, p_ring->Head, 1U) == p_ring->Tail);
#endif
}

extern void Ring_Init(Ring_T * p_ring);
#ifdef CONFIG_RING_DYNAMIC_MEMORY_ALLOCATION
extern Ring_T * Ring_New(size_t unitCount, size_t unitSize);
#endif
extern void Ring_Clear(Ring_T * p_ring);
extern bool Ring_Enqueue(Ring_T * p_ring, const void * p_unit);
extern bool Ring_Dequeue(Ring_T * p_ring, void * p_dest);
extern bool Ring_EnqueueN(Ring_T * p_ring, const void * p_unit, size_t nUnits);
extern bool Ring_DequeueN(Ring_T * p_ring, void * p_dest, size_t nUnits);
extern size_t Ring_EnqueueMax(Ring_T * p_ring, const void * p_units, size_t nUnits);
extern size_t Ring_DequeueMax(Ring_T * p_ring, void * p_dest, size_t nUnits);
extern bool Ring_PushFront(Ring_T * p_ring, const void * p_unit);
extern bool Ring_PopBack(Ring_T * p_ring, void * p_dest);
extern bool Ring_PeekFront(Ring_T * p_ring, void * p_dest);
extern bool Ring_PeekBack(Ring_T * p_ring, void * p_dest);
extern bool Ring_PeekIndex(Ring_T * p_ring, void * p_dest, size_t index);
extern bool Ring_RemoveFront(Ring_T * p_ring, size_t nUnits);
extern bool Ring_RemoveBack(Ring_T * p_ring, size_t nUnits);
extern bool Ring_Seek(Ring_T * p_ring, void * p_dest, size_t index);
extern void * Ring_PeekPtrFront(Ring_T * p_ring);
extern void * Ring_PeekPtrBack(Ring_T * p_ring);
extern void * Ring_PeekPtrIndex(Ring_T * p_ring, size_t index);
extern void * Ring_SeekPtr(Ring_T * p_ring, size_t index);
extern uint8_t * Ring_AcquireBuffer(Ring_T * p_ring);
extern void Ring_ReleaseBuffer(Ring_T * p_ring, size_t writeSize);

#endif
