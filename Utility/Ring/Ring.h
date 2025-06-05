/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Ring.h
    @author FireSourcery
    @brief  Ring Buffer. Optimized implementation using POW2_MASK over modulus

*/
/******************************************************************************/
#ifndef RING_UTILITY_H
#define RING_UTILITY_H

#include "Config.h"

#include "Type/Array/void_array.h"

#if defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
#include "System/Critical/Critical.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>


/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef const struct Ring_Type
{
    size_t UNIT_SIZE;     /* Bytes */
    size_t LENGTH;        /* In UNIT_SIZE counts (NOT bytes) */
#if defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
    size_t POW2_MASK;     /* Index Mask */
#endif
}
Ring_Type_T;

#if     defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
#define _RING_INIT_POW2(Pow2Mask) .POW2_MASK = Pow2Mask,
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
#define _RING_INIT_POW2(Pow2Mask)
#endif

#define _RING_TYPE_INIT(UnitSize, Length) { .UNIT_SIZE = UnitSize, .LENGTH = Length, _RING_INIT_POW2(Length - 1U) }

/*
    Run time contigous implementation.
*/
typedef
// __attribute__((aligned(sizeof(intptr_t)))) /* align to pointer size */
struct Ring
{
    Ring_Type_T Type;
    volatile uint32_t Head;    /* FIFO Out/Front. */
    volatile uint32_t Tail;    /* FIFO In/Back. */
#if defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    volatile critical_lock_t Lock;
#endif
    uint8_t Buffer[]; /* MISRA violation. Rationale: Compile-time allocated. */
}
Ring_T;

/*
    For Generic Handling or Runtime Init
    In the case that Runtime state stores [Ring_Type_T Type], it must be copied at run time.
*/
typedef const struct Ring_Context
{
    const Ring_Type_T TYPE;
    Ring_T * const P_STATE;
}
Ring_Context_T;

#define IS_POW2(x) (((x) & ((x) - 1U)) == 0U)
/* alternatively use int/sizeof(int) in case of ascii fill */
#define _RING_BUFFER_ALLOC(Bytes) ((uint8_t[(Bytes)]){})
// static_assert(IS_POW2(Length), "Ring length must be power of 2");

#define RING_STATE_ALLOC(UnitSize, Length) ((Ring_T *)(_RING_BUFFER_ALLOC(sizeof(Ring_T) + (UnitSize)*(Length))))
#define RING_CONTEXT_INIT(UnitSize, Length, p_State) { .TYPE = _RING_TYPE_INIT(UnitSize, Length), .P_STATE = p_State, }
#define RING_CONTEXT_ALLOC(UnitSize, Length) RING_CONTEXT_INIT(UnitSize, Length, RING_STATE_ALLOC(UnitSize, Length))


/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
static inline size_t _Ring_Length(const Ring_T * p_ring) { return p_ring->Type.LENGTH; }

static inline size_t _Ring_Mask(const Ring_T * p_ring)
{
#if defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
    return p_ring->Type.POW2_MASK;
#else
    (void)p_ring;
    return 0U;  /* Not used in LENGTH_COMPARE mode */
#endif
}

static inline size_t _Ring_IndexMaskedOf(const Ring_T * p_ring, size_t index)
{
#if defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
    return (index & p_ring->Type.POW2_MASK);
#endif
}

static inline size_t _Ring_IndexWrapOf(const Ring_T * p_ring, size_t index)
{
#if     defined(CONFIG_RING_POW2_COUNTER) || defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexMaskedOf(p_ring, index);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return (index % p_ring->Type.LENGTH);
#endif
}

/* on storage */
static inline size_t _Ring_IndexIncOf(const Ring_T * p_ring, size_t index, size_t inc)
{
#if     defined(CONFIG_RING_POW2_COUNTER)
    (void)p_ring;
    return index + inc;
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexMaskedOf(p_ring, index + inc);
#elif defined(CONFIG_RING_LENGTH_COMPARE)
    return (index + inc >= p_ring->Type.LENGTH) ? index + inc - p_ring->Type.LENGTH : index + inc;
#endif
}

static inline size_t _Ring_IndexDecOf(const Ring_T * p_ring, size_t index, size_t dec)
{
#if     defined(CONFIG_RING_POW2_COUNTER)
    (void)p_ring;
    return index - dec;
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexMaskedOf(p_ring, index - dec);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return ((int32_t)index - dec < 0) ? p_ring->Type.LENGTH + index - dec : index - dec;
#endif
}

static inline size_t _Ring_ArrayIndexOf(const Ring_T * p_ring, size_t ringIndex)
{
#if     defined(CONFIG_RING_POW2_COUNTER)
    return _Ring_IndexMaskedOf(p_ring, ringIndex);
#elif   defined(CONFIG_RING_POW2_WRAP) || defined(CONFIG_RING_LENGTH_COMPARE)
    return ringIndex;
#endif
}


/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
/*
    CONFIG_RING_POW2_COUNTER
    Max usable capacity is length

    CONFIG_RING_POW2_WRAP, CONFIG_RING_LENGTH_COMPARE
    Empty space detection method. Tail always points to empty space. Max usable capacity is length - 1
*/
static inline size_t Ring_GetFullCount(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_POW2_COUNTER)
    return (p_ring->Tail - p_ring->Head);
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, p_ring->Tail - p_ring->Head);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return (p_ring->Tail >= p_ring->Head) ? (p_ring->Tail - p_ring->Head) : (p_ring->Type.LENGTH - p_ring->Head + p_ring->Tail);
#endif
}

/*
    later 2 cases: -1 to account for 1 space used for detection
*/
static inline size_t Ring_GetEmptyCount(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_POW2_COUNTER)
    return p_ring->Type.LENGTH + p_ring->Head - p_ring->Tail;
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, p_ring->Type.LENGTH + p_ring->Head - p_ring->Tail - 1U);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return (p_ring->Tail >= p_ring->Head) ? (p_ring->Type.LENGTH + p_ring->Head - p_ring->Tail - 1U) : (p_ring->Head - p_ring->Tail - 1U);
#endif
}

static inline bool Ring_IsEmpty(const Ring_T * p_ring)
{
    return (p_ring->Tail == p_ring->Head);
}

static inline bool Ring_IsFull(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_POW2_COUNTER)
    return (Ring_GetFullCount(p_ring) == p_ring->Type.LENGTH);
#elif   defined(CONFIG_RING_POW2_WRAP) || defined(CONFIG_RING_LENGTH_COMPARE)
    return (_Ring_IndexIncOf(p_ring, p_ring->Tail, 1U) == p_ring->Head);
#endif
}



/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
/*

*/
extern void * Ring_Front(const Ring_T * p_ring);
extern void * Ring_Back(const Ring_T * p_ring);
extern void * Ring_At(const Ring_T * p_ring, size_t index);

extern void _Ring_PushBack(Ring_T * p_ring, const void * p_unit);
extern void _Ring_PushFront(Ring_T * p_ring, const void * p_unit);
extern void * _Ring_PopFront(Ring_T * p_ring);
extern void * _Ring_PopBack(Ring_T * p_ring);
extern void * _Ring_Seek(Ring_T * p_ring, size_t index);

/*

*/
extern void Ring_Init(const Ring_Context_T * p_ring);
extern void Ring_InitFrom(Ring_T * p_ring, const Ring_Type_T * p_type);
extern void Ring_Clear(Ring_T * p_ring);
extern bool Ring_PushFront(Ring_T * p_ring, const void * p_unit);
extern bool Ring_PopFront(Ring_T * p_ring, void * p_result);
extern bool Ring_PushBack(Ring_T * p_ring, const void * p_unit);
extern bool Ring_PopBack(Ring_T * p_ring, void * p_result);

extern bool Ring_PeekFront(Ring_T * p_ring, void * p_result);
extern bool Ring_PeekBack(Ring_T * p_ring, void * p_result);
extern bool Ring_PeekAt(Ring_T * p_ring, size_t index, void * p_result);

extern bool Ring_RemoveFront(Ring_T * p_ring, size_t unitCount);
extern bool Ring_RemoveBack(Ring_T * p_ring, size_t unitCount);
extern bool Ring_Enqueue(Ring_T * p_ring, const void * p_unit);
extern bool Ring_Dequeue(Ring_T * p_ring, void * p_result);

extern bool Ring_EnqueueN(Ring_T * p_ring, const void * p_units, size_t unitCount);
extern size_t Ring_EnqueueMax(Ring_T * p_ring, const void * p_units, size_t unitCount);

extern bool Ring_DequeueN(Ring_T * p_ring, void * p_result, size_t unitCount);
extern size_t Ring_DequeueMax(Ring_T * p_ring, void * p_result, size_t unitCount);



#endif



