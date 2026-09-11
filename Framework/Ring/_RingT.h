#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   _Ring.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Type/Array/void_array.h"
#include "Type/Array/Array.h"

#if defined(RING_LOCAL_CRITICAL_ENABLE)
#include "System/Critical/Critical.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

/******************************************************************************/
/*!
    Generic Compile-Time Optimized Ring Buffer Implementation
    All functions are templated on Ring_Type_T for maximum performance

    This becomes like a C++ template - each Ring_Type_T generates optimized code
    const Ring_Type_T UINT32_RING_TYPE = _RING_TYPE_INIT(sizeof(uint32_t), 256);

    Compiler generates specialized function for this exact type
    _RingT_PushBack(UINT32_RING_TYPE, &myRing, &value);
    Becomes equivalent to:
    memcpy(ring->Buffer + (4 * ring->Tail), &value, 4);
    ring->Tail = (ring->Tail + 1) & 255;
 */
/******************************************************************************/

/******************************************************************************/
/* Level 1 — strided array access (reusable anywhere) */
/******************************************************************************/
/* Strided element access is void_array_at / void_array_assign_at (Type/Array/void_array.h) */

/* Ring index -> array index. One mapping per index representation. */
static inline size_t array_index_of_counter(size_t pow2len, size_t ringIndex) { return (ringIndex & (pow2len - 1U)); }
static inline size_t array_index_of_wrap(size_t length, size_t ringIndex) { return (ringIndex >= length) ? ringIndex - length : ringIndex; }
static inline size_t array_index_of_diff(size_t length, ptrdiff_t diff) { return (size_t)((diff < 0) ? diff + (ptrdiff_t)length : ((diff >= (ptrdiff_t)length) ? diff - (ptrdiff_t)length : diff)); }

/*
    RING_INDEX_POW2_COUNTER is the only representation implemented here.
    Cursors are free-running counters, never wrapped on storage, masked on access.
    Capacity is the full LENGTH. FullCount is the plain difference Tail - Head.
*/
static inline size_t ring_index_on_access(size_t pow2len, size_t ringIndex) { return array_index_of_counter(pow2len, ringIndex); }
static inline size_t ring_index_on_storage(size_t length, size_t ringIndex) { (void)length; return ringIndex; }

/* paired access */
/* size_t stride, size_t pow2len are fixed */
/* pow2 length only */
static inline void * ring_at(size_t stride, const void * p_array, size_t pow2len, size_t ringIndex) { return void_array_at(stride, p_array, ring_index_on_access(pow2len, ringIndex)); }
static inline void ring_assign_at(size_t stride, void * p_array, size_t pow2len, size_t ringIndex, const void * p_unit) { void_array_assign_at(stride, p_array, ring_index_on_access(pow2len, ringIndex), p_unit); }

static inline void * ring_buffer_at(size_t stride, const void * p_buffer, size_t bytes, size_t ringIndex) { return ring_at(stride, p_buffer, bytes / stride, ringIndex); }
static inline void ring_buffer_assign_at(size_t stride, void * p_buffer, size_t bytes, size_t ringIndex, const void * p_unit) { ring_assign_at(stride, p_buffer, bytes / stride, ringIndex, p_unit); }


/******************************************************************************/
/* Level 2 — ring state (cursors + optional lock) */
/* — span (buffer + capacity) */
/* — ring operations (stride isolated, span by value, state by pointer) */
/******************************************************************************/

/* Flyweight shape descriptor. LENGTH is in TYPE_SIZE counts (NOT bytes). Always declared const. */
typedef const struct Ring_Type { size_t TYPE_SIZE; size_t LENGTH; } Ring_Type_T;

/* 0 is excluded: it is not a power of 2 here, and (0 - 1U) would mask to SIZE_MAX */
#define RING_IS_POW2(x) (((x) & ((x) - 1U)) == 0U)
#define RING_IS_ALIGNED(x, align) (((x) & ((align) - 1U)) == 0U)

/* Evaluates to 0. Fails the translation for a non-power-of-2 LENGTH, which array_index_of_counter would silently corrupt. */
#define _RING_ASSERT_POW2(Length) (0U * sizeof(struct { static_assert(RING_IS_POW2(Length), "Ring LENGTH must be a non-zero power of 2"); int _; }))

#define RING_TYPE_INIT(UnitSize, Length) { .TYPE_SIZE = (UnitSize), .LENGTH = (Length) + _RING_ASSERT_POW2(Length) }

#define RING_VALIDATE_PARAMS(TypeSize, Length) \
    static_assert(_RING_POW2_DEF(RING_IS_POW2(Length), true), "POW2 mode requires power-of-2 Length"); \
    static_assert(RING_IS_ALIGNED(TypeSize, sizeof(uintptr_t)) || RING_IS_ALIGNED(Length, sizeof(uintptr_t)), "Ring unit size must be aligned to uintptr_t size"); \


/*
    Cursors are free-running counters. Only their masked value indexes the buffer.
    Concurrency contract: single-producer (Tail) / single-consumer (Head) only.
    'volatile' stops the peer's cursor being cached, it does not order the buffer
    access against the cursor update. Multi-producer or multi-consumer use needs
    RING_LOCAL_CRITICAL_ENABLE and the locking wrappers in Ring.c.
*/
typedef struct __attribute__((aligned(sizeof(uintptr_t)))) Ring_State
{
    volatile uintptr_t Head;    /* FIFO Out/Front. */
    volatile uintptr_t Tail;    /* FIFO In/Back. */
#if defined(RING_LOCAL_CRITICAL_ENABLE)
    volatile critical_lock_t Lock;
#endif
    uint8_t Buffer[]; /* MISRA violation. Rationale: Compile-time allocated. */
}
Ring_State_T;

/* Round up: plain division truncates, which would short the Buffer whenever BytesSize is not a multiple of the word size */
#define _RING_BUFFER_ALLOC(BytesSize) ((uintptr_t[((BytesSize) + sizeof(uintptr_t) - 1U) / sizeof(uintptr_t)]){}) /* guarantees align and no ascii fill */
#define RING_STATE_ALLOC(UnitSize, Length) ((Ring_State_T *)(_RING_BUFFER_ALLOC(sizeof(Ring_State_T) + ((UnitSize) * (Length)))))


static inline size_t _RingT_ArrayIndexOf(Ring_Type_T type, size_t index) { return array_index_of_counter(type.LENGTH, index); }
static inline size_t _RingT_IndexOnAccess(Ring_Type_T type, size_t index) { return ring_index_on_access(type.LENGTH, index); }
static inline size_t _RingT_IndexOnStorage(Ring_Type_T type, size_t index) { return ring_index_on_storage(type.LENGTH, index); }

static inline size_t _RingT_IndexIncOf(Ring_Type_T type, size_t index, size_t inc) { return _RingT_IndexOnStorage(type, index + inc); }
static inline size_t _RingT_IndexDecOf(Ring_Type_T type, size_t index, size_t dec) { return _RingT_IndexOnStorage(type, index - dec); }

/******************************************************************************/
/*!
    Core Operations - Fully Compile-Time Optimized
*/
/******************************************************************************/
// static inline void * _RingT_ArrayAt(Ring_Type_T type, Ring_State_T * p_state, size_t index) { return ring_at(type.TYPE_SIZE, p_state->Buffer, type.LENGTH, index); }

/* generic access from length in bytes */
// static inline void * _RingT_Buffer_At(size_t cast, Ring_Type_T type, Ring_State_T * p_state, size_t index) { return ring_buffer_at(cast, p_state->Buffer, type.LENGTH * type.TYPE_SIZE, index); }
// static inline void * _RingT_Buffer_At(size_t cast, RingT_T ring, size_t index) { return ring_buffer_at(cast, ring.P_STATE->Buffer, ring.SIZE_BYTES, index); }

/* Head/Tail pointer access */
static inline void * _RingT_Head(Ring_Type_T type, const Ring_State_T * p_ring) { return ring_at(type.TYPE_SIZE, p_ring->Buffer, type.LENGTH, p_ring->Head); }
static inline void * _RingT_Tail(Ring_Type_T type, const Ring_State_T * p_ring) { return ring_at(type.TYPE_SIZE, p_ring->Buffer, type.LENGTH, p_ring->Tail); }

/* Peek operations */
static inline void _RingT_PeekHead(Ring_Type_T type, const Ring_State_T * p_ring, void * p_result) { pointer_assign(type.TYPE_SIZE, p_result, _RingT_Head(type, p_ring)); }
static inline void _RingT_PeekTail(Ring_Type_T type, const Ring_State_T * p_ring, void * p_result) { pointer_assign(type.TYPE_SIZE, p_result, _RingT_Tail(type, p_ring)); }

/* Place operations */
static inline void _RingT_PlaceHead(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit) { pointer_assign(type.TYPE_SIZE, _RingT_Head(type, p_ring), p_unit); }
static inline void _RingT_PlaceTail(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit) { pointer_assign(type.TYPE_SIZE, _RingT_Tail(type, p_ring), p_unit); }

/* Index operations */
static inline void _RingT_AddFront(Ring_Type_T type, Ring_State_T * p_ring, size_t count)       { p_ring->Head = _RingT_IndexDecOf(type, p_ring->Head, count); }
static inline void _RingT_RemoveFront(Ring_Type_T type, Ring_State_T * p_ring, size_t count)    { p_ring->Head = _RingT_IndexIncOf(type, p_ring->Head, count); }
static inline void _RingT_AddBack(Ring_Type_T type, Ring_State_T * p_ring, size_t count)        { p_ring->Tail = _RingT_IndexIncOf(type, p_ring->Tail, count); }
static inline void _RingT_RemoveBack(Ring_Type_T type, Ring_State_T * p_ring, size_t count)     { p_ring->Tail = _RingT_IndexDecOf(type, p_ring->Tail, count); }

/* FIFO operations */
static inline void _RingT_PushBack(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)    { _RingT_PlaceTail(type, p_ring, p_unit); _RingT_AddBack(type, p_ring, 1U); }
static inline void _RingT_PopFront(Ring_Type_T type, Ring_State_T * p_ring, void * p_result)        { _RingT_PeekHead(type, p_ring, p_result); _RingT_RemoveFront(type, p_ring, 1U); }
static inline void _RingT_PushFront(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)   { _RingT_AddFront(type, p_ring, 1U); _RingT_PlaceHead(type, p_ring, p_unit); }
static inline void _RingT_PopBack(Ring_Type_T type, Ring_State_T * p_ring, void * p_result)         { _RingT_RemoveBack(type, p_ring, 1U); _RingT_PeekTail(type, p_ring, p_result); }

// static inline void * _RingT_PopFront(Ring_Type_T type, Ring_State_T * p_ring) { void * p_front = _RingT_Head(type, p_ring); _RingT_RemoveFront(type, p_ring, 1U); return p_front; }
// static inline void * _RingT_PopBack(Ring_Type_T type, Ring_State_T * p_ring) { _RingT_RemoveBack(type, p_ring, 1U); return _RingT_Tail(type, p_ring); }

/* Random access */
static inline void * _RingT_At(Ring_Type_T type, const Ring_State_T * p_ring, size_t index) { return ring_at(type.TYPE_SIZE, p_ring->Buffer, type.LENGTH, p_ring->Head + index); }
static inline void _RingT_PeekAt(Ring_Type_T type, const Ring_State_T * p_ring, size_t index, void * p_result)  { pointer_assign(type.TYPE_SIZE, p_result, _RingT_At(type, p_ring, index)); }
static inline void _RingT_PlaceAt(Ring_Type_T type, Ring_State_T * p_ring, size_t index, const void * p_unit)   { pointer_assign(type.TYPE_SIZE, _RingT_At(type, p_ring, index), p_unit); }

/* Alias */
static inline void * _RingT_Front(Ring_Type_T type, const Ring_State_T * p_ring) { return _RingT_Head(type, p_ring); }
static inline void * _RingT_Back(Ring_Type_T type, const Ring_State_T * p_ring) { return ring_at(type.TYPE_SIZE, p_ring->Buffer, type.LENGTH, _RingT_IndexDecOf(type, p_ring->Tail, 1U)); }

/* Value access */
static inline int _RingT_GetValueAt(Ring_Type_T type, const Ring_State_T * p_ring, size_t index)        { return pointer_value_as(type.TYPE_SIZE, _RingT_At(type, p_ring, index)); }
static inline void _RingT_SetValueAt(Ring_Type_T type, Ring_State_T * p_ring, size_t index, int value)  { pointer_assign_value_as(type.TYPE_SIZE, _RingT_At(type, p_ring, index), value); }

/* FIFO Each */
static inline void _RingT_PushBackEach(Ring_Type_T type, Ring_State_T * p_ring, const void * p_array, size_t count)     { for (size_t i = 0U; i < count; ++i) { _RingT_PushBack(type, p_ring, void_array_at(type.TYPE_SIZE, p_array, i)); } }
static inline void _RingT_PopFrontEach(Ring_Type_T type, Ring_State_T * p_ring, void * p_buffer, size_t count)          { for (size_t i = 0U; i < count; ++i) { _RingT_PopFront(type, p_ring, void_array_at(type.TYPE_SIZE, p_buffer, i)); } }
static inline void _RingT_PushFrontEach(Ring_Type_T type, Ring_State_T * p_ring, const void * p_array, size_t count)    { for (size_t i = 0U; i < count; ++i) { _RingT_PushFront(type, p_ring, void_array_at(type.TYPE_SIZE, p_array, i)); } }
static inline void _RingT_PopBackEach(Ring_Type_T type, Ring_State_T * p_ring, void * p_buffer, size_t count)           { for (size_t i = 0U; i < count; ++i) { _RingT_PopBack(type, p_ring, void_array_at(type.TYPE_SIZE, p_buffer, i)); } }

/*  */
// static inline size_t _RingT_ContiguousEnd(Ring_Type_T type, const Ring_State_T * p_ring)
// {
//     return _RingT_ArrayIndexOf(type, p_ring->Head) < _RingT_ArrayIndexOf(type, p_ring->Tail) ? _RingT_ArrayIndexOf(type, p_ring->Tail) : type.LENGTH;
// }

/*
    Split of a [unitCount] run into its two contiguous segments.
    Returns the unit count of the first segment, the run from the cursor to the end of the array.
    Bounded by unitCount, so a run that does not wrap leaves a second segment of 0.
*/
static inline size_t _RingT_SplitOf(Ring_Type_T type, size_t cursor, size_t unitCount)
{
    size_t contiguous = type.LENGTH - _RingT_ArrayIndexOf(type, cursor);
    return (unitCount < contiguous) ? unitCount : contiguous;
}

/*
    The same run as its two segments.
    _RingT_SpanOf       the first, from [cursor] to the end of the array
    _RingT_SpanWrapOf   the remainder, always based at Buffer[0], LENGTH 0 when the run does not wrap
*/
static inline ArraySpanT_T _RingT_SpanOf(Ring_Type_T type, const Ring_State_T * p_ring, size_t cursor, size_t unitCount)
{
    return (ArraySpanT_T) { .TYPE_SIZE = type.TYPE_SIZE, .P_BUFFER = ring_at(type.TYPE_SIZE, p_ring->Buffer, type.LENGTH, cursor), .LENGTH = _RingT_SplitOf(type, cursor, unitCount) };
}

static inline ArraySpanT_T _RingT_SpanWrapOf(Ring_Type_T type, const Ring_State_T * p_ring, size_t cursor, size_t unitCount)
{
    return (ArraySpanT_T) { .TYPE_SIZE = type.TYPE_SIZE, .P_BUFFER = (void *)p_ring->Buffer, .LENGTH = unitCount - _RingT_SplitOf(type, cursor, unitCount) };
}

/* Caller guarantees unitCount <= EmptyCount */
static inline void _RingT_PlaceBackWrap(Ring_Type_T type, Ring_State_T * p_ring, const void * p_units, size_t unitCount)
{
    ArraySpanT_T first = _RingT_SpanOf(type, p_ring, p_ring->Tail, unitCount);
    ArraySpan_CopyFrom(first, p_units);
    ArraySpan_CopyFrom(_RingT_SpanWrapOf(type, p_ring, p_ring->Tail, unitCount), void_array_at(type.TYPE_SIZE, p_units, first.LENGTH));
}

/* Caller guarantees unitCount <= FullCount */
static inline void _RingT_PeekFrontWrap(Ring_Type_T type, const Ring_State_T * p_ring, void * p_results, size_t unitCount)
{
    ArraySpanT_T first = _RingT_SpanOf(type, p_ring, p_ring->Head, unitCount);
    ArraySpan_CopyTo(first, p_results);
    ArraySpan_CopyTo(_RingT_SpanWrapOf(type, p_ring, p_ring->Head, unitCount), void_array_at(type.TYPE_SIZE, p_results, first.LENGTH));
}

