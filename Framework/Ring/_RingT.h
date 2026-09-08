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
static inline void * array_at(size_t type, const void * p_array, size_t arrayIndex) { return ((uint8_t *)p_array + (type * arrayIndex)); }

// RING_INDEX_POW2_COUNTER
static inline size_t ring_array_index_of(size_t length, size_t ringIndex) { return(ringIndex & (length - 1U)); }
static inline size_t ring_index_inc(size_t length, size_t ringIndex, size_t inc) { return ringIndex + inc; }

static inline void * ring_array_at(size_t type, const void * p_array, size_t pow2len, size_t ringIndex) { return array_at(type, p_array, (ringIndex & (pow2len - 1U))); }

// static inline size_t _RingT_IndexIncOf(size_t stride, Span_T span, size_t index, size_t inc)
// {
// #if defined(RING_INDEX_POW2_COUNTER)
//     (void)stride; (void)span; return index + inc;
// #elif defined(RING_INDEX_POW2_WRAP)
//     return _RingT_IndexWrapOf(stride, span, index + inc);
// #elif defined(RING_INDEX_LENGTH_COMPARE)
//     return (index + inc >= span.LENGTH) ? index + inc - span.LENGTH : index + inc;
// #endif
// }


/******************************************************************************/
/* Level 2 — ring state (cursors + optional lock) */
/******************************************************************************/
typedef struct __attribute__((aligned(sizeof(uintptr_t)))) Ring_State
{
    volatile uintptr_t Head;    /* FIFO Out/Front. */
    volatile uintptr_t Tail;    /* FIFO In/Back. */
#if defined(RING_LOCAL_CRITICAL_ENABLE)
    volatile critical_lock_t Lock;
#endif
    /*  uint8_t Buffer[]; */
}
Ring_State_T;

/* Level 3 — span (buffer + capacity) */
typedef const struct { void * P_BUFFER; size_t LENGTH; } Span_T;
typedef const struct { void * P_BUFFER; size_t LENGTH; size_t POW2_MASK; } Ring_Span_T;

static inline size_t _RingT_GetCapacity(Span_T type)
{
#if defined(RING_INDEX_POW2_COUNTER)
    return type.LENGTH;  /* Full capacity usable */
#else
    return type.LENGTH - 1U;  /* One slot reserved for empty detection */
#endif
}

static inline size_t _RingT_GetFullCount(Span_T type, const Ring_State_T * p_ring)
{
#if defined(RING_INDEX_POW2_COUNTER)
    (void)type; return p_ring->Tail - p_ring->Head;
#else
    size_t head = ring_array_index_of(type.LENGTH, p_ring->Head);
    size_t tail = ring_array_index_of(type.LENGTH, p_ring->Tail);
    return (tail >= head) ? (tail - head) : (type.LENGTH - head + tail);
#endif
}

static inline bool _RingT_IsFull(Span_T type, const Ring_State_T * p_ring) { return _RingT_GetFullCount(type, p_ring) == _RingT_GetCapacity(type); }


/*
    Private
*/
/* Level 4 — ring operations (stride isolated, span by value, state by pointer) */
static inline void * RingT_At(size_t stride, Span_T span, const Ring_State_T * p_state, size_t index) { return ring_array_at(stride, span.P_BUFFER, span.LENGTH, p_state->Head + index); }
static inline void * _RingT_Head(size_t stride, Span_T span, const Ring_State_T * p_state) { return RingT_At(stride, span, p_state, p_state->Head); }
static inline void * _RingT_Tail(size_t stride, Span_T span, const Ring_State_T * p_state) { return RingT_At(stride, span, p_state, p_state->Tail); }

/* Place operations */
static inline void _RingT_PlaceHead(size_t stride, Span_T span, Ring_State_T * p_state, const void * p_unit) { void_pointer_assign(stride, _RingT_Head(stride, span, p_state), p_unit); }
static inline void _RingT_PlaceTail(size_t stride, Span_T span, Ring_State_T * p_state, const void * p_unit) { void_pointer_assign(stride, _RingT_Tail(stride, span, p_state), p_unit); }

/* Index operations */
// static inline void _RingT_AddFront(Ring_Type_T type, Ring_T * p_ring, size_t count)     { p_ring->Head = _RingT_IndexDecOf(type, p_ring->Head, count); }
// static inline void _RingT_RemoveFront(Ring_Type_T type, Ring_T * p_ring, size_t count)  { p_ring->Head = _RingT_IndexIncOf(type, p_ring->Head, count); }
static inline void _RingT_AddBack(size_t stride, Span_T span, Ring_State_T * p_state, size_t count) { p_state->Tail = ring_index_inc(span.LENGTH, p_state->Tail, count); }
// static inline void _RingT_RemoveBack(Ring_Type_T type, Ring_T * p_ring, size_t count)   { p_ring->Tail = _RingT_IndexDecOf(type, p_ring->Tail, count); }

/* Pointer-path read for > sizeof(intptr_t) elements */
static inline void _RingT_PushBack(size_t stride, Span_T span, Ring_State_T * p_state, const void * p_unit) { _RingT_PlaceTail(stride, span, p_state, p_unit); _RingT_AddBack(stride, span, p_state, 1U); }
static inline void _RingT_PeekAt(size_t stride, Span_T span, const Ring_State_T * p_state, size_t index, void * p_out);

/* Value-path write for ≤ sizeof(intptr_t) elements */
static inline intptr_t _RingT_ValueAt(size_t stride, Span_T span, const Ring_State_T * p_state, size_t index) { return void_pointer_as_value(stride, RingT_At(stride, span, p_state, index)); }
static inline bool _RingT_PushBackV(size_t stride, Span_T span, Ring_State_T * p_state, intptr_t value);

/*
    Public multi content
*/
static bool _RingT_TryPushBack(size_t stride, Span_T span, Ring_State_T * p_state, const void * p_unit) { if (!_RingT_IsFull(span, p_state)) { _RingT_PushBack(span.LENGTH, span, p_state, p_unit); return true; } else { return false; } }



// /******************************************************************************/
// /*!
//     Private - Compile-Time Optimized (Pass Ring_Type_T by value)
// */
// /******************************************************************************/
// static inline size_t _RingT_Length(Ring_Type_T type) { return type.LENGTH; }

// static inline size_t _RingT_Mask(Ring_Type_T type)
// {
// #if defined(RING_INDEX_POW2_COUNTER) || defined(RING_INDEX_POW2_WRAP)
//     return type.POW2_MASK;
// #else
//     (void)type; return 0U;
// #endif
// }

// static inline size_t _RingT_IndexWrapOf(Ring_Type_T type, size_t index)
// {
// #if defined(RING_INDEX_POW2_COUNTER) || defined(RING_INDEX_POW2_WRAP)
//     return (index & type.POW2_MASK);
// #elif defined(RING_INDEX_LENGTH_COMPARE)
//     return (index % type.LENGTH);
// #endif
// }

// static inline size_t _RingT_IndexIncOf(Ring_Type_T type, size_t index, size_t inc)
// {
// #if defined(RING_INDEX_POW2_COUNTER)
//     (void)type; return index + inc;
// #elif defined(RING_INDEX_POW2_WRAP)
//     return _RingT_IndexWrapOf(type, index + inc);
// #elif defined(RING_INDEX_LENGTH_COMPARE)
//     return (index + inc >= type.LENGTH) ? index + inc - type.LENGTH : index + inc;
// #endif
// }

// static inline size_t _RingT_IndexDecOf(Ring_Type_T type, size_t index, size_t dec)
// {
// #if defined(RING_INDEX_POW2_COUNTER)
//     (void)type; return index - dec;
// #elif defined(RING_INDEX_POW2_WRAP)
//     return _RingT_IndexWrapOf(type, index - dec);
// #elif defined(RING_INDEX_LENGTH_COMPARE)
//     return ((int32_t)index - dec < 0) ? type.LENGTH + index - dec : index - dec;
// #endif
// }

// static inline size_t _RingT_ArrayIndexOf(Ring_Type_T type, size_t ringIndex)
// {
// #if defined(RING_INDEX_POW2_COUNTER)
//     return _RingT_IndexWrapOf(type, ringIndex);
// #elif defined(RING_INDEX_POW2_WRAP) || defined(RING_INDEX_LENGTH_COMPARE)
//     (void)type; return ringIndex;
// #endif
// }



// /******************************************************************************/
// /*!
//     Core Operations - Fully Compile-Time Optimized
// */
// /******************************************************************************/
// static inline void * _RingT_ArrayAt(Ring_Type_T type, const void * p_array, size_t arrayIndex) { return ((uint8_t *)p_array + (type.UNIT_SIZE * arrayIndex)); }
// // static inline void * _RingT_ArrayAt(Ring_Type_T type, const Ring_T * p_ring, size_t arrayIndex) { return ((uint8_t *)p_ring->Buffer + (type.UNIT_SIZE * arrayIndex)); }

// static inline void * _RingT_PtrOf(Ring_Type_T type, const Ring_T * p_ring, size_t ringIndex) { return _RingT_ArrayAt(type, p_ring->Buffer, _RingT_ArrayIndexOf(type, ringIndex)); }
// static inline void * _RingT_At(Ring_Type_T type, const Ring_T * p_ring, size_t index) { return _RingT_PtrOf(type, p_ring, _RingT_IndexIncOf(type, p_ring->Head, index)); }

// /* Head/Tail pointer access */
// static inline void * _RingT_Head(Ring_Type_T type, const Ring_T * p_ring) { return _RingT_PtrOf(type, p_ring, p_ring->Head); }
// static inline void * _RingT_Tail(Ring_Type_T type, const Ring_T * p_ring) { return _RingT_PtrOf(type, p_ring, p_ring->Tail); }

// /* Peek operations */
// static inline void _RingT_PeekHead(Ring_Type_T type, const Ring_T * p_ring, void * p_result) { _RingT_Copy(type, p_result, _RingT_Head(type, p_ring)); }
// static inline void _RingT_PeekTail(Ring_Type_T type, const Ring_T * p_ring, void * p_result) { _RingT_Copy(type, p_result, _RingT_Tail(type, p_ring)); }

// /* Place operations */
// static inline void _RingT_PlaceHead(Ring_Type_T type, const Ring_T * p_ring, const void * p_unit) { _RingT_Copy(type, _RingT_Head(type, p_ring), p_unit); }
// static inline void _RingT_PlaceTail(Ring_Type_T type, const Ring_T * p_ring, const void * p_unit) { _RingT_Copy(type, _RingT_Tail(type, p_ring), p_unit); }

// /* Index operations */
// static inline void _RingT_AddFront(Ring_Type_T type, Ring_T * p_ring, size_t count)     { p_ring->Head = _RingT_IndexDecOf(type, p_ring->Head, count); }
// static inline void _RingT_RemoveFront(Ring_Type_T type, Ring_T * p_ring, size_t count)  { p_ring->Head = _RingT_IndexIncOf(type, p_ring->Head, count); }
// static inline void _RingT_AddBack(Ring_Type_T type, Ring_T * p_ring, size_t count)      { p_ring->Tail = _RingT_IndexIncOf(type, p_ring->Tail, count); }
// static inline void _RingT_RemoveBack(Ring_Type_T type, Ring_T * p_ring, size_t count)   { p_ring->Tail = _RingT_IndexDecOf(type, p_ring->Tail, count); }

// /* FIFO operations */
// static inline void _RingT_PushBack(Ring_Type_T type, Ring_T * p_ring, const void * p_unit)  { _RingT_PlaceTail(type, p_ring, p_unit); _RingT_AddBack(type, p_ring, 1U); }
// static inline void _RingT_PopFront(Ring_Type_T type, Ring_T * p_ring, void * p_result)      { _RingT_PeekHead(type, p_ring, p_result); _RingT_RemoveFront(type, p_ring, 1U); }
// static inline void _RingT_PushFront(Ring_Type_T type, Ring_T * p_ring, const void * p_unit) { _RingT_AddFront(type, p_ring, 1U); _RingT_PlaceHead(type, p_ring, p_unit); }
// static inline void _RingT_PopBack(Ring_Type_T type, Ring_T * p_ring, void * p_result)       { _RingT_RemoveBack(type, p_ring, 1U); _RingT_PeekTail(type, p_ring, p_result); }

// /* FIFO Each */
// static inline void _RingT_PushBackEach(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)   { for (size_t i = 0U; i < count; ++i) { _RingT_PushBack(type, p_ring, _RingT_ArrayAt(type, p_array, i)); } }
// static inline void _RingT_PopFrontEach(Ring_Type_T type, Ring_T * p_ring, void * p_buffer, size_t count)        { for (size_t i = 0U; i < count; ++i) { _RingT_PopFront(type, p_ring, _RingT_ArrayAt(type, p_buffer, i)); } }
// static inline void _RingT_PushFrontEach(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)  { for (size_t i = 0U; i < count; ++i) { _RingT_PushFront(type, p_ring, _RingT_ArrayAt(type, p_array, i)); } }
// static inline void _RingT_PopBackEach(Ring_Type_T type, Ring_T * p_ring, void * p_buffer, size_t count)         { for (size_t i = 0U; i < count; ++i) { _RingT_PopBack(type, p_ring, _RingT_ArrayAt(type, p_buffer, i)); } }

// /* Random access */
// static inline void _RingT_PeekAt(Ring_Type_T type, const Ring_T * p_ring, size_t index, void * p_result) { _RingT_Copy(type, p_result, _RingT_At(type, p_ring, index)); }
// static inline void _RingT_PlaceAt(Ring_Type_T type, Ring_T * p_ring, size_t index, const void * p_unit) { _RingT_Copy(type, _RingT_At(type, p_ring, index), p_unit); }

// /* Value access */
// // static inline int _RingT_ValueOf(Ring_Type_T type, const Ring_T * p_ring, size_t ringIndex) { return void_pointer_as_value(type.UNIT_SIZE, _RingT_PtrOf(type, p_ring, ringIndex)); }
// // static inline int _RingT_GetValueAt(Ring_Type_T type, const Ring_T * p_ring, size_t index) { _RingT_ValueOf(type, p_ring, _RingT_IndexIncOf(type, p_ring->Head, index)); }
// // static inline void _RingT_SetValueAt(Ring_Type_T type, Ring_T * p_ring, size_t index, int value)


// /*
// */
// // static inline size_t ContiguousEnd(const Ring_T * p_ring) { return IndexHead(p_ring ) < IndexTail(p_ring) ? IndexTail(p_ring) : p_ring->CONST.LENGTH ; }

// // static inline void PlaceBackWrap(const Ring_T * p_ring, const void * p_units, size_t unitCount)
// // {
// //     size_t split = ContiguousEnd(p_ring);
// //     memcpy(Tail(p_ring), p_units, split);
// //     memcpy(p_ring->CONST.P_BUFFER, PtrOf(p_ring, split), unitCount - split);
// // }

// // static inline void PeekFrontWrap(const Ring_T * p_ring, void * p_results, size_t unitCount)
// // {
// //     size_t split = ContiguousEnd(p_ring);
// //     memcpy(p_results, Front(p_ring), split);
// //     memcpy(void_array_at(p_ring->CONST.UNIT_SIZE, p_results, split), p_ring->CONST.P_BUFFER, unitCount - split);
// // }
