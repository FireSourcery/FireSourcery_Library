#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   RingT.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "_RingT.h"


/******************************************************************************/
/* Level 3 — typed wrapper (user-facing, Correct-Pairing guarantee) */
/******************************************************************************/
/*
    Type-erased storage takes stride from the caller, not from its own state.
    Container types whose storage is mechanically byte-addressable (ring buffers, pools, queues of untyped bytes) should accept stride as a parameter rather than storing it as a field.

    When a typed call-site is wanted, provide a declaration macro (kfifo-style) that pairs a typed buffer pointer with the container's shape descriptor.
        Operation macros derive sizeof(*typed_ptr) at the call site — compile-time literal, zero runtime cost, identical source to the "typed handle" form without its storage overhead.
    Cited: Linux kernel DECLARE_KFIFO; Stroustrup, The C++ Programming Language 4e §25.3.4.1; Stepanov, Elements of Programming §7.
*/

typedef const struct RingT
{
    Ring_Type_T TYPE;
    Ring_State_T * P_STATE; /* State allocation includes its flexible Buffer[] */
}
RingT_T;

/* Caller validates that p_State was allocated for (UnitSize, Length) */
#define RING_T_INIT(UnitSize, Length, p_State) { .TYPE = RING_TYPE_INIT(UnitSize, Length), .P_STATE = (p_State), }
#define RING_T_ALLOC(UnitSize, Length) RING_T_INIT(UnitSize, Length, RING_STATE_ALLOC(UnitSize, Length))

/*
    The (TYPE, P_STATE) argument pair every RingT_ operation takes, drawn from one RingT_T.
    This is the Correct-Pairing guarantee in practice: a descriptor and a state taken from
    different rings is a silent corruption the type system cannot catch, and sourcing both
    from a single RingT_T makes it unrepresentable.

        RingT_PushBack(RING_T_ARGS(p_serial->TX_RING), &txChar);
*/
#define RING_T_ARGS(ring) (ring).TYPE, (ring).P_STATE

/******************************************************************************/
/*!
    Status Operations - Compile-Time Optimized
*/
/******************************************************************************/
/*
    Counter representation (the only one _RingT.h implements).
    Cursors are free-running, so Tail - Head is the exact occupancy and no slot is
    reserved for empty detection. Max usable capacity is the full LENGTH.
*/
static inline size_t RingT_GetCapacity(Ring_Type_T type, const Ring_State_T * p_ring) { (void)p_ring; return type.LENGTH; }
static inline size_t RingT_GetFullCount(Ring_Type_T type, const Ring_State_T * p_ring) { (void)type; return p_ring->Tail - p_ring->Head; }

static inline size_t RingT_GetEmptyCount(Ring_Type_T type, const Ring_State_T * p_ring) { return RingT_GetCapacity(type, p_ring) - RingT_GetFullCount(type, p_ring); }

static inline bool RingT_IsFull(Ring_Type_T type, const Ring_State_T * p_ring) { return RingT_GetFullCount(type, p_ring) == RingT_GetCapacity(type, p_ring); }

static inline bool RingT_IsEmpty(Ring_Type_T type, const Ring_State_T * p_ring) { (void)type; return (p_ring->Tail == p_ring->Head); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline void RingT_Clear(Ring_Type_T type, Ring_State_T * p_ring) { (void)type; p_ring->Head = 0U; p_ring->Tail = 0U; }

/******************************************************************************/
/*!
    Boundary-Checked Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline bool RingT_PushBack(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)   { if (RingT_IsFull(type, p_ring)) { return false; } else { _RingT_PushBack(type, p_ring, p_unit); return true; } }
static inline bool RingT_PopFront(Ring_Type_T type, Ring_State_T * p_ring, void * p_result)       { if (RingT_IsEmpty(type, p_ring)) { return false; } else { _RingT_PopFront(type, p_ring, p_result); return true; } }
static inline bool RingT_PushFront(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)  { if (RingT_IsFull(type, p_ring)) { return false; } else { _RingT_PushFront(type, p_ring, p_unit); return true; } }
static inline bool RingT_PopBack(Ring_Type_T type, Ring_State_T * p_ring, void * p_result)        { if (RingT_IsEmpty(type, p_ring)) { return false; } else { _RingT_PopBack(type, p_ring, p_result); return true; } }
static inline bool RingT_RemoveFront(Ring_Type_T type, Ring_State_T * p_ring, size_t count)       { if (count > RingT_GetFullCount(type, p_ring)) { return false; } else { _RingT_RemoveFront(type, p_ring, count); return true; } }
static inline bool RingT_RemoveBack(Ring_Type_T type, Ring_State_T * p_ring, size_t count)        { if (count > RingT_GetFullCount(type, p_ring)) { return false; } else { _RingT_RemoveBack(type, p_ring, count); return true; } }
/*
    returns the popped pointer
    concurrent push/pop may overwrite contents
*/
// inline void * _Ring_PopFront(Ring_State_T * p_ring) { return (Ring_IsEmpty(p_ring) == false) ? (_PopFront(p_ring)) : NULL; }
// inline void * _Ring_PopBack(Ring_State_T * p_ring) { return (Ring_IsEmpty(p_ring) == false) ? (_PopBack(p_ring)) : NULL; }

/******************************************************************************/
/*!
    Peek Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline bool RingT_PeekAt(Ring_Type_T type, const Ring_State_T * p_ring, size_t index, void * p_result)   { if (index >= RingT_GetFullCount(type, p_ring)) { return false; } else { _RingT_PeekAt(type, p_ring, index, p_result); return true; } }
static inline bool RingT_PeekFront(Ring_Type_T type, const Ring_State_T * p_ring, void * p_result)              { if (RingT_IsEmpty(type, p_ring)) { return false; } else { _RingT_PeekHead(type, p_ring, p_result); return true; } }


/******************************************************************************/
/*!
    Pointer Access Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline void * RingT_At(Ring_Type_T type, const Ring_State_T * p_ring, size_t index) { return (index >= RingT_GetFullCount(type, p_ring)) ? NULL : _RingT_At(type, p_ring, index); }
static inline void * RingT_Front(Ring_Type_T type, const Ring_State_T * p_ring) { return RingT_IsEmpty(type, p_ring) ? NULL : _RingT_Front(type, p_ring); }
static inline void * RingT_Back(Ring_Type_T type, const Ring_State_T * p_ring)  { return RingT_IsEmpty(type, p_ring) ? NULL :_RingT_Back(type, p_ring); }


/******************************************************************************/
/*!
    Overwrite Operations

    Full-policy: evict instead of reject. The counterpart to the reject policy above,
    for latest-data-wins buffers (telemetry, scope traces) where a stalled consumer
    must not stall the producer.

    SINGLE CONTEXT ONLY. Eviction makes the producer advance Head, which breaks the
    single-writer-per-cursor invariant the lock-free SPSC contract depends on.
    Never call these on a ring shared across an ISR boundary.

    Cited: boost::circular_buffer::push_back; std::ring_span (P0059) overwrite semantics.
*/
/******************************************************************************/
/*! @return true if a unit was evicted to make room */
static inline bool RingT_PushBackOverwrite(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)
{
    bool isEvict = RingT_IsFull(type, p_ring);
    if (isEvict) { _RingT_RemoveFront(type, p_ring, 1U); }
    _RingT_PushBack(type, p_ring, p_unit);
    return isEvict;
}

/*! @return true if a unit was evicted to make room */
static inline bool RingT_PushFrontOverwrite(Ring_Type_T type, Ring_State_T * p_ring, const void * p_unit)
{
    bool isEvict = RingT_IsFull(type, p_ring);
    if (isEvict) { _RingT_RemoveBack(type, p_ring, 1U); }
    _RingT_PushFront(type, p_ring, p_unit);
    return isEvict;
}

/*!
    Always accepts the whole array. Only the last [Capacity] units can survive,
    so a count above Capacity discards the leading source units rather than the ring.
    @return units lost - evicted from the ring plus source units skipped
*/
static inline size_t RingT_PushBackOverwriteArray(Ring_Type_T type, Ring_State_T * p_ring, const void * p_array, size_t count)
{
    size_t capacity = RingT_GetCapacity(type, p_ring);
    size_t keep = (count < capacity) ? count : capacity;    /* units of p_array that fit */
    size_t skip = count - keep;                             /* leading source units the ring cannot hold */
    size_t empty = RingT_GetEmptyCount(type, p_ring);
    size_t evict = (keep > empty) ? keep - empty : 0U;

    _RingT_RemoveFront(type, p_ring, evict);
    _RingT_PlaceBackWrap(type, p_ring, void_array_at(type.TYPE_SIZE, p_array, skip), keep);
    _RingT_AddBack(type, p_ring, keep);
    return skip + evict;
}


/******************************************************************************/
/*!
    Contiguous Segment Access - zero copy

    Occupied units and free units each occupy at most two contiguous segments of the
    array. These hand back the segments directly so a DMA engine or driver can read
    from / write into the ring with no intermediate buffer.

        ArraySpanT_T span = RingT_FrontSpan(TYPE, p_ring);        // claim
        DMA_Send(span.P_BUFFER, ArraySpan_Size(span));
        RingT_RemoveFront(TYPE, p_ring, span.LENGTH);            // finish

    The second segment is empty (LENGTH 0) whenever the run does not wrap, so a caller
    that handles both unconditionally is always correct.

    The spans are only valid until the ring is next modified.

    Cited: boost::circular_buffer array_one()/array_two(); kfifo_dma_in_prepare();
    Zephyr ring_buf_put_claim()/get_claim().
*/
/******************************************************************************/
/* Occupied units, to read out. Front then its wrapped remainder. */
static inline ArraySpanT_T RingT_FrontSpan(Ring_Type_T type, const Ring_State_T * p_ring)     { return _RingT_SpanOf(type, p_ring, p_ring->Head, RingT_GetFullCount(type, p_ring)); }
static inline ArraySpanT_T RingT_FrontSpanWrap(Ring_Type_T type, const Ring_State_T * p_ring) { return _RingT_SpanWrapOf(type, p_ring, p_ring->Head, RingT_GetFullCount(type, p_ring)); }

/* Free units, to write into. Back then its wrapped remainder. */
static inline ArraySpanT_T RingT_BackSpan(Ring_Type_T type, const Ring_State_T * p_ring)      { return _RingT_SpanOf(type, p_ring, p_ring->Tail, RingT_GetEmptyCount(type, p_ring)); }
static inline ArraySpanT_T RingT_BackSpanWrap(Ring_Type_T type, const Ring_State_T * p_ring)  { return _RingT_SpanWrapOf(type, p_ring, p_ring->Tail, RingT_GetEmptyCount(type, p_ring)); }


/******************************************************************************/
/*!
    Advanced Operations - Compile-Time Optimized
*/
/******************************************************************************/
// static inline void * RingT_Seek(Ring_Type_T type, Ring_State_T * p_ring, size_t index)
// {
//     if (index >= RingT_GetFullCount(type, p_ring)) { return NULL; }
//    return _RingT_Seek(type, p_ring);
// }

/******************************************************************************/
/*!
    Batch Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline bool RingT_PeekFrontArray(Ring_Type_T type, const Ring_State_T * p_ring, void * p_array, size_t count)
{
    if (count > RingT_GetFullCount(type, p_ring)) { return false; }
    _RingT_PeekFrontWrap(type, p_ring, p_array, count);
    return true;
}

static inline bool RingT_PopFrontArray(Ring_Type_T type, Ring_State_T * p_ring, void * p_array, size_t count)
{
    if (RingT_PeekFrontArray(type, p_ring, p_array, count) == false) { return false; }
    _RingT_RemoveFront(type, p_ring, count);
    return true;
}

static inline bool RingT_PushBackArray(Ring_Type_T type, Ring_State_T * p_ring, const void * p_array, size_t count)
{
    if (count > RingT_GetEmptyCount(type, p_ring)) { return false; }
    _RingT_PlaceBackWrap(type, p_ring, p_array, count);
    _RingT_AddBack(type, p_ring, count);
    return true;
}

static inline size_t RingT_PushBackMax(Ring_Type_T type, Ring_State_T * p_ring, const void * p_array, size_t maxCount)
{
    size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
    size_t pushCount = (maxCount < emptyCount) ? maxCount : emptyCount;
    _RingT_PlaceBackWrap(type, p_ring, p_array, pushCount);
    _RingT_AddBack(type, p_ring, pushCount);
    return pushCount;
}

static inline size_t RingT_PopFrontMax(Ring_Type_T type, Ring_State_T * p_ring, void * p_array, size_t maxCount)
{
    size_t fullCount = RingT_GetFullCount(type, p_ring);
    size_t popCount = (maxCount < fullCount) ? maxCount : fullCount;
    _RingT_PeekFrontWrap(type, p_ring, p_array, popCount);
    _RingT_RemoveFront(type, p_ring, popCount);
    return popCount;
}
