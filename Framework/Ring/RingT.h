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

// #if defined(RING_LOCAL_CRITICAL_ENABLE)
// #include "System/Critical/Critical.h"
// #endif

// #include <stdint.h>
// #include <stdbool.h>
// #include <stddef.h>
// #include <string.h>
// #include <assert.h>


/******************************************************************************/
/* Level 5 — typed wrapper (user-facing, Correct-Pairing guarantee) */
/******************************************************************************/
/*
    3 forms to consider:
    RingT_At(size_t stride, Span_T span, const Ring_State_T * p_state, size_t index)

    RingT as Augmented Array without stride size:
    RingT_At(size_t stride, RingT_T array, size_t index)

    RingT as complete handle with stride size:
    RingT_At(RingT_T ring, size_t index)

    Type-erased storage takes stride from the caller, not from its own state.
    Container types whose storage is mechanically byte-addressable (ring buffers, pools, queues of untyped bytes) should accept stride as a parameter rather than storing it as a field.

    When a typed call-site is wanted, provide a declaration macro (kfifo-style) that pairs a typed buffer pointer with the container's shape descriptor.
        Operation macros derive sizeof(*typed_ptr) at the call site — compile-time literal, zero runtime cost, identical source to the "typed handle" form without its storage overhead.
    Cited: Linux kernel DECLARE_KFIFO; Stroustrup, The C++ Programming Language 4e §25.3.4.1; Stepanov, Elements of Programming §7.

    RingT_At(size_t stride, RingT_T array, size_t index) is selected as the final API form. This keeps stride as a generic parameter.
    A generic container should carry information about its shape, not about the types it transports.
*/

typedef const struct RingT
{
    Ring_Span_T SPAN;
    Ring_State_T * P_STATE; /* optionally include uint8_t Buffer[]; */
}
RingT_T;

// typedef const union RingT
// {
//     struct { void * P_BUFFER; size_t LENGTH; Ring_State_T * P_STATE; };
//     Array_T ARRAY;
// }
// RingT_T;

static inline void * RingT_At(size_t stride, RingT_T * p_ring, size_t index) { return ring_array_at(stride, p_ring->SPAN.P_BUFFER, p_ring->SPAN.LENGTH, p_ring->P_STATE->Head + index); }

static inline bool RingT_PushBack(size_t stride, RingT_T ring, const void * p_unit) {}


// #define _RING_BUFFER_ALLOC(BytesSize) ((uintptr_t[(BytesSize) / sizeof(uintptr_t)]){}) /* guarantees align and no ascii fill */
// #define RING_STATE_ALLOC(UnitSize, Length) ((Ring_State_T *)(_RING_BUFFER_ALLOC(sizeof(Ring_State_T) + ((UnitSize) * (Length)))))
// #define RING_T_ALLOC(UnitSize, Length) RING_T_INIT(UnitSize, Length, RING_STATE_ALLOC(UnitSize, Length))






// /******************************************************************************/
// /*!
//     Protected
// */
// /******************************************************************************/
// static inline void RingT_Clear(Ring_Type_T type, Ring_T * p_ring) { (void)type; p_ring->Head = 0U; p_ring->Tail = 0U; }

// /******************************************************************************/
// /*!
//     Status Operations - Compile-Time Optimized
// */
// /******************************************************************************/
// static inline size_t RingT_GetCapacity(Ring_Type_T type, const Ring_T * p_ring)
// {
//     (void)p_ring;
// #if defined(RING_INDEX_POW2_COUNTER)
//     return type.LENGTH;  /* Full capacity usable */
// #else
//     return type.LENGTH - 1U;  /* One slot reserved for empty detection */
// #endif
// }

// /*
//     RING_INDEX_POW2_COUNTER
//     Max usable capacity is length

//     RING_INDEX_POW2_WRAP, RING_INDEX_LENGTH_COMPARE
//     Empty space detection method. Tail always points to empty space. Max usable capacity is length - 1
// */
// static inline size_t RingT_GetFullCount(Ring_Type_T type, const Ring_T * p_ring)
// {
// #if defined(RING_INDEX_POW2_COUNTER)
//     (void)type; return p_ring->Tail - p_ring->Head;
// #else
//     size_t head = _RingT_ArrayIndexOf(type, p_ring->Head);
//     size_t tail = _RingT_ArrayIndexOf(type, p_ring->Tail);
//     return (tail >= head) ? (tail - head) : (type.LENGTH - head + tail);
// #endif
// }

// static inline size_t RingT_GetEmptyCount(Ring_Type_T type, const Ring_T * p_ring)
// {
//     return RingT_GetCapacity(type, p_ring) - RingT_GetFullCount(type, p_ring);
// }

// static inline bool RingT_IsFull(Ring_Type_T type, const Ring_T * p_ring) { return RingT_GetFullCount(type, p_ring) == RingT_GetCapacity(type, p_ring); }

// static inline bool RingT_IsEmpty(Ring_Type_T type, const Ring_T * p_ring) { return (p_ring->Tail == p_ring->Head); }


// /******************************************************************************/
// /*!
//     Boundary-Checked Operations - Compile-Time Optimized
// */
// /******************************************************************************/
// static inline bool RingT_PushBack(Ring_Type_T type, Ring_T * p_ring, const void * p_unit)   { return (RingT_IsFull(type, p_ring) ? false : ({ _RingT_PushBack(type, p_ring, p_unit); true; })); }
// static inline bool RingT_PopFront(Ring_Type_T type, Ring_T * p_ring, void * p_result)       { return (RingT_IsEmpty(type, p_ring) ? false : ({ _RingT_PopFront(type, p_ring, p_result); true; })); }
// static inline bool RingT_PushFront(Ring_Type_T type, Ring_T * p_ring, const void * p_unit)  { return (RingT_IsFull(type, p_ring) ? false : ({ _RingT_PushFront(type, p_ring, p_unit); true; })); }
// static inline bool RingT_PopBack(Ring_Type_T type, Ring_T * p_ring, void * p_result)        { return (RingT_IsEmpty(type, p_ring) ? false : ({ _RingT_PopBack(type, p_ring, p_result); true; })); }
// static inline bool RingT_RemoveFront(Ring_Type_T type, Ring_T * p_ring, size_t count)       { return (count > RingT_GetFullCount(type, p_ring) ? false : ({ _RingT_RemoveFront(type, p_ring, count); true; })); }
// static inline bool RingT_RemoveBack(Ring_Type_T type, Ring_T * p_ring, size_t count)        { return (count > RingT_GetFullCount(type, p_ring) ? false : ({ _RingT_RemoveBack(type, p_ring, count); true; })); }

// /*
//     returns the popped pointer
//     concurrent push/pop may overwrite contents
// */
// // inline void * _Ring_PopFront(Ring_T * p_ring) { return (Ring_IsEmpty(p_ring) == false) ? (_PopFront(p_ring)) : NULL; }
// // inline void * _Ring_PopBack(Ring_T * p_ring) { return (Ring_IsEmpty(p_ring) == false) ? (_PopBack(p_ring)) : NULL; }

// /******************************************************************************/
// /*!
//     Pointer Access Operations - Compile-Time Optimized
// */
// /******************************************************************************/
// static inline void * RingT_Front(Ring_Type_T type, const Ring_T * p_ring) { return RingT_IsEmpty(type, p_ring) ? NULL : _RingT_Head(type, p_ring); }
// static inline void * RingT_Back(Ring_Type_T type, const Ring_T * p_ring) { return RingT_IsEmpty(type, p_ring) ? NULL : _RingT_PtrOf(type, p_ring, _RingT_IndexDecOf(type, p_ring->Tail, 1U)); }
// static inline void * RingT_At(Ring_Type_T type, const Ring_T * p_ring, size_t index) { return (index >= RingT_GetFullCount(type, p_ring)) ? NULL : _RingT_At(type, p_ring, index); }

// // /******************************************************************************/
// // /*!
// //     Peek Operations - Compile-Time Optimized
// // */
// // /******************************************************************************/
// // static inline bool RingT_PeekFront(Ring_Type_T type, const Ring_T * p_ring, void * p_result)
// // {
// //     if (_RingT_IsEmpty(type, p_ring)) return false;
// //     _RingT_PeekHead(type, p_ring, p_result);
// //     return true;
// // }

// // static inline bool RingT_PeekBack(Ring_Type_T type, const Ring_T * p_ring, void * p_result)
// // {
// //     if (_RingT_IsEmpty(type, p_ring)) return false;
// //     _RingT_Copy(type, p_result, _RingT_PtrOf(type, p_ring, _RingT_IndexDecOf(type, p_ring->Tail, 1U)));
// //     return true;
// // }

// // static inline bool RingT_PeekAt(Ring_Type_T type, const Ring_T * p_ring, size_t index, void * p_result)
// // {
// //     if (index >= _RingT_GetFullCount(type, p_ring)) return false;
// //     _RingT_GetAt(type, p_ring, index, p_result);
// //     return true;
// // }

// // /******************************************************************************/
// // /*!
// //     Advanced Operations - Compile-Time Optimized
// // */
// // /******************************************************************************/
// // static inline void * RingT_Seek(Ring_Type_T type, Ring_T * p_ring, size_t index)
// // {
// //     if (index >= RingT_GetFullCount(type, p_ring)) return NULL;
// //     _RingT_RemoveFront(type, p_ring, index);
// //     return _RingT_Head(type, p_ring);
// // }


// /******************************************************************************/
// /*!
//     Batch Operations - Compile-Time Optimized
// */
// /******************************************************************************/
// // static inline size_t RingT_PushBackArray(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
// static inline size_t RingT_PushBackMax(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
// {
//     size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
//     size_t pushCount = (count <= emptyCount) ? count : emptyCount;
//     _RingT_PushBackEach(type, p_ring, p_array, pushCount);
//     return pushCount;
// }

// static inline bool RingT_PushBackAll(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
// {
//     return (count <= Ring_GetEmptyCount(p_ring)) ? ({ _RingT_PushBackEach(type, p_ring, p_array, count); true; }) : false;
// }

// // static inline size_t _RingT_PopFrontArray(Ring_Type_T type, Ring_T * p_ring, void * p_array, size_t count)
// // {
// //     size_t fullCount = RingT_GetFullCount(type, p_ring);
// //     size_t readCount = (count <= fullCount) ? count : fullCount;
// //     uint8_t * p_dest = (uint8_t *)p_array;

// //     for (size_t i = 0U; i < readCount; i++)
// //     {
// //         _RingT_PopFront(type, p_ring, p_dest);
// //         p_dest += type.UNIT_SIZE;
// //     }
// //     return readCount;
// // }

// // static inline size_t _RingT_PushFrontArray(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
// // {
// //     size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
// //     size_t writeCount = (count <= emptyCount) ? count : emptyCount;
// //     const uint8_t * p_src = (const uint8_t *)p_array;

// //     for (size_t i = 0U; i < writeCount; i++)
// //     {
// //         _RingT_PushFront(type, p_ring, p_src);
// //         p_src += type.UNIT_SIZE;
// //     }
// //     return writeCount;
// // }

// // static inline size_t _RingT_PopBackArray(Ring_Type_T type, Ring_T * p_ring, void * p_array, size_t count)
// // {
// //     size_t fullCount = RingT_GetFullCount(type, p_ring);
// //     size_t readCount = (count <= fullCount) ? count : fullCount;
// //     uint8_t * p_dest = (uint8_t *)p_array;

// //     for (size_t i = 0U; i < readCount; i++)
// //     {
// //         _RingT_PopBack(type, p_ring, p_dest);
// //         p_dest += type.UNIT_SIZE;
// //     }
// //     return readCount;
// // }

// // static inline size_t _RingT_PushBackMax(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t maxCount)
// // {
// //     size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
// //     size_t writeCount = (maxCount <= emptyCount) ? maxCount : emptyCount;
// //     const uint8_t * p_src = (const uint8_t *)p_array;

// //     for (size_t i = 0U; i < writeCount; i++)
// //     {
// //         _RingT_PushBack(type, p_ring, p_src);
// //         p_src += type.UNIT_SIZE;
// //     }
// //     return writeCount;
// // }

// // static inline size_t _RingT_PopFrontMax(Ring_Type_T type, Ring_T * p_ring, void * p_array, size_t maxCount)
// // {
// //     size_t fullCount = RingT_GetFullCount(type, p_ring);
// //     size_t readCount = (maxCount <= fullCount) ? maxCount : fullCount;
// //     uint8_t * p_dest = (uint8_t *)p_array;

// //     for (size_t i = 0U; i < readCount; i++)
// //     {
// //         _RingT_PopFront(type, p_ring, p_dest);
// //         p_dest += type.UNIT_SIZE;
// //     }
// //     return readCount;
// // }



