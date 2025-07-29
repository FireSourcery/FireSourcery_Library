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
#include "Config.h"
#include "Ring.h"

#include "Type/Array/void_array.h"

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
*/
// This becomes like a C++ template - each Ring_Type_T generates optimized code
// const Ring_Type_T UINT32_RING_TYPE = _RING_TYPE_INIT(sizeof(uint32_t), 256);

// Compiler generates specialized function for this exact type
// _RingT_PushBack(UINT32_RING_TYPE, &myRing, &value);
// Becomes equivalent to:
// memcpy(ring->Buffer + (4 * ring->Tail), &value, 4);
// ring->Tail = (ring->Tail + 1) & 255;
/******************************************************************************/

/******************************************************************************/
/*!
    Private - Compile-Time Optimized (Pass Ring_Type_T by value)
*/
/******************************************************************************/
static inline size_t _RingT_Length(Ring_Type_T type) { return type.LENGTH; }

static inline size_t _RingT_Mask(Ring_Type_T type)
{
#if defined(RING_INDEX_POW2_COUNTER) || defined(RING_INDEX_POW2_WRAP)
    return type.POW2_MASK;
#else
    (void)type; return 0U;
#endif
}

static inline size_t _RingT_IndexWrapOf(Ring_Type_T type, size_t index)
{
#if defined(RING_INDEX_POW2_COUNTER) || defined(RING_INDEX_POW2_WRAP)
    return (index & type.POW2_MASK);
#elif defined(RING_INDEX_LENGTH_COMPARE)
    return (index % type.LENGTH);
#endif
}

static inline size_t _RingT_IndexIncOf(Ring_Type_T type, size_t index, size_t inc)
{
#if defined(RING_INDEX_POW2_COUNTER)
    (void)type; return index + inc;
#elif defined(RING_INDEX_POW2_WRAP)
    return _RingT_IndexWrapOf(type, index + inc);
#elif defined(RING_INDEX_LENGTH_COMPARE)
    return (index + inc >= type.LENGTH) ? index + inc - type.LENGTH : index + inc;
#endif
}

static inline size_t _RingT_IndexDecOf(Ring_Type_T type, size_t index, size_t dec)
{
#if defined(RING_INDEX_POW2_COUNTER)
    (void)type; return index - dec;
#elif defined(RING_INDEX_POW2_WRAP)
    return _RingT_IndexWrapOf(type, index - dec);
#elif defined(RING_INDEX_LENGTH_COMPARE)
    return ((int32_t)index - dec < 0) ? type.LENGTH + index - dec : index - dec;
#endif
}

static inline size_t _RingT_ArrayIndexOf(Ring_Type_T type, size_t ringIndex)
{
#if defined(RING_INDEX_POW2_COUNTER)
    return _RingT_IndexWrapOf(type, ringIndex);
#elif defined(RING_INDEX_POW2_WRAP) || defined(RING_INDEX_LENGTH_COMPARE)
    (void)type; return ringIndex;
#endif
}

static inline void _RingT_Copy(Ring_Type_T type, void * p_dest, const void * p_src)
{
    switch (type.UNIT_SIZE)
    {
        case sizeof(uint8_t) : *((uint8_t  *)p_dest) = *((const uint8_t  *)p_src); break;
        case sizeof(uint16_t): *((uint16_t *)p_dest) = *((const uint16_t *)p_src); break;
        case sizeof(uint32_t): *((uint32_t *)p_dest) = *((const uint32_t *)p_src); break;
#if (REGISTER_SIZE_64)
        case sizeof(uint64_t) : *((uint64_t *)p_dest) = *((const uint64_t *)p_src); break;
#endif
        default: memcpy(p_dest, p_src, type.UNIT_SIZE); break;
    }
}


/******************************************************************************/
/*!
    Core Operations - Fully Compile-Time Optimized
*/
/******************************************************************************/
static inline void * _RingT_ArrayAt(Ring_Type_T type, const void * p_array, size_t arrayIndex) { return ((uint8_t *)p_array + (type.UNIT_SIZE * arrayIndex)); }
// static inline void * _RingT_ArrayAt(Ring_Type_T type, const Ring_T * p_ring, size_t arrayIndex) { return ((uint8_t *)p_ring->Buffer + (type.UNIT_SIZE * arrayIndex)); }

static inline void * _RingT_PtrOf(Ring_Type_T type, const Ring_T * p_ring, size_t ringIndex) { return _RingT_ArrayAt(type, p_ring->Buffer, _RingT_ArrayIndexOf(type, ringIndex)); }

/* Head/Tail pointer access */
static inline void * _RingT_Head(Ring_Type_T type, const Ring_T * p_ring) { return _RingT_PtrOf(type, p_ring, p_ring->Head); }
static inline void * _RingT_Tail(Ring_Type_T type, const Ring_T * p_ring) { return _RingT_PtrOf(type, p_ring, p_ring->Tail); }

/* Peek operations */
static inline void _RingT_PeekHead(Ring_Type_T type, const Ring_T * p_ring, void * p_result) { _RingT_Copy(type, p_result, _RingT_Head(type, p_ring)); }
static inline void _RingT_PeekTail(Ring_Type_T type, const Ring_T * p_ring, void * p_result) { _RingT_Copy(type, p_result, _RingT_Tail(type, p_ring)); }

/* Place operations */
static inline void _RingT_PlaceHead(Ring_Type_T type, const Ring_T * p_ring, const void * p_unit) { _RingT_Copy(type, _RingT_Head(type, p_ring), p_unit); }
static inline void _RingT_PlaceTail(Ring_Type_T type, const Ring_T * p_ring, const void * p_unit) { _RingT_Copy(type, _RingT_Tail(type, p_ring), p_unit); }

/* Index operations */
static inline void _RingT_AddFront(Ring_Type_T type, Ring_T * p_ring, size_t count)     { p_ring->Head = _RingT_IndexDecOf(type, p_ring->Head, count); }
static inline void _RingT_RemoveFront(Ring_Type_T type, Ring_T * p_ring, size_t count)  { p_ring->Head = _RingT_IndexIncOf(type, p_ring->Head, count); }
static inline void _RingT_AddBack(Ring_Type_T type, Ring_T * p_ring, size_t count)      { p_ring->Tail = _RingT_IndexIncOf(type, p_ring->Tail, count); }
static inline void _RingT_RemoveBack(Ring_Type_T type, Ring_T * p_ring, size_t count)   { p_ring->Tail = _RingT_IndexDecOf(type, p_ring->Tail, count); }

/* FIFO operations */
static inline void _RingT_PushBack(Ring_Type_T type, Ring_T * p_ring, const void * p_unit)  { _RingT_PlaceTail(type, p_ring, p_unit); _RingT_AddBack(type, p_ring, 1U); }
static inline void _RingT_PopFront(Ring_Type_T type, Ring_T * p_ring, void * p_result)      { _RingT_PeekHead(type, p_ring, p_result); _RingT_RemoveFront(type, p_ring, 1U); }
static inline void _RingT_PushFront(Ring_Type_T type, Ring_T * p_ring, const void * p_unit) { _RingT_AddFront(type, p_ring, 1U); _RingT_PlaceHead(type, p_ring, p_unit); }
static inline void _RingT_PopBack(Ring_Type_T type, Ring_T * p_ring, void * p_result)       { _RingT_RemoveBack(type, p_ring, 1U); _RingT_PeekTail(type, p_ring, p_result); }

/* FIFO Each */
static inline void _RingT_PushBackEach(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)   { for (size_t i = 0U; i < count; ++i) { _RingT_PushBack(type, p_ring, _RingT_ArrayAt(type, p_array, i)); } }
static inline void _RingT_PopFrontEach(Ring_Type_T type, Ring_T * p_ring, void * p_buffer, size_t count)        { for (size_t i = 0U; i < count; ++i) { _RingT_PopFront(type, p_ring, _RingT_ArrayAt(type, p_buffer, i)); } }
static inline void _RingT_PushFrontEach(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)  { for (size_t i = 0U; i < count; ++i) { _RingT_PushFront(type, p_ring, _RingT_ArrayAt(type, p_array, i)); } }
static inline void _RingT_PopBackEach(Ring_Type_T type, Ring_T * p_ring, void * p_buffer, size_t count)         { for (size_t i = 0U; i < count; ++i) { _RingT_PopBack(type, p_ring, _RingT_ArrayAt(type, p_buffer, i)); } }

/* Random access */
static inline void * _RingT_At(Ring_Type_T type, const Ring_T * p_ring, size_t index) { return _RingT_PtrOf(type, p_ring, _RingT_IndexIncOf(type, p_ring->Head, index)); }
static inline void _RingT_PeekAt(Ring_Type_T type, const Ring_T * p_ring, size_t index, void * p_result) { _RingT_Copy(type, p_result, _RingT_At(type, p_ring, index)); }
static inline void _RingT_PlaceAt(Ring_Type_T type, Ring_T * p_ring, size_t index, const void * p_unit) { _RingT_Copy(type, _RingT_At(type, p_ring, index), p_unit); }

/* Value access */
// static inline int _RingT_ValueOf(Ring_Type_T type, const Ring_T * p_ring, size_t ringIndex) { return void_pointer_as_value(_RingT_PtrOf(type, p_ring, ringIndex), type.UNIT_SIZE); }
// static inline int _RingT_GetValueAt(Ring_Type_T type, const Ring_T * p_ring, size_t index) { _RingT_ValueOf(type, p_ring, _RingT_IndexIncOf(type, p_ring->Head, index)); }
// static inline void _RingT_SetValueAt(Ring_Type_T type, Ring_T * p_ring, size_t index, int value)


/*
*/
// static inline size_t ContiguousEnd(const Ring_T * p_ring) { return IndexHead(p_ring ) < IndexTail(p_ring) ? IndexTail(p_ring) : p_ring->CONST.LENGTH ; }

// static inline void PlaceBackWrap(const Ring_T * p_ring, const void * p_units, size_t unitCount)
// {
//     size_t split = ContiguousEnd(p_ring);
//     memcpy(Tail(p_ring), p_units, split);
//     memcpy(p_ring->CONST.P_BUFFER, PtrOf(p_ring, split), unitCount - split);
// }

// static inline void PeekFrontWrap(const Ring_T * p_ring, void * p_results, size_t unitCount)
// {
//     size_t split = ContiguousEnd(p_ring);
//     memcpy(p_results, Front(p_ring), split);
//     memcpy(void_pointer_at(p_results, p_ring->CONST.UNIT_SIZE, split), p_ring->CONST.P_BUFFER, unitCount - split);
// }


/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
static inline void RingT_Clear(Ring_Type_T type, Ring_T * p_ring) { (void)type; p_ring->Head = 0U; p_ring->Tail = 0U; }

/******************************************************************************/
/*!
    Status Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline size_t RingT_GetCapacity(Ring_Type_T type, const Ring_T * p_ring)
{
    (void)p_ring;
#if defined(RING_INDEX_POW2_COUNTER)
    return type.LENGTH;  /* Full capacity usable */
#else
    return type.LENGTH - 1U;  /* One slot reserved for empty detection */
#endif
}

/*
    RING_INDEX_POW2_COUNTER
    Max usable capacity is length

    RING_INDEX_POW2_WRAP, RING_INDEX_LENGTH_COMPARE
    Empty space detection method. Tail always points to empty space. Max usable capacity is length - 1
*/
static inline size_t RingT_GetFullCount(Ring_Type_T type, const Ring_T * p_ring)
{
#if defined(RING_INDEX_POW2_COUNTER)
    (void)type; return p_ring->Tail - p_ring->Head;
#else
    size_t head = _RingT_ArrayIndexOf(type, p_ring->Head);
    size_t tail = _RingT_ArrayIndexOf(type, p_ring->Tail);
    return (tail >= head) ? (tail - head) : (type.LENGTH - head + tail);
#endif
}

static inline size_t RingT_GetEmptyCount(Ring_Type_T type, const Ring_T * p_ring)
{
    return RingT_GetCapacity(type, p_ring) - RingT_GetFullCount(type, p_ring);
}

static inline bool RingT_IsFull(Ring_Type_T type, const Ring_T * p_ring) { return RingT_GetFullCount(type, p_ring) == RingT_GetCapacity(type, p_ring); }

static inline bool RingT_IsEmpty(Ring_Type_T type, const Ring_T * p_ring) { return (p_ring->Tail == p_ring->Head); }


/******************************************************************************/
/*!
    Boundary-Checked Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline bool RingT_PushBack(Ring_Type_T type, Ring_T * p_ring, const void * p_unit)   { return (RingT_IsFull(type, p_ring) ? false : ({ _RingT_PushBack(type, p_ring, p_unit); true; })); }
static inline bool RingT_PopFront(Ring_Type_T type, Ring_T * p_ring, void * p_result)       { return (RingT_IsEmpty(type, p_ring) ? false : ({ _RingT_PopFront(type, p_ring, p_result); true; })); }
static inline bool RingT_PushFront(Ring_Type_T type, Ring_T * p_ring, const void * p_unit)  { return (RingT_IsFull(type, p_ring) ? false : ({ _RingT_PushFront(type, p_ring, p_unit); true; })); }
static inline bool RingT_PopBack(Ring_Type_T type, Ring_T * p_ring, void * p_result)        { return (RingT_IsEmpty(type, p_ring) ? false : ({ _RingT_PopBack(type, p_ring, p_result); true; })); }
static inline bool RingT_RemoveFront(Ring_Type_T type, Ring_T * p_ring, size_t count)       { return (count > RingT_GetFullCount(type, p_ring) ? false : ({ _RingT_RemoveFront(type, p_ring, count); true; })); }
static inline bool RingT_RemoveBack(Ring_Type_T type, Ring_T * p_ring, size_t count)        { return (count > RingT_GetFullCount(type, p_ring) ? false : ({ _RingT_RemoveBack(type, p_ring, count); true; })); }

/*
    returns the popped pointer
    concurrent push/pop may overwrite contents
*/
// inline void * _Ring_PopFront(Ring_T * p_ring) { return (Ring_IsEmpty(p_ring) == false) ? (_PopFront(p_ring)) : NULL; }
// inline void * _Ring_PopBack(Ring_T * p_ring) { return (Ring_IsEmpty(p_ring) == false) ? (_PopBack(p_ring)) : NULL; }

/******************************************************************************/
/*!
    Pointer Access Operations - Compile-Time Optimized
*/
/******************************************************************************/
static inline void * RingT_Front(Ring_Type_T type, const Ring_T * p_ring) { return RingT_IsEmpty(type, p_ring) ? NULL : _RingT_Head(type, p_ring); }
static inline void * RingT_Back(Ring_Type_T type, const Ring_T * p_ring) { return RingT_IsEmpty(type, p_ring) ? NULL : _RingT_PtrOf(type, p_ring, _RingT_IndexDecOf(type, p_ring->Tail, 1U)); }
static inline void * RingT_At(Ring_Type_T type, const Ring_T * p_ring, size_t index) { return (index >= RingT_GetFullCount(type, p_ring)) ? NULL : _RingT_At(type, p_ring, index); }

// /******************************************************************************/
// /*!
//     Peek Operations - Compile-Time Optimized
// */
// /******************************************************************************/
// static inline bool RingT_PeekFront(Ring_Type_T type, const Ring_T * p_ring, void * p_result)
// {
//     if (_RingT_IsEmpty(type, p_ring)) return false;
//     _RingT_PeekHead(type, p_ring, p_result);
//     return true;
// }

// static inline bool RingT_PeekBack(Ring_Type_T type, const Ring_T * p_ring, void * p_result)
// {
//     if (_RingT_IsEmpty(type, p_ring)) return false;
//     _RingT_Copy(type, p_result, _RingT_PtrOf(type, p_ring, _RingT_IndexDecOf(type, p_ring->Tail, 1U)));
//     return true;
// }

// static inline bool RingT_PeekAt(Ring_Type_T type, const Ring_T * p_ring, size_t index, void * p_result)
// {
//     if (index >= _RingT_GetFullCount(type, p_ring)) return false;
//     _RingT_GetAt(type, p_ring, index, p_result);
//     return true;
// }

// /******************************************************************************/
// /*!
//     Advanced Operations - Compile-Time Optimized
// */
// /******************************************************************************/
// static inline void * RingT_Seek(Ring_Type_T type, Ring_T * p_ring, size_t index)
// {
//     if (index >= RingT_GetFullCount(type, p_ring)) return NULL;
//     _RingT_RemoveFront(type, p_ring, index);
//     return _RingT_Head(type, p_ring);
// }


/******************************************************************************/
/*!
    Batch Operations - Compile-Time Optimized
*/
/******************************************************************************/
// static inline size_t RingT_PushBackArray(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
static inline size_t RingT_PushBackMax(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
{
    size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
    size_t pushCount = (count <= emptyCount) ? count : emptyCount;
    _RingT_PushBackEach(type, p_ring, p_array, pushCount);
    return pushCount;
}

static inline bool RingT_PushBackAll(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
{
    return (count <= Ring_GetEmptyCount(p_ring)) ? ({ _RingT_PushBackEach(type, p_ring, p_array, count); true; }) : false;
}

// static inline size_t _RingT_PopFrontArray(Ring_Type_T type, Ring_T * p_ring, void * p_array, size_t count)
// {
//     size_t fullCount = RingT_GetFullCount(type, p_ring);
//     size_t readCount = (count <= fullCount) ? count : fullCount;
//     uint8_t * p_dest = (uint8_t *)p_array;

//     for (size_t i = 0U; i < readCount; i++)
//     {
//         _RingT_PopFront(type, p_ring, p_dest);
//         p_dest += type.UNIT_SIZE;
//     }
//     return readCount;
// }

// static inline size_t _RingT_PushFrontArray(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t count)
// {
//     size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
//     size_t writeCount = (count <= emptyCount) ? count : emptyCount;
//     const uint8_t * p_src = (const uint8_t *)p_array;

//     for (size_t i = 0U; i < writeCount; i++)
//     {
//         _RingT_PushFront(type, p_ring, p_src);
//         p_src += type.UNIT_SIZE;
//     }
//     return writeCount;
// }

// static inline size_t _RingT_PopBackArray(Ring_Type_T type, Ring_T * p_ring, void * p_array, size_t count)
// {
//     size_t fullCount = RingT_GetFullCount(type, p_ring);
//     size_t readCount = (count <= fullCount) ? count : fullCount;
//     uint8_t * p_dest = (uint8_t *)p_array;

//     for (size_t i = 0U; i < readCount; i++)
//     {
//         _RingT_PopBack(type, p_ring, p_dest);
//         p_dest += type.UNIT_SIZE;
//     }
//     return readCount;
// }

// static inline size_t _RingT_PushBackMax(Ring_Type_T type, Ring_T * p_ring, const void * p_array, size_t maxCount)
// {
//     size_t emptyCount = RingT_GetEmptyCount(type, p_ring);
//     size_t writeCount = (maxCount <= emptyCount) ? maxCount : emptyCount;
//     const uint8_t * p_src = (const uint8_t *)p_array;

//     for (size_t i = 0U; i < writeCount; i++)
//     {
//         _RingT_PushBack(type, p_ring, p_src);
//         p_src += type.UNIT_SIZE;
//     }
//     return writeCount;
// }

// static inline size_t _RingT_PopFrontMax(Ring_Type_T type, Ring_T * p_ring, void * p_array, size_t maxCount)
// {
//     size_t fullCount = RingT_GetFullCount(type, p_ring);
//     size_t readCount = (maxCount <= fullCount) ? maxCount : fullCount;
//     uint8_t * p_dest = (uint8_t *)p_array;

//     for (size_t i = 0U; i < readCount; i++)
//     {
//         _RingT_PopFront(type, p_ring, p_dest);
//         p_dest += type.UNIT_SIZE;
//     }
//     return readCount;
// }



