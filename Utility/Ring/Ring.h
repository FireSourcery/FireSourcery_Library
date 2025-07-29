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
    @brief  Ring Buffer. Optimized implementation using POW2_MASK

*/
/******************************************************************************/
#ifndef RING_UTILITY_H
#define RING_UTILITY_H

#include "Config.h"

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

*/
/******************************************************************************/
// #ifndef RING_ALIGNED
// #define RING_ALIGNED __attribute__((aligned(sizeof(intptr_t)))) /* Align to register size */
// #endif

#if defined(RING_INDEX_POW2_COUNTER) || defined(RING_INDEX_POW2_WRAP)
#define _RING_POW2_DEF(code, ...) code
#define _RING_NON_POW2_DEF(code)
// #define _RING_POW2_DEF_INIT(code) code,
#else
#define _RING_POW2_DEF(code, ...) __VA_ARGS__
#define _RING_NON_POW2_DEF(code) code
#endif

/*  */
typedef const struct Ring_Type
{
    size_t UNIT_SIZE;     /* Bytes */
    size_t LENGTH;        /* In UNIT_SIZE counts (NOT bytes) */
    _RING_POW2_DEF(size_t POW2_MASK;)
}
Ring_Type_T;

#define RING_TYPE_INIT(UnitSize, Length) { .UNIT_SIZE = UnitSize, .LENGTH = Length, _RING_POW2_DEF(.POW2_MASK = (Length - 1U)) }

/*  */
typedef struct Ring_State
{
    volatile uintptr_t Head;    /* FIFO Out/Front. */
    volatile uintptr_t Tail;    /* FIFO In/Back. */
#if defined(RING_LOCAL_CRITICAL_ENABLE)
    volatile critical_lock_t Lock;
#endif
    uint8_t Buffer[]; /* MISRA violation. Rationale: Compile-time allocated. */
}
Ring_State_T;

#define _RING_BUFFER_ALLOC(BytesSize) ((uint8_t[(BytesSize)]){})
// #define _RING_BUFFER_ALLOC(BytesSize) ((uintptr_t[(BytesSize) / sizeof(uintptr_t)]){}) /* /sizeof(int) guarantees align and no ascii fill */
#define _RING_STATE_ALLOC(UnitSize, Length) ((Ring_T *)(_RING_BUFFER_ALLOC(sizeof(Ring_T) + ((UnitSize)*(Length)))))


/*
    Run time contigous implementation.
*/
typedef struct Ring
{
    const Ring_Type_T Type;
    union //temp wrap
    {
        Ring_State_T State;
        struct
        {
            volatile uintptr_t Head;    /* FIFO Out/Front. */
            volatile uintptr_t Tail;    /* FIFO In/Back. */
        #if defined(RING_LOCAL_CRITICAL_ENABLE)
            volatile critical_lock_t Lock;
        #endif
            uint8_t Buffer[]; /* MISRA violation. Rationale: Compile-time allocated. */
        };
    };
}
Ring_T;

#define RING_STATE_ALLOC(UnitSize, Length) ((Ring_T *)(_RING_BUFFER_ALLOC(sizeof(Ring_T) + ((UnitSize)*(Length)))))

/*
    For Generic Handling or Runtime Init
    In the case that Runtime state stores [Ring_Type_T Type], it must be copied at run time.
*/
typedef const struct Ring_Context
{
    Ring_Type_T TYPE;
    Ring_T * P_STATE;
    // Ring_State_T * P_STATE;
}
Ring_Context_T;

#define RING_CONTEXT_INIT(UnitSize, Length, p_State) { .TYPE = RING_TYPE_INIT(UnitSize, Length), .P_STATE = p_State, } /* Caller validates State with Length */
#define RING_CONTEXT_ALLOC(UnitSize, Length) RING_CONTEXT_INIT(UnitSize, Length, RING_STATE_ALLOC(UnitSize, Length))


#define IS_POW2(x) (((x) & ((x) - 1U)) == 0U)
#define IS_ALIGNED(x, align) (((x) & ((align) - 1U)) == 0U)

#define RING_VALIDATE_PARAMS(unit_size, length) \
    static_assert(_RING_POW2_DEF(IS_POW2(length), true), "POW2 mode requires power-of-2 length"); \
    static_assert(IS_ALIGNED(unit_size, sizeof(uintptr_t)) || IS_ALIGNED(length, sizeof(uintptr_t)), "Ring unit size must be aligned to uintptr_t size"); \


/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
static inline size_t _Ring_Length(const Ring_T * p_ring) { return p_ring->Type.LENGTH; }
static inline size_t _Ring_Mask(const Ring_T * p_ring) { return _RING_POW2_DEF(p_ring->Type.POW2_MASK, 0U); }
// _RING_POW2_DEF(return p_ring->Type.POW2_MASK, (void)p_ring; return 0U); // if unused parameter warning
static inline size_t _Ring_IndexMaskedOf(const Ring_T * p_ring, size_t index) { return _RING_POW2_DEF((index & p_ring->Type.POW2_MASK), 0U); }

static inline size_t _Ring_IndexWrapOf(const Ring_T * p_ring, size_t index)
{
    return _RING_POW2_DEF(_Ring_IndexMaskedOf(p_ring, index), (index % p_ring->Type.LENGTH));
    // _RING_POW2_DEF(return _Ring_IndexMaskedOf(p_ring, index);)
    // _RING_NON_POW2_DEF(return (index % p_ring->Type.LENGTH);)
}

/* on storage */
static inline size_t _Ring_IndexIncOf(const Ring_T * p_ring, size_t index, size_t inc)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    (void)p_ring;
    return index + inc;
#elif   defined(RING_INDEX_POW2_WRAP)
    return _Ring_IndexMaskedOf(p_ring, index + inc);
#elif defined(RING_INDEX_LENGTH_COMPARE)
    return (index + inc >= p_ring->Type.LENGTH) ? index + inc - p_ring->Type.LENGTH : index + inc;
#endif
}

static inline size_t _Ring_IndexDecOf(const Ring_T * p_ring, size_t index, size_t dec)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    (void)p_ring;
    return index - dec;
#elif   defined(RING_INDEX_POW2_WRAP)
    return _Ring_IndexMaskedOf(p_ring, index - dec);
#elif   defined(RING_INDEX_LENGTH_COMPARE)
    return ((int32_t)index - (int32_t)dec < 0) ? p_ring->Type.LENGTH + index - dec : index - dec;
#endif
}

static inline size_t _Ring_ArrayIndexOf(const Ring_T * p_ring, size_t ringIndex)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    return _Ring_IndexMaskedOf(p_ring, ringIndex);
#elif   defined(RING_INDEX_POW2_WRAP) || defined(RING_INDEX_LENGTH_COMPARE)
    return ringIndex;
#endif
}


/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
/*
    RING_INDEX_POW2_COUNTER
    Max usable capacity is length

    RING_INDEX_POW2_WRAP, RING_INDEX_LENGTH_COMPARE
    Empty space detection method. Tail always points to empty space. Max usable capacity is length - 1
*/
static inline size_t Ring_GetFullCount(const Ring_T * p_ring)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    return (p_ring->Tail - p_ring->Head);
#elif   defined(RING_INDEX_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, p_ring->Tail - p_ring->Head);
#elif   defined(RING_INDEX_LENGTH_COMPARE)
    return (p_ring->Tail >= p_ring->Head) ? (p_ring->Tail - p_ring->Head) : (p_ring->Type.LENGTH - p_ring->Head + p_ring->Tail);
#endif
}

/*
    later 2 cases: -1 to account for 1 space used for detection
*/
static inline size_t Ring_GetEmptyCount(const Ring_T * p_ring)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    return p_ring->Type.LENGTH + p_ring->Head - p_ring->Tail;
#elif   defined(RING_INDEX_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, p_ring->Type.LENGTH + p_ring->Head - p_ring->Tail - 1U);
#elif   defined(RING_INDEX_LENGTH_COMPARE)
    return (p_ring->Tail >= p_ring->Head) ? (p_ring->Type.LENGTH + p_ring->Head - p_ring->Tail - 1U) : (p_ring->Head - p_ring->Tail - 1U);
#endif
}

static inline bool Ring_IsEmpty(const Ring_T * p_ring)
{
    return (p_ring->Tail == p_ring->Head);
}

static inline bool Ring_IsFull(const Ring_T * p_ring)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    return (Ring_GetFullCount(p_ring) == p_ring->Type.LENGTH);
#elif   defined(RING_INDEX_POW2_WRAP) || defined(RING_INDEX_LENGTH_COMPARE)
    return (_Ring_IndexIncOf(p_ring, p_ring->Tail, 1U) == p_ring->Head);
#endif
}

// #include "_RingT.h"

// //todo as wrap around RingT
// static inline size_t Ring_GetFullCount(const Ring_T * p_ring)
// {
//     return RingT_GetFullCount(p_ring->Type, p_ring);
// }

// // todo with context call ringT
// static inline size_t Ring_Context_GetFullCount(const Ring_Context_T * p_ring)
// {
//     return RingT_GetFullCount(p_ring->TYPE, p_ring->P_STATE);
// }

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



