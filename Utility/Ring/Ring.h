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
    @version V0
*/
/******************************************************************************/
#ifndef RING_UTILITY_H
#define RING_UTILITY_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef const struct Ring_Const
{
    const size_t UNIT_SIZE;     /* Bytes */
    const size_t LENGTH;        /* In UNIT_SIZE counts (NOT bytes) */
#if defined(CONFIG_RING_POW2_MASK) || defined(CONFIG_RING_POW2_WRAP)
    const size_t POW2_MASK;     /* Index Mask */
#endif
    void * const P_BUFFER; /* P_STATE */
}
Ring_Const_T;

typedef struct Ring
{
    const Ring_Const_T CONST;
    volatile size_t Head;    /* FIFO Out/Front. */
    volatile size_t Tail;    /* FIFO In/Back. */
#if defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    volatile critical_signal_t Mutex;
#endif
}
Ring_T;

#if     defined(CONFIG_RING_POW2_MASK) || defined(CONFIG_RING_POW2_WRAP)
#define _RING_INIT_POW2(Pow2Mask) .POW2_MASK = Pow2Mask,
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
#define _RING_INIT_POW2(Pow2Mask)
#endif

#define RING_INIT(p_Buffer, UnitSize, Length, ...)  \
{                                                           \
    .CONST =                                                \
    {                                                       \
        .P_BUFFER       = p_Buffer,                         \
        .LENGTH         = Length,                           \
        .UNIT_SIZE      = UnitSize,                         \
        _RING_INIT_POW2(Length - 1U)                        \
    },                                                      \
}

#define RING_ALLOC(UnitSize, Length, ...) RING_INIT((uint8_t[(Length * UnitSize)]){ }, UnitSize, Length, __VA_ARGS__)
#define RING_AS(TYPE, Length, ...) RING_INIT((TYPE[Length]){ }, sizeof(TYPE), Length, __VA_ARGS__)


/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
// static inline size_t _Ring_Length(const Ring_T * p_ring) { return p_ring->CONST.LENGTH; }
// static inline size_t _Ring_Mask(const Ring_T * p_ring) { return p_ring->CONST.POW2_MASK; }

static inline size_t _Ring_IndexWrapOf(const Ring_T * p_ring, size_t index)
{
#if     defined(CONFIG_RING_POW2_MASK) || defined(CONFIG_RING_POW2_WRAP)
    return (index & p_ring->CONST.POW2_MASK);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return (index % p_ring->CONST.LENGTH);
#endif
}

static inline size_t _Ring_IndexIncOf(const Ring_T * p_ring, size_t index, size_t inc)
{
#if     defined(CONFIG_RING_POW2_MASK)
    (void)p_ring;
    return index + inc;
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, index + inc);
#elif defined(CONFIG_RING_LENGTH_COMPARE)
    return (index + inc > p_ring->CONST.LENGTH) ? index + inc - p_ring->CONST.LENGTH : index + inc;
#endif
}

static inline size_t _Ring_IndexDecOf(const Ring_T * p_ring, size_t index, size_t dec)
{
#if     defined(CONFIG_RING_POW2_MASK)
    (void)p_ring;
    return index - dec;
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, index - dec);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return ((int32_t)index - dec < 0) ? p_ring->CONST.LENGTH + index - dec : index - dec;
#endif
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
/*
    CONFIG_RING_POW2_MASK
    Max usable capacity is length

    CONFIG_RING_POW2_WRAP, CONFIG_RING_LENGTH_COMPARE
    Empty space detection method. Tail always points to empty space. Max usable capacity is length - 1
*/
/*
    later 2 cases: returns max of buffer length - 1, to account for 1 space used for detection
*/
static inline size_t Ring_GetFullCount(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_POW2_MASK)
    return (p_ring->Tail - p_ring->Head);
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, p_ring->Tail - p_ring->Head);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return (p_ring->Tail >= p_ring->Head) ? (p_ring->Tail - p_ring->Head) : (p_ring->CONST.LENGTH - p_ring->Head + p_ring->Tail);
#endif
}

/*
    later 2 cases: -1 to account for 1 space used for detection
*/
static inline size_t Ring_GetEmptyCount(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_POW2_MASK)
    return p_ring->CONST.LENGTH + p_ring->Head - p_ring->Tail;
#elif   defined(CONFIG_RING_POW2_WRAP)
    return _Ring_IndexWrapOf(p_ring, p_ring->CONST.LENGTH + p_ring->Head - p_ring->Tail - 1U);
#elif   defined(CONFIG_RING_LENGTH_COMPARE)
    return (p_ring->Tail >= p_ring->Head) ? (p_ring->CONST.LENGTH + p_ring->Head - p_ring->Tail - 1U) : (p_ring->Head - p_ring->Tail - 1U);
#endif
}

static inline bool Ring_IsEmpty(const Ring_T * p_ring)
{
    return (p_ring->Tail == p_ring->Head);
}

static inline bool Ring_IsFull(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_POW2_MASK)
    return (Ring_GetFullCount(p_ring) == p_ring->CONST.LENGTH);
#elif   defined(CONFIG_RING_POW2_WRAP) || defined(CONFIG_RING_LENGTH_COMPARE)
    return (_Ring_IndexIncOf(p_ring, p_ring->Tail, 1U) == p_ring->Head);
#endif
}

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void * _Ring_Front(const Ring_T * p_ring);
extern void * _Ring_Back(const Ring_T * p_ring);
extern void * _Ring_At(const Ring_T * p_ring, size_t index);
extern void * _Ring_Seek(Ring_T * p_ring, size_t index);
// extern bool _Ring_PeekAt(const Ring_T * p_ring, size_t index, void * p_result);

extern void Ring_Init(Ring_T * p_ring);
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

extern bool Ring_DequeueN(Ring_T * p_ring, void * p_result, size_t unitCount); // move p_result to last?
extern size_t Ring_DequeueMax(Ring_T * p_ring, void * p_result, size_t unitCount);


#endif



