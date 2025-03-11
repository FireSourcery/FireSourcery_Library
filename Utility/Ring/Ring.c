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
    @file   Ring.c
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#include "Ring.h"

#include "Type/Array/void_array.h"

#if defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
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
    Optionally implement Critical at Ring Buffer instance level.
    Alternatively, Critical can be implement in upper layer.
*/
/******************************************************************************/
static inline void EnterCritical(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    _Critical_DisableIrq();
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

static inline void ExitCritical(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    _Critical_EnableIrq();
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

/* Multithreaded may use disable interrupts, if-test, or spin-wait with thread scheduler */
static inline bool AcquireSignal(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    return Critical_AcquireSignal(&p_ring->Mutex);
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
    return true;
#endif
}

static inline void ReleaseSignal(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    Critical_ReleaseSignal(&p_ring->Mutex);
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

static inline bool AcquireCritical(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    return Critical_AcquireEnter(&p_ring->Mutex);
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
    return true;
#endif
}

static inline void ReleaseCritical(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_LOCAL_CRITICAL_ENABLE)
    Critical_ReleaseExit(&p_ring->Mutex);
#elif   defined(CONFIG_RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

/******************************************************************************/
/*!
    Private
    Handle data and index.
    No input boundary checking.
*/
/******************************************************************************/
/* Effective index */
static inline size_t IndexOf(const Ring_T * p_ring, size_t index)
{
#if     defined(CONFIG_RING_POW2_MASK)
    return _Ring_IndexWrapOf(p_ring, index);
#elif   defined(CONFIG_RING_POW2_WRAP) || defined(CONFIG_RING_LENGTH_COMPARE)
    return index;
#endif
}

static inline void * _At(const Ring_T * p_ring, size_t index) { return void_pointer_at(p_ring->CONST.P_BUFFER, p_ring->CONST.UNIT_SIZE, index); }
static inline void * Front(const Ring_T * p_ring)             { return _At(p_ring, IndexOf(p_ring, p_ring->Head)); }
static inline void * Back(const Ring_T * p_ring)              { return _At(p_ring, IndexOf(p_ring, p_ring->Tail)); }
static inline void * At(const Ring_T * p_ring, size_t index)  { return _At(p_ring, IndexOf(p_ring, p_ring->Head + index)); }

/* Array Get */
static inline void PeekFront(const Ring_T * p_ring, void * p_result)                { void_copy(p_result, Front(p_ring), p_ring->CONST.UNIT_SIZE); }
static inline void PeekBack(const Ring_T * p_ring, void * p_result)                 { void_copy(p_result, Back(p_ring), p_ring->CONST.UNIT_SIZE); }
static inline void PeekIndex(const Ring_T * p_ring, size_t index, void * p_result)  { void_copy(p_result, At(p_ring, index), p_ring->CONST.UNIT_SIZE); }
/* Array Set */
static inline void PlaceFront(const Ring_T * p_ring, const void * p_unit)           { void_copy(Front(p_ring), p_unit, p_ring->CONST.UNIT_SIZE); }
static inline void PlaceBack(const Ring_T * p_ring, const void * p_unit)            { void_copy(Back(p_ring), p_unit, p_ring->CONST.UNIT_SIZE); }

/* Seek/Inc/Dec Indexs */
// static inline void _SeekFront(Ring_T * p_ring, size_t index)                       { p_ring->Head = _Ring_IndexIncOf(p_ring, p_ring->Head, index); }
// static inline void _SeekBack(Ring_T * p_ring, size_t count)                        { p_ring->Tail = _Ring_IndexDecOf(p_ring, p_ring->Tail, count); }
static inline void AddFront(Ring_T * p_ring, size_t count)                           { p_ring->Head = _Ring_IndexDecOf(p_ring, p_ring->Head, count); }
static inline void RemoveFront(Ring_T * p_ring, size_t count)                        { p_ring->Head = _Ring_IndexIncOf(p_ring, p_ring->Head, count); }
static inline void AddBack(Ring_T * p_ring, size_t count)                            { p_ring->Tail = _Ring_IndexIncOf(p_ring, p_ring->Tail, count); }
static inline void RemoveBack(Ring_T * p_ring, size_t count)                         { p_ring->Tail = _Ring_IndexDecOf(p_ring, p_ring->Tail, count); }
/* Combined */
static inline void PushFront(Ring_T * p_ring, const void * p_unit)                  { AddFront(p_ring, 1U); PlaceFront(p_ring, p_unit); }
static inline void PopFront(Ring_T * p_ring, void * p_result)                       { PeekFront(p_ring, p_result); RemoveFront(p_ring, 1U); }
static inline void PushBack(Ring_T * p_ring, const void * p_unit)                   { PlaceBack(p_ring, p_unit); AddBack(p_ring, 1U); }
static inline void PopBack(Ring_T * p_ring, void * p_result)                        { RemoveBack(p_ring, 1U); PeekBack(p_ring, p_result); }
// static inline void * _PopFront(Ring_T * p_ring)                                     { void * p_front = Front(p_ring); RemoveFront(p_ring, 1U); return p_front; }
// static inline void * _PopBack(Ring_T * p_ring)                                      { void * p_back = Back(p_ring); RemoveBack(p_ring, 1U); return p_back; }
/*  */
static inline void * SeekPtr(Ring_T * p_ring, size_t index)                         { RemoveFront(p_ring, index); return Front(p_ring); }


/******************************************************************************/
/*!
    Protected
    Handle input boundary checking.
    Caller handle lock
    @return a pointer to buffer data.
        concurrent push/pop may overwrite
        Caller handle lock, or single threaded.
*/
/******************************************************************************/
/*
    Peek Ptr
*/
inline void * _Ring_Front(const Ring_T * p_ring)               { return (Ring_IsEmpty(p_ring) == false) ? Front(p_ring) : NULL; }
inline void * _Ring_Back(const Ring_T * p_ring)                { return (Ring_IsEmpty(p_ring) == false) ? Back(p_ring) : NULL; }
inline void * _Ring_At(const Ring_T * p_ring, size_t index)    { return (index < Ring_GetFullCount(p_ring)) ? At(p_ring, index) : NULL; }
inline void * _Ring_Seek(Ring_T * p_ring, size_t index)        { return (index < Ring_GetFullCount(p_ring)) ? SeekPtr(p_ring, index) : NULL; }

// #define RING_PEEK_AS(T, p_Ring, index) (*(T*)_Ring_At(p_Ring, index))

// inline void _Ring_PushBack(Ring_T * p_ring, const void * p_unit)    { if (Ring_IsFull(p_ring) == false) { PushBack(p_ring, p_unit); } }
// inline void _Ring_PushFront(Ring_T * p_ring, const void * p_unit)   { if (Ring_IsFull(p_ring) == false) { PushFront(p_ring, p_unit); } }
// inline void * _Ring_PopFront(Ring_T * p_ring)                       { return (Ring_IsEmpty(p_ring) == false) ? (_PopFront(p_ring)) : NULL; }
// inline void * _Ring_PopBack(Ring_T * p_ring)                        { return (Ring_IsEmpty(p_ring) == false) ? (_PopBack(p_ring)) : NULL; }


/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void Ring_Init(Ring_T * p_ring)
{
    Ring_Clear(p_ring);
}

void Ring_Clear(Ring_T * p_ring)
{
    p_ring->Tail = 0U;
    p_ring->Head = 0U;
}

bool Ring_PushBack(Ring_T * p_ring, const void * p_unit)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsFull(p_ring) == false) { PushBack(p_ring, p_unit); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PopFront(Ring_T * p_ring, void * p_result)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PopFront(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PushFront(Ring_T * p_ring, const void * p_unit)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsFull(p_ring) == false) { PushFront(p_ring, p_unit); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PopBack(Ring_T * p_ring, void * p_result)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PopBack(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

inline bool Ring_Enqueue(Ring_T * p_ring, const void * p_unit)  { return Ring_PushBack(p_ring, p_unit); }
inline bool Ring_Dequeue(Ring_T * p_ring, void * p_result)      { return Ring_PopFront(p_ring, p_result); }

bool Ring_RemoveFront(Ring_T * p_ring, size_t unitCount)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (unitCount <= Ring_GetFullCount(p_ring)) { RemoveFront(p_ring, unitCount); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_RemoveBack(Ring_T * p_ring, size_t unitCount)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (unitCount <= Ring_GetFullCount(p_ring)) { RemoveBack(p_ring, unitCount); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_EnqueueN(Ring_T * p_ring, const void * p_units, size_t unitCount)
{
    bool isSuccess = false;
    if (AcquireCritical(p_ring) == true)
    {
        if (unitCount <= Ring_GetEmptyCount(p_ring))
        {
            for (size_t iUnit = 0U; iUnit < unitCount; ++iUnit) { PushBack(p_ring, void_pointer_at(p_units, p_ring->CONST.UNIT_SIZE, iUnit)); }
            isSuccess = true;
        }
        ReleaseCritical(p_ring);
    }
    return isSuccess;
}

bool Ring_DequeueN(Ring_T * p_ring, void * p_result, size_t unitCount)
{
    bool isSuccess = false;
    if (AcquireCritical(p_ring) == true)
    {
        if (unitCount <= Ring_GetFullCount(p_ring))
        {
            for (size_t iUnit = 0U; iUnit < unitCount; ++iUnit) { PopFront(p_ring, void_pointer_at(p_result, p_ring->CONST.UNIT_SIZE, iUnit)); }
            isSuccess = true;
        }
        ReleaseCritical(p_ring);
    }
    return isSuccess;
}

/*

*/
size_t Ring_EnqueueMax(Ring_T * p_ring, const void * p_units, size_t unitCount)
{
    size_t count = 0U;
    if (AcquireCritical(p_ring) == true)
    {
        count = Ring_GetEmptyCount(p_ring);
        if (count > unitCount) { count = unitCount; }
        for (size_t iUnit = 0U; iUnit < count; ++iUnit) { PushBack(p_ring, void_pointer_at(p_units, p_ring->CONST.UNIT_SIZE, iUnit)); }
        ReleaseCritical(p_ring);
    }
    return count;
}

/*

*/
size_t Ring_DequeueMax(Ring_T * p_ring, void * p_result, size_t unitCount)
{
    size_t count = 0U;
    if (AcquireCritical(p_ring) == true)
    {
        count = Ring_GetFullCount(p_ring);
        if (count > unitCount) { count = unitCount; }
        for (size_t iUnit = 0U; iUnit < count; ++iUnit) { PopFront(p_ring, void_pointer_at(p_result, p_ring->CONST.UNIT_SIZE, iUnit)); }
        ReleaseCritical(p_ring);
    }
    return count;
}


bool Ring_PeekFront(Ring_T * p_ring, void * p_result)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PeekFront(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekBack(Ring_T * p_ring, void * p_result)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PeekBack(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekAt(Ring_T * p_ring, size_t index, void * p_result)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if (index < Ring_GetFullCount(p_ring)) { PeekIndex(p_ring, index, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

