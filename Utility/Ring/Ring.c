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

#if defined(CONFIG_RING_MULTITHREADED_ENABLE)
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
#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
    if(p_ring.CONST.USE_CRITICAL == true) { Critical_Enter(); }
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE)
    (void)p_ring;
#endif
}

static inline void ExitCritical(const Ring_T * p_ring)
{
#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
    if(p_ring.CONST.USE_CRITICAL == true) { Critical_Exit(); }
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE)
    (void)p_ring;
#endif
}

static inline bool AcquireMutex(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
    return (p_ring.CONST.USE_CRITICAL == true) ? Critical_AcquireMutex(&p_ring->Mutex) : true;
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE)
    (void)p_ring;
    return true;
#endif
}

static inline void ReleaseMutex(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
    if(p_ring.CONST.USE_CRITICAL == true) { Critical_ReleaseMutex(&p_ring->Mutex) };
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE)
    (void)p_ring;
#endif
}

static inline bool AcquireCritical(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
    return (p_ring.CONST.USE_CRITICAL == true) ? Critical_AcquireEnter(&p_ring->Mutex) : true;
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE)
    (void)p_ring;
    return true;
#endif
}

static inline void ReleaseCritical(Ring_T * p_ring)
{
#if     defined(CONFIG_RING_MULTITHREADED_ENABLE)
    if(p_ring.CONST.USE_CRITICAL == true) { Critical_ReleaseExit(&p_ring->Mutex) };
#elif   defined(CONFIG_RING_MULTITHREADED_DISABLE)
    (void)p_ring;
#endif
}

/******************************************************************************/
/*!
    Private Common Ring Buffer Functions
    No input boundary checking, expected in calling function
*/
/******************************************************************************/
static inline void * PtrOf(const void * p_units, size_t unitSize, size_t unitIndex) { return ((uint8_t *)p_units + (unitIndex * unitSize)); }

static inline size_t IndexOf(const Ring_T * p_ring, size_t index)
{
#if     defined(CONFIG_RING_LENGTH_POW2_INDEX_UNBOUNDED)
    return _Ring_IndexWrappedOf(p_ring, index);
#elif   defined(CONFIG_RING_LENGTH_POW2_INDEX_WRAPPED) || defined(CONFIG_RING_LENGTH_ANY)
    return index;
#endif
}

static inline void * GetPtrFront(const Ring_T * p_ring)              { return PtrOf(p_ring->CONST.P_BUFFER, p_ring->CONST.UNIT_SIZE, IndexOf(p_ring, p_ring->Tail)); }
static inline void * GetPtrBack(const Ring_T * p_ring)               { return PtrOf(p_ring->CONST.P_BUFFER, p_ring->CONST.UNIT_SIZE, IndexOf(p_ring, p_ring->Head)); }
static inline void * GetPtrAt(const Ring_T * p_ring, size_t index)   { return PtrOf(p_ring->CONST.P_BUFFER, p_ring->CONST.UNIT_SIZE, IndexOf(p_ring, p_ring->Tail + index)); }

static inline void UnitCopy(void * p_dest, const void * p_source, size_t unitSize)
{
    switch(unitSize)
    {
        case sizeof(uint32_t): *((uint32_t *)p_dest) = *((uint32_t *)p_source); break;
        case sizeof(uint16_t): *((uint16_t *)p_dest) = *((uint16_t *)p_source); break;
        case sizeof(uint8_t) : *((uint8_t  *)p_dest) = *((uint8_t  *)p_source); break;
        default: memcpy(p_dest, p_source, unitSize); break;
    }
}

static inline void PeekFront(const Ring_T * p_ring, void * p_dest)                  { UnitCopy(p_dest, GetPtrFront(p_ring), p_ring->CONST.UNIT_SIZE); }
static inline void PeekBack(const Ring_T * p_ring, void * p_dest)                   { UnitCopy(p_dest, GetPtrBack(p_ring), p_ring->CONST.UNIT_SIZE); }
static inline void PeekIndex(const Ring_T * p_ring, void * p_dest, size_t index)    { UnitCopy(p_dest, GetPtrAt(p_ring, index), p_ring->CONST.UNIT_SIZE); }
static inline void PlaceFront(Ring_T * p_ring, const void * p_unit)                 { UnitCopy(GetPtrFront(p_ring), p_unit, p_ring->CONST.UNIT_SIZE); }
static inline void PlaceBack(Ring_T * p_ring, const void * p_unit)                  { UnitCopy(GetPtrBack(p_ring), p_unit, p_ring->CONST.UNIT_SIZE); }
static inline void RemoveFront(Ring_T * p_ring, size_t unitCount)                   { p_ring->Tail = _Ring_IndexIncOf(p_ring, p_ring->Tail, unitCount); }
static inline void RemoveBack(Ring_T * p_ring, size_t unitCount)                    { p_ring->Head = _Ring_IndexDecOf(p_ring, p_ring->Head, unitCount); }
static inline void AddFront(Ring_T * p_ring, size_t unitCount)                      { p_ring->Tail = _Ring_IndexDecOf(p_ring, p_ring->Tail, unitCount); }
static inline void AddBack(Ring_T * p_ring, size_t unitCount)                       { p_ring->Head = _Ring_IndexIncOf(p_ring, p_ring->Head, unitCount); }
static inline void Enqueue(Ring_T * p_ring, const void * p_unit)                    { PlaceBack(p_ring, p_unit); AddBack(p_ring, 1U); }
static inline void Dequeue(Ring_T * p_ring, void * p_dest)                          { PeekFront(p_ring, p_dest); RemoveFront(p_ring, 1U); }
static inline void PushFront(Ring_T * p_ring, const void * p_unit)                  { AddFront(p_ring, 1U); PlaceFront(p_ring, p_unit); }
static inline void PopBack(Ring_T * p_ring, void * p_dest)                          { RemoveBack(p_ring, 1U); PeekBack(p_ring, p_dest); }
static inline void Seek(Ring_T * p_ring, void * p_dest, size_t index)               { RemoveFront(p_ring, index); PeekFront(p_ring, p_dest); }
static inline void * SeekPtr(Ring_T * p_ring, size_t index)                         { RemoveFront(p_ring, index); return GetPtrFront(p_ring); }



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
    p_ring->Head = 0U;
    p_ring->Tail = 0U;
}

bool Ring_Enqueue(Ring_T * p_ring, const void * p_unit)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(Ring_IsFull(p_ring) == false) { Enqueue(p_ring, p_unit); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_Dequeue(Ring_T * p_ring, void * p_dest)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(Ring_IsEmpty(p_ring) == false) { Dequeue(p_ring, p_dest); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_EnqueueN(Ring_T * p_ring, const void * p_units, size_t unitCount)
{
    bool isSuccess = false;
    if(AcquireCritical(p_ring) == true)
    {
        if(unitCount <= Ring_GetEmptyCount(p_ring))
        {
            for(size_t iUnit = 0U; iUnit < unitCount; iUnit++) { Enqueue(p_ring, PtrOf(p_units, p_ring->CONST.UNIT_SIZE, iUnit)); }
            isSuccess = true;
        }
        ReleaseCritical(p_ring);
    }
    return isSuccess;
}

bool Ring_DequeueN(Ring_T * p_ring, void * p_dest, size_t unitCount)
{
    bool isSuccess = false;
    if(AcquireCritical(p_ring) == true)
    {
        if(unitCount <= Ring_GetFullCount(p_ring))
        {
            for(size_t iUnit = 0U; iUnit < unitCount; iUnit++) { Dequeue(p_ring, PtrOf(p_dest, p_ring->CONST.UNIT_SIZE, iUnit)); }
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
    if(AcquireCritical(p_ring) == true)
    {
        count = Ring_GetEmptyCount(p_ring);
        if(count > unitCount) { count = unitCount; }
        for(size_t iCount = 0U; iCount < count; iCount++) { Enqueue(p_ring, PtrOf(p_units, p_ring->CONST.UNIT_SIZE, iCount)); }
        ReleaseCritical(p_ring);
    }
    return count;
}

/*

*/
size_t Ring_DequeueMax(Ring_T * p_ring, void * p_dest, size_t unitCount)
{
    size_t count = 0U;
    if(AcquireCritical(p_ring) == true)
    {
        count = Ring_GetFullCount(p_ring);
        if(count > unitCount) { count = unitCount; }
        for(size_t iCount = 0U; iCount < count; iCount++) { Dequeue(p_ring, PtrOf(p_dest, p_ring->CONST.UNIT_SIZE, iCount)); }
        ReleaseCritical(p_ring);
    }
    return count;
}

bool Ring_PushFront(Ring_T * p_ring, const void * p_unit)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(Ring_IsFull(p_ring) == false) { PushFront(p_ring, p_unit); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PopBack(Ring_T * p_ring, void * p_dest)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(Ring_IsEmpty(p_ring) == false) { PopBack(p_ring, p_dest); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekFront(Ring_T * p_ring, void * p_dest)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(Ring_IsEmpty(p_ring) == false) { PeekFront(p_ring, p_dest); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekBack(Ring_T * p_ring, void * p_dest)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(Ring_IsEmpty(p_ring) == false) { PeekBack(p_ring, p_dest); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekIndex(Ring_T * p_ring, void * p_dest, size_t index)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(index < Ring_GetFullCount(p_ring)) { PeekIndex(p_ring, p_dest, index); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_Seek(Ring_T * p_ring, void * p_dest, size_t index)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(index < Ring_GetFullCount(p_ring)) { Seek(p_ring, p_dest, index); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_RemoveFront(Ring_T * p_ring, size_t unitCount)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(unitCount <= Ring_GetFullCount(p_ring)) { RemoveFront(p_ring, unitCount); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_RemoveBack(Ring_T * p_ring, size_t unitCount)
{
    bool isSuccess = false;
    EnterCritical(p_ring);
    if(unitCount <= Ring_GetFullCount(p_ring)) { RemoveBack(p_ring, unitCount); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

/******************************************************************************/
/*!
    Experimental
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
/*
    When a pointer is returned, there is no guarantee that the data will not be overwritten, unless the Ring is locked, or single threaded.
*/
void * Ring_PeekPtrFront(Ring_T * p_ring)                   { return (Ring_IsEmpty(p_ring) == false) ? GetPtrFront(p_ring) : NULL; }
void * Ring_PeekPtrBack(Ring_T * p_ring)                    { return (Ring_IsEmpty(p_ring) == false) ? GetPtrBack(p_ring) : NULL; }
void * Ring_PeekPtrIndex(Ring_T * p_ring, size_t index)     { return (index < Ring_GetFullCount(p_ring)) ? GetPtrAt(p_ring, index) : NULL; }
void * Ring_SeekPtr(Ring_T * p_ring, size_t index)          { return (index < Ring_GetFullCount(p_ring)) ? SeekPtr(p_ring, index) : NULL; }

/*
    Give caller exclusive buffer write. Will flush buffer to start from index 0
*/
void * Ring_AcquireBuffer(Ring_T * p_ring)
{
    void * p_buffer = 0U;

    if(AcquireMutex(p_ring) == true)
    {
        Ring_Clear(p_ring);
        p_buffer = p_ring->CONST.P_BUFFER;
    }

    return p_buffer;
}

void Ring_ReleaseBuffer(Ring_T * p_ring, size_t writeSize)
{
    p_ring->Head = writeSize;
    ReleaseMutex(p_ring);
}

#ifdef CONFIG_RING_DYNAMIC_MEMORY_ALLOCATION
Ring_T * Ring_New(size_t unitCount, size_t unitSize)
{
    Ring_T * p_ring = malloc(sizeof(Ring_T));
    void * p_buffer = malloc(unitCount * unitSize);

    if(p_ring != 0U && p_buffer != 0U)
    {
        *(void *)&p_ring->CONST.P_BUFFER = p_buffer; /* bypass const typedef */
        *(size_t)&p_ring->CONST.LENGTH = unitCount;
        *(size_t)&p_ring->CONST.UNIT_SIZE = unitSize;
        Ring_Init(p_ring);
    }

    return p_ring;
}

void Ring_Free(Ring_T * p_ring)
{
    free(p_ring);
}
#endif
