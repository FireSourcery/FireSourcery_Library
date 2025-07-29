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

    @brief
*/
/******************************************************************************/
#include "Ring.h"


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
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    _Critical_DisableIrq();
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

static inline void ExitCritical(const Ring_T * p_ring)
{
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    _Critical_EnableIrq();
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

/* Multithreaded may use disable interrupts, if-test, or spin-wait with thread scheduler */
static inline bool AcquireSignal(Ring_T * p_ring)
{
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    return Critical_AcquireLock(&p_ring->Lock);
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
    return true;
#endif
}

static inline void ReleaseSignal(Ring_T * p_ring)
{
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    Critical_ReleaseLock(&p_ring->Lock);
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
#endif
}

static inline bool AcquireCritical(Ring_T * p_ring)
{
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    return Critical_AcquireEnter(&p_ring->Lock);
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
    (void)p_ring;
    return true;
#endif
}

static inline void ReleaseCritical(Ring_T * p_ring)
{
#if     defined(RING_LOCAL_CRITICAL_ENABLE)
    Critical_ReleaseExit(&p_ring->Lock);
#elif   defined(RING_LOCAL_CRITICAL_DISABLE)
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
/*
    Buffer Access
    Shrothand wrapper
    abstract away UNIT_SIZE
*/
static inline void * PtrOf(const Ring_T * p_ring, size_t arrayIndex) { return ((uint8_t *)p_ring->Buffer + (p_ring->Type.UNIT_SIZE * arrayIndex)); }
static inline void Copy(const Ring_T * p_ring, void * p_dest, const void * p_src) { void_copy(p_dest, p_src, p_ring->Type.UNIT_SIZE); }
/* value array only */
// static inline intptr_t ValueOf(const Ring_T * p_ring, size_t arrayIndex) { return void_array_get(p_ring->P_BUFFER, p_ring->Type.UNIT_SIZE, arrayIndex); }

/* Effective array index of virtual index */
static inline size_t IndexOf(const Ring_T * p_ring, size_t ringIndex)
{
#if     defined(RING_INDEX_POW2_COUNTER)
    return _Ring_IndexWrapOf(p_ring, ringIndex);
#elif   defined(RING_INDEX_POW2_WRAP) || defined(RING_INDEX_LENGTH_COMPARE)
    return ringIndex;
#endif
}

// static inline void * PtrOf(const Ring_T * p_ring, size_t ringIndex) { return PtrOf(p_ring, IndexOf(p_ring, ringIndex)); }
// static inline void * Head(const Ring_T * p_ring) { return PtrOf(p_ring, p_ring->Head); }
// static inline void * Tail(const Ring_T * p_ring) { return PtrOf(p_ring, p_ring->Tail); }

/* Virtual access. Map virtual index to array index */
static inline size_t IndexHead(const Ring_T * p_ring)                       { return IndexOf(p_ring, p_ring->Head); }
static inline size_t IndexTail(const Ring_T * p_ring)                       { return IndexOf(p_ring, p_ring->Tail); }
static inline void * Head(const Ring_T * p_ring)                            { return PtrOf(p_ring, IndexHead(p_ring)); }
static inline void * Tail(const Ring_T * p_ring)                            { return PtrOf(p_ring, IndexTail(p_ring)); }
static inline void PeekHead(const Ring_T * p_ring, void * p_result)         { Copy(p_ring, p_result, Head(p_ring)); }
static inline void PeekTail(const Ring_T * p_ring, void * p_result)         { Copy(p_ring, p_result, Tail(p_ring)); }
static inline void PlaceHead(const Ring_T * p_ring, const void * p_unit)    { Copy(p_ring, Head(p_ring), p_unit); }
static inline void PlaceTail(const Ring_T * p_ring, const void * p_unit)    { Copy(p_ring, Tail(p_ring), p_unit); }

/* Seek/Inc/Dec Indexs */
static inline void AddFront(Ring_T * p_ring, size_t count)                          { p_ring->Head = _Ring_IndexDecOf(p_ring, p_ring->Head, count); }
static inline void RemoveFront(Ring_T * p_ring, size_t count)                       { p_ring->Head = _Ring_IndexIncOf(p_ring, p_ring->Head, count); }
static inline void AddBack(Ring_T * p_ring, size_t count)                           { p_ring->Tail = _Ring_IndexIncOf(p_ring, p_ring->Tail, count); }
static inline void RemoveBack(Ring_T * p_ring, size_t count)                        { p_ring->Tail = _Ring_IndexDecOf(p_ring, p_ring->Tail, count); }

/* Interface preliminary */
static inline void PushFront(Ring_T * p_ring, const void * p_unit)                  { AddFront(p_ring, 1U); PlaceHead(p_ring, p_unit); }
static inline void PushBack(Ring_T * p_ring, const void * p_unit)                   { PlaceTail(p_ring, p_unit); AddBack(p_ring, 1U); }

static inline void PopFront(Ring_T * p_ring, void * p_result)                       { PeekHead(p_ring, p_result); RemoveFront(p_ring, 1U); }
static inline void PopBack(Ring_T * p_ring, void * p_result)                        { RemoveBack(p_ring, 1U); PeekTail(p_ring, p_result); }

/* test */
static inline void * _PopFront(Ring_T * p_ring)                                     { void * p_front = Head(p_ring); RemoveFront(p_ring, 1U); return p_front; }
static inline void * _PopBack(Ring_T * p_ring)                                      { RemoveBack(p_ring, 1U); return Tail(p_ring); }

/*  */
static inline void * Seek(Ring_T * p_ring, size_t index)                            { RemoveFront(p_ring, index); return Head(p_ring); }

/*
    Additional
*/
static inline size_t IndexAt(const Ring_T * p_ring, size_t index)                       { return IndexOf(p_ring, _Ring_IndexIncOf(p_ring, p_ring->Head, index)); }
static inline size_t IndexFront(const Ring_T * p_ring)                                  { return IndexOf(p_ring, p_ring->Head); }
static inline size_t IndexBack(const Ring_T * p_ring)                                   { return IndexOf(p_ring, _Ring_IndexDecOf(p_ring, p_ring->Tail, 1)); } /* Tail - 1U */
static inline size_t IndexBackWrite(const Ring_T * p_ring)                              { return IndexOf(p_ring, p_ring->Tail); }
static inline size_t IndexFrontWrite(const Ring_T * p_ring)                             { return IndexOf(p_ring, _Ring_IndexDecOf(p_ring, p_ring->Head, 1)); }
static inline void * At(const Ring_T * p_ring, size_t index)                            { return PtrOf(p_ring, IndexAt(p_ring, index)); }
static inline void * Front(const Ring_T * p_ring)                                       { return PtrOf(p_ring, IndexFront(p_ring)); }
static inline void * Back(const Ring_T * p_ring)                                        { return PtrOf(p_ring, IndexBack(p_ring)); }
static inline void PeekAt(const Ring_T * p_ring, size_t index, void * p_result)         { Copy(p_ring, p_result, At(p_ring, index)); }
static inline void PeekFront(const Ring_T * p_ring, void * p_result)                    { Copy(p_ring, p_result, Front(p_ring)); }
static inline void PeekBack(const Ring_T * p_ring, void * p_result)                     { Copy(p_ring, p_result, Back(p_ring)); }
static inline void PlaceAt(const Ring_T * p_ring, size_t index, const void * p_unit)    { Copy(p_ring, At(p_ring, index), p_unit); }
/* Overwrite */
static inline void ReplaceFront(const Ring_T * p_ring, const void * p_unit)               { Copy(p_ring, Front(p_ring), p_unit); }
static inline void ReplaceBack(const Ring_T * p_ring, const void * p_unit)                { Copy(p_ring, Back(p_ring), p_unit); }



/******************************************************************************/
/*!
    Protected
    Handle input boundary checking.
    Caller handle lock
    @return a pointer to buffer data.
*/
/******************************************************************************/
/*
*/
inline void * Ring_Front(const Ring_T * p_ring)     { return (Ring_IsEmpty(p_ring) == false) ? Front(p_ring) : NULL; }
inline void * Ring_Back(const Ring_T * p_ring)      { return (Ring_IsEmpty(p_ring) == false) ? Back(p_ring) : NULL; }
inline void * Ring_At(const Ring_T * p_ring, size_t index) { return (index < Ring_GetFullCount(p_ring)) ? At(p_ring, index) : NULL; }

inline void * _Ring_Seek(Ring_T * p_ring, size_t index)  { return (index < Ring_GetFullCount(p_ring)) ? Seek(p_ring, index) : NULL; }


/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void Ring_Init(const Ring_Context_T * p_ring)
{
    Ring_InitFrom(p_ring->P_STATE, &p_ring->TYPE);
}

void Ring_InitFrom(Ring_T * p_ring, const Ring_Type_T * p_type)
{
    /* Ok to cast away const. It is the RAM copy */
    memcpy((void *)&p_ring->Type, p_type, sizeof(Ring_Type_T));
    p_ring->Head = 0U;
    p_ring->Tail = 0U;
    Ring_Clear(p_ring);
}

void Ring_Clear(Ring_T * p_ring)
{
    p_ring->Tail = 0U;
    p_ring->Head = 0U;
}

bool Ring_PushBack(Ring_T * p_ring, const void * p_unit)
{
    assert(p_unit != NULL);

    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsFull(p_ring) == false) { PushBack(p_ring, p_unit); isSuccess = true; }
    // _Ring_PushBack(p_ring, p_unit);
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PopFront(Ring_T * p_ring, void * p_result)
{
    assert(p_result != NULL);

    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PopFront(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PushFront(Ring_T * p_ring, const void * p_unit)
{
    assert(p_unit != NULL);

    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsFull(p_ring) == false) { PushFront(p_ring, p_unit); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PopBack(Ring_T * p_ring, void * p_result)
{
    assert(p_result != NULL);

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
    assert(p_units != NULL);

    bool isSuccess = false;
    if (AcquireCritical(p_ring) == true)
    {
        if (unitCount <= Ring_GetEmptyCount(p_ring))
        {
            for (size_t iUnit = 0U; iUnit < unitCount; ++iUnit) { PushBack(p_ring, void_pointer_at(p_units, p_ring->Type.UNIT_SIZE, iUnit)); }
            isSuccess = true;
        }
        ReleaseCritical(p_ring);
    }
    return isSuccess;
}

bool Ring_DequeueN(Ring_T * p_ring, void * p_result, size_t unitCount)
{
    assert(p_result != NULL);

    bool isSuccess = false;
    if (AcquireCritical(p_ring) == true)
    {
        if (unitCount <= Ring_GetFullCount(p_ring))
        {
            for (size_t iUnit = 0U; iUnit < unitCount; ++iUnit) { PopFront(p_ring, void_pointer_at(p_result, p_ring->Type.UNIT_SIZE, iUnit)); }
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
    assert(p_units != NULL);

    size_t count = 0U;
    if (AcquireCritical(p_ring) == true)
    {
        count = Ring_GetEmptyCount(p_ring);
        if (count > unitCount) { count = unitCount; }
        for (size_t iUnit = 0U; iUnit < count; ++iUnit) { PushBack(p_ring, void_pointer_at(p_units, p_ring->Type.UNIT_SIZE, iUnit)); }
        ReleaseCritical(p_ring);
    }
    return count;
}

/*

*/
size_t Ring_DequeueMax(Ring_T * p_ring, void * p_result, size_t unitCount)
{
    assert(p_result != NULL);

    size_t count = 0U;
    if (AcquireCritical(p_ring) == true)
    {
        count = Ring_GetFullCount(p_ring);
        if (count > unitCount) { count = unitCount; }
        for (size_t iUnit = 0U; iUnit < count; ++iUnit) { PopFront(p_ring, void_pointer_at(p_result, p_ring->Type.UNIT_SIZE, iUnit)); }
        ReleaseCritical(p_ring);
    }
    return count;
}


bool Ring_PeekFront(Ring_T * p_ring, void * p_result)
{
    assert(p_result != NULL);

    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PeekFront(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekBack(Ring_T * p_ring, void * p_result)
{
    assert(p_result != NULL);

    bool isSuccess = false;
    EnterCritical(p_ring);
    if (Ring_IsEmpty(p_ring) == false) { PeekBack(p_ring, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}

bool Ring_PeekAt(Ring_T * p_ring, size_t index, void * p_result)
{
    assert(p_result != NULL);

    bool isSuccess = false;
    EnterCritical(p_ring);
    if (index < Ring_GetFullCount(p_ring)) { PeekAt(p_ring, index, p_result); isSuccess = true; }
    ExitCritical(p_ring);
    return isSuccess;
}




// const int8_t * _Ring8_At(const Ring_T * p_ring, size_t index) { return (index < Ring_GetFullCount(p_ring)) ? &as_array_at(int8_t *, p_ring->CONST.P_BUFFER, IndexOf(p_ring, index)) : NULL; }
