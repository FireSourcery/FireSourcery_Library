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
    @file   Limit.h
    @author FireSourcery

    @brief Array with O(1) Min/Max, calculated on set/clear
*/
/******************************************************************************/
#ifndef LIMIT_ARRAY_H
#define LIMIT_ARRAY_H

#include "Type/Array/Array.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef LIMIT_ARRAY_SIGNED
typedef int32_t limit_t;
#define LIMIT_ARRAY_MIN (-INT32_MAX)
#define LIMIT_ARRAY_MAX (INT32_MAX)
#define LIMIT_ARRAY_CLEAR (INT32_MIN) /* (LIMIT_ARRAY_MIN - 1) is reserved  */
#else
/*
    limits as unsigned
*/
typedef uint16_t limit_t;
#define LIMIT_ARRAY_MIN (0U)
#define LIMIT_ARRAY_MAX (UINT16_MAX)
#define LIMIT_ARRAY_CLEAR (LIMIT_ARRAY_MAX) /* UpperLimit unaffected. LowerLimit may incur additional logic */
#endif

typedef size_t limit_id_t;

/*
    Augments
*/
typedef struct LimitArray_Augments
{
    /*
        calculated on add/remove
        get is O(1)
    */
    size_t MinId;
    size_t MaxId;
    limit_t Min; /* Min of the array, upper of [Limit] */
    limit_t Max; /* Max of the array, lower of [Limit] */
}
LimitArray_Augments_T;

/******************************************************************************/
/*
    inline version
*/
/******************************************************************************/
static inline void _LimitArray_ClearValues(limit_t * p_values, size_t length)
{
    for (uint8_t index = 0U; index < length; index++) { p_values[index] = LIMIT_ARRAY_CLEAR; }
}

static inline void _LimitArray_ClearState(LimitArray_Augments_T * p_state)
{
    p_state->Min = LIMIT_ARRAY_MAX;
    p_state->Max = LIMIT_ARRAY_MIN;
    p_state->MinId = (limit_id_t)-1;
    p_state->MaxId = (limit_id_t)-1;
}

static void _LimitArray_Init(LimitArray_Augments_T * p_state, limit_t * p_values)
{
    p_state->Min = LIMIT_ARRAY_MAX;
    p_state->Max = LIMIT_ARRAY_MIN;
}

static inline interval_t _LimitArray_GetBand(const LimitArray_Augments_T * p_state) { return (interval_t) { .low = p_state->Min, .high = p_state->Max, }; }
static inline limit_t _LimitArray_Apply(const LimitArray_Augments_T * p_limits, int req) { return math_clamp(req, p_limits->Min, p_limits->Max); }

static void _LimitArray_ClearAll(LimitArray_Augments_T * p_state, limit_t * p_values, size_t length)
{
    _LimitArray_ClearState(p_state);
    _LimitArray_ClearValues(p_values, length);
}

static inline limit_t _LimitArray_Upper(const LimitArray_Augments_T * p_state) { return p_state->Min; }
static inline limit_t _LimitArray_Lower(const LimitArray_Augments_T * p_state) { return p_state->Max; }

static inline bool _LimitArray_TestSetUpper(LimitArray_Augments_T * p_state, limit_t * p_values, limit_id_t id, limit_t value)
{
    p_values[id] = value;
    if (value > p_state->Max) { p_state->Max = value; p_state->MaxId = id; return true; }
    return false;
}

static inline bool _LimitArray_TestSetLower(LimitArray_Augments_T * p_state, limit_t * p_values, limit_id_t id, limit_t value)
{
    p_values[id] = value;
    if (value < p_state->Min) { p_state->Min = value; p_state->MinId = id; return true; }
    return false;
}

static void _LimitArray_ProcCompare(LimitArray_Augments_T * p_state, limit_t * p_values, size_t length)
{
    limit_t bufferMin = LIMIT_ARRAY_MAX;
    limit_t bufferMax = LIMIT_ARRAY_MIN;
    limit_t bufferValue;

    for (uint8_t index = 0U; index < length; index++)
    {
        bufferValue = p_values[index];
        if (bufferValue != LIMIT_ARRAY_CLEAR) /* Simultaneous no compare for signed limit_t */
        {
            if (bufferValue < bufferMin) { bufferMin = bufferValue; }
            if (bufferValue > bufferMax) { bufferMax = bufferValue; }
        }
    }

    p_state->Min = bufferMin;
    p_state->Max = bufferMax;
}

static inline bool _LimitArray_TryClearEntry(LimitArray_Augments_T * p_state, limit_t * p_values, size_t length, limit_id_t id)
{
    limit_t value = p_values[id];
    bool isLimit = (value == p_state->Min) || (value == p_state->Max);
    // bool isLimit = (id == p_state->MinId) || (id == p_state->MaxId);
    p_values[id] = LIMIT_ARRAY_CLEAR;
    if (isLimit == true) { _LimitArray_ProcCompare(p_state, p_values, length); }
    return isLimit;
}

/******************************************************************************/
/*
    Contiguous allocation
*/
/******************************************************************************/
// typedef struct __attribute__((aligned(sizeof(uintptr_t)))) LimitArray
// {
//     LimitArray_Augments_T State;
//     limit_t Values[]; /* Flexible array member for contiguous storage with augments. Caller handles allocation and indexing. */
// }
// LimitArray_T;

// #define _LIMIT_ARRAY_SIZE(Length) (sizeof(LimitArray_Augments_T) + (sizeof(limit_t) * (Length)))
// #define _LIMIT_ARRAY_ALLOC(Length) (LimitArray_T *)_BUFFER_ALLOC(sizeof(LimitArray_Augments_T) + (sizeof(limit_t) * (Length)))

// /* Caller def uint8[_LIMIT_ARRAY_SIZE(Length)] */
// typedef __attribute__((aligned(sizeof(uintptr_t)))) uint8_t LimitArray_Alloc_T[];

// /*
//     alternative to macro def
//     struct
//     {
//         ...
//         LimitArray_Alloc_T[_LIMIT_ARRAY_SIZE(Length)] LimitArray;
//     }
// */
// LimitArray_T * LimitArray_Cast(LimitArray_Alloc_T * p_alloc) { return (LimitArray_T *)p_alloc; }

/******************************************************************************/
/*
    Descriptor handler
*/
/******************************************************************************/
typedef const struct LimitArray
{
    limit_t * P_BUFFER;   // Pointer to the p_array buffer
    size_t LENGTH;        // Length of the p_array
    /* alternatively single pointer to top */
    LimitArray_Augments_T * P_AUGMENTS;
}
LimitArray_T;

#define LIMIT_ARRAY_INIT(p_Buffer, Length, p_Augments) (LimitArray_T){ .P_BUFFER = (p_Buffer), .LENGTH = (Length), .P_AUGMENTS = (p_Augments) }
#define LIMIT_ARRAY_ALLOC(Length) LIMIT_ARRAY_INIT(_BUFFER_ALLOC(sizeof(limit_t) * (Length)), Length, &(LimitArray_Augments_T){0})


/*

*/
static inline limit_t * _LimitArray_Values(LimitArray_T * p_limit) { return (limit_t *)p_limit->P_BUFFER; }
static inline LimitArray_Augments_T * _LimitArray_State(LimitArray_T * p_limit) { return (LimitArray_Augments_T *)p_limit->P_AUGMENTS; }


/*!
    @brief Get the arrayMinimum value from the p_limit control.
    @param p_limit Pointer to the LimitArray_T structure.
    @return The arrayMinimum value, or UINT16_MAX if the array is empty.
*/
static inline limit_t LimitArray_Upper(LimitArray_T * p_limit) { return _LimitArray_State(p_limit)->Min; }

/*!
    @brief Get the arrayMaximum value from the p_limit control.
    @param p_limit Pointer to the Limit_T structure.
    @return The arrayMaximum value, or 0 if the array is empty.
*/
static inline limit_t LimitArray_Lower(LimitArray_T * p_limit) { return _LimitArray_State(p_limit)->Max; }



static inline bool LimitArray_IsUpperActive(LimitArray_T * p_limit) { return (_LimitArray_State(p_limit)->Min != LIMIT_ARRAY_MAX); }
static inline bool LimitArray_IsLowerActive(LimitArray_T * p_limit) { return (_LimitArray_State(p_limit)->Max != LIMIT_ARRAY_MIN); }
static inline bool LimitArray_IsActive(LimitArray_T * p_limit) { return (LimitArray_IsUpperActive(p_limit) || LimitArray_IsLowerActive(p_limit)); }

static inline void LimitArray_Apply(LimitArray_T * p_limits, limit_t req) { _LimitArray_Apply(_LimitArray_State(p_limits), req); }

static inline limit_t LimitArray_UpperComposed(LimitArray_T * p_a, LimitArray_T * p_b) { return math_min(LimitArray_Upper(p_a), LimitArray_Upper(p_b)); }
static inline limit_t LimitArray_LowerComposed(LimitArray_T * p_a, LimitArray_T * p_b) { return math_max(LimitArray_Lower(p_a), LimitArray_Lower(p_b)); }
// void LimitArray_ProcCompareComposed(LimitArray_T * p_limit, LimitArray_T * p_b)

/*

*/
extern void LimitArray_Init(LimitArray_T * p_limit);
extern void LimitArray_ClearAll(LimitArray_T * p_limit);

extern bool LimitArray_TestSetEntry(LimitArray_T * p_limit, limit_id_t id, limit_t value);
extern bool LimitArray_TestSetUpper(LimitArray_T * p_limit, limit_id_t id, limit_t value);
extern bool LimitArray_TestClearEntry(LimitArray_T * p_limit, limit_id_t id);

#endif // LIMIT_ARRAY_H
//fanout
// typedef void (*Notify_Fn_T)(void * p_subscriber);

// typedef const struct Notify_Sub
// {
//     Notify_Fn_T ON_EVENT;
//     void * P_SUBSCRIBER;
// }
// Notify_Sub_T;

// typedef const struct Notify
// {
//     const Notify_Sub_T * P_SUBS;
//     size_t COUNT;
// }
// Notify_T;

// static inline void Notify_Fire(const Notify_T * p)
// {
//     for (size_t i = 0; i < p->COUNT; i++) p->P_SUBS[i].ON_EVENT(p->P_SUBS[i].P_SUBSCRIBER);
// }