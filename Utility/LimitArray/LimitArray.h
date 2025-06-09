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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef CONFIG_LIMIT_ARRAY_SIGNED
typedef int32_t limit_t;
#define LIMIT_ARRAY_MIN (-INT32_MAX)
#define LIMIT_ARRAY_MAX (INT32_MAX)
#define LIMIT_ARRAY_CLEAR (INT32_MIN) /* (LIMIT_ARRAY_MIN - 1) is reserved  */
#else
/*
    limits as a percent of uint16_t
*/
typedef uint16_t limit_t;
#define LIMIT_ARRAY_MIN (0U)
#define LIMIT_ARRAY_MAX (UINT16_MAX)
#define LIMIT_ARRAY_CLEAR (LIMIT_ARRAY_MAX) /* UpperLimit unaffected. LowerLimit may incur additional logic */
#endif

typedef size_t limit_id_t;

// typedef struct
// {
//     limit_t Lower;
//     limit_t Upper;
// }
// LimitPair_T;

/*
    Augments
*/
typedef struct LimitArray_Augments
{
    // size_t count;
    /*
        calculated on add/remove
        get is O(1)
    */
    size_t MinId;
    size_t MaxId;
    int Min; /* Min of the array, upper of [Limit] */
    int Max; /* Max of the array, lower of [Limit] */
}
LimitArray_Augments_T;

// typedef Array_T LimitArray_T;

typedef const union
{
    Array_T ARRAY;
    struct
    {
        limit_t * P_BUFFER;   // Pointer to the p_array buffer
        size_t LENGTH;        // Length of the p_array
        LimitArray_Augments_T * P_AUGMENTS; // Pointer to the p_array buffer
    };
}
LimitArray_T;

#define LIMIT_ARRAY_INIT(p_Buffer, Length, p_Augments) { .ARRAY = ARRAY_INIT(p_Buffer, Length, p_Augments) }
#define LIMIT_ARRAY_ALLOC(Length) { .ARRAY = ARRAY_ALLOC_AS(limit_t, Length, &(LimitArray_Augments_T){0}) }

/*  */
static inline LimitArray_Augments_T * _LimitArray_State(const LimitArray_T * p_limit) { return (LimitArray_Augments_T *)p_limit->P_AUGMENTS; }
static inline limit_t * _LimitArray_Values(const LimitArray_T * p_limit) { return (limit_t *)p_limit->P_BUFFER; }

/*  */
static inline void _LimitArray_ClearState(LimitArray_Augments_T * p_state)
{
    p_state->Min = LIMIT_ARRAY_MAX;
    p_state->Max = LIMIT_ARRAY_MIN;
    p_state->MinId = -1;
    p_state->MaxId = -1;
}

static inline void _LimitArray_ClearValues(limit_t * p_values, size_t length)
{
    for (uint8_t index = 0U; index < length; index++) { p_values[index] = LIMIT_ARRAY_CLEAR; }
}


// static inline int16_t LimitArray_Apply(const LimitArray_T * p_limits, int16_t req) { return math_clamp(req, p_limits->Lower, p_limits->Upper); }


/*!
    @brief Get the arrayMinimum value from the p_limit control.
    @param p_limit Pointer to the LimitArray_T structure.
    @return The arrayMinimum value, or UINT16_MAX if the array is empty.
*/
static inline limit_t LimitArray_GetUpper(const LimitArray_T * p_limit) { return _LimitArray_State(p_limit)->Min; }

/*!
    @brief Get the arrayMaximum value from the p_limit control.
    @param p_limit Pointer to the Limit_T structure.
    @return The arrayMaximum value, or 0 if the array is empty.
*/
static inline limit_t LimitArray_GetLower(const LimitArray_T * p_limit) { return _LimitArray_State(p_limit)->Max; }

static inline bool LimitArray_IsUpperActive(const LimitArray_T * p_limit) { return (_LimitArray_State(p_limit)->Min != LIMIT_ARRAY_MAX); }
static inline bool LimitArray_IsLowerActive(const LimitArray_T * p_limit) { return (_LimitArray_State(p_limit)->Max != LIMIT_ARRAY_MIN); }
static inline bool LimitArray_IsActive(const LimitArray_T * p_limit) { return (LimitArray_IsUpperActive(p_limit) || LimitArray_IsLowerActive(p_limit)); }

extern void LimitArray_Init(const LimitArray_T * p_limit);
extern void LimitArray_ClearAll(const LimitArray_T * p_limit);

extern bool LimitArray_SetEntry(const LimitArray_T * p_limit, limit_id_t id, limit_t value);
extern bool LimitArray_ClearEntry(const LimitArray_T * p_limit, limit_id_t id);
// extern bool LimitArray_SetEntryUpper(LimitArray_T * p_limit, limit_id_t id, limit_t value);

#endif // LIMIT_ARRAY_H