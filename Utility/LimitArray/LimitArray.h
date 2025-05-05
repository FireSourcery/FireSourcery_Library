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
    @version V0
    @brief Array with O(1) Min/Max, calculated on set/clear
*/
/******************************************************************************/
#ifndef LIMIT_ARRAY_H
#define LIMIT_ARRAY_H

// #include "Type/Array/Array.h"

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
// LimitArray_Pair_T;
// static inline int16_t LimitArray_Of(const LimitArray_Pair_T * p_limits, int16_t req) { return math_clamp(req, p_limits->Lower, p_limits->Upper); }

typedef struct
{
    const struct { limit_t * const P_ARRAY; const size_t LENGTH; };
    // const struct { LimitArray_Pair_T * const P_ARRAY; const size_t LENGTH; };

    // size_t count;
    /*
        limits calculated on add/remove
        get is O(1)
    */
    // limit_id_t minIndex;
    // limit_id_t maxId;
    limit_t Min; /* Min of the array, upper of [Limit] */
    limit_t Max; /* Max of the array, lower of [Limit] */

}
LimitArray_T;

#define LIMIT_ARRAY_INIT(p_buffer, length) { .P_ARRAY = (p_buffer), .LENGTH = (length) }
#define LIMIT_ARRAY_ALLOC(length) { .P_ARRAY = (limit_t[length]){0}, .LENGTH = (length) }


/*!
    @brief Get the arrayMinimum value from the p_limit control.
    @param p_limit Pointer to the LimitArray_T structure.
    @return The arrayMinimum value, or UINT16_MAX if the array is empty.
*/
static inline limit_t LimitArray_GetUpper(const LimitArray_T * p_limit) { return p_limit->Min; }

/*!
    @brief Get the arrayMaximum value from the p_limit control.
    @param p_limit Pointer to the Limit_T structure.
    @return The arrayMaximum value, or 0 if the array is empty.
*/
static inline limit_t LimitArray_GetLower(const LimitArray_T * p_limit) { return p_limit->Max; }

static inline bool LimitArray_IsActive(const LimitArray_T * p_limit) { return (p_limit->Min != LIMIT_ARRAY_MAX || p_limit->Max != LIMIT_ARRAY_MIN); }
static inline bool LimitArray_IsUpperActive(const LimitArray_T * p_limit) { return (p_limit->Min != LIMIT_ARRAY_MAX); }
static inline bool LimitArray_IsLowerActive(const LimitArray_T * p_limit) { return (p_limit->Max != LIMIT_ARRAY_MIN); }

extern void LimitArray_Init(LimitArray_T * p_limit);
extern void LimitArray_ClearAll(LimitArray_T * p_limit);

extern bool LimitArray_SetEntry(LimitArray_T * p_limit, limit_id_t id, limit_t value);
extern bool LimitArray_ClearEntry(LimitArray_T * p_limit, limit_id_t id);
// extern bool LimitArray_SetEntryUpper(LimitArray_T * p_limit, limit_id_t id, limit_t value);

#endif // LIMIT_ARRAY_H