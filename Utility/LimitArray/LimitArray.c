
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
    @file   LimitArray.h
    @author FireSourcery

    @brief  Array with O(1) Min/Max, calculated on set/clear
*/
/******************************************************************************/
#include "LimitArray.h"

#include <stdint.h>
#include <stdbool.h>
// #include <limits.h>
#include <assert.h>

/*!
    @brief Initialize the LimitArray_T structure.
    @param p_limit Pointer to the LimitArray_T structure.
*/
void LimitArray_Init(const LimitArray_T * p_limit)
{
    // p_limit->count = 0U;
    _LimitArray_State(p_limit)->Min = LIMIT_ARRAY_MAX;
    _LimitArray_State(p_limit)->Max = LIMIT_ARRAY_MIN;
}

void LimitArray_ClearAll(const LimitArray_T * p_limit)
{
    _LimitArray_ClearState(_LimitArray_State(p_limit));
    _LimitArray_ClearValues(_LimitArray_Values(p_limit), p_limit->LENGTH);
}

/*
    Entry Sub Module
    Functions should not be mixed with non Id functions
    directly use index as id, this way O(n) compare is not needed.
*/
bool TestSetUpper(LimitArray_Augments_T * p_state, limit_id_t id, limit_t value)
{
    return (value > p_state->Max) ? ({ p_state->Max = value; p_state->MaxId = id; true; }) : false;
}

bool TestSetLower(LimitArray_Augments_T * p_state, limit_id_t id, limit_t value)
{
    return (value < p_state->Min) ? ({ p_state->Min = value; p_state->MinId = id; true; }) : false;
}


/*!
   @brief a value to the p_limit control.
   @param p_limit Pointer to the LimitArray_T structure.
   @param value The value to add.
   @param id The ID associated with the value.
   @return True if the value was a new min or max. The value of the entry is always set.
*/
bool LimitArray_SetEntry(const LimitArray_T * p_limit, limit_id_t id, limit_t value)
{
    assert(id < p_limit->LENGTH); // Ensure id is within bounds. Compile-time constant

    _LimitArray_Values(p_limit)[id] = value;

    return TestSetUpper(_LimitArray_State(p_limit), id, value) || TestSetLower(_LimitArray_State(p_limit), id, value);
}

bool LimitArray_TestSetUpper(const LimitArray_T * p_limit, limit_id_t id, limit_t value)
{
    _LimitArray_Values(p_limit)[id] = value;
    return TestSetUpper(_LimitArray_State(p_limit), id, value);
    //return  LimitArray_GetUpper(p_limit);
}


/*
    Alternatively, periodically call this function to update the min/max values
    Entries may set async
*/
void LimitArray_ProcCompare(const LimitArray_T * p_limit)
{
    limit_t bufferMin = LIMIT_ARRAY_MAX;
    limit_t bufferMax = LIMIT_ARRAY_MIN;
    limit_t bufferValue;

    for (uint8_t index = 0U; index < p_limit->LENGTH; index++)
    {
        bufferValue = _LimitArray_Values(p_limit)[index];
        if (bufferValue != LIMIT_ARRAY_CLEAR)
        {
            if (bufferValue < bufferMin) { bufferMin = bufferValue; }
            if (bufferValue > bufferMax) { bufferMax = bufferValue; }
        }
    }

    _LimitArray_State(p_limit)->Min = bufferMin;
    _LimitArray_State(p_limit)->Max = bufferMax;
}

// static inline limit_t LimitArray_ProcCompareUpper(const LimitArray_T * p_limit)
// {
//     _LimitArray_State(p_limit)->Min = _LimitArray_ProcCompareUpper(p_limit);
// }

/*!
    @brief Remove a value from the p_limit control by ID.
    @param p_limit Pointer to the LimitArray_T structure.
    @param id The ID of the value to remove.
    @return True if the value was a active limit. The value of the entry is always cleared.
*/
bool LimitArray_ClearEntry(const LimitArray_T * p_limit, limit_id_t id)
{
    limit_t value = _LimitArray_Values(p_limit)[id];
    bool isLimit = (value == _LimitArray_State(p_limit)->Min) || (value == _LimitArray_State(p_limit)->Max);

    _LimitArray_Values(p_limit)[id] = LIMIT_ARRAY_CLEAR;

    if (isLimit == true) { LimitArray_ProcCompare(p_limit); }

    return isLimit;
}

