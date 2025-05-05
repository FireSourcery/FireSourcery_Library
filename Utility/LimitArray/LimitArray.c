
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
    @version V0
    @brief  Array with O(1) Min/Max, calculated on set/clear
*/
/******************************************************************************/
#include "LimitArray.h"

#include <stdint.h>
#include <stdbool.h>
// #include <limits.h>

/*!
    @brief Initialize the LimitArray_T structure.
    @param p_limit Pointer to the LimitArray_T structure.
*/
void LimitArray_Init(LimitArray_T * p_limit)
{
    // p_limit->count = 0U;
    p_limit->Min = LIMIT_ARRAY_MAX;
    p_limit->Max = LIMIT_ARRAY_MIN;
}

void LimitArray_ClearAll(LimitArray_T * p_limit)
{
    p_limit->Min = LIMIT_ARRAY_MAX;
    p_limit->Max = LIMIT_ARRAY_MIN;

    for (uint8_t index = 0U; index < p_limit->LENGTH; index++) { p_limit->P_ARRAY[index] = LIMIT_ARRAY_CLEAR; }
}

/*
    Entry Sub Module
    Functions should not be mixed with non Id functions
    directly use index as id, this way O(n) compare is not needed.
*/

/*!
   @brief a value to the p_limit control.
   @param p_limit Pointer to the LimitArray_T structure.
   @param value The value to add.
   @param id The ID associated with the value.
   @return True if the value was a new min or max. The value of the entry is always set.
*/
bool LimitArray_SetEntry(LimitArray_T * p_limit, limit_id_t id, limit_t value)
{
    bool isLimit = false;
    p_limit->P_ARRAY[id] = value;
    if (value < p_limit->Min) { p_limit->Min = value; isLimit = true; }
    if (value > p_limit->Max) { p_limit->Max = value; isLimit = true; }
    return isLimit;
}

bool LimitArray_SetEntryUpper(LimitArray_T * p_limit, limit_id_t id, limit_t value)
{
    bool isLimit = false;
    p_limit->P_ARRAY[id] = value;
    if (value < p_limit->Min) { p_limit->Min = value; isLimit = true; }
    return isLimit;
}

/*!
    @brief Remove a value from the p_limit control by ID.
    @param p_limit Pointer to the LimitArray_T structure.
    @param id The ID of the value to remove.
    @return True if the value was a active limit. The value of the entry is always cleared.
*/
bool LimitArray_ClearEntry(LimitArray_T * p_limit, limit_id_t id)
{
    bool isLimit = false;
    limit_t value = p_limit->P_ARRAY[id];
    limit_t bufferValue;
    limit_t bufferMin;
    limit_t bufferMax;

    p_limit->P_ARRAY[id] = LIMIT_ARRAY_CLEAR;

    isLimit = (value == p_limit->Min || value == p_limit->Max); /*  */

    if (isLimit == true)
    {
        bufferMin = LIMIT_ARRAY_MAX;
        bufferMax = LIMIT_ARRAY_MIN;
        for (uint8_t index = 0U; index < p_limit->LENGTH; index++)
        {
            bufferValue = p_limit->P_ARRAY[index];
            if (bufferValue != LIMIT_ARRAY_CLEAR)
            {
                if (bufferValue < p_limit->Min) { bufferMin = bufferValue; }
                if (bufferValue > p_limit->Max) { bufferMax = bufferValue; }
            }
        }

        p_limit->Min = bufferMin;
        p_limit->Max = bufferMax;
    }

    return isLimit;
}

