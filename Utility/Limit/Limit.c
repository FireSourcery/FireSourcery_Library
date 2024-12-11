
#include "Limit.h"
#include "Type/Array/Array.h"
#include "Type/Array/array_generic.h"

#include <stdint.h>
#include <stdbool.h>
// #include <limits.h>

/*!
    @brief Initialize the Limit_T structure.
    @param p_limit Pointer to the Limit_T structure.
*/
void Limit_Init(Limit_T * p_limit)
{
    p_limit->count = 0U;
    p_limit->valuesMin = LIMIT_MAX;
    p_limit->valuesMax = LIMIT_MIN;
}

void Limit_ClearAll(Limit_T * p_limit)
{
    p_limit->valuesMin = LIMIT_MAX;
    p_limit->valuesMax = LIMIT_MIN;

    for (uint8_t index = 0U; index < p_limit->LENGTH; index++) { p_limit->P_ARRAY[index] = LIMIT_CLEAR; }
}

/*
    Entry Sub Module
    Functions should not be mixed with non Id functions
    directly use index as id, this way O(n) compare is not needed.
*/

/*!
   @brief   a value to the p_limit control.
   @param p_limit Pointer to the Limit_T structure.
   @param value The value to add.
   @param id The ID associated with the value.
   @return True if the value was a new min or max. The value of the entry is always set.
*/
bool Limit_Entry_Set(Limit_T * p_limit, uint8_t id, limit_t value)
{
    bool isLimit = false;
    p_limit->P_ARRAY[id] = value;
    if (value < p_limit->valuesMin) { p_limit->valuesMin = value; isLimit = true; }
    if (value > p_limit->valuesMax) { p_limit->valuesMax = value; isLimit = true; }
    return isLimit;
}

bool Limit_Entry_SetUpper(Limit_T * p_limit, uint8_t id, limit_t value)
{
    bool isLimit = false;
    p_limit->P_ARRAY[id] = value;
    if (value < p_limit->valuesMin) { p_limit->valuesMin = value; isLimit = true; }
    return isLimit;
}

/*!
    @brief Remove a value from the p_limit control by ID.
    @param p_limit Pointer to the Limit_T structure.
    @param id The ID of the value to remove.
    @return True if the value was a active limit. The value of the entry is always cleared.
*/
bool Limit_Entry_Clear(Limit_T * p_limit, uint8_t id)
{
    // static const limit_t LIMIT_CLEAR_VALUE = LIMIT_CLEAR;
    bool isLimit = false;
    limit_t value;
    limit_t bufferValue;
    limit_t bufferMin;
    limit_t bufferMax;

    value = p_limit->P_ARRAY[id];
    p_limit->P_ARRAY[id] = LIMIT_CLEAR;

    isLimit = (value == p_limit->valuesMin || value == p_limit->valuesMax); /*  */

    if (isLimit == true)
    {
        bufferMin = LIMIT_MAX;
        bufferMax = LIMIT_MIN;
        for (uint8_t id = 0U; id < p_limit->LENGTH; id++)
        {
            bufferValue = p_limit->P_ARRAY[id];
            if (bufferValue != LIMIT_CLEAR)
            {
                if (bufferValue < p_limit->valuesMin) { bufferMin = bufferValue; }
                if (bufferValue > p_limit->valuesMax) { bufferMax = bufferValue; }
            }
        }

        p_limit->valuesMin = bufferMin;
        p_limit->valuesMax = bufferMax;
    }

    return isLimit;
}

// /*!
//     @brief Add a value to the p_limit control.
//     @param p_limit Pointer to the Limit_T structure.
//     @param value The value to add.
//     @return True if the value was added, false if the array is full.
// */
// bool Limit_Add(Limit_T * p_limit, limit_t value)
// {
//     if(p_limit->count < MAX_LIMIT_ENTRIES)
//     {
//         p_limit->values[p_limit->count++] = value;

//         if(value < p_limit->valuesMin) { p_limit->valuesMin = value; }
//         if(value > p_limit->valuesMax) { p_limit->valuesMax = value; }
//         return true;
//     }
//     return false;
// }

// /*!
//     @brief Remove a value from the p_limit control.
//     @param p_limit Pointer to the Limit_T structure.
//     @param value The value to remove.
//     @return True if the value was removed, false if the value was not found.
// */
// bool Limit_Remove(Limit_T * p_limit, limit_t value)
// {
//     bool found = false;
//     for(size_t id = 0; id < p_limit->count; ++id)
//     {
//         if(p_limit->values[id] == value)
//         {
//             found = true;
//             // Shift remaining values to fill the gap
//             for(size_t j = id; j < p_limit->count - 1; ++j)
//             {
//                 p_limit->values[j] = p_limit->values[j + 1];
//             }
//             --p_limit->count;
//             break;
//         }
//     }

//     if(found)
//     {
//         // Recalculate valuesMin and valuesMax values
//         p_limit->valuesMin = UINT16_MAX;
//         p_limit->valuesMax = 0;
//         for(size_t id = 0; id < p_limit->count; ++id)
//         {
//             if(p_limit->values[id] < p_limit->valuesMin)
//             {
//                 p_limit->valuesMin = p_limit->values[id];
//             }
//             if(p_limit->values[id] > p_limit->valuesMax)
//             {
//                 p_limit->valuesMax = p_limit->values[id];
//             }
//         }
//     }

//     return found;
// }

/*!
    @brief Add a value to the p_limit control.
    @param p_limit Pointer to the Limit_T structure.
    @param value The value to add.
    @param id The ID associated with the value.
    @return True if the value was added, false if the array is full.
*/
//
// bool Limit_AddEntry(Limit_T * p_limit, uint8_t id, limit_t value)
// bool Limit_RemoveEntry(Limit_T * p_limit, uint8_t id)