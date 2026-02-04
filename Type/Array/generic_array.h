#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   generic_array.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "void_array.h"

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof((array)[0]))
#define ARRAY_FOR_EACH(array, index) for (size_t index = 0; (index) < ARRAY_LENGTH(array); ++(index))

/******************************************************************************/
/*!
    value array
*/
/******************************************************************************/
/*
    Ensures all type checks are handled at compile time.
*/

/* shorthand the implementation */
#define _ARRAY_FOREACH_IMPL(P_BUFFER, LENGTH, OP) for (size_t index = 0U; index < LENGTH; ++index) { OP(&(P_BUFFER)[index]); }

static inline void uint8_foreach(uint8_t * p_buffer, size_t length, proc_t proc) { _ARRAY_FOREACH_IMPL(p_buffer, length, proc) }
static inline void uint16_foreach(uint16_t * p_buffer, size_t length, proc_t proc) { _ARRAY_FOREACH_IMPL(p_buffer, length, proc) }
static inline void array_foreach_int(int * p, size_t len, proc_t proc) { for (size_t i = 0; i < len; i++) proc(&(p)[i]); }
static inline void array_foreach_void(void * p, size_t len, proc_t) {  }

// #define _value_array_foreach(p_buffer, length, op) \
//     _Generic((p_buffer), \
//         uint8_t*:   uint8_foreach,       \
//         uint16_t*:  uint16_foreach,      \
//         int*:       array_foreach_int,      \
//         default:    array_foreach_int    \
//     )

#define value_array_foreach(p_buffer, length, op) \
    _Generic((p_buffer), \
        uint8_t*:   uint8_foreach,       \
        uint16_t*:  uint16_foreach,      \
        int*:       array_foreach_int,      \
        default:    array_foreach_int    \
    )(p_buffer, length, op)



/******************************************************************************/
/*!
    select value type
*/
/******************************************************************************/
#define generic_array_foreach(p_buffer, length, op) \
    _Generic((p_buffer), \
        uint8_t*:  value_array_foreach(p_buffer, length, op),      \
        uint16_t*: value_array_foreach(p_buffer, length, op),      \
        default:   void_array_foreach(p_buffer, sizeof(*(p_buffer)), length, op)    \
    )

/******************************************************************************/
/*!
    select operation type
*/
/******************************************************************************/


// #define generic_array_foreach_call(p_buffer, length, function, ...) \
//     _Generic((p_buffer, function), \

//     )(p_buffer, sizeof(*(p_buffer)), length, function __VA_OPT__(,) __VA_ARGS__)




