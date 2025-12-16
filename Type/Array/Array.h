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
    @file   Array.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "void_array.h"
#include "generic_array.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>


/******************************************************************************/
/*
    Wrapper around values array, preset functions for int types and operations
*/
/******************************************************************************/
typedef const struct ArrayMeta
{
    // const size_t TYPE_SIZE; // Size of each element in the p_array
    const union
    {
        void * P_BUFFER;
        uint8_t * P_ARRAY8;
        uint16_t * P_ARRAY16;
        uint32_t * P_ARRAY32;
        uint64_t * P_ARRAY64;
    };
    size_t LENGTH;    // Length of the p_array
    void * P_AUGMENTS; // P_HEADER/P_STATE
}
Array_T;

// static allocation only, for inline convenience
#define _BUFFER_ALLOC(Bytes) ((void *)(uint8_t[(Bytes)]){})
#define _ARRAY_ALLOC(T, Length) ((void *)(T[(Length)]){})

/*
    Length in Units
*/
#define ARRAY_INIT(p_Buffer, Length, p_Augments) { .P_BUFFER = (p_Buffer), .LENGTH = (Length), .P_AUGMENTS = (p_Augments) }
#define ARRAY_INIT_ALLOC(TypeSize, Length, p_Augments) ARRAY_INIT(_BUFFER_ALLOC((TypeSize) * (Length)), Length, p_Augments)
#define ARRAY_INIT_ALLOC_AS(T, Length, p_Augments) ARRAY_INIT(_ARRAY_ALLOC(T, Length), Length, p_Augments)

/*
    Void * or Struct
    rely on compiler optimization to inline type size
*/
static inline void _Array_Set(const size_t type, Array_T * p_array, size_t index, void * p_value) { memcpy(void_pointer_at(p_array->P_BUFFER, type, index), p_value, type); }
static inline void _Array_ForEach(const size_t type, Array_T * p_array, proc_t op) { void_array_foreach(p_array->P_BUFFER, type, p_array->LENGTH, op); }
static inline void _Array_SetEach(const size_t type, Array_T * p_array, set_t op, value_t value) { void_array_foreach_set(p_array->P_BUFFER, type, p_array->LENGTH, op, value); }

/*
    Value Array
    Generic array implementation - using Macros with _Generic selection for type safety

    Compile time typed
    effectively void array expressed with type prior to what compiler may optimize
    alternative to handling size parameter, may be faster depending on compiler.
*/
static inline int8_t * Array8_At(Array_T * p_array, size_t index) { return &(((int8_t *)p_array->P_BUFFER)[index]); }
// static inline void Array8_Assign(Array_T * p_array, int8_t * p_values, size_t length) { memcpy(p_array->P_BUFFER, p_values, length * sizeof(int8_t)); }

static inline int8_t Array8_Get(Array_T * p_array, size_t index) { return ((int8_t *)p_array->P_BUFFER)[index]; }
static inline void Array8_Set(Array_T * p_array, size_t index, int8_t value) { ((int8_t *)p_array->P_BUFFER)[index] = value; }

static inline int16_t Array16_Get(Array_T * p_array, size_t index) { return ((int16_t *)p_array->P_BUFFER)[index]; }
static inline void Array16_Set(Array_T * p_array, size_t index, int16_t value) { ((int16_t *)p_array->P_BUFFER)[index] = value; }

static inline int32_t Array32_Get(Array_T * p_array, size_t index)  { return ((int32_t *)p_array->P_BUFFER)[index]; }
static inline void Array32_Set(Array_T * p_array, size_t index, int32_t value) { ((int32_t *)p_array->P_BUFFER)[index] = value; }

// static inline void Array32_ForEach(Array_T * p_array, proc_t op) { for (size_t index = 0U; index < p_array->LENGTH; index++) { op(&((int32_t *)p_array->P_BUFFER)[index]); } }

static inline void Array32_ForEach(Array_T * p_array, proc_t op) { void_array_foreach_call(p_array->P_ARRAY32, p_array->LENGTH, op); }


/*
    Typed signature for Init and handle with _Generic
*/
typedef const Array_T Array8_T;
typedef const Array_T Array16_T;
typedef const Array_T Array32_T;
typedef const Array_T Array64_T;

/* cast the wrapper type */
#define ValueArray_Get(p_array, index) \
    _Generic(p_array, \
        Array8_T *: Array8_Get, \
        Array16_T *: Array16_Get, \
        Array32_T *: Array32_Get, \
        Array64_T *: Array64_Get,  \
        default : ArrayPtr_Get \
    )(p_array, index)

#define ValueArray_Set(p_array, index, value) \
    _Generic(p_array, \
        Array8_T *: Array8_Set, \
        Array16_T *: Array16_Set, \
        Array32_T *: Array32_Set, \
        Array64_T *: Array64_Set, \
        default  : ArrayPtr_Set \
    )(p_array, index, value)

// #define Array_GetAs(T, p_ArrayMeta, index) \
//     _Generic((T)0, \
//         int8_t  : Array8_Get, \
//         int16_t : Array16_Get, \
//         int32_t : Array32_Get, \
//         default  :   \
//     )(p_ArrayMeta, index)

// #define Array_SetAs(T, p_ArrayMeta, index, value) (Array_At(T, p_ArrayMeta, index) = value)
