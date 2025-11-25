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

// typedef const union
// {
//     void * P_BUFFER;
//     uint8_t * P_ARRAY8;
//     uint16_t * P_ARRAY16;
//     uint32_t * P_ARRAY32;
//     uint64_t * P_ARRAY64;
// }
// ValueArray_T;

// typedef const struct ArrayContext
typedef const struct ArrayMeta
{
    // const size_t TYPE_SIZE; // Size of each element in the p_array
    void * P_BUFFER;
    size_t LENGTH;    // Length of the p_array
    void * P_AUGMENTS; // P_HEADER/P_STATE

    // effective add to the unified interface, non value types
    // Array_State_T * const P_STATE;  // Pointer to the p_array buffer, with augments
    /* or separate for extensions */
    // void * const P_BUFFER;
}
Array_T;

// static allocation only, for inline convenience
#define _BUFFER_ALLOC(Bytes) ((void *)(uint8_t[(Bytes)]){})
#define _ARRAY_ALLOC(T, Length) ((void *)(T[(Length)]){})
// #define _BUFFER_ALLOC_STATIC(Bytes) ((void *)((static uint8_t[(Bytes)]){0}))
// #define _ARRAY_ALLOC_STATIC(T, Length) ((void *)((static T[(Length)]){0}))

/*
    Length in Units
*/
#define ARRAY_INIT(p_Buffer, Length, p_Augments) { .P_BUFFER = (p_Buffer), .LENGTH = (Length), .P_AUGMENTS = (p_Augments) }
#define ARRAY_ALLOC_AS(T, Length, p_Augments) ARRAY_INIT(_ARRAY_ALLOC(T, Length), Length, p_Augments)



/*
    Generic array implementation - using Macros with _Generic selection for type safety

    Compile time typed
    effectively void array expressed with type prior to what compiler may optimize
    alternative to handling size parameter, may be faster depending on compiler.
*/
/*
    Allow sub-module to configure type using a single parameter
*/
// #define as_pointer(T, p_buffer) ((T *)p_buffer)
#define as_pointer(T, p_buffer) \
    _Generic((T)0, \
        int8_t  : ((int8_t *)p_buffer), \
        int16_t : ((int16_t *)p_buffer), \
        int32_t : ((int32_t *)p_buffer), \
        int8_t *: ((int8_t *)p_buffer), \
        int16_t *: ((int16_t *)p_buffer), \
        int32_t *: ((int32_t *)p_buffer), \
        int64_t *: ((int64_t *)p_buffer) \
    )

// #define as_pointer(T, p_buffer) \
//     _Generic((T)0, \
//         int8_t    : ((int8_t *)p_buffer), \
//         uint8_t   : ((uint8_t *)p_buffer), \
//         int16_t   : ((int16_t *)p_buffer), \
//         uint16_t  : ((uint16_t *)p_buffer), \
//         int32_t   : ((int32_t *)p_buffer), \
//         uint32_t  : ((uint32_t *)p_buffer), \
//         int64_t   : ((int64_t *)p_buffer), \
//         uint64_t  : ((uint64_t *)p_buffer), \
//         float     : ((float *)p_buffer), \
//         double    : ((double *)p_buffer), \
//         default   : ((void *)p_buffer) \
//     )

#define as_ref(T, p_buffer) (as_pointer(T, p_buffer)[0U])

#define as_value(T, p_buffer) as_ref(T, p_buffer)

// #define as(T, p_buffer) \
//     _Generic((T)0, \
//         int8_t  : as_value(T, p_buffer), \
//         int16_t : as_value(T, p_buffer), \
//         int32_t : as_value(T, p_buffer), \
//         int8_t * : as_pointer(T, p_buffer), \
//         int16_t *: as_pointer(T, p_buffer) \
//     )


#define as_array_at(T, p_buffer, index) (as_pointer(T, p_buffer)[index])

#define Array_At(T, p_ArrayMeta, index) (as_array_at(T, p_ArrayMeta->P_BUFFER, index))

#define ArrayContextT(p_ArrayMeta, index) \
    _Generic(p_ArrayMeta, \
        Array8_T *: int8_t, \
        Array16_T *: int16_t, \
        Array32_T *: int32_t \
    )

#define ArrayContext_At_(p_ArrayMeta, index) (as_array_at(ArrayContextT(p_ArrayMeta, index), p_ArrayMeta->P_BUFFER, index))

/*!
    @brief Generically Get the value of an element at a specific index.
    @param p_array Pointer to the Array_T structure.
    @param index Index of the element to get.
    @return The value of the element at the specified index.
*/
#define Array_GetAs(T, p_ArrayMeta, index) (Array_At(T, p_ArrayMeta, index))

/*!
    @brief Generically Set the value of an element at a specific index.
    @param p_array Pointer to the Array_T structure.
    @param index Index of the element to set.
    @param value Value to set at the specified index.
*/
#define Array_SetAs(T, p_ArrayMeta, index, value) (Array_At(T, p_ArrayMeta, index) = value)

/*
    Type safe API wrapper around common implementation
*/
static inline int8_t Array8_Get(const Array_T * p_array, size_t index) { return ((int8_t *)p_array->P_BUFFER)[index]; }
static inline void Array8_Set(Array_T * p_array, size_t index, int8_t value) { ((int8_t *)p_array->P_BUFFER)[index] = value; }

static inline int16_t Array16_Get(const Array_T * p_array, size_t index) { return as_pointer(int16_t, p_array->P_BUFFER)[index]; }
static inline void Array16_Set(Array_T * p_array, size_t index, int16_t value) { as_pointer(int16_t, p_array->P_BUFFER)[index] = value; }

static inline int32_t Array32_Get(const Array_T * p_array, size_t index) { return Array_At(int32_t, p_array, index); }
static inline void Array32_Set(Array_T * p_array, size_t index, int32_t value) { Array_At(int32_t, p_array, index) = value; }

static inline void ArrayStruct_Set(size_t type, Array_T * p_array, size_t index, void * p_value) { memcpy(void_pointer_at(p_array->P_BUFFER, type, index), p_value, type); }

// static inline int8_t Array8_Get(const Array_T * p_array, size_t index) { return as_pointer(int8_t, p_array->P_BUFFER)[index]; }
// static inline void Array8_Set(Array_T * p_array, size_t index, int8_t value) { as_pointer(int8_t, p_array->P_BUFFER)[index] = value; }

// static inline int16_t Array16_Get(const Array_T * p_array, size_t index) { return as_pointer(int16_t, p_array->P_BUFFER)[index]; }
// static inline void Array16_Set(Array_T * p_array, size_t index, int16_t value) { as_pointer(int16_t, p_array->P_BUFFER)[index] = value; }

// #define Array_GetAs(T, p_ArrayMeta, index) (as_pointer(T, p_ArrayMeta->P_BUFFER)[index])
// #define Array_GetAs(T, p_ArrayMeta, index) \
//     _Generic((T)0, \
//         int8_t  : Array8_Get, \
//         int16_t : Array16_Get, \
//         int32_t : Array32_Get, \
//         default  : void_pointer_at(p_ArrayMeta->P_BUFFER, sizeof(T), index) \
//     )(p_ArrayMeta, index)






/*
    Typed signiture for Init and handle with _Generic
*/
// typedef const struct Array8 { uint8_t * const P_ARRAY; const size_t LENGTH; } Array8_T;
// typedef const struct Array16 { uint16_t * const P_ARRAY; const size_t LENGTH; } Array16_T;
// typedef const struct Array32 { uint32_t * const P_ARRAY; const size_t LENGTH; } Array32_T;
// typedef const struct Array64 { uint64_t * const P_ARRAY; const size_t LENGTH; } Array64_T;
// typedef const struct _Array { void * const P_ARRAY; const size_t LENGTH; } _Array_T;
// typedef const struct ArrayPtr { void ** const P_ARRAY; const size_t LENGTH; } ArrayPtr_T;
// typedef const struct Array8 { Array_State_T * const P_STATE; const size_t LENGTH; } Array8_T;
// typedef const struct Array16 { Array_State_T * const P_STATE; const size_t LENGTH; } Array16_T;
// typedef const struct Array32 { Array_State_T * const P_STATE; const size_t LENGTH; } Array32_T;
// typedef const struct Array64 { Array_State_T * const P_STATE; const size_t LENGTH; } Array64_T;
// typedef const struct ArrayPtr { Array_State_T * const P_STATE; const size_t LENGTH; } ArrayPtr_T;

// typedef const struct GenericArray
// {
//     union
//     {
//         void * P_BUFFER;
//         uint8_t * P_ARRAY8;
//         uint16_t * P_ARRAY16;
//         uint32_t * P_ARRAY32;
//         uint64_t * P_ARRAY64;
//     };
//     size_t LENGTH;    // Length of the p_array
//     void * P_AUGMENTS;

//     // effective add to the unified interface, non value types
//     // Array_State_T * const P_STATE;  // Pointer to the p_array buffer, with augments
//     /* or seperate for extensions */
//     // void * const P_BUFFER;
// }
// GenericArray_T;

/* cast the wrapper type */
// #define Array_Get(p_array, index) \
//     _Generic(p_array, \
//         Array8_T *: Array8_Get, \
//         Array16_T *: Array16_Get, \
//         Array32_T *: Array32_Get, \
//         Array64_T *: Array64_Get  \
//         default : ArrayPtr_Get \
//     )(p_array, index)


// #define Array_Set(p_array, index, value) \
//     _Generic(value, \
//         int8_t  : Array8_Set, \
//         int16_t : Array16_Set, \
//         int32_t : Array32_Set, \
//         int64_t : Array64_Set, \
//         default  : ArrayPtr_Set \
//     )(p_array, index, value)


// #define Array_ForEach(T, p_array, unit_op) (for (size_t index = 0U; index < p_array->LENGTH; index++) { unit_op(((T *)p_array->P_BUFFER)); })

// static inline void Array_ForEach(size_t type, const Array_T * p_array, proc_t unit_op)
// {
//     for (size_t index = 0U; index < p_array->LENGTH; index++) { unit_op(void_pointer_at(p_array->P_BUFFER, type, index)); }
// }

// #define array_foreach(T, p_buffer, count, unit_op) \
//     _Generic((T)0, \
//         int8_t  : void_array_foreach(p_buffer, 1, count, unit_op), \
//         default  : void_array_foreach(p_buffer, sizeof(T), count, unit_op), \
//     )

// static inline void Array_SetEach(const Array_T * p_array, set_t unit_op, intptr_t value)
// {
//     void_array_foreach_set(p_array->P_BUFFER, p_array->UNIT_SIZE, p_array->LENGTH, unit_op, value);
// }

