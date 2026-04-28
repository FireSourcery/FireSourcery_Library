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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>


/******************************************************************************/
/*
    Wrapper around values array, preset functions for int types and operations
*/
/******************************************************************************/
typedef const union
{
    uint8_t * P_ARRAY8;
    uint16_t * P_ARRAY16;
    uint32_t * P_ARRAY32;
    uint64_t * P_ARRAY64;
}_Array_Cast;

typedef const struct Array
{
    const union { void * P_BUFFER; _Array_Cast CAST; };
    size_t LENGTH;    // Length of the p_array
    void * P_AUGMENTS; // P_HEADER/P_STATE
}
Array_T;

typedef const struct
{
    const union { void * P_BUFFER; _Array_Cast CAST; };
    size_t LENGTH;
}
Array_Span_T;

typedef const struct Array
{
    const union { void * P_BUFFER; _Array_Cast CAST; };
    size_t LENGTH;    // Length of the p_array
    void * P_AUGMENTS; // P_HEADER/P_STATE
}
// Array_Handle_T;
Array_Params_T;



// static allocation only, for inline convenience
#define _BUFFER_ALLOC(Bytes) ((void *)(alignas(uintptr_t) uint8_t[(Bytes)]){})
#define _ARRAY_ALLOC(T, Length) ((void *)(alignas(uintptr_t) T[(Length)]){})

/*
    Length in Units
*/
#define ARRAY_INIT(p_Buffer, Length, p_Augments) { .P_BUFFER = (p_Buffer), .LENGTH = (Length), .P_AUGMENTS = (p_Augments) }
#define ARRAY_INIT_ALLOC(TypeSize, Length, p_Augments) ARRAY_INIT(_BUFFER_ALLOC((TypeSize) * (Length)), Length, p_Augments)
#define ARRAY_INIT_ALLOC_AS(T, Length, p_Augments) ARRAY_INIT(_ARRAY_ALLOC(T, Length), Length, p_Augments)


#define _ARRAY_AUGMENTED_ALLOC(AugmentsSize, Length) ((void *)(alignas(uintptr_t) uint8_t[(AugmentsSize) * (Length)]){})
#define ARRAY_AUGMENTED_INIT(p_Alloc, AugmentsSize, Length) { .P_BUFFER = ((uint8_t *)(p_Alloc) + (AugmentsSize)), .LENGTH = (Length), .P_AUGMENTS = (p_Alloc) }

/*
    Void * or Struct
    compiler optimization to inline type size
*/
static inline intptr_t Array_GetV(size_t type, Array_T * p_array, size_t index) { return void_array_get(type, p_array->P_BUFFER, index); }
static inline void Array_SetV(size_t type, Array_T * p_array, size_t index, value_t value) { void_array_set(type, p_array->P_BUFFER, index, value); }

static inline void Array_Set(size_t type, Array_T * p_array, size_t index, const void * p_value) { memcpy(void_array_at(type, p_array->P_BUFFER, index), p_value, type); }
static inline void Array_Get(size_t type, Array_T * p_array, size_t index, void * p_value) { memcpy(p_value, void_array_at(type, p_array->P_BUFFER, index), type); }

// static inline void Array_Set(size_t type, Array_T array, size_t index, const void * p_value) { memcpy(void_array_at(type, array.P_BUFFER, index), p_value, type); }
// static inline void Array_Get(size_t type, Array_T array, size_t index, void * p_value) { memcpy(p_value, void_array_at(type, array.P_BUFFER, index), type); }
