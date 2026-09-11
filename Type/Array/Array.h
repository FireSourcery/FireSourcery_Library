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
typedef struct { void * P_BUFFER; size_t LENGTH; } ArraySpan_T;

// static allocation only, for inline convenience
#define _BUFFER_ALLOC(Bytes) ((void *)(alignas(uintptr_t) uint8_t[(Bytes)]){})
#define _ARRAY_ALLOC(T, Length) ((void *)(alignas(uintptr_t) T[(Length)]){})

/*
    Length in Units
*/
#define ARRAY_SPAN(p_Buffer, Length) { .P_BUFFER = (p_Buffer), .LENGTH = (Length) }
#define ARRAY_SPAN_ALLOC(TypeSize, Length) ARRAY_SPAN(_BUFFER_ALLOC((TypeSize) * (Length)), Length)
#define ARRAY_SPAN_ALLOC_AS(T, Length) ARRAY_SPAN(_ARRAY_ALLOC(T, Length), Length)



/*
    A contiguous run of LENGTH units of TYPE_SIZE at P_BUFFER. A view, not a descriptor:
    it is returned by value, so it is NOT const-qualified (a qualifier on a by-value return is ignored, and warns under -Wextra).
*/
typedef struct ArraySpanT { size_t TYPE_SIZE; void * P_BUFFER; size_t LENGTH; } ArraySpanT_T;

static inline size_t ArraySpan_Size(ArraySpanT_T span) { return span.LENGTH * span.TYPE_SIZE; }
static inline void * ArraySpan_At(ArraySpanT_T span, size_t index) { return void_array_at(span.TYPE_SIZE, span.P_BUFFER, index); }
static inline void ArraySpan_CopyTo(ArraySpanT_T span, void * p_dest) { memcpy(p_dest, span.P_BUFFER, ArraySpan_Size(span)); }
static inline void ArraySpan_CopyFrom(ArraySpanT_T span, const void * p_src) { memcpy(span.P_BUFFER, p_src, ArraySpan_Size(span)); }

static inline ArraySpanT_T ArraySpanT_Cast(size_t type, ArraySpan_T arrayBuffer) { return (ArraySpanT_T) { .TYPE_SIZE = type, .P_BUFFER = arrayBuffer.P_BUFFER, .LENGTH = arrayBuffer.LENGTH / type }; }


/*
    Void * or Struct
    compiler optimization to inline type size
*/
static inline intptr_t _ArrayT_GetV(size_t type, ArraySpan_T array, size_t index) { return void_array_get(type, array.P_BUFFER, index); }
static inline void _ArrayT_SetV(size_t type, ArraySpan_T array, size_t index, value_t value) { void_array_set(type, array.P_BUFFER, index, value); }

static inline void _ArrayT_Set(size_t type, ArraySpan_T array, size_t index, const void * p_value) { switch_copy(void_array_at(type, array.P_BUFFER, index), p_value, type); }
static inline void _ArrayT_Get(size_t type, ArraySpan_T array, size_t index, void * p_value) { switch_copy(p_value, void_array_at(type, array.P_BUFFER, index), type); }

static inline void ArrayT_CopyTo(size_t type, ArraySpan_T arrayBuffer, void * p_value) { ArraySpan_CopyTo(ArraySpanT_Cast(type, arrayBuffer), p_value); }
static inline void ArrayT_CopyFrom(size_t type, ArraySpan_T arrayBuffer, const void * p_value) { ArraySpan_CopyFrom(ArraySpanT_Cast(type, arrayBuffer), p_value); }


/*

*/
typedef const struct
{
    void * P_BUFFER;
    size_t LENGTH;
    void * P_AUGMENTS; // P_HEADER/P_STATE
}
ArrayContiguous_T;

#define _ARRAY_CONTIG_ALLOC(AugmentsSize, Length) _BUFFER_ALLOC((AugmentsSize) + (Length))
#define ARRAY_CONTIG_INIT(p_Alloc, AugmentsSize, Length) { .P_BUFFER = ((uint8_t *)(p_Alloc) + (AugmentsSize)), .LENGTH = (Length), .P_AUGMENTS = (p_Alloc) }