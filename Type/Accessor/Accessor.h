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
    @file   Accessor.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Type/accessor.h"

/* IO Map */
/*
    compatibility with sub modules using switch()
    includes/circumvents handling function pointers with different signatures
*/
typedef const struct Accessor
{
    get_field_t GET_FIELD;
    set_field_t SET_FIELD;
}
Accessor_T;

static inline int Accessor_Get(Accessor_T * p_this, void * p_context, int id) { return p_this->GET_FIELD(p_context, id); }
static inline void Accessor_Set(Accessor_T * p_this, void * p_context, int id, int value) { p_this->SET_FIELD(p_context, id, value); }
// static inline int Accessor_SetWithGuard(Accessor_T * p_this, void * p_context, int id, int value) { p_this->SET_FIELD(p_context, id, value); }


/*
    Struct per Var implementation
    Each Var must have generically defined getter/setter
*/
typedef struct VField
{
    get_t GET;
    set_t SET;
}
VField_T;

/*
    Grouped Implementation
    Single layer of wraping with generically typed function pointers
*/
typedef const struct VField_Table
{
    VField_T * P_VARS;
    size_t COUNT;
    // proc_t ON_SET;
    // test_t TEST_SET;
}
VField_Table_T;

// static inline int _VarAccess_GetAt(VField_Table_T * p_varAccess, void * p_context, int varId) { return p_varAccess->P_VARS[varId].GET(p_context); }


/*
    Field Accessor
     - Generic field getter/setter by offset and size
     - Can be used for direct struct field access, or as a layer of indirection for virtual properties
     - Handles 1/2/4 byte fields, zero-extends to int32_t for getter, truncates from int32_t for setter
     - Can be used in tables for indexed access
*/

/* Field Descriptor */
typedef const struct Field
{
    size_t SIZE;
    size_t OFFSET;
}
Field_T;

#define FIELD(type, field) ((Field_T){ .SIZE = sizeof(((type *)0)->field), .OFFSET = offsetof(type, field) })

static inline int32_t get_field(const void * p_context, size_t size, size_t offset)
{
    const uint8_t * p_base = (const uint8_t *)p_context + offset;
    int32_t value;
    switch (size)
    {
        case 1U: value = (int32_t)(*(const int8_t *)p_base);   break;
        case 2U: value = (int32_t)(*(const int16_t *)p_base);  break;
        case 4U: value = *(const int32_t *)p_base;             break;
        default: value = 0;                                    break;
    }
    return value;
}

static inline void set_field(void * p_context, size_t size, size_t offset, int32_t value)
{
    uint8_t * p_base = (uint8_t *)p_context + offset;
    switch (size)
    {
        case 1U: *(int8_t *)p_base = (int8_t)value;     break;
        case 2U: *(int16_t *)p_base = (int16_t)value;   break;
        case 4U: *(int32_t *)p_base = value;            break;
        default:                                        break;
    }
}

/*
    Read a field from a struct by descriptor.
    Returns value zero-extended to int32_t. Handles 1/2/4 byte fields.
*/
static inline int32_t Field_Get(Field_T field, const void * p_context) { return get_field(p_context, field.SIZE, field.OFFSET); }

/*
    Write a field in a struct by descriptor.
*/
static inline void Field_Set(Field_T field, void * p_context, int32_t value) { set_field(p_context, field.SIZE, field.OFFSET, value); }

// static inline int32_t Field_Table_Get(const void * p_context, const Field_T * p_table, int id) { return Field_Get(p_table[id], p_context); }

// static inline void Field_Table_Set(void * p_context, const Field_T * p_table, int id, int32_t value) { Field_Set(p_table[id], p_context, value); }