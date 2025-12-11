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
/* compatibility with sub modules using switch() */
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
