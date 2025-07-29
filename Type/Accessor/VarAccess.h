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
    @file   VarAccess.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#include "Type/accessor.h"

/*
    check state - restrict state by interface
    map to int accessor, or keyed accessor
    call accessor handle error checking only

    map to int accessor
    check state - restrict state by accessor/id
    call accessor handle check state - state restriction per accessor/id
*/

/*
    Struct per Var implementation
    Each Var must have generically defined getter/setter
*/
// typedef struct Accessor_Var
// typedef struct Accessor_Field
typedef struct VarAccess_Var
{
    int ID;
    get_t GET;
    set_t SET;
    // proc_t ON_SET;
    // int (*CALL)(void * p_context, voit * p_value); /* return a status */
}
VarAccess_Var_T;

/*
    Grouped Implementation
    Single layer of wraping with generically typed function pointers
*/
// typedef const struct VarAccess_TypeGroup
typedef const struct VarAccess_VarTable
{
    VarAccess_Var_T * P_VARS;
    size_t COUNT;
    // proc_t ON_SET;
    // test_t TEST_SET;
}
VarAccess_VarTable_T;

/* compatibility with sub modules using switch() */
typedef const struct VarAccess_FieldHandler
{
    get_field_t GET_FIELD;
    set_field_t SET_FIELD;
    test_t TEST_SET;
}
VarAccess_FieldHandler_T;

typedef enum VarAccess_Mode
{
    VAR_ACCESS_MODE_READ_ONLY,
    VAR_ACCESS_MODE_READ_WRITE,
}
VarAccess_Mode_T;

/* Runtime state */
typedef struct
{
    VarAccess_Mode_T Mode; /* Enable/Disable */
    // VarAccess_Var_T * p_LastAccess;
    int PrevAccessId;
    int Status;
}
VarAccess_State_T;

/*
    Handle, an instance implementing the VTable
*/
typedef const struct
{
    void * P_BASE; /* context instance */
    const VarAccess_FieldHandler_T * P_VIRTUAL;
    VarAccess_State_T * P_STATE;
}
VarAccess_T;

/* Multiple groups share the same runtime state, or provide direct state access to base */
#define VAR_ACCESS_INIT(p_Base, p_VTable, p_RunTime) { .P_BASE = ((void *)(p_Base)), .P_VIRTUAL = (p_VTable), .P_STATE = (p_RunTime), }
// #define VAR_ACCESS_ALLOC(p_Base, p_VTable) VAR_ACCESS_INIT(p_Base, p_VTable, &(VarAccess_State_T){0})

static inline void _VarAccess_EnableSet(VarAccess_State_T * p_state) { p_state->Mode = VAR_ACCESS_MODE_READ_WRITE; }
static inline void _VarAccess_DisableSet(VarAccess_State_T * p_state) { p_state->Mode = VAR_ACCESS_MODE_READ_ONLY; }

// static inline int _VarAccess_GetAt(const VarAccess_T * p_varAccess, int varId) { return p_varAccess->P_VIRTUAL->P_VARS[varId].GET(p_varAccess->P_BASE); }

/* a getter is always defined */
static inline int _VarAccess_GetAt(const VarAccess_T * p_varAccess, int varId) { return p_varAccess->P_VIRTUAL->GET_FIELD(p_varAccess->P_BASE, varId); }
/* caller handle additional testset */
// if (p_varAccess->P_VIRTUAL->TEST_SET(p_varAccess->P_BASE) == true)
static inline void _VarAccess_SetAt(const VarAccess_T * p_varAccess, int varId, int varValue) { p_varAccess->P_VIRTUAL->SET_FIELD(p_varAccess->P_BASE, varId, varValue); }

static inline void VarAccess_EnableSet(const VarAccess_T * p_varAccess) { _VarAccess_EnableSet(p_varAccess->P_STATE); }
static inline void VarAccess_DisableSet(const VarAccess_T * p_varAccess) { _VarAccess_EnableSet(p_varAccess->P_STATE); }

static inline int VarAccess_GetAt(const VarAccess_T * p_varAccess, int varId) { return call_get_at(p_varAccess->P_VIRTUAL->GET_FIELD, p_varAccess->P_BASE, varId); }
/* per compilation unit without inline */
static void VarAccess_SetAt(const VarAccess_T * p_varAccess, int varId, int varValue)
{
    if (p_varAccess->P_STATE->Mode != VAR_ACCESS_MODE_READ_ONLY)
    {
        if (call_test(p_varAccess->P_VIRTUAL->TEST_SET, p_varAccess->P_BASE) == true)
            { call_set_at(p_varAccess->P_VIRTUAL->SET_FIELD, p_varAccess->P_BASE, varId, varValue); }
    }
}