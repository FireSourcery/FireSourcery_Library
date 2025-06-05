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
// typedef struct VarAccess_Var
// {
//     int ID;
//     get_t GET;
//     set_t SET;
//     proc_t ON_SET;
//     // int (*CALL)(void * p_context, voit * p_value);
// }
// VarAccess_Var_T;

/*
    Grouped Implementation
    compatibility with sub modules using switch()
    Single layer of wraping with generically typed function pointers
*/
// typedef const struct VarAccess_Def
typedef const struct VarAccess_VTable
{
    // struct { VarAccess_Var_T * P_VARS; size_t COUNT; };
    // test_t TEST_SET;

    get_at_t GET_AT;
    set_at_t SET_AT;
    test_t TEST_SET;
}
VarAccess_VTable_T;


/* Runtime state */
typedef struct
{
    // VarAccess_Instance_T Instance;
    // VarAccess_Var_T * p_LastAccess;
    int PrevAccess;
    int Mode; /* Enable/Disable */
}
VarAccess_State_T;

/*
    Handle, an instance implementing the VTable
*/
typedef const struct
{
    void * const P_BASE;
    const VarAccess_VTable_T * const P_VIRTUAL;
    VarAccess_State_T * const P_STATE;
}
VarAccess_T;

/* Multiple groups share the same runtime state, or provide direct state access to base */
#define VAR_ACCESS_INIT(p_Base, p_VTable, p_RunTime) { .P_BASE = (p_Base), .P_VIRTUAL = (p_VTable), .P_STATE = (p_RunTime), }
#define VAR_ACCESS_ALLOC(p_Base, p_VTable) VAR_ACCESS_INIT(p_Base, p_VTable, &(VarAccess_State_T){ 0 })

static inline void _VarAccess_EnableSet(VarAccess_State_T * p_state) { p_state->Mode = 1; }
static inline void _VarAccess_DisableSet(VarAccess_State_T * p_state) { p_state->Mode = 0; }

/* a getter is always defined */
static inline int _VarAccess_GetAt(const VarAccess_T * p_varAccess, int varId) { return p_varAccess->P_VIRTUAL->GET_AT(p_varAccess->P_BASE, varId); }

static inline int VarAccess_GetAt(const VarAccess_T * p_varAccess, int varId) { return call_get_at(p_varAccess->P_VIRTUAL->GET_AT, p_varAccess->P_BASE, varId); }
static inline void VarAccess_EnableSet(const VarAccess_T * p_varAccess) { _VarAccess_EnableSet(p_varAccess->P_STATE); }
static inline void VarAccess_DisableSet(const VarAccess_T * p_varAccess) { _VarAccess_EnableSet(p_varAccess->P_STATE); }


extern void VarAccess_SetAt(const VarAccess_T * p_varAccess, int varId, int varValue);
