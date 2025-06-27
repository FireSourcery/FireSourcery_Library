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
    @file   VarAccess.c
    @author FireSourcery

    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "VarAccess.h"



void VarAccess_SetAt(const VarAccess_T * p_varAccess, int varId, int varValue)
{
    if (p_varAccess->P_STATE->Mode != 0)
    {
        if (call_test(p_varAccess->P_VIRTUAL->TEST_SET, p_varAccess->P_BASE) == true)
            { call_set_at(p_varAccess->P_VIRTUAL->SET_AT, p_varAccess->P_BASE, varId, varValue); }
    }
}