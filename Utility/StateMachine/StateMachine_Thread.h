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
    @file   StateMachine_Thread.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "StateMachine.h"
#include "_StateMachine.h" /* Include the private header for inline defs */

/******************************************************************************/
/*
    StateMachine Thread
    inline for compile time expansion
    [Combined Semi-Synchronous Machine]
    supports both operation at expense of additional condition check
*/
/******************************************************************************/
static inline void _StateMachine_Synchronous_Thread(StateMachine_Active_T * p_active, void * p_context)
{
    if (_StateMachine_AcquireAsyncIsr(p_active) == true)
    {
        _StateMachine_ProcState(p_active, p_context);
        _StateMachine_ReleaseAsyncIsr(p_active);
    }
}

static inline void StateMachine_Synchronous_Thread(StateMachine_T * p_stateMachine)
{
    _StateMachine_Synchronous_Thread(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT);
}

// hsm
// static inline void StateMachine_TreeSynchronous_Thread(  StateMachine_T * p_stateMachine)
// {
//     StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
//     if (_StateMachine_AcquireAsyncIsr(p_active) == true)
//     {
//         _StateMachine_ProcBranchSyncInput(p_active, p_stateMachine->P_CONTEXT);
//         _StateMachine_ProcBranchSyncOutput(p_active, p_stateMachine->P_CONTEXT, NULL);
//         _StateMachine_ReleaseAsyncIsr(p_active);
//     }
// }

