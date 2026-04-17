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
    @file   StateMachine_Sync.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../StateMachine.h"


/******************************************************************************/
/*
    [Synchronous Machine]
    Synchronous Proc State - [ProcState] synchronous to clock/timer
    Synchronous Proc Input - Async user [SetInput] proc synchronously during [ProcState]
        [SetInput] will noy delay periodic [ProcSyncOutput]
        Proc last [SetInput], inputs overwrite. always single threaded proc

    Lock Input buffer only.

    Does not need EnterCritical/DisableIRQ, when:
        [ProcState] thread is higher priority than [SetInput] thread
        [SetInput] calls are on the same thread
        Check accept on [ProcState]

    Atomic signal skip [ProcState], or spin-wait in low priority input thread

*/
/******************************************************************************/
static void StateMachine_Sync_ProcState(const StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;

    if (_StateMachine_AcquireSyncInput(p_active) == true)  /* set clear/1, lock until next input */
    {
        _StateMachine_ProcSyncInput(p_active, p_stateMachine->P_CONTEXT);
        _StateMachine_ReleaseSyncInput(p_active);
    }

    /* ISR wont interrupt an Input transition in this case. All Inputs are [SetSyncInput], all transition are processed Sync, [ProcSyncOutput] will not interrupt a transition  */
    _StateMachine_ProcSyncOutput(p_active, p_stateMachine->P_CONTEXT);
}



