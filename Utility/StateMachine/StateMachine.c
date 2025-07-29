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
    @file   StateMachine.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "StateMachine.h"
#include "_StateMachine.h"

#include "_State.h"
#include "_State_Node.h"



/******************************************************************************/
/*!
    Public Functions
*/
/******************************************************************************/
/*
    States const strut defined at compile time
*/
void StateMachine_Init(StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
    /* sync mode uses no lock as valid input, no sentenial, set to run dummy input once */
    /* same as case of async unlock with write through dummy input */
    _StateMachine_SetSyncInput(p_active, 0, 0);
    Critical_ReleaseLock(&p_active->InputSignal);
    _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
}


void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
    StateMachine_Active_T * p_active = p_stateMachine->P_ACTIVE;
    if (_StateMachine_AcquireAsyncInput(p_active) == true)
    {
        _StateMachine_Init(p_active, p_stateMachine->P_CONTEXT, p_stateMachine->P_MACHINE->P_STATE_INITIAL);
        _StateMachine_ReleaseAsyncInput(p_active);
    }
}






