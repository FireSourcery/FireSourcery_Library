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
    @file   MotDrive_StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotDrive.h"
#include "Utility/StateMachine/StateMachine.h"

/*
    State Machine
*/
typedef enum MotDrive_StateMachine_Input
{
    MOT_DRIVE_STATE_INPUT_DIRECTION,    /* Drive Direction */
    MOT_DRIVE_STATE_INPUT_CMD_START,    /* Edge */
}
MotDrive_State_Input_T;

typedef enum MotDrive_StateId
{
    MOT_DRIVE_STATE_ID_DRIVE,
    MOT_DRIVE_STATE_ID_NEUTRAL,
}
MotDrive_StateId_T;

extern const StateMachine_Machine_T MOT_DRIVE_MACHINE;

#define MOT_DRIVE_STATE_MACHINE_INIT(p_MotDriveContext, MotDriveActiveStruct) STATE_MACHINE_INIT((p_MotDriveContext), &MOT_DRIVE_MACHINE, &((MotDriveActiveStruct).StateMachine))


/* proc 1-10ms */
/*
    No sync lock. inputs are on the same thread
    handle edge actions
    Proc synchronous
*/
static inline void MotDrive_StatMachine_Proc(const MotDrive_T * p_motDrive)
{
    if (MotDrive_Input_PollCmdEdge(&p_motDrive->P_MOT_DRIVE_STATE->Input) == true)
    {
        _StateMachine_ApplyAsyncInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd);
    }

    _StateMachine_ProcState(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive);
}


/******************************************************************************/
/*!

*/
/******************************************************************************/

extern MotDrive_Direction_T MotDrive_StateMachine_GetDirection(const MotDrive_T * p_motDrive);