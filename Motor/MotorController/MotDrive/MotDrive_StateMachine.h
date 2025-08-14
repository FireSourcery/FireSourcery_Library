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

// typedef struct MotDrive MotDrive_T;
// typedef struct MotDrive_State MotDrive_State_T;

/*
    State Machine
*/
typedef enum MotDrive_StateMachine_Input
{
    MOT_DRIVE_STATE_INPUT_DIRECTION,    /* Drive Direction */
    MOT_DRIVE_STATE_INPUT_CMD_START,    /* Edge */
    // MOT_DRIVE_STATE_INPUT_THROTTLE, /* Polling inputs */
    // MOT_DRIVE_STATE_INPUT_BRAKE,
}
MotDrive_State_Input_T;

typedef enum MotDrive_StateId
{
    MOT_DRIVE_STATE_ID_PARK,
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
*/
static inline void MotDrive_StatMachine_Proc(const MotDrive_T * p_motDrive)
{
    if (MotDrive_Input_PollCmdEdge(&p_motDrive->P_MOT_DRIVE_STATE->Input) == true)
    {
        switch (p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd)
        {
            case MOT_DRIVE_CMD_BRAKE: _StateMachine_ApplyAsyncInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, MOT_DRIVE_CMD_BRAKE); break;
            case MOT_DRIVE_CMD_THROTTLE: _StateMachine_ApplyAsyncInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, MOT_DRIVE_CMD_THROTTLE); break;
            case MOT_DRIVE_CMD_RELEASE: _StateMachine_ApplyAsyncInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, MOT_DRIVE_CMD_RELEASE); break;
            default: break;
        }
    }

    _StateMachine_ProcState(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive);
}

/*
    transition from park
*/
// static inline bool MotDrive_StateMachine_CheckExit(const MotDrive_T * p_motDrive)
// {
//     return StateMachine_IsActiveStateId(p_motDrive->STATE_MACHINE.P_ACTIVE, MOT_DRIVE_STATE_ID_PARK);
// }

// static inline MotDrive_Status_T MotDrive_StateMachine_GetStatus(const MotDrive_T * p_handle)
// {
//     // StateMachine_GetActiveStateId(&p_handle->STATE_MACHINE);
//     return (p_handle->STATE_MACHINE.P_MOT_DRIVE_STATE->p_ActiveState == &MOT_DRIVE_STATE_PARK) ? MOT_DRIVE_STATUS_FAULT : MOT_DRIVE_STATUS_OK;
// }


/******************************************************************************/
/*!

*/
/******************************************************************************/
extern MotDrive_Direction_T MotDrive_StateMachine_GetDirection(const MotDrive_T * p_motDrive);