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
// typedef struct MotDrive_Active MotDrive_Active_T;

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

#define MOT_DRIVE_STATE_MACHINE_INIT(p_MotDriveConst, p_MotDriveActive) STATE_MACHINE_INIT((p_MotDriveConst), &MOT_DRIVE_MACHINE, &((p_MotDriveActive)->StateMachine))


/* proc 1-10ms */
/*
    No sync lock. inputs are on the same thread
    handle edge actions
*/
static inline void MotDrive_Proc_Thread(const MotDrive_T * p_motDrive)
{
    if (MotDrive_Input_PollCmdEdge(&p_motDrive->P_MOT_DRIVE->Input) == true)
    {
        switch (p_motDrive->P_MOT_DRIVE->Input.Cmd)
        {
            case MOT_DRIVE_CMD_BRAKE: _StateMachine_ProcInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, MOT_DRIVE_CMD_BRAKE); break;
            case MOT_DRIVE_CMD_THROTTLE: _StateMachine_ProcInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, MOT_DRIVE_CMD_THROTTLE); break;
            case MOT_DRIVE_CMD_RELEASE: _StateMachine_ProcInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, MOT_DRIVE_CMD_RELEASE); break;
            default: break;
        }
    }

    _StateMachine_ProcState(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive);
}


/*

*/
static inline MotDrive_Direction_T _MotDrive_GetDirection(const MotDrive_T * p_motDrive)
{
    MotDrive_Direction_T direction;
    if      (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) == true)  { direction = MOT_DRIVE_DIRECTION_FORWARD; }
    else if (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse) == true)  { direction = MOT_DRIVE_DIRECTION_REVERSE; }
    else                                                                                { direction = MOT_DRIVE_DIRECTION_ERROR; }
    return direction;
}

static inline MotDrive_Direction_T MotDrive_StateMachine_GetDirection(const MotDrive_T * p_motDrive)
{
    MotDrive_Direction_T direction;
    switch (StateMachine_GetActiveStateId(&p_motDrive->P_MOT_DRIVE->StateMachine))
    {
        case MOT_DRIVE_STATE_ID_PARK:       direction = MOT_DRIVE_DIRECTION_PARK;            break;
        case MOT_DRIVE_STATE_ID_NEUTRAL:    direction = MOT_DRIVE_DIRECTION_NEUTRAL;         break;
        case MOT_DRIVE_STATE_ID_DRIVE:      direction = _MotDrive_GetDirection(p_motDrive);  break;
        default:                            direction = MOT_DRIVE_DIRECTION_ERROR;           break;
    }
    return direction;
}


/*
    transition form park
*/
// static inline MotDrive_Status_T MotDrive_StateMachine_GetStatus(const MotDrive_T * p_handle)
// {
//     // StateMachine_GetActiveStateId(&p_handle->STATE_MACHINE);
//     return (p_handle->STATE_MACHINE.P_ACTIVE->p_ActiveState == &MOT_DRIVE_STATE_PARK) ? MOT_DRIVE_STATUS_FAULT : MOT_DRIVE_STATUS_OK;
// }