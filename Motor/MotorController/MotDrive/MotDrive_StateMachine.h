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
static inline void MotDrive_StateMachine_Proc(const MotDrive_T * p_motDrive)
{
    if (MotDrive_Input_PollCmdEdge(&p_motDrive->P_MOT_DRIVE_STATE->Input) == true)
    {
        _StateMachine_ApplyAsyncInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_CMD_START, p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd);
    }

    _StateMachine_ProcState(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive);
}


static Motor_User_Direction_T MotDrive_StateMachine_GetDirection(const MotDrive_T * p_motDrive)
{
    Motor_User_Direction_T direction;
    switch (StateMachine_GetActiveStateId(p_motDrive->STATE_MACHINE.P_ACTIVE))
    {
        case MOT_DRIVE_STATE_ID_NEUTRAL:    direction = MOTOR_DIRECTION_NONE; break;
        case MOT_DRIVE_STATE_ID_DRIVE:      direction = (Motor_User_Direction_T)_MotMotors_GetDirectionAll(&p_motDrive->MOTORS);    break;
        default:                            direction = 0;           break;
    }
    return direction;
}


static inline void MotDrive_StateMachine_ApplyDirection(const MotDrive_T * p_motDrive, Motor_User_Direction_T direction)
{
    // if (MotDrive_StateMachine_GetDirection(p_motDrive) != direction)
    // {
        _StateMachine_ProcInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_DIRECTION, direction);
        // MotDrive_User_CheckDirection(p_motDrive, direction); /* effective on motor async transition only */
        // bool isSuccess = (MotDrive_User_GetDirection(p_motDrive) == direction);
            // if (isSuccess == false) { Blinky_Blink(p_motDrive->P_BUZZER, 500U); }
    // }
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
// extern MotDrive_Direction_T MotDrive_StateMachine_GetDirection(const MotDrive_T * p_motDrive);/* Drive Direction */
// Motor_User_Direction_T MotDrive_User_GetDirection(const MotDrive_T * p_motDrive)
// {
//     return MotDrive_StateMachine_GetDirection(p_motDrive);
// }

// /* Separate Check direction with alarm, so Motor set can use SetSyncInput */
// bool MotDrive_User_CheckDirection(const MotDrive_T * p_motDrive, Motor_User_Direction_T direction)
// {
//     bool isSuccess = (MotDrive_User_GetDirection(p_motDrive) == direction);
//     if (isSuccess == false) { Blinky_Blink(p_motDrive->P_BUZZER, 500U); }
//     return isSuccess;
// }

// /*
//     set to state machine async
// */
// void MotDrive_User_ApplyDirection(const MotDrive_T * p_motDrive, Motor_User_Direction_T direction)
// {
//     if (MotDrive_User_GetDirection(p_motDrive) != direction)
//     {
//         _StateMachine_ProcInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_DIRECTION, direction);
//         MotDrive_User_CheckDirection(p_motDrive, direction); /* effective on motor async transition only */
//     }
// }