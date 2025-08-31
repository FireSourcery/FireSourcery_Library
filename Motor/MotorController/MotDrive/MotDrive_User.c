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
    @file   MotDrive_User.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotDrive_User.h"

/* Drive Direction */
MotDrive_Direction_T MotDrive_User_GetDirection(const MotDrive_T * p_motDrive)
{
    return MotDrive_StateMachine_GetDirection(p_motDrive);
}

/* Separate Check direction with alarm, so Motor set can use SetSyncInput */
bool MotDrive_User_CheckDirection(const MotDrive_T * p_motDrive, MotDrive_Direction_T direction)
{
    bool isSuccess = (MotDrive_User_GetDirection(p_motDrive) == direction);
    if (isSuccess == false) { Blinky_Blink(p_motDrive->P_BUZZER, 500U); }
    return isSuccess;
}

/*
    set to state machine async
*/
void MotDrive_User_ApplyDirection(const MotDrive_T * p_motDrive, MotDrive_Direction_T direction)
{
    if (MotDrive_User_GetDirection(p_motDrive) != direction)
    {
        _StateMachine_ProcInput(p_motDrive->STATE_MACHINE.P_ACTIVE, (void *)p_motDrive, MOT_DRIVE_STATE_INPUT_DIRECTION, direction);
        MotDrive_User_CheckDirection(p_motDrive, direction); /* effective on motor async transition only */
    }
}

/******************************************************************************/
/* Var Id */
/******************************************************************************/
void MotDrive_VarId_Set(const MotDrive_T * p_motDrive, MotDrive_VarId_T id, int value)
{
    switch (id)
    {
        case MOT_DRIVE_VAR_DIRECTION:   MotDrive_User_ApplyDirection(p_motDrive, (MotDrive_Direction_T)value);            break;
        case MOT_DRIVE_VAR_THROTTLE:    MotDrive_User_SetThrottle(p_motDrive->P_MOT_DRIVE_STATE, (uint16_t)value);      break;
        case MOT_DRIVE_VAR_BRAKE:       MotDrive_User_SetBrake(p_motDrive->P_MOT_DRIVE_STATE, (uint16_t)value);         break;
    }
}

int MotDrive_VarId_Get(const MotDrive_T * p_motDrive, MotDrive_VarId_T id)
{
    int value = 0;
    switch (id)
    {
        case MOT_DRIVE_VAR_DIRECTION:   value = MotDrive_User_GetDirection(p_motDrive);     break;
        case MOT_DRIVE_VAR_THROTTLE:    value = p_motDrive->P_MOT_DRIVE_STATE->Input.ThrottleValue;  break;
        case MOT_DRIVE_VAR_BRAKE:       value = p_motDrive->P_MOT_DRIVE_STATE->Input.BrakeValue;     break;
    }
    return value;
}


/******************************************************************************/
/* */
/******************************************************************************/
int MotDrive_ConfigId_Get(const MotDrive_State_T * p_motDriveState, MotDrive_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case MOT_DRIVE_CONFIG_THROTTLE_MODE:    value = p_motDriveState->Config.ThrottleMode;                 break;
        case MOT_DRIVE_CONFIG_BRAKE_MODE:       value = p_motDriveState->Config.BrakeMode;                    break;
        case MOT_DRIVE_CONFIG_ZERO_MODE:        value = p_motDriveState->Config.ZeroMode;                     break;
    }
    return value;
}

void MotDrive_ConfigId_Set(MotDrive_State_T * p_motDriveState, MotDrive_ConfigId_T id, int value)
{
    switch (id)
    {
        case MOT_DRIVE_CONFIG_THROTTLE_MODE:    p_motDriveState->Config.ThrottleMode = (MotDrive_ThrottleMode_T)value;     break;
        case MOT_DRIVE_CONFIG_BRAKE_MODE:       p_motDriveState->Config.BrakeMode = (MotDrive_BrakeMode_T)value;           break;
        case MOT_DRIVE_CONFIG_ZERO_MODE:        p_motDriveState->Config.ZeroMode = (MotDrive_ZeroMode_T)value;             break;
        default: break;
    }
}

