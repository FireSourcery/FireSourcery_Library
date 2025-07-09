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
    @file   MotDrive_User.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#include "MotDrive.h"
#include "MotDrive_StateMachine.h"


/******************************************************************************/
/*!
    DriveCmd - Throttle, Brake, Zero
    @param[in] driveCmd Throttle, Brake, Zero
    @param[in] cmdValue [0:65535]
*/
/******************************************************************************/
/*
    set to sync buffer. proc in thread
*/
static inline void MotDrive_User_SetThrottle(MotDrive_State_T * p_motDriveState, uint16_t userCmd) { p_motDriveState->Input.ThrottleValue = userCmd; }
static inline void MotDrive_User_SetBrake(MotDrive_State_T * p_motDriveState, uint16_t userCmd) { p_motDriveState->Input.BrakeValue = userCmd; }
static inline void MotDrive_User_SetZero(MotDrive_State_T * p_motDriveState)
{
    p_motDriveState->Input.ThrottleValue = 0U;
    p_motDriveState->Input.BrakeValue = 0U;
}

extern MotDrive_Direction_T MotDrive_User_GetDirection(const MotDrive_T * p_this);
extern void MotDrive_User_SetDirection(const MotDrive_T * p_this, MotDrive_Direction_T direction);

/******************************************************************************/
/*!
    Config Options
*/
/******************************************************************************/
// static inline MotDrive_BrakeMode_T _MotDrive_Config_GetBrakeMode(const MotDrive_Config_T * p_config) { return p_config; }

// static inline MotDrive_BrakeMode_T MotDrive_Config_GetBrakeMode(const MotDrive_State_T * p_motDriveState) { return p_motDriveState->Config.BrakeMode; }
// static inline MotDrive_ThrottleMode_T MotDrive_Config_GetThrottleMode(const MotDrive_State_T * p_motDriveState) { return p_motDriveState->Config.ThrottleMode; }
// static inline MotDrive_ZeroMode_T MotDrive_Config_GetZeroMode(const MotDrive_State_T * p_motDriveState) { return p_motDriveState->Config.ZeroMode; }

// static inline void MotDrive_Config_SetBrakeMode(MotDrive_State_T * p_motDriveState, MotDrive_BrakeMode_T mode) { p_motDriveState->Config.BrakeMode = mode; }
// static inline void MotDrive_Config_SetThrottleMode(MotDrive_State_T * p_motDriveState, MotDrive_ThrottleMode_T mode) { p_motDriveState->Config.ThrottleMode = mode; }
// static inline void MotDrive_Config_SetZeroMode(MotDrive_State_T * p_motDriveState, MotDrive_ZeroMode_T mode) { p_motDriveState->Config.ZeroMode = mode; }


/******************************************************************************/
/*
    VarId Interface
*/
/******************************************************************************/
typedef enum MotDrive_VarId
{
    MOT_DRIVE_VAR_DIRECTION,          // MotDrive_Direction_T,
    MOT_DRIVE_VAR_THROTTLE,           // [0:65535]
    MOT_DRIVE_VAR_BRAKE,              // [0:65535]
}
MotDrive_VarId_T;

typedef enum MotDrive_ConfigId
{
    MOT_DRIVE_CONFIG_THROTTLE_MODE,     /* MotDrive_ThrottleMode_T */
    MOT_DRIVE_CONFIG_BRAKE_MODE,        /* MotDrive_BrakeMode_T */
    MOT_DRIVE_CONFIG_ZERO_MODE,         /* MotDrive_ZeroMode_T */
}
MotDrive_ConfigId_T;

/* IO vars use full context */
extern void MotDrive_VarId_Set(const MotDrive_T * p_motDrive, MotDrive_VarId_T id, int value);
extern int MotDrive_VarId_Get(const MotDrive_T * p_motDrive, MotDrive_VarId_T id);

extern int MotDrive_ConfigId_Get(const MotDrive_State_T * p_motDriveState, MotDrive_ConfigId_T id);
extern void MotDrive_ConfigId_Set(MotDrive_State_T * p_motDriveState, MotDrive_ConfigId_T id, int value);

