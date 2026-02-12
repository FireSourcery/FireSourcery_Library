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
    @file   Vehicle_User.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Vehicle.h"
#include "Vehicle_StateMachine.h"


/******************************************************************************/
/*!
    DriveCmd - Throttle, Brake, Zero
    "state" changes on edge detection
    @param[in] driveCmd Throttle, Brake, Zero
    @param[in] cmdValue [0:65535]
*/
/******************************************************************************/
/*
    set to sync buffer. proc in thread
    Direction handle in state machine
*/
static inline void Vehicle_User_SetThrottle(Vehicle_State_T * p_this, uint16_t userCmd) { p_this->Input.ThrottleValue = userCmd; }
static inline void Vehicle_User_SetBrake(Vehicle_State_T * p_this, uint16_t userCmd) { p_this->Input.BrakeValue = userCmd; }
static inline void Vehicle_User_SetZero(Vehicle_State_T * p_this) { p_this->Input.ThrottleValue = 0U; p_this->Input.BrakeValue = 0U; }


/******************************************************************************/
/*!

*/
/******************************************************************************/
extern void Vehicle_User_PollStartCmd(Vehicle_T * p_vehicle);
extern void Vehicle_User_ApplyThrottle(Vehicle_T * p_vehicle, uint16_t userCmd);
extern void Vehicle_User_ApplyBrake(Vehicle_T * p_vehicle, uint16_t userCmd);
extern void Vehicle_User_ApplyZero(Vehicle_T * p_vehicle);

/******************************************************************************/
/*
    VarId Interface
*/
/******************************************************************************/
typedef enum Vehicle_VarId
{
    VEHICLE_VAR_DIRECTION,          // sign_t,
    VEHICLE_VAR_THROTTLE,           // [0:65535]
    VEHICLE_VAR_BRAKE,              // [0:65535]
}
Vehicle_VarId_T;

typedef enum Vehicle_ConfigId
{
    VEHICLE_CONFIG_THROTTLE_MODE,     /* Vehicle_ThrottleMode_T */
    VEHICLE_CONFIG_BRAKE_MODE,        /* Vehicle_BrakeMode_T */
    VEHICLE_CONFIG_ZERO_MODE,         /* Vehicle_ZeroMode_T */
}
Vehicle_ConfigId_T;

/* IO vars use full context */
extern int Vehicle_VarId_Get(const Vehicle_T * p_vehicle, Vehicle_VarId_T id);
extern void Vehicle_VarId_Set(const Vehicle_T * p_vehicle, Vehicle_VarId_T id, int value);

extern int Vehicle_ConfigId_Get(const Vehicle_State_T * p_this, Vehicle_ConfigId_T id);
extern void Vehicle_ConfigId_Set(Vehicle_State_T * p_this, Vehicle_ConfigId_T id, int value);
