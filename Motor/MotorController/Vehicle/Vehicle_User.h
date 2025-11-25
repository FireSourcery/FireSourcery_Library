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
static inline void Vehicle_User_SetThrottle(Vehicle_State_T * p_vehicleState, uint16_t userCmd) { p_vehicleState->Input.ThrottleValue = userCmd; }
static inline void Vehicle_User_SetBrake(Vehicle_State_T * p_vehicleState, uint16_t userCmd) { p_vehicleState->Input.BrakeValue = userCmd; }
static inline void Vehicle_User_SetZero(Vehicle_State_T * p_vehicleState)
{
    p_vehicleState->Input.ThrottleValue = 0U;
    p_vehicleState->Input.BrakeValue = 0U;
}

/* Protocol Call sets Inner Machine only */
/* Will not exit park */
/* Caller Handle Edge Detection */
static inline void Vehicle_User_ApplyDirection(const Vehicle_T * p_vehicle, Motor_User_Direction_T direction)
{
    // if (Vehicle_StateMachine_GetDirection(p_vehicle) != direction)
    // {
    _StateMachine_ProcInput(p_vehicle->STATE_MACHINE.P_ACTIVE, (void *)p_vehicle, VEHICLE_STATE_INPUT_DIRECTION, direction);
    // Vehicle_User_CheckDirection(p_vehicle, direction); /* effective on motor async transition only */
    // bool isSuccess = (Vehicle_User_GetDirection(p_vehicle) == direction);
        // if (isSuccess == false) { Blinky_Blink(p_vehicle->P_BUZZER, 500U); }
// }
}

// Call on Park transition. caller handle Park state
// static inline void _Vehicle_User_ApplyPark(const Vehicle_T * p_vehicle)
// {
//     _StateMachine_ProcInput(p_vehicle->STATE_MACHINE.P_ACTIVE, (void *)p_vehicle, VEHICLE_STATE_INPUT_DIRECTION, MOTOR_DIRECTION_NONE);
//     // MotorController_User_EnterPark(p_vehicle->P_MC_STATE);
// }

/* Caller Handle Edge Detection */
static inline void Vehicle_User_ApplyStartCmd(const Vehicle_T * p_vehicle, Vehicle_Cmd_T cmd)
{
    _StateMachine_ProcInput(p_vehicle->STATE_MACHINE.P_ACTIVE, (void *)p_vehicle, VEHICLE_STATE_INPUT_DRIVE_CMD, cmd);
}

static inline void Vehicle_User_StartThrottle(const Vehicle_T * p_vehicle)  { Vehicle_User_ApplyStartCmd(p_vehicle, VEHICLE_CMD_THROTTLE); }
static inline void Vehicle_User_StartBrake(const Vehicle_T * p_vehicle)     { Vehicle_User_ApplyStartCmd(p_vehicle, VEHICLE_CMD_BRAKE); }


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
    VEHICLE_VAR_DIRECTION,          // Motor_User_Direction_T,
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

extern int Vehicle_ConfigId_Get(const Vehicle_State_T * p_vehicleState, Vehicle_ConfigId_T id);
extern void Vehicle_ConfigId_Set(Vehicle_State_T * p_vehicleState, Vehicle_ConfigId_T id, int value);



/******************************************************************************/
/*!
    Config Options
*/
/******************************************************************************/
// static inline Vehicle_BrakeMode_T _Vehicle_Config_GetBrakeMode(const Vehicle_Config_T * p_config) { return p_config; }
// static inline Vehicle_BrakeMode_T Vehicle_Config_GetBrakeMode(const Vehicle_State_T * p_vehicleState) { return p_vehicleState->Config.BrakeMode; }
// static inline Vehicle_ThrottleMode_T Vehicle_Config_GetThrottleMode(const Vehicle_State_T * p_vehicleState) { return p_vehicleState->Config.ThrottleMode; }
// static inline Vehicle_ZeroMode_T Vehicle_Config_GetZeroMode(const Vehicle_State_T * p_vehicleState) { return p_vehicleState->Config.ZeroMode; }

// static inline void Vehicle_Config_SetBrakeMode(Vehicle_State_T * p_vehicleState, Vehicle_BrakeMode_T mode) { p_vehicleState->Config.BrakeMode = mode; }
// static inline void Vehicle_Config_SetThrottleMode(Vehicle_State_T * p_vehicleState, Vehicle_ThrottleMode_T mode) { p_vehicleState->Config.ThrottleMode = mode; }
// static inline void Vehicle_Config_SetZeroMode(Vehicle_State_T * p_vehicleState, Vehicle_ZeroMode_T mode) { p_vehicleState->Config.ZeroMode = mode; }

