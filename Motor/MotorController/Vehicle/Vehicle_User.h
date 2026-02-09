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

/* Caller Handle Edge Detection */
static inline void Vehicle_User_ApplyStartCmd(const Vehicle_T * p_vehicle, Vehicle_Cmd_T cmd)
{
    _StateMachine_ProcInput(p_vehicle->STATE_MACHINE.P_ACTIVE, (void *)p_vehicle, VEHICLE_STATE_INPUT_DRIVE_CMD, cmd);
}

static inline void Vehicle_User_StartThrottle(const Vehicle_T * p_vehicle)  { Vehicle_User_ApplyStartCmd(p_vehicle, VEHICLE_CMD_THROTTLE); }
static inline void Vehicle_User_StartBrake(const Vehicle_T * p_vehicle)     { Vehicle_User_ApplyStartCmd(p_vehicle, VEHICLE_CMD_BRAKE); }


/* Protocol Call sets Inner Machine only */
/* Will not exit park */
/* Caller Handle Edge Detection */
static inline void Vehicle_User_ApplyDirection(const Vehicle_T * p_vehicle, Motor_Direction_T direction)
{
    // if (Vehicle_StateMachine_GetDirection(p_vehicle) != direction)
    // {
    _StateMachine_ProcInput(p_vehicle->STATE_MACHINE.P_ACTIVE, (void *)p_vehicle, VEHICLE_STATE_INPUT_DIRECTION, direction);
    // Vehicle_User_CheckDirection(p_vehicle, direction); /* effective on motor async transition only */
    // bool isSuccess = (Vehicle_User_GetDirection(p_vehicle) == direction);
        // if (isSuccess == false) { Blinky_Blink(p_vehicle->P_BUZZER, 500U); }
    // }
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* also returns NEUTRAL on error */
/* Alternatively use substates */
static Motor_Direction_T Vehicle_StateMachine_GetDirection(const Vehicle_T * p_vehicle)
{
    Motor_Direction_T direction;
    switch (StateMachine_GetActiveStateId(p_vehicle->STATE_MACHINE.P_ACTIVE))
    {
        case VEHICLE_STATE_ID_NEUTRAL:    direction = MOTOR_DIRECTION_NULL; break;
        case VEHICLE_STATE_ID_DRIVE:      direction = _Motor_Table_GetDirectionAll(&p_vehicle->MOTORS); break; /* NULL is error */
        default:                          direction = 0;           break;
    }
    return direction;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
// /* Separate Check direction with alarm, so Motor set can use SetSyncInput */
// bool Vehicle_User_CheckDirection(const Vehicle_T * p_vehicle, Motor_Direction_T direction)
// {
//     bool isSuccess = (Vehicle_User_GetDirection(p_vehicle) == direction);
//     if (isSuccess == false) { Blinky_Blink(p_vehicle->P_BUZZER, 500U); }
//     return isSuccess;
// }


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
    VEHICLE_VAR_DIRECTION,          // Motor_Direction_T,
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
