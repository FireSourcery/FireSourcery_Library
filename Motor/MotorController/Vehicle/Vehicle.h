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
    @file   Vehicle.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor/Motor/Motor_Table.h"
#include "Motor/Motor/Motor_Include.h"

#include "Transducer/Blinky/Blinky.h"



/*
    [Vehicle_Input_T]
*/
/* Cmd SubState use edge detection - DriveState */
typedef enum Vehicle_Cmd
{
    VEHICLE_CMD_RELEASE,
    VEHICLE_CMD_THROTTLE,
    VEHICLE_CMD_BRAKE,
}
Vehicle_Cmd_T;

/* alternatively convert to common interface */
typedef struct Vehicle_Input
{
    // sign_t Direction;
    uint16_t ThrottleValue;
    uint16_t BrakeValue;
    Vehicle_Cmd_T Cmd;
    Vehicle_Cmd_T CmdPrev;
}
Vehicle_Input_T;

static inline Vehicle_Cmd_T Vehicle_Input_EvaluateCmd(const Vehicle_Input_T * p_user)
{
    if      (p_user->BrakeValue > 0U)       { return VEHICLE_CMD_BRAKE; } // optionally check throttle active error
    else if (p_user->ThrottleValue > 0U)    { return VEHICLE_CMD_THROTTLE; }
    else                                    { return VEHICLE_CMD_RELEASE; }
}

/*
    For cases where Module handles edge detect
*/
static inline Vehicle_Cmd_T Vehicle_Input_PollCmd(Vehicle_Input_T * p_user)
{
    p_user->CmdPrev = p_user->Cmd; /* Save for Async Edge Check */
    p_user->Cmd = Vehicle_Input_EvaluateCmd(p_user);
    return p_user->Cmd;
}

static inline bool Vehicle_Input_PollCmdEdge(Vehicle_Input_T * p_user) { return (p_user->Cmd != Vehicle_Input_PollCmd(p_user)); }
static inline bool Vehicle_Input_IsCmdEdge(Vehicle_Input_T * p_user) { return (p_user->Cmd != p_user->CmdPrev); }


/*
    Config States
*/
typedef enum Vehicle_BrakeMode
{
    VEHICLE_BRAKE_MODE_PASSIVE,
    VEHICLE_BRAKE_MODE_TORQUE,
    VEHICLE_BRAKE_MODE_VOLTAGE,
}
Vehicle_BrakeMode_T;

typedef enum Vehicle_ThrottleMode
{
    VEHICLE_THROTTLE_MODE_SPEED,
    VEHICLE_THROTTLE_MODE_TORQUE,
    VEHICLE_THROTTLE_MODE_VOLTAGE,
}
Vehicle_ThrottleMode_T;

/* Release Input */
typedef enum Vehicle_ZeroMode
{
    VEHICLE_ZERO_MODE_FLOAT,       /* "Coast". MOSFETS non conducting. Same as Neutral. */
    VEHICLE_ZERO_MODE_REGEN,       /* Regen Brake */
    VEHICLE_ZERO_MODE_CRUISE,      /* Voltage following, Zero current/torque */
    VEHICLE_ZERO_MODE_ZERO,        /* Setpoint Zero. No cmd overwrite */
}
Vehicle_ZeroMode_T;

typedef struct Vehicle_Config
{
    Vehicle_ThrottleMode_T ThrottleMode;
    Vehicle_BrakeMode_T BrakeMode;
    Vehicle_ZeroMode_T ZeroMode;
}
Vehicle_Config_T;

typedef struct Vehicle_State
{
    Vehicle_Input_T Input;
    Vehicle_Config_T Config;
    // Vehicle_Input_T InputPrev;
    // Vehicle_Status_T Status;
    // StateMachine_Active_T StateMachine;
}
Vehicle_State_T;

/*
    [Vehicle Context]
    StateMachine with Sync Input
*/
typedef const struct Vehicle
{
    Vehicle_State_T * P_VEHICLE_STATE;
    // StateMachine_T STATE_MACHINE;
    Motor_Table_T MOTORS;
    // const Blinky_T * P_BUZZER;
    // const MotAnalogUser_T * P_ANALOG_USER;
    const Vehicle_Config_T * const P_NVM_CONFIG;
}
Vehicle_T;

// #define VEHICLE_INIT()

/******************************************************************************/
/*!
    DriveCmd - Throttle, Brake, Zero
    "state" changes on edge detection
    @param[in] driveCmd Throttle, Brake, Zero
    @param[in] cmdValue [0:65535]
*/
/******************************************************************************/
/*
    set to sync buffer. proc in state or input
    Direction handle in state machine
*/
static inline bool Vehicle_User_PollCmdEdge(Vehicle_State_T * p_this) { return Vehicle_Input_PollCmdEdge(&p_this->Input); }
static inline void Vehicle_User_SetThrottle(Vehicle_State_T * p_this, uint16_t userCmd) { p_this->Input.ThrottleValue = userCmd; }
static inline void Vehicle_User_SetBrake(Vehicle_State_T * p_this, uint16_t userCmd) { p_this->Input.BrakeValue = userCmd; }
static inline void Vehicle_User_SetZero(Vehicle_State_T * p_this) { p_this->Input.ThrottleValue = 0U; p_this->Input.BrakeValue = 0U; }


/******************************************************************************/
/*!

*/
/******************************************************************************/
extern void Vehicle_Init(const Vehicle_T * p_vehicle);

extern void Vehicle_StartThrottleMode(const Vehicle_T * p_vehicle);
extern void Vehicle_ApplyThrottleValue(const Vehicle_T * p_vehicle, uint16_t value);
extern void Vehicle_StartBrakeMode(const Vehicle_T * p_vehicle);
extern void Vehicle_ApplyBrakeValue(const Vehicle_T * p_vehicle, uint16_t value);
extern void Vehicle_StartDriveZero(const Vehicle_T * p_vehicle);
extern void Vehicle_ProcDriveZero(const Vehicle_T * p_vehicle);
extern void Vehicle_ProcInputCmd(const Vehicle_T * p_vehicle);
extern void Vehicle_StartCmdMode(const Vehicle_T * p_vehicle, Vehicle_Cmd_T mode);
extern void Vehicle_ProcThrottleValue(const Vehicle_T * p_vehicle);
extern void Vehicle_ProcBrakeValue(const Vehicle_T * p_vehicle);

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
// extern int Vehicle_VarId_Get(const Vehicle_T * p_vehicle, Vehicle_VarId_T id);
// extern void _Vehicle_VarId_Set(const Vehicle_T * p_vehicle, Vehicle_VarId_T id, int value);

extern int Vehicle_ConfigId_Get(const Vehicle_State_T * p_this, Vehicle_ConfigId_T id);
extern void Vehicle_ConfigId_Set(Vehicle_State_T * p_this, Vehicle_ConfigId_T id, int value);
