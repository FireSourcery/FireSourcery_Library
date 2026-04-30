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
    @file   Traction.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor/Motor/Motor_Table.h"
#include "Motor/Motor/Motor_Include.h"

#include "Transducer/Blinky/Blinky.h"


/*
    [Traction_Input_T]
*/
/* Cmd SubState use edge detection - DriveState */
typedef enum Traction_Cmd
{
    TRACTION_CMD_RELEASE,
    TRACTION_CMD_THROTTLE,
    TRACTION_CMD_BRAKE,
}
Traction_Cmd_T;

/*
    Traction_User
*/
/* alternatively convert to MotorInput */
typedef struct Traction_Input
{
    sign_t Direction;
    uint16_t ThrottleValue;
    uint16_t BrakeValue;
    Traction_Cmd_T DriveCmd;
}
Traction_Input_T;
/* optionally handle park,fnr in 1 id */

static inline Traction_Cmd_T Traction_Input_EvaluateCmd(const Traction_Input_T * p_input)
{
    if      (p_input->BrakeValue > 0U)       { return TRACTION_CMD_BRAKE; } // optionally check throttle active error
    else if (p_input->ThrottleValue > 0U)    { return TRACTION_CMD_THROTTLE; }
    else                                     { return TRACTION_CMD_RELEASE; }
}

/*
    For cases where Module handles cmd edge detect
*/
static inline Traction_Cmd_T Traction_Input_PollCmd(Traction_Input_T * p_input)
{
    p_input->DriveCmd = Traction_Input_EvaluateCmd(p_input);
    return p_input->DriveCmd;
}

static inline bool Traction_Input_PollCmdEdge(Traction_Input_T * p_input) { return (p_input->DriveCmd != Traction_Input_PollCmd(p_input)); }

/*
    Individually set inputs
*/
static inline bool Traction_Input_PollThrottle(Traction_Input_T * p_input, uint16_t userCmd)
{
    p_input->ThrottleValue = userCmd;
    return Traction_Input_PollCmdEdge(p_input);
}

static inline bool Traction_Input_PollBrake(Traction_Input_T * p_input, uint16_t userCmd)
{
    p_input->BrakeValue = userCmd;
    return Traction_Input_PollCmdEdge(p_input);
}

static inline bool Traction_Input_PollDirectionEdge(Traction_Input_T * p_input, sign_t direction)
{
    if (direction != p_input->Direction) { p_input->Direction = direction; return true; }
    else { return false; }
}

static inline sign_t Traction_Input_GetDirectionCmd(const Traction_Input_T * p_input) { return p_input->Direction; }



/*
    Config States
*/
typedef enum Traction_BrakeMode
{
    TRACTION_BRAKE_MODE_PASSIVE,
    TRACTION_BRAKE_MODE_TORQUE,
    TRACTION_BRAKE_MODE_VOLTAGE,
}
Traction_BrakeMode_T;

typedef enum Traction_ThrottleMode
{
    TRACTION_THROTTLE_MODE_SPEED,
    TRACTION_THROTTLE_MODE_TORQUE,
    TRACTION_THROTTLE_MODE_VOLTAGE,
}
Traction_ThrottleMode_T;

/* Release Input */
typedef enum Traction_ZeroMode
{
    TRACTION_ZERO_MODE_FLOAT,       /* "Coast". MOSFETS non conducting. Same as Neutral. */
    TRACTION_ZERO_MODE_REGEN,       /* Regen Brake */
    TRACTION_ZERO_MODE_IZERO,      /* Zero current/torque */
    TRACTION_ZERO_MODE_ZERO,        /* Setpoint Zero. No cmd overwrite */
}
Traction_ZeroMode_T;

typedef struct Traction_Config
{
    Traction_ThrottleMode_T ThrottleMode;
    Traction_BrakeMode_T BrakeMode;
    Traction_ZeroMode_T ZeroMode;
    // uint8_t RequireZeroOnEntry;
}
Traction_Config_T;

typedef struct Traction_State
{
    Traction_Input_T Input;
    Traction_Config_T Config;
    // StateMachine_Active_T StateMachine;
}
Traction_State_T;

/*
    [Traction Context]
    StateMachine with Sync Input
*/
typedef const struct Traction
{
    Traction_State_T * P_TRACTION_STATE;
    Motor_Table_T MOTORS;
    const Traction_Config_T * P_NVM_CONFIG;
}
Traction_T;


// #define TRACTION_INIT()


/******************************************************************************/
/*!

*/
/******************************************************************************/
extern void Traction_Init(const Traction_T * p_vehicle);

extern void Traction_StartThrottleMode(const Traction_T * p_vehicle);
extern void Traction_ApplyThrottleValue(const Traction_T * p_vehicle, uint16_t value);
extern void Traction_StartBrakeMode(const Traction_T * p_vehicle);
extern void Traction_ApplyBrakeValue(const Traction_T * p_vehicle, uint16_t value);
extern void Traction_StartDriveZero(const Traction_T * p_vehicle);
extern void Traction_ProcDriveZero(const Traction_T * p_vehicle);
extern void Traction_ProcInputCmd(const Traction_T * p_vehicle);
extern void Traction_StartCmdMode(const Traction_T * p_vehicle, Traction_Cmd_T mode);
extern void Traction_ProcThrottleValue(const Traction_T * p_vehicle);
extern void Traction_ProcBrakeValue(const Traction_T * p_vehicle);

/******************************************************************************/
/*
    VarId Interface
*/
/******************************************************************************/
typedef enum Traction_VarId
{
    TRACTION_VAR_DIRECTION,          // sign_t,
    TRACTION_VAR_THROTTLE,           // [0:65535]
    TRACTION_VAR_BRAKE,              // [0:65535]
    TRACTION_VAR_RELEASE,
    TRACTION_VAR_STATE_ID,
    // TRACTION_VAR_THROTTLE_ONLY,           // [0:65535]
    // TRACTION_VAR_BRAKE_ONLY,              // [0:65535]
}
Traction_VarId_T;

typedef enum Traction_ConfigId
{
    TRACTION_CONFIG_THROTTLE_MODE,     /* Traction_ThrottleMode_T */
    TRACTION_CONFIG_BRAKE_MODE,        /* Traction_BrakeMode_T */
    TRACTION_CONFIG_ZERO_MODE,         /* Traction_ZeroMode_T */
}
Traction_ConfigId_T;


extern int _Traction_VarId_Get(const Traction_Input_T * p_vehicle, Traction_VarId_T id);
extern void _Traction_VarId_Set(Traction_Input_T * p_vehicle, Traction_VarId_T id, int value);

extern int Traction_ConfigId_Get(const Traction_Config_T * p_this, Traction_ConfigId_T id);
extern void Traction_ConfigId_Set(Traction_Config_T * p_this, Traction_ConfigId_T id, int value);

