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
typedef struct Traction_Input
{
    sign_t Direction;
    uint16_t ThrottleValue;
    uint16_t BrakeValue;
    int16_t LeverValue;     /* Single axis [-32768:32767], positive as user Forward */
    Traction_Cmd_T DriveCmd;
}
Traction_Input_T;

static inline Traction_Cmd_T Traction_Input_EvaluateCmd(const Traction_Input_T * p_input)
{
    if (p_input->BrakeValue > 0U) { return TRACTION_CMD_BRAKE; }
    if (p_input->ThrottleValue > 0U) { return TRACTION_CMD_THROTTLE; }
    return TRACTION_CMD_RELEASE;
}

/*
    For cases where Module handles cmd edge detect
*/
static inline Traction_Cmd_T Traction_Input_PollCmd(Traction_Input_T * p_input)
{
    p_input->DriveCmd = Traction_Input_EvaluateCmd(p_input);
    return p_input->DriveCmd;
}

static inline bool Traction_Input_PollCmdEdge(Traction_Input_T * p_input)
{
    Traction_Cmd_T prev = p_input->DriveCmd;
    return (prev != Traction_Input_PollCmd(p_input));
}

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
    Lever
    Magnitude along a reference direction, in [ThrottleValue]/[BrakeValue] scale [0:65535]
*/
static inline uint16_t Traction_LeverAlong(sign_t reference, int16_t lever) { return math_clamp((int32_t)reference * lever * 2, 0, UINT16_MAX); }

/* Along the reference as Throttle, against as Brake */
static inline void Traction_Input_ResolveLever(Traction_Input_T * p_input, sign_t reference)
{
    p_input->ThrottleValue = Traction_LeverAlong(reference, p_input->LeverValue);
    p_input->BrakeValue = Traction_LeverAlong(0 - reference, p_input->LeverValue);
}

/* Prior to PollCmd */
static inline bool Traction_Input_IsLeverEngage(const Traction_Input_T * p_input) { return (p_input->DriveCmd == TRACTION_CMD_RELEASE) && (p_input->LeverValue != 0); }


/*
    Config
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
    TRACTION_ZERO_MODE_IZERO,       /* Zero current/torque */
    TRACTION_ZERO_MODE_ZERO,        /* Setpoint Zero. No cmd overwrite */
}
Traction_ZeroMode_T;

/* Lever Input */
typedef enum Traction_LeverMode
{
    TRACTION_LEVER_MODE_BRAKE_TO_STOP,  /* Against the drive direction brakes. Reverse on engage from release at standstill */
    TRACTION_LEVER_MODE_THROUGH_ZERO,   /* Signed cmd continuous through zero speed. Motor bounds plugging */
}
Traction_LeverMode_T;

/* Direction the lever is read against. [driveDirection] NULL when unresolved */
static inline sign_t Traction_LeverReference(Traction_LeverMode_T mode, sign_t driveDirection, int16_t lever)
{
    switch (mode)
    {
        case TRACTION_LEVER_MODE_BRAKE_TO_STOP: return (driveDirection != 0) ? driveDirection : math_sign(lever);
        case TRACTION_LEVER_MODE_THROUGH_ZERO:  return math_sign(lever);
        default:                                return 0;
    }
}

typedef struct Traction_Config
{
    Traction_ThrottleMode_T ThrottleMode;
    Traction_BrakeMode_T BrakeMode;
    Traction_ZeroMode_T ZeroMode;
    Traction_LeverMode_T LeverMode;
    // uint8_t RequireZeroOnEntry;
    // uint16_t SwitchBrakeFloor_Percent16;
}
Traction_Config_T;

typedef struct Traction
{
    Traction_Input_T Input;
    Traction_Config_T Config;
}
Traction_T;



/******************************************************************************/
/*!

*/
/******************************************************************************/
extern void Traction_InitFrom(Traction_T * p_traction, const Traction_Config_T * p_config);


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
    TRACTION_VAR_COMMAND,           // Traction_Cmd_T
    TRACTION_VAR_STATE_ID,          // Traction_StateId_T
    TRACTION_VAR_LEVER,             // [-32768:32767]
}
Traction_VarId_T;

typedef enum Traction_ConfigId
{
    TRACTION_CONFIG_THROTTLE_MODE,     /* Traction_ThrottleMode_T */
    TRACTION_CONFIG_BRAKE_MODE,        /* Traction_BrakeMode_T */
    TRACTION_CONFIG_ZERO_MODE,         /* Traction_ZeroMode_T */
    TRACTION_CONFIG_LEVER_MODE,        /* Traction_LeverMode_T */
}
Traction_ConfigId_T;


extern int Traction_ConfigId_Get(const Traction_Config_T * p_this, Traction_ConfigId_T id);
extern void Traction_ConfigId_Set(Traction_Config_T * p_this, Traction_ConfigId_T id, int value);

// extern int _Traction_VarId_Get(const Traction_Input_T * p_vehicle, Traction_VarId_T id);
// extern void _Traction_VarId_Set(Traction_Input_T * p_vehicle, Traction_VarId_T id, int value);


