#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   MotorController_OptPin.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/

#include "../../MotorController.h"
#include "../../MotorController_User.h"
#include "../ParkPin/MotorController_ParkPin.h"
#include "../SwitchBrake/MotorController_SwitchBrake.h"
#include "Transducer/UserIn/UserDIn_Cmd.h"

typedef enum MotorController_OptDinMode
{
    MOTOR_CONTROLLER_OPT_DIN_DISABLE,
    MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_I_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_PARK,
    MOTOR_CONTROLLER_OPT_DIN_FORWARD,
    MOTOR_CONTROLLER_OPT_DIN_REVERSE,
    MOTOR_CONTROLLER_OPT_DIN_SWITCH_BRAKE,
    MOTOR_CONTROLLER_OPT_DIN_RELAY,
    MOTOR_CONTROLLER_OPT_DIN_BUZZER,
    MOTOR_CONTROLLER_OPT_DIN_METER,
    MOTOR_CONTROLLER_OPT_DIN_MODE_COUNT,
    // MOTOR_CONTROLLER_OPT_DIN_SERVO,
}
MotorController_OptDinMode_T;

typedef struct MotorController_OptDinConfig
{
    MotorController_OptDinMode_T FunctionId;
    // bool           ActiveHigh; /* polarity */
    // uint16_t       DebounceMs;
    uint16_t       SpeedPreset_Fract16;
    uint16_t       SpeedLimit_Fract16;
    uint16_t       TorqueLimit_Fract16;
    uint16_t       SwitchBrakeScalar;
    bool           AutoParkOnSto;
}
MotorController_OptDinConfig_T;

static inline void MotorController_SpeedLimitPin(MotorController_T * p_dev, UserDIn_Edge_T edge)
{
    switch (edge)
    {
        case USER_DIN_EDGE_RISING:  MotorController_SetUserSpeedLimitAll(p_dev, p_dev->P_MC->Config.OptSpeedLimit_Fract16); break;
        case USER_DIN_EDGE_FALLING: MotorController_ClearUserSpeedLimitAll(p_dev); break;
        default: break;
    };
}

// static inline void _MotorController_ProcOptDin_Runtime(const MotorController_T * p_dev)
// static inline void _MotorController_ProcOptDin_Compiletime(const MotorController_T * p_dev)
// {
//     UserDIn_PollEdgeCmd(&p_dev->OPT_DIN, p_dev-> );
// }

static inline void _MotorController_ProcOptDin(const MotorController_T * p_dev)
{
    // p_dev->P_MC->OptDinCmd(p_dev, UserDIn_PollEdge(&p_dev->OPT_DIN));
    // UserDIn_PollEdgeCmd(&p_dev->OPT_DIN, p_dev->P_MC->OptDinCmd);
    const UserDIn_Cmd_T cmd = { .P_CONTEXT = (void *)p_dev, .CMD = p_dev->P_MC->OptDinCmd, };
    UserDIn_PollEdgeCmd(&p_dev->OPT_DIN, &cmd);
}

static const UserDIn_Fn_T OPT_PIN_TABLE[] =
{
    [MOTOR_CONTROLLER_OPT_DIN_DISABLE]      =  (UserDIn_Fn_T)UserDIn_CmdNull,
    [MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT]  =  (UserDIn_Fn_T)MotorController_SpeedLimitPin,
    [MOTOR_CONTROLLER_OPT_DIN_PARK]         =  (UserDIn_Fn_T)MotorController_CallParkPin,
    [MOTOR_CONTROLLER_OPT_DIN_SWITCH_BRAKE] =  (UserDIn_Fn_T)MotorController_CallSwitchBrakePin,
};


static inline void _MotorController_InitOptDin(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    if (p_mc->Config.OptDinMode < MOTOR_CONTROLLER_OPT_DIN_MODE_COUNT)
    {
        p_mc->OptDinCmd = OPT_PIN_TABLE[p_mc->Config.OptDinMode];
    }
    else
    {
        p_mc->OptDinCmd = (UserDIn_Fn_T)UserDIn_CmdNull;
    }
}