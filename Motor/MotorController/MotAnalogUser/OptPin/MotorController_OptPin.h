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
    @brief  Functions Table
*/
/******************************************************************************/
#include "../../MotorController.h"
#include "../../MotorController_User.h"
#include "MotorController_ParkPin.h"
#include "MotorController_SwitchBrake.h"
#include "Transducer/UserIn/UserDIn_Cmd.h"

static inline void MotorController_CallSpeedLimitPin(MotorController_T * p_dev, UserDIn_Edge_T edge)
{
    switch (edge)
    {
        case USER_DIN_EDGE_RISING:  MotorController_SetUserSpeedLimitAll(p_dev, p_dev->P_MC->Config.OptDinConfig.SpeedLimit_Fract16); break;
        case USER_DIN_EDGE_FALLING: MotorController_ClearUserSpeedLimitAll(p_dev); break;
        default: break;
    };
}
static inline void MotorController_CallILimitPin(MotorController_T * p_dev, UserDIn_Edge_T edge)
{
    switch (edge)
    {
        case USER_DIN_EDGE_RISING:  MotorController_SetUserILimitAll(p_dev, p_dev->P_MC->Config.OptDinConfig.TorqueLimit_Fract16); break;
        case USER_DIN_EDGE_FALLING: MotorController_ClearUserILimitAll(p_dev); break;
        default: break;
    };
}


static inline UserDIn_Fn_T _MotorController_OptDinFn(MotorController_OptDinMode_T mode)
{
    static const UserDIn_Fn_T OPT_PIN_TABLE[] =
    {
        [MOTOR_CONTROLLER_OPT_DIN_DISABLE]      = (UserDIn_Fn_T)UserDIn_CmdNull,
        [MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT]  = (UserDIn_Fn_T)MotorController_CallSpeedLimitPin,
        [MOTOR_CONTROLLER_OPT_DIN_I_LIMIT]      = (UserDIn_Fn_T)MotorController_CallILimitPin,
        [MOTOR_CONTROLLER_OPT_DIN_PARK]         = (UserDIn_Fn_T)MotorController_CallParkPin,
        [MOTOR_CONTROLLER_OPT_DIN_SWITCH_BRAKE] = (UserDIn_Fn_T)UserDIn_CmdNull,
        // [MOTOR_CONTROLLER_OPT_DIN_FORWARD]      = (UserDIn_Fn_T)UserDIn_CmdNull,
        // [MOTOR_CONTROLLER_OPT_DIN_REVERSE]      = (UserDIn_Fn_T)UserDIn_CmdNull,
    };

    return (mode < MOTOR_CONTROLLER_OPT_DIN_MODE_COUNT) ? OPT_PIN_TABLE[mode] : (UserDIn_Fn_T)UserDIn_CmdNull;
}



