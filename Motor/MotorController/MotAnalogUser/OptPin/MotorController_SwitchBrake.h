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
    @file   MotorController_SwitchBrake.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../../MotorController_StateMachine.h"

static inline UserDIn_T * MotorController_SwitchBrakePin(MotorController_T * p_dev) { return OptDin_SwitchBrake(&p_dev->DINS[0U], &p_dev->P_MC->OptDinState); }

static inline uint16_t MotorController_FuseSwitchBrake(MotorController_T * p_dev, uint16_t brake_Percent16)
{
    return OptDin_FuseSwitchBrake(MotorController_SwitchBrakePin(p_dev), p_dev->P_MC->Config.OptDinConfig.SwitchBrakeFloor_Percent16, brake_Percent16);
}
