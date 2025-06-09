/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotorController_Include.h
    @author FireSourcery
    @brief    Includes all headers

*/
/******************************************************************************/
#include "Config.h"
#include "MotorController_Thread.h"
#include "MotorController_MotProtocol.h"
#include "MotorController_User.h"
// #include "MotorController_Analog.h"
#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
#include "MotorController_Shell.h"
#endif
#include "Motor/Motor/Motor_Include.h"
#include "../MotProtocol/MotProtocol.h"