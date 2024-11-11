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
    @file
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_SHELL_H
#define MOTOR_CONTROLLER_SHELL_H

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE

#include "Utility/Shell/Shell.h"

#define MC_SHELL_CMD_COUNT 30U

extern const Cmd_T MC_CMD_TABLE[MC_SHELL_CMD_COUNT];

#define MOTOR_CONTROLLER_SHELL_INIT(p_MotorController, p_Timer, TimerFreq, p_Config, p_XcvrTable, TableLength)  \
    SHELL_INIT(MC_CMD_TABLE, MC_SHELL_CMD_COUNT, p_MotorController, p_Timer, TimerFreq, p_Config, p_XcvrTable, TableLength)

#endif

#endif /* MOTOR_SHELL_H */
