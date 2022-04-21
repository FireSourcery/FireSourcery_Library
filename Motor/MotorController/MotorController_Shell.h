/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_SHELL_H
#define MOT_SHELL_H

#include "Utility/Shell/Shell.h"

#include <stdint.h>

#define MC_SHELL_CMD_COUNT		 	20U
#define MC_SHELL_CMD_STATUS_COUNT 	1U
#define MC_SHELL_PERIOD_MILLIS		1000U

//typedef enum
//{
//	MOTOR_SHELL_CMD_RETURN_CODE_SUCCESS 		= CMD_RESERVED_RETURN_CODE_SUCCESS,
//	MOTOR_SHELL_CMD_RETURN_CODE_INVALID_ARGS 	= CMD_RESERVED_RETURN_CODE_INVALID_ARGS,
//	MOTOR_SHELL_CMD_RETURN_CODE_ERROR_1 		= 1,
//}
//McShell_CmdCode_T;

extern const Cmd_T MC_CMD_TABLE[MC_SHELL_CMD_COUNT];

#define MOTOR_CONTROLLER_SHELL_CONFIG(p_MotorController, p_Timer, TimerFreq, p_Params, p_XcvrTable, TableLength)  SHELL_CONFIG(MC_CMD_TABLE, MC_SHELL_CMD_COUNT, p_MotorController, p_Timer, TimerFreq, p_Params, p_XcvrTable, TableLength)

#endif /* MOTOR_SHELL_H */
