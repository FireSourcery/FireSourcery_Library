/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/
#ifndef MOT_SHELL_H
#define MOT_SHELL_H

//#include "System/Shell/Shell.h"
#include "System/Shell/Terminal.h"
#include "System/Shell/Cmd.h"

#include <stdint.h>

typedef const struct
{
	 Terminal_T * const P_CMD_TERMINAL;
	 void * P_DATA_LINK;
//	 MotorUser_T * P_CMD_MOTOR_USER;
//	 Motor_Parameters_T * CMD_MOTOR_PARAMETERS_ARRAY[CONFIG_MOTOR_SHELL_MOTOR_COUNT];// array of pointers
//	 uint8_t CMD_MOTOR_COUNT;
//	 MotorInterface_T * const P_MOTOR_INTERFACE;
} MotShell_Config_T;

typedef enum
{
	MOTOR_SHELL_CMD_RETURN_CODE_SUCCESS 		= CMD_RESERVED_RETURN_CODE_SUCCESS,
	MOTOR_SHELL_CMD_RETURN_CODE_INVALID_ARGS 	= CMD_RESERVED_RETURN_CODE_INVALID_ARGS,
	MOTOR_SHELL_CMD_RETURN_CODE_ERROR_1 		= 1,
}
MotShell_CmdReturnCode_T;


#endif /* MOTOR_SHELL_H */
