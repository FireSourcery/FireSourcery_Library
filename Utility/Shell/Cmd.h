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
	@file  	Cmd.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#ifndef CMD_H
#define CMD_H

#include <stdint.h>

typedef enum
{
	CMD_STATUS_SUCCESS,
	CMD_STATUS_INVALID_ARGS,
	CMD_STATUS_PROCESS_LOOP,
	CMD_STATUS_PROCESS_END,
	CMD_STATUS_ERROR,
}
Cmd_Status_T;

typedef Cmd_Status_T (*Cmd_Function_T)(void * p_context, int argc, char * argv[]);
typedef Cmd_Status_T (*Cmd_ProcessFunction_T)(void * p_context);

typedef const struct
{
    const Cmd_ProcessFunction_T FUNCTION; //loop function
    const uint32_t FREQ; //dHz
}
Cmd_Process_T;

typedef const struct
{
    const char * const P_NAME;
    const char * const P_HELP;
    const Cmd_Function_T FUNCTION;
    const Cmd_Process_T PROCESS;
    // ArgCMax;
}
Cmd_T;

#define CMD_ENTRY(CmdName, CmdHelpString, CmdFunction) { (#CmdName), (CmdHelpString), (CmdFunction) }

extern Cmd_T * Cmd_Search(const Cmd_T * p_cmdTable, uint8_t tableLength, const char * p_cmdName);
#endif
