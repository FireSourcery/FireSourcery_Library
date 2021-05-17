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
	@file  	Shell.h
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#ifndef SHELL_H
#define SHELL_H

//#ifdef SHELL_OPTION_USE_LIST
//#include "List.h"
//#endif
#include "Cmd.h"

typedef enum
{
//	SHELL_STATUS_SUCCESS,
	SHELL_STATUS_TERMINAL_PARSER_FAIL,
	SHELL_STATUS_CMD_INVALID,
	SHELL_STATUS_CMD_ACCEPTED,
	SHELL_STATUS_CMD_PROCESSING,
} Shell_Status_T;


extern int Shell_Proc(void);
//extern void Shell_Init(uint16_t cmdLoopFreq, uint16_t shellProcFreq);

//extern Cmd_Function_T Cmd_help;
//extern Cmd_Function_T Cmd_exit;

extern Cmd_T CmdEntry_exit;
extern Cmd_T CmdEntry_help;

#endif
