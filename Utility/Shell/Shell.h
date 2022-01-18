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

#include "Cmd.h"
#include "Terminal.h"

typedef enum
{
	SHELL_STATUS_OK,

//	SHELL_STATUS_TERMINAL_PARSER_FAIL,
	SHELL_STATUS_CMD_ACCEPTED,
	SHELL_STATUS_CMD_INVALID,
	SHELL_STATUS_CMD_INVALID_ARGS,
	SHELL_STATUS_CMD_ERROR,
	SHELL_STATUS_CMD_EXCEPTION,
	SHELL_STATUS_CMD_PROCESSING,
	SHELL_STATUS_CMD_COMPLETE,
	//	SHELL_STATUS_SUCCESS,
}
Shell_Status_T;

typedef enum
{
	SHELL_STATE_PROMPT,
	SHELL_STATE_WAIT_INPUT,
//	SHELL_STATE_PARSE_INPUT,
//	SHELL_STATE_SEARCH_CMD,
	SHELL_STATE_PROCESS_CMD,
	SHELL_STATE_PROCESS_CMD_LOOP,
	SHELL_STATE_INACTIVE,
}
Shell_State_T;

/******************************************************************************/
/*!

 */
/******************************************************************************/

typedef struct
{
	Serial_T * p_Xcvr;
	uint32_t BaudRate;
	bool IsEnable;
	//	bool PrintReturnCode;
}
Shell_Params_T;

typedef const struct
{
	//Shell Array Table Mode -Set up Shell to use Array tables defined by the user
	const Cmd_T * P_CMD_TABLE;
	uint8_t CMD_COUNT;
	void * P_CMD_CONTEXT;

	const Cmd_Status_T * P_CMD_STATUS_TABLE;
	uint8_t CMD_STATUS_COUNT;

	volatile const uint32_t * P_TIMER;
	uint32_t LOOP_PERIOD;

	const Shell_Params_T * const P_PARAMS;
}
Shell_Config_T;

typedef struct
{
	Shell_Config_T CONFIG;
	Shell_Params_T Params;
	Terminal_T Terminal;
	Shell_State_T State;
	Cmd_T * p_Cmd; /*!< Cmd  in process preserve across state change */
	int CmdReturnCode;
	uint32_t LoopModeTimeRef;
}
Shell_T;

#define SHELL_CONFIG(p_CmdTable, CmdCount, p_Context, p_CmdStatusTable, CmdStatusCount, p_Timer, LoopPeriod, p_Params)	\
{															\
	.CONFIG = 												\
	{														\
		.P_CMD_TABLE 			= p_CmdTable,				\
		.CMD_COUNT 				= CmdCount,					\
		.P_CMD_CONTEXT 			= p_Context,				\
		.P_CMD_STATUS_TABLE 	= p_CmdStatusTable,			\
		.CMD_STATUS_COUNT 		= CmdStatusCount,			\
		.P_TIMER 				= p_Timer,					\
		.LOOP_PERIOD 			= LoopPeriod,				\
		.P_PARAMS				= p_Params					\
	}														\
}


extern Shell_Status_T Shell_Proc(Shell_T * p_shell);
extern void Shell_Init(Shell_T * p_shell);

#endif
