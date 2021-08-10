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
//#ifdef SHELL_OPTION_USE_LIST
//#include "List.h"
//#endif

typedef enum
{
	SHELL_STATUS_OK, //todo refine

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
	SHELL_STATE_PARSE_INPUT,
	SHELL_STATE_SEARCH_CMD,
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
	//Shell Array Table Mode -Set up Shell to use Array tables defined by the user
	const Cmd_T * p_CmdTable;
	uint8_t CmdCount;
	const Cmd_Return_T * p_CmdReturnTable;
	uint8_t CmdReturnCount;

	Terminal_T Terminal;
	volatile void * p_TypeData;

	//processing
	volatile Shell_State_T State;
	volatile Cmd_Function_T CmdFunction; /*!< Cmd in process */ //preserve across state change
	volatile int CmdReturnCode;

	//loop mode
	uint32_t ProcFreq; //loop mode reference
	bool IsLoopModeEnable;
	volatile uint32_t LoopModePeriod;
	volatile uint32_t LoopModeCounter;

//	bool PrintReturnCode;
}
Shell_T;

extern Shell_Status_T Shell_Proc(Shell_T * p_shell);
//extern void Shell_Init(uint16_t cmdLoopFreq, uint16_t shellProcFreq);

//extern int Cmd_help(Shell_T p_shell, int argc, char * argv[]);
//extern int Cmd_exit(Shell_T p_shell, int argc, char * argv[]);
//extern int Cmd_echo(int argc, char * argv[]);

//extern Cmd_T Cmd_exit;
//extern Cmd_T Cmd_help;

#endif
