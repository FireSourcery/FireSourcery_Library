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
	@file  	Shell.c
	@author FireSourcery
	@brief
	@version V0
 */
/******************************************************************************/
#include "Shell.h"
#include "Cmd.h"
#include "Terminal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include <stddef.h>

/******************************************************************************/
/*!
 *  @name ShellState
 *  Variables containing Shell state
 */
/******************************************************************************/
/*! @{ */
static void PrintCmdReturnCode(Shell_T * p_shell, int cmdReturnCode)
{
	Terminal_SendString(&p_shell->Terminal, "Command returned error code ");
	Terminal_SendString(&p_shell->Terminal, Cmd_SearchReturnString(p_shell->CONFIG.P_CMD_STATUS_TABLE, p_shell->CONFIG.CMD_STATUS_COUNT, cmdReturnCode));
	Terminal_SendString(&p_shell->Terminal, "\r\n");
}

//static void PrintShellStatus(Shell_T * p_shell, Shell_Status_T status)
//{
////if (PrintShellStatusError)
////{
//	switch(status)
//	{
//			case SHELL_STATUS_CMD_INVALID:
//				Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
//
////			case SHELL_STATUS_TERMINAL_PARSER_FAIL:
////				break;
//
//			case SHELL_STATUS_CMD_INVALID:
//				Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
//				break;
//
//			case SHELL_STATUS_CMD_ACCEPTED:
//				break;
//
//			case SHELL_STATUS_CMD_PROCESSING:
//				break;
//	}
//
////case SHELL_STATUS_CMD_EXCEPTION:
////	 PrintCmdReturnCode(Shell_T * p_shell, int cmdReturnCode)
//
//}

//void Shell_PollEscape(Shell_T * p_shell)
//{
//	if (Terminal_PollCmdlineEsc(&p_shell->Terminal))
//	{
//		Terminal_SendString(&p_shell->Terminal, "Exit\r\n");
//		p_shell->State = SHELL_STATE_PROMPT;
//	}
//}

//non blocking proc
Shell_Status_T Shell_Proc(Shell_T * p_shell)
{
	Shell_Status_T status = 0U;

//	if(p_shell->State != SHELL_STATE_INACTIVE)
//	{
//		Shell_PollEscape(p_shell);
//	}

	switch (p_shell->State)
	{
		case SHELL_STATE_PROMPT:
			Terminal_SendString(&p_shell->Terminal, "cmd> ");
			p_shell->State = SHELL_STATE_WAIT_INPUT;
//			status = SHELL_STATUS_OK;
			break;

		case SHELL_STATE_WAIT_INPUT:
			if (Terminal_ProcCmdline(&p_shell->Terminal))
			{
				p_shell->State = SHELL_STATE_PROCESS_CMD;		//input complete
			}
//			status = SHELL_STATUS_OK;
			break;

//		case SHELL_STATE_PARSE_INPUT:
//
//
////			p_shell->State = SHELL_STATE_SEARCH_CMD;
////			status = SHELL_STATUS_OK;
//			break;
//
//		case SHELL_STATE_SEARCH_CMD:
//
//			break;

		case SHELL_STATE_PROCESS_CMD:
			Terminal_ParseCmdline(&p_shell->Terminal);
			p_shell->p_Cmd = Cmd_Search(p_shell->CONFIG.P_CMD_TABLE, p_shell->CONFIG.CMD_COUNT, Terminal_GetCmdlineArgV(&p_shell->Terminal, 0U));

			p_shell->State = SHELL_STATE_PROMPT;

			if (p_shell->p_Cmd != 0U)
			{
				p_shell->CmdReturnCode = p_shell->p_Cmd->FUNCTION(p_shell->CONFIG.P_CMD_CONTEXT, Terminal_GetCmdlineArgC(&p_shell->Terminal), Terminal_GetPtrCmdlineArgV(&p_shell->Terminal));

				switch(p_shell->CmdReturnCode)
				{
					case CMD_RESERVED_RETURN_CODE_SUCCESS:
						p_shell->State = SHELL_STATE_PROMPT;
						break;

					case CMD_RESERVED_RETURN_CODE_INVALID_ARGS:
//						if(PrintCmdReturnCode)
						if (p_shell->CmdReturnCode == CMD_RESERVED_RETURN_CODE_INVALID_ARGS)
						{
							Terminal_SendString(&p_shell->Terminal, "Invalid args\r\n");
							status = SHELL_STATUS_CMD_INVALID_ARGS;
						}
						else
						{
							PrintCmdReturnCode(p_shell, p_shell->CmdReturnCode);
							status = SHELL_STATUS_CMD_EXCEPTION;
						}
						break;

					case CMD_RESERVED_RETURN_CODE_LOOP:
						p_shell->State = SHELL_STATE_PROCESS_CMD_LOOP;
//							status = SHELL_STATUS_CMD_PROCESSING;
						break;
					default:
//						if(PrintCmdReturnCode)
						PrintCmdReturnCode(p_shell, p_shell->CmdReturnCode);
						break;
				}
			}
			else
			{
				Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
				p_shell->State = SHELL_STATE_PROMPT;
				status = SHELL_STATUS_CMD_INVALID;
			}
			break;

		case SHELL_STATE_PROCESS_CMD_LOOP:
			if (Terminal_PollCmdlineEsc(&p_shell->Terminal))
			{
				Terminal_SendString(&p_shell->Terminal, "Exit\r\n");
				p_shell->State = SHELL_STATE_PROMPT;
//				status = SHELL_STATUS_CMD_PROCESSING;
			}
			else
			{
				if(*p_shell->CONFIG.P_TIMER - p_shell->LoopModeTimeRef > p_shell->CONFIG.TIMER_FREQ * 10U / p_shell->p_Cmd->PROCESS.FREQ )
				{
					p_shell->LoopModeTimeRef = *p_shell->CONFIG.P_TIMER;
					p_shell->CmdReturnCode = p_shell->p_Cmd->PROCESS.FUNCTION(p_shell->CONFIG.P_CMD_CONTEXT);
				}
			}
			break;


		case SHELL_STATE_INACTIVE:
			break;

		default: break;
	}

	return status;
}

void Shell_Disable(Shell_T * p_shell)
{
	p_shell->State = SHELL_STATE_INACTIVE;
}

void Shell_Enable(Shell_T * p_shell)
{
	p_shell->State = SHELL_STATE_PROMPT;
}

void Shell_Init(Shell_T * p_shell)
{
	if(p_shell->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_shell->Params, p_shell->CONFIG.P_PARAMS, sizeof(Shell_Params_T));
	}

	//need xcvr module to validate
	if (p_shell->Params.p_Xcvr != 0U)
	{
		Terminal_SetXcvr(&p_shell->Terminal, p_shell->Params.p_Xcvr);

		if (p_shell->Params.BaudRate != 0U)
		{
			Serial_ConfigBaudRate(p_shell->Params.p_Xcvr, p_shell->Params.BaudRate);
		}
	}

	if (p_shell->Params.IsEnable == true)
	{
		Protocol_Enable(p_shell);
	}
	else
	{
		p_shell->State = SHELL_STATE_INACTIVE;
	}
}

//void Shell_SetXcvr(Shell_T * p_shell, void * p_xcvr)
//{
//	Terminal_SetXcvr(&p_shell->Terminal, p_xcvr);
//}


/******************************************************************************/
/*!
 *
 */
/******************************************************************************/

