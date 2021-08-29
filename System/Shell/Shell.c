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
//#include <string.h>
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
	Terminal_SendString(&p_shell->Terminal, Cmd_SearchReturnString(p_shell->p_CmdReturnTable, p_shell->CmdReturnCount, cmdReturnCode));
	Terminal_SendString(&p_shell->Terminal, "\r\n");
}

//static void PrintShellException(Shell_T * p_shell, Shell_Status_T status)
//{

//if (PrintShellStatusError)
//{
//	switch(status)
//	{
//	case SHELL_STATUS_CMD_INVALID:
//	Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");



//	switch (status)
//	{
//		case SHELL_STATUS_TERMINAL_PARSER_FAIL:
//			break;
//
//		case SHELL_STATUS_CMD_INVALID:
//			Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
//			break;
//
//		case SHELL_STATUS_CMD_ACCEPTED:
//			break;
//
//		case SHELL_STATUS_CMD_PROCESSING:
//			break;
//	}
//}

void Shell_PollEscape(Shell_T * p_shell) //poll escape on separate thread
{
	if (Terminal_PollCmdlineEsc(&p_shell->Terminal))
	{
		Terminal_SendString(&p_shell->Terminal, "Exit\r\n");
		p_shell->State = SHELL_STATE_PROMPT;
	}
}

//non blocking proc
Shell_Status_T Shell_Proc(Shell_T * p_shell)
{
	Shell_Status_T status = 0U;
//	Cmd_Function_T * p_cmdSearch;

	switch(p_shell->State)
	{
		case SHELL_STATE_PROMPT:
			Terminal_SendString(&p_shell->Terminal, "cmd> ");
			p_shell->State = SHELL_STATE_WAIT_INPUT;
			status = SHELL_STATUS_OK;
			break;

		case SHELL_STATE_WAIT_INPUT:
//			if (Shell_KeyPressed())
//			switch(Terminal_ProcCmdline())
			if (Terminal_ProcCmdline(&p_shell->Terminal))
			{
				p_shell->State = SHELL_STATE_PARSE_INPUT;
				//input complete
			}
			status = SHELL_STATUS_OK;
			break;

		case SHELL_STATE_PARSE_INPUT:
			Terminal_ParseCmdline(&p_shell->Terminal);
			p_shell->State = SHELL_STATE_SEARCH_CMD;
			status = SHELL_STATUS_OK;
			break;

		case SHELL_STATE_SEARCH_CMD:
			p_shell->CmdFunction = Cmd_SearchFunction(p_shell->p_CmdTable, p_shell->CmdCount, Terminal_GetCmdlineArgV(&p_shell->Terminal)[0U]);
			if (p_shell->CmdFunction != 0U)
			{
				p_shell->State = SHELL_STATE_PROCESS_CMD;
				status =  SHELL_STATUS_CMD_ACCEPTED;
			}
			else
			{
				Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
				p_shell->State = SHELL_STATE_PROMPT;
				status = SHELL_STATUS_CMD_INVALID;
			}
			break;

		case SHELL_STATE_PROCESS_CMD:
//			if (Terminal_PollCmdlineEsc(&p_shell->Terminal))
//			{
//				Terminal_SendString(&p_shell->Terminal, "Exit\r\n");
////				p_shell->State = SHELL_STATE_PROMPT;
//			}
//			else
			{
				//switch (function type) pass shell context

				p_shell->CmdReturnCode = p_shell->CmdFunction(Terminal_GetCmdlineArgC(&p_shell->Terminal), Terminal_GetCmdlineArgV(&p_shell->Terminal));

				if (p_shell->CmdReturnCode != CMD_RESERVED_RETURN_CODE_SUCCESS)
				{

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



				}
				else
				{
//					if (p_shell->IsLoopModeEnable)
//					{
//						p_shell->State = SHELL_STATE_CMD_PROCESS_LOOP;
//						//how should user app enter this mode
//						//reserved return code?
//						//public shell function?
//						//cmd type? - same cmd name cannot share 2 modes
//					status = SHELL_STATUS_CMD_PROCESSING;
//					}
//					else
					{
						p_shell->State = SHELL_STATE_PROMPT;
						status = SHELL_STATUS_CMD_COMPLETE;
					}
				}
			}
			break;

		case SHELL_STATE_PROCESS_CMD_LOOP:
//			if (Terminal_WaitCmdline(&p_shell->Terminal))
//			{
//				p_shell->State = SHELL_STATE_PROMPT;
//
////				status = SHELL_STATUS_CMD_PROCESSING;
//			}
//			else
//			{
//				if (p_shell->LoopModeCounter > p_shell->LoopModePeriod)
//				{
//					p_shell->LoopModeCounter = 0U;
//					p_shell->CmdReturnCodeId = p_shell->CmdFunction(Terminal_GetCmdlineArgC(&p_shell->Terminal), Terminal_GetCmdlineArgV(&p_shell->Terminal));
//				}
//				else
//				{
//					p_shell->LoopModeCounter++;
////					status = SHELL_STATUS_CMD_PROCESSING;
//				}
//			}
			break;

//		case SHELL_STATE_PRINT_EXCEPTION:
//			PrintShellException()

		//	case SHELL_STATUS_CMD_ERROR:
		//	if(PrintCmdReturnCode)
		//		PrintCmdReturnCode(p_shell, p_shell->CmdReturnCode);
		//	}
			//


		case SHELL_STATE_INACTIVE:
			break;

		default: break;
	}

	return status;
}

void Shell_Init
(
	Shell_T * p_shell,
	Cmd_T * p_cmdTable,
	uint8_t cmdCount,
	Cmd_Return_T * p_returnCodeTable,
	uint8_t returnCodeCount,
	void * p_terminalConnect,
//	void * p_typeData,
	uint16_t shellProcFreq
)
{
	p_shell->p_CmdTable = p_cmdTable;
	p_shell->CmdCount = cmdCount;
	p_shell->p_CmdReturnTable = p_returnCodeTable;
	p_shell->CmdReturnCount = returnCodeCount;
	Terminal_Init(&p_shell->Terminal, p_terminalConnect);
//	p_shell->p_TypeData = p_typeData;
	p_shell->ProcFreq = shellProcFreq;
	p_shell->State = SHELL_STATE_PROMPT;
}

//void Shell_SetProcessModeLoop(Shell_T * p_shell, uint16_t cmdLoopFreq)
//{
//	p_shell->IsLoopModeEnable = true;
//	p_shell->LoopModePeriod = p_shell->ProcFreq/cmdLoopFreq;
//}

//void Shell_SetPrintReturnCode(bool print)
//{
//	PrintReturnCode = print;
//}


/******************************************************************************/
/*!
 */
/******************************************************************************/
/*! @{ */
//static const char CMD_PROMPT_STRING[] = "cmd> ";
////static const char * const CMD_PROMPT_STRING = "cmd> ";
/*! @} */



/******************************************************************************/
/*!
 * @name  	ShellListMode
 * @brief 	Set up Shell to use linked list defined by the user
 *
 * Add to the list using Shell_Register
 */
/******************************************************************************/
/*! @{ */
//#ifdef SHELL_OPTION_USE_LIST
//LIST_T CmdList;
//LIST_T ReturnCodeList;
//#endif
/*! @} */

//#ifdef SHELL_OPTION_USE_LIST
//void Shell_RegisterReturnCodeEntry(RETURN_CODE_ENTRY_T * entry)
//{
//	List_AddTail(&ReturnCodeList, &entry->Link);
//}
//
////void Shell_RegisterReturnCodeEntry(RETURN_CODE_ENTRY_T * entry, uint8_t partition)
////{
////	entry->ReturnCode = partition * 100 + 1 + entry->ReturnCode; //+1, 0 is reserved
////	List_AddTail(&ReturnCodeList, &entry->Link);
////}
//
////void Shell_RegisterReturnCodeList(RETURN_CODE_ENTRY_T * entry, uint8_t partition)
////{
////    LIST_NODE_HANDLE_T node = &entry->Link;
////
////	while (node != NULL)
////	{
////		GET_CONTAINER_STRUCT_POINTER(node, RETURN_CODE_ENTRY_T, Link)->ReturnCode = partition * 100 + GET_CONTAINER_STRUCT_POINTER(node, RETURN_CODE_ENTRY_T, Link)->ReturnCode;
////		node = List_GetNext(node);
////	}
////
////    List_AddTail(&ReturnCodeList, &entry->Link);
////}
//
//void Shell_RegisterCmdLineEntry(CMDLINE_ENTRY_T * cmd)
//{
//	List_AddTail(&CmdList, &cmd->Link);
//}
//
////void Shell_RegisterCmdLineList(CMDLINE_ENTRY_T * cmd)
////{
////	List_AddTail(&CmdList, &cmd->Link);
////}
//
////void Shell_InitRegisterReturnCodeEntry(RETURN_CODE_ENTRY_T * entry, int resultCode, char * resultString)
////{
////	entry->ReturnCode = resultCode;
////	entry->ReturnString = resultString;
////	List_AddTail(&ReturnCodeList, &entry->Link);
////}
////
////void Shell_InitRegisterCmdLineEntry(CMDLINE_ENTRY_T * cmd, char * cmdName, char * cmdHelp, CMDLINE_FUNCTION_T cmdFunction)
////{
////	cmd->CmdName = cmdName;
////	cmd->CmdHelp = cmdHelp;
////	cmd->CmdFunction = cmdFunction;
////	List_AddTail(&CmdList, &cmd->Link);
////}
//#endif



