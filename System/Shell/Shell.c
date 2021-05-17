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

typedef enum
{
	SHELL_STATE_PROMPT,
	SHELL_STATE_WAIT_INPUT,
	SHELL_STATE_PARSE_INPUT,
	SHELL_STATE_SEARCH_INPUT,
	SHELL_STATE_CMD_PROCESS,
	SHELL_STATE_CMD_PROCESS_LOOP,
	SHELL_STATE_INACTIVE,
} Shell_State_T;

/******************************************************************************/
/*!
	@name  	ShellArrayTableMode
	@brief 	Set up Shell to use Array tables defined by the user
 */
/******************************************************************************/
typedef struct
{
    Cmd_T * p_CmdTable;
    uint8_t CmdCount;
    Cmd_ReturnCode_T * p_ReturnCodeTable;
    uint8_t ReturnCodeCount;

	Cmd_Function_T CmdFunction; 			/*!<  */

	Shell_State_T State;
//	bool PrintReturnCode;
	bool LoopMode;
	uint32_t LoopControlPeriod;
	uint32_t LoopControlCounter;

	uint32_t ProcFreq;

	// ShellFunctionConfig; //pass through cmd function to allow user function to adjust shell options
}
Shell_T;

Shell_T MainShell;



/******************************************************************************/
/*!
 */
/******************************************************************************/
/*! @{ */
//static const char CMD_PROMPT_STRING[] = "cmd> ";
////static const char * CMD_PROMPT_STRING = "cmd> ";
/*! @} */

static inline int32_t StringCompare(const char * p_str1, const char * p_str2)
{
	const char* p_string1 = p_str1;
	const char* p_string2 = p_str2;
	int32_t diff = 0;

//	for (uint32_t index = 0; index < length; index++)
//	{
//		if (*p_string1 != *p_string2)
//		{
//			diff = (int32_t) *(p_string1) - (int32_t) *(p_string2);
//		}
//		p_string1++;
//		p_string2++;
//	}

	while ((*p_string1 != '\0') && (*p_string2 != '\0'))
	{
		if (*p_string1 != *p_string2)
		{
			diff = (int32_t) *(p_string1) - (int32_t) *(p_string2);
			break;
		}
		p_string1++;
		p_string2++;
	}

	return diff;
}

static inline bool SearchCmd(void)
{
	uint8_t argc = Terminal_GetCmdlineArgC();
	char ** p_argv = Terminal_GetCmdlineArgV();

	bool status;

	if (argc > 0U)
	{
		//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
		for (uint8_t index; index < MainShell.CmdCount; index++)
		{
			if (StringCompare(p_argv[0], MainShell.p_CmdTable[index].p_CmdName) == 0)
			{
				MainShell.CmdFunction = MainShell.p_CmdTable[index].CmdFunction;
				status = true;
				//				return SHELL_CMD_ACCEPTED;
			}
			else
			{
				status = false;
				//				return SHELL_CMD_INVALID;
			}
		}
		//#endif

		//#ifdef SHELL_OPTION_USE_LIST
		//    	// ((CMDLINE_ENTRY_HANDLE_T)(listIterator->Data))->CmdName
		//        node = List_GetHead(&CmdList);
		//        while (node != NULL)
		//        {
		//        	if(!strcmp(Argv[0], GET_CONTAINER_STRUCT_POINTER(node, CMDLINE_ENTRY_T, Link)->CmdName))
		//        	{
		//            	CmdFunction = GET_CONTAINER_STRUCT_POINTER(node, CMDLINE_ENTRY_T, Link)->CmdFunction;
		//            	return CMDLINE_OK;
		//            }
		//            node = List_GetNext(node);
		//        }
		//#endif
	}

	return status;
}


/******************************************************************************/
/*!
 *  @name ShellState
 *  Variables containing Shell state
 */
/******************************************************************************/
/*! @{ */
static void PrintTerminalStatus(int returnCode)
{
	switch (returnCode)
	{
		case TERMINAL_CMDLINE_PARSE_SUCESS:
			break;
		case TERMINAL_CMDLINE_INVALID_CMD:
			Terminal_SendString("Invalid format\r\n");
			break;
		case TERMINAL_CMDLINE_INVALID_ARGS:
			Terminal_SendString("Invalid command arguments\r\n");
			break;
	}
}

static void PrintShellStatus(int returnCode)
{
	switch (returnCode)
	{
		case SHELL_STATUS_TERMINAL_PARSER_FAIL:
			break;
		case SHELL_STATUS_CMD_INVALID:
			Terminal_SendString("Invalid command\r\n");
			break;
		case SHELL_STATUS_CMD_ACCEPTED:
			break;

		case SHELL_STATUS_CMD_PROCESSING:
			break;
	}
}

static const char * GetCmdReturnCodeString(int returnCode)
{
//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
//	uint8_t idx;
	const char * p_returnCodeString = "Invalid Return Code";

//	for(idx = 0; idx < MainShell.ReturnCodeCount; idx++)
//	{
//		if (MainShell.p_ReturnCodeTable[idx].ReturnCode == returnCode)
//		{
//			p_returnCodeString = MainShell.p_ReturnCodeTable[idx].p_ReturnCodeString;
//			break;
//		}
//	}
	// if table index all entries defined
	if (returnCode < MainShell.ReturnCodeCount)
	{
		return (MainShell.p_ReturnCodeTable[returnCode].p_ReturnCodeString);
	}
//#endif

//#ifdef SHELL_OPTION_USE_LIST
//	LIST_NODE_HANDLE_T node;
//	node = List_GetHead(&ReturnCodeList);
//	while (node != NULL)
//	{
//		if( GET_CONTAINER_STRUCT_POINTER(node, RETURN_CODE_ENTRY_T, Link)->ReturnCode == returnCode )
//			return GET_CONTAINER_STRUCT_POINTER(node, RETURN_CODE_ENTRY_T, Link)->ReturnCodeString;
//		node = List_GetNext(node);
//	}
//#endif

	return p_returnCodeString;
}

static void PrintCmdReturnCode(int returnCode)
{
	Terminal_SendString("Command returned error code ");
	Terminal_SendString(GetCmdReturnCodeString(returnCode));
	Terminal_SendString("\r\n");
}

int Shell_Proc(void)
{
 	int returnCode;

	switch(MainShell.State)
	{
		case SHELL_STATE_PROMPT:
			Terminal_SendString("cmd> ");
			MainShell.State = SHELL_STATE_WAIT_INPUT;
			returnCode = SHELL_STATUS_CMD_PROCESSING;
			break;

		case SHELL_STATE_WAIT_INPUT:
//			if (Shell_KeyPressed())
//			switch(Terminal_ProcCmdline())
			if (Terminal_ProcCmdline())
			{
				MainShell.State = SHELL_STATE_PARSE_INPUT;
			}
			returnCode = SHELL_STATUS_CMD_PROCESSING;
			break;

		case SHELL_STATE_PARSE_INPUT:
			Terminal_ParseCmdline();
			//	returnCode = status;
			//	returnCode = SHELL_STATUS_CMD_PROCESSING;
			//	PrintReturnCodeTerminal(status);
			//	if (status == CMDLINE_PARSE_SUCESS)
			break;

		case SHELL_STATE_SEARCH_INPUT:
			if (SearchCmd())
			{
				MainShell.State = SHELL_STATE_CMD_PROCESS;
				//	returnCode = status;
				returnCode = SHELL_STATUS_CMD_PROCESSING;
			}
			else
			{
				MainShell.State = SHELL_STATE_PROMPT;
				//	returnCode = status;
				returnCode = SHELL_STATUS_CMD_PROCESSING;
			}

			break;

		case SHELL_STATE_CMD_PROCESS:
			returnCode = MainShell.CmdFunction(Terminal_GetCmdlineArgC(), Terminal_GetCmdlineArgV());
//			if (PrintReturnCode)
//			{
//				if (status != RESERVED_SHELL_RETURN_CODE_SUCCESS)
					PrintCmdReturnCode(returnCode);
//			}
			if (MainShell.LoopMode)
			{
				MainShell.State = SHELL_STATE_CMD_PROCESS_LOOP;
				//how should user app enter this mode reserved return code? public shell function?
				//				MainShell.LoopMode = 0;
			}
			else
			{
				MainShell.State = SHELL_STATE_PROMPT;
			}
			break;

		case SHELL_STATE_CMD_PROCESS_LOOP:
			if (Terminal_WaitCmdline())
			{
				MainShell.State = SHELL_STATE_PROMPT;

				returnCode = SHELL_STATUS_CMD_PROCESSING;
			}
			else
			{
				if (MainShell.LoopControlCounter > MainShell.LoopControlPeriod)
				{
					MainShell.LoopControlCounter = 0U;
					returnCode = MainShell.CmdFunction(Terminal_GetCmdlineArgC(), Terminal_GetCmdlineArgV());
				}
				else
				{
					MainShell.LoopControlCounter++;
					returnCode = SHELL_STATUS_CMD_PROCESSING;
				}
			}
			break;

		case SHELL_STATE_INACTIVE:
			break;

		default: break;
	}

	return returnCode;
}

void Shell_Init
(
	Cmd_T * p_cmdTable,
	uint8_t cmdCount,
	Cmd_ReturnCode_T * p_returnCodeTable,
	uint8_t returnCodeCount,
	uint16_t cmdLoopFreq,
	uint16_t shellProcFreq
)
{
	MainShell.p_CmdTable = p_cmdTable;
	MainShell.CmdCount = cmdCount;
	MainShell.p_ReturnCodeTable = p_returnCodeTable;
	MainShell.ReturnCodeCount = returnCodeCount;

	MainShell.LoopControlPeriod = shellProcFreq / cmdLoopFreq;
	MainShell.ProcFreq = shellProcFreq;

	MainShell.State = SHELL_STATE_PROMPT;
}

//void Shell_SetProcessModeLoop(uint16_t cmdLoopFreq)
//{
//	LoopMode = 1;
//	LoopControlPeriod = ProcFreq/cmdLoopFreq;
//}

//void Shell_SetPrintReturnCode(bool print)
//{
//	PrintReturnCode = print;
//}

/******************************************************************************/
/*!
 *  @name Cmds
 *  Default commands
 */
/******************************************************************************/
/*! @{ */
int Cmd_help(int argc, char * argv[])
{
//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
	uint8_t idx = 0;
//#endif

//#ifdef SHELL_OPTION_USE_LIST
//	LIST_NODE_HANDLE_T node;
//#endif

    (void)argc;
    (void)argv;

    Terminal_SendString("\r\nAvailable commands\r\n");
//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
    while(MainShell.p_CmdTable[idx].p_CmdName)
    {
        Terminal_SendString(MainShell.p_CmdTable[idx].p_CmdName);
        Terminal_SendString("	");
        Terminal_SendString(MainShell.p_CmdTable[idx].p_CmdHelp);
        Terminal_SendString("\r\n");
        idx++;
    }
//#endif
//#ifdef SHELL_OPTION_USE_LIST
//    node = List_GetHead(&CmdList);
//    while(node != NULL)
//    {
//        Terminal_SendString(GET_CONTAINER_STRUCT_POINTER(node, CMDLINE_ENTRY_T, Link)->CmdName);
//        Terminal_SendString("	");
//        Terminal_SendString(GET_CONTAINER_STRUCT_POINTER(node, CMDLINE_ENTRY_T, Link)->CmdHelp);
//        Terminal_SendString("\r\n");
//        node = List_GetNext(node);
//    }
//#endif

    Terminal_SendString("\r\n");

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

//int Cmd_echo(int argc, char * argv[])
//{
//	if(argc == 2)
//	{
//	    Terminal_SendString("\r\n");
//	    Terminal_SendString(argv[1]);
//	    Terminal_SendString("\r\n");
//	}
//
//	return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}

int Cmd_exit(int argc, char * argv[])
{
    (void)argc;
    (void)argv;

	MainShell.State = SHELL_STATE_INACTIVE;
    Terminal_SendString("\r\nShell exited\r\n");

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}
/*! @} */

Cmd_T CmdEntry_help 	= { "help", 	"Display list of available commands",  	Cmd_help };
Cmd_T CmdEntry_exit 	= { "exit", 	"Exit program",  						Cmd_exit };



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
