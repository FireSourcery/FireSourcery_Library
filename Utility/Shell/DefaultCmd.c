#include "Shell.h"
#include "Cmd.h"
#include "Terminal.h"

#include <stdint.h>
#include <stdbool.h>


//extern int Cmd_help(Shell_T p_shell, int argc, char * argv[]);
//extern int Cmd_exit(Shell_T p_shell, int argc, char * argv[]);
//extern int Cmd_echo(int argc, char * argv[]);

//extern Cmd_T Cmd_exit;
//extern Cmd_T Cmd_help;

/******************************************************************************/
/*!
 *  @name Cmds
 *  Default commands
 */
/******************************************************************************/
/*! @{ */

////fixed scope cmd scope data, init before using cmd.
//static Cmd_T * p_CmdHelpCmdTable;
//static uint8_t CmdHelpCmdTableLength;
//static Terminal_T * p_CmdDefaultTerminal;
//
//void DefaultCmd_Init(Terminal_T * p_cmdDefaultTerminal, Cmd_T * p_cmdHelpCmdTable, uint8_t cmdHelpCmdTableLength)
//{
//	p_CmdDefaultTerminal = p_cmdDefaultTerminal;
//	p_CmdHelpCmdTable = p_cmdHelpCmdTable;
//	CmdHelpCmdTableLength = cmdHelpCmdTableLength;
//}
//
//
//int Cmd_help(int argc, char * argv[])
//{
//    (void)argc;
//    (void)argv;
//
//    Terminal_SendString(p_CmdDefaultTerminal, "\r\nCommands Available\r\n--------------------\r\n");
//
////#ifdef SHELL_OPTION_USE_ARRAY_TABLE
//	for (uint8_t idx; idx < CmdHelpCmdTableLength; idx++)
//	{
//		if(p_CmdHelpCmdTable[idx].P_NAME != 0U)
//		{
//			Terminal_SendString(p_CmdDefaultTerminal, p_CmdHelpCmdTable[idx].P_NAME);
//			Terminal_SendString(p_CmdDefaultTerminal, "	");
//			Terminal_SendString(p_CmdDefaultTerminal, p_CmdHelpCmdTable[idx].P_HELP);
//			Terminal_SendString(p_CmdDefaultTerminal, "\r\n");
//		}
//	}
//
//
//    Terminal_SendString(p_CmdDefaultTerminal, "\r\n");
//
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}
//
//int Cmd_echo(int argc, char * argv[])
//{
//	if(argc == 2U)
//	{
//	    Terminal_SendString(p_CmdDefaultTerminal, "\r\n");
//	    Terminal_SendString(p_CmdDefaultTerminal, argv[1U]);
//	    Terminal_SendString(p_CmdDefaultTerminal, "\r\n");
//	}
//
//	return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}
//
//int Cmd_exit(Shell_T * p_shell, int argc, char * argv[])
//{
//    (void)argc;
//    (void)argv;
//
//    p_shell->State = SHELL_STATE_INACTIVE;
//    Terminal_SendString(&p_shell->Terminal, "\r\nShell exited\r\n");
//
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}


/*! @} */

//Cmd_T Cmd_help 	= { "help", 	"Display list of available commands",  	Cmd_Function_help };
//Cmd_T Cmd_exit 	= { "exit", 	"Exit program",  						Cmd_Function_exit };
