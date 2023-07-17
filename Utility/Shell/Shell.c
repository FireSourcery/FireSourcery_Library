/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file      Shell.c
    @author FireSourcery
    @brief
    @version V0
 */
/******************************************************************************/
#include "Shell.h"

void Shell_Init(Shell_T * p_shell)
{
    if(p_shell->CONFIG.P_PARAMS != 0U)
    {
        memcpy(&p_shell->Params, p_shell->CONFIG.P_PARAMS, sizeof(Shell_Params_T));
    }

    // Terminal_Init(&p_shell->Terminal);
#ifdef CONFIG_SHELL_XCVR_ENABLE
    Terminal_SetXcvr(&p_shell->Terminal, p_shell->Params.XcvrId);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Terminal_SetSerial(&p_shell->Terminal, p_shell->Params.p_Serial);     /* need xcvr module to validate xcvr pointer */
#endif

    if(p_shell->Params.IsEnableOnInit == true)
    {
        Shell_ResetBaudRate(p_shell);
        p_shell->State = SHELL_STATE_PROMPT;
    }
    else
    {
        p_shell->State = SHELL_STATE_INACTIVE;
    }
}

/*
    non blocking proc
*/
Shell_Status_T Shell_Proc(Shell_T * p_shell)
{
    Shell_Status_T status = 0U;

    switch (p_shell->State)
    {
        case SHELL_STATE_PROMPT:
            Terminal_SendString(&p_shell->Terminal, "cmd> ");
            p_shell->State = SHELL_STATE_WAIT_INPUT;
//            status = SHELL_STATUS_OK;
            break;

        case SHELL_STATE_WAIT_INPUT:
            if(Terminal_ProcCmdline(&p_shell->Terminal) == true) { p_shell->State = SHELL_STATE_PROCESS_CMD; }
//            status = SHELL_STATUS_OK;
            break;

        case SHELL_STATE_PROCESS_CMD:
            if(Terminal_ParseCmdline(&p_shell->Terminal) == TERMINAL_SUCCESS)
            {
                p_shell->p_Cmd = Cmd_Search(p_shell->CONFIG.P_CMD_TABLE, p_shell->CONFIG.CMD_COUNT, Terminal_GetCmdlineVar(&p_shell->Terminal, 0U));

                if (p_shell->p_Cmd != 0U)
                {
                    p_shell->CmdReturnCode = p_shell->p_Cmd->FUNCTION(p_shell->CONFIG.P_CMD_CONTEXT, Terminal_GetCmdlineArgC(&p_shell->Terminal), Terminal_GetCmdlineArgV(&p_shell->Terminal));

                    switch(p_shell->CmdReturnCode)
                    {
                        case CMD_STATUS_SUCCESS:
                            p_shell->State = SHELL_STATE_PROMPT;
                            break;

                        case CMD_STATUS_INVALID_ARGS: /* Must be implemented at cmd function level / per cmd function */
                            Terminal_SendString(&p_shell->Terminal, "Invalid args\r\n");
                            p_shell->State = SHELL_STATE_PROMPT;
//                            status = SHELL_STATUS_CMD_INVALID_ARGS;
                            break;

                        case CMD_STATUS_PROCESS_LOOP:
                            p_shell->State = SHELL_STATE_PROCESS_CMD_LOOP;
//                            status = SHELL_STATUS_CMD_PROCESSING;
                            break;

                        default:
//                            p_shell->State = SHELL_STATE_PROMPT;
//                            status = SHELL_STATUS_CMD_EXCEPTION;
                            break;
                    }
                }
                else
                {
                    Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
                    p_shell->State = SHELL_STATE_PROMPT;
//                    status = SHELL_STATUS_CMD_INVALID;
                }
            }
            else
            {
                Terminal_SendString(&p_shell->Terminal, "Invalid input\r\n");
                p_shell->State = SHELL_STATE_PROMPT;
//                status = SHELL_STATUS_CMDLINE_INVALID;
            }
            break;

        case SHELL_STATE_PROCESS_CMD_LOOP:
            if (Terminal_PollEsc(&p_shell->Terminal))
            {
                Terminal_SendString(&p_shell->Terminal, "Exit\r\n");
                p_shell->State = SHELL_STATE_PROMPT;
//                status = SHELL_STATUS_CMD_PROCESSING;
            }
            else
            {
                if (*p_shell->CONFIG.P_TIMER - p_shell->ProcTimeRef > p_shell->p_Cmd->PROCESS.PERIOD)
                {
                    p_shell->CmdReturnCode = p_shell->p_Cmd->PROCESS.FUNCTION(p_shell->CONFIG.P_CMD_CONTEXT);
                    if (p_shell->CmdReturnCode == CMD_STATUS_PROCESS_END) { p_shell->State = SHELL_STATE_PROMPT; }
                    p_shell->ProcTimeRef = *p_shell->CONFIG.P_TIMER;
                }
            }
            break;

        case SHELL_STATE_INACTIVE:
            break;

        default: break;
    }

    return status;
}

#ifdef CONFIG_SHELL_XCVR_ENABLE
bool Shell_SetXcvr(Shell_T * p_shell, uint8_t xcvrId)
{
    bool isSuccess = Terminal_SetXcvr(&p_shell->Terminal, xcvrId);
    if(isSuccess == true) { p_shell->Params.XcvrId = xcvrId; }
    return isSuccess;
}
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
void Shell_SetSerial(Shell_T * p_shell, Serial_T * p_serial)
{
    p_shell->Params.p_Serial = p_serial;
    Terminal_SetSerial(&p_shell->Terminal, p_serial);
}
#endif

void Shell_ConfigBaudRate(Shell_T * p_shell, uint16_t baudRate)
{
    p_shell->Params.BaudRate = baudRate;
    Shell_ResetBaudRate(p_shell);
}

void Shell_ResetBaudRate(Shell_T * p_shell)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
    if(Xcvr_CheckIsSet(&p_shell->Terminal.Xcvr, p_shell->Params.XcvrId))
#elif defined(CONFIG_PROTOCOL_XCVR_SERIAL)
    if(p_protocol->Params.p_Serial != 0U))
#endif
    {
        Terminal_ConfigBaudRate(&p_shell->Terminal, p_shell->Params.BaudRate);
    }
}

//const char * const INVALID_RETURN_CODE_STRING = "Invalid Return Code";
//static void PrintShellStatus(Shell_T * p_shell, Shell_Status_T status)
//{
////if (PrintShellStatusError)
////{
//    switch(status)
//    {
//            case SHELL_STATUS_CMD_INVALID:
//                Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
//
////            case SHELL_STATUS_TERMINAL_PARSER_FAIL:
////                break;
//
//            case SHELL_STATUS_CMD_INVALID:
//                Terminal_SendString(&p_shell->Terminal, "Invalid command\r\n");
//                break;
//
//            case SHELL_STATUS_CMD_ACCEPTED:
//                break;
//
//            case SHELL_STATUS_CMD_PROCESSING:
//                break;
//    }
//
////case SHELL_STATUS_CMD_EXCEPTION:
////     PrintCmdReturnCode(Shell_T * p_shell, int cmdReturnCode)
//
//}
