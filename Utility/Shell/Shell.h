/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file      Shell.h
    @author FireSourcery
    @brief
    @version V0
 */
/******************************************************************************/
#ifndef SHELL_H
#define SHELL_H

#include "Config.h"
#include "Cmd.h"
#include "Terminal.h"

typedef enum
{
    SHELL_STATUS_SUCCESS,
    SHELL_STATUS_CMDLINE_INVALID,
    SHELL_STATUS_CMD_ACCEPTED,
    SHELL_STATUS_CMD_INVALID,
    SHELL_STATUS_CMD_INVALID_ARGS,
    SHELL_STATUS_CMD_ERROR,
    SHELL_STATUS_CMD_EXCEPTION,
    SHELL_STATUS_CMD_PROCESSING,
    SHELL_STATUS_CMD_COMPLETE,
}
Shell_Status_T;

typedef enum
{
    SHELL_STATE_PROMPT,
    SHELL_STATE_WAIT_INPUT,
    SHELL_STATE_PROCESS_CMD,
    SHELL_STATE_PROCESS_CMD_LOOP,
    SHELL_STATE_INACTIVE,
}
Shell_State_T;

typedef struct __attribute__((aligned(2U)))
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    uint8_t XcvrId;
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Serial_T * p_Serial;
#endif
    uint32_t BaudRate;
    bool IsEnableOnInit;
    bool EnablePrintStatus;
}
Shell_Params_T;

typedef const struct
{
    const Cmd_T * const P_CMD_TABLE;    /* Cmds table defined by the user */
    const uint8_t CMD_COUNT;
    void * const P_CMD_CONTEXT;
    volatile const uint32_t * const P_TIMER;
    const uint32_t TIMER_FREQ;
    const Shell_Params_T * const P_PARAMS;
}
Shell_Config_T;

typedef struct
{
    Shell_Config_T CONFIG;
    Shell_Params_T Params;
    Terminal_T Terminal;
    Shell_State_T State;
    Cmd_T * p_Cmd;             /*!< Cmd  in process preserve across state change */
    Cmd_Status_T CmdReturnCode;
    uint32_t ProcTimeRef;
}
Shell_T;

#if defined(CONFIG_SHELL_XCVR_ENABLE)
    #define _SHELL_INIT_XCVR(p_XcvrTable, TableLength) .Xcvr = XCVR_INIT(p_XcvrTable, TableLength)
#else
    #define _SHELL_INIT_XCVR(p_XcvrTable, TableLength)
#endif

#define SHELL_INIT(p_CmdTable, CmdCount, p_Context, p_Timer, TimerFreq, p_Params, p_XcvrTable, TableLength)    \
{                                                            \
    .CONFIG =                                                 \
    {                                                        \
        .P_CMD_TABLE             = p_CmdTable,                \
        .CMD_COUNT                 = CmdCount,                    \
        .P_CMD_CONTEXT             = p_Context,                \
        .P_TIMER                 = p_Timer,                    \
        .TIMER_FREQ             = TimerFreq,                \
        .P_PARAMS                = p_Params                    \
    },                                                        \
    .Terminal =                                                \
    {                                                        \
        _SHELL_INIT_XCVR(p_XcvrTable, TableLength)             \
    },                                                        \
}

static inline void Shell_Disable(Shell_T * p_shell) { p_shell->State = SHELL_STATE_INACTIVE; }
static inline void Shell_Enable(Shell_T * p_shell) { p_shell->State = SHELL_STATE_PROMPT; Terminal_Reset(&p_shell->Terminal);}
static inline void Shell_EnableOnInit(Shell_T * p_shell) { p_shell->Params.IsEnableOnInit = true; }
static inline void Shell_DisableOnInit(Shell_T * p_shell) { p_shell->Params.IsEnableOnInit = false; }

extern void Shell_Init(Shell_T * p_shell);
extern Shell_Status_T Shell_Proc(Shell_T * p_shell);
#ifdef CONFIG_SHELL_XCVR_ENABLE
extern bool Shell_SetXcvr(Shell_T * p_shell, uint8_t xcvrId);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
extern void Shell_SetSerial(Shell_T * p_shell, Serial_T * p_serial);
#endif
extern void Shell_ConfigBaudRate(Shell_T * p_shell, uint16_t baudRate);
extern void Shell_ResetBaudRate(Shell_T * p_shell);

#endif
