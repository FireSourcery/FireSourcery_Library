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
	@file  	Terminal.h
	@author FireSourcery
	@brief	text io functions for terminal ui
	@version V0
 */
/******************************************************************************/
#ifndef TERMINAL_H
#define TERMINAL_H

#include "Config.h"

#ifdef CONFIG_SHELL_XCVR_ENABLE
	#include "Peripheral/Xcvr/Xcvr.h"
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	#include "Peripheral/Serial/Serial.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h> /* for snprintf */

#define KEY_ESC (0x1BU)
#define KEY_DEL (0x7FU)

typedef enum
{
	TERMINAL_SUCCESS,
	TERMINAL_PARSER_FAIL,
}
Terminal_Status_T;

typedef struct
{
	/* TxRx string functions */
#ifdef CONFIG_SHELL_XCVR_ENABLE
	Xcvr_T Xcvr;
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	Serial_T * p_Serial;
#endif
	char Cmdline[CMDLINE_CHAR_MAX]; /*!< Cmdline buffer */
	char * (ArgV[CMDLINE_ARG_MAX]); /*!<  */
	uint8_t ArgC;
	uint8_t CursorIndex;
}
Terminal_T;

static inline uint8_t Terminal_GetCmdlineArgC(const Terminal_T * p_terminal)
{
	return p_terminal->ArgC;
}

static inline char ** Terminal_GetCmdlineArgV(const Terminal_T * p_terminal)
{
	return p_terminal->ArgV;
}

static inline char * Terminal_GetCmdlineVar(const Terminal_T * p_terminal, uint8_t varIndex)
{
	return p_terminal->ArgV[varIndex];
}

extern void Terminal_Init(Terminal_T * p_terminal);
extern void Terminal_SetXcvr(Terminal_T * p_terminal, uint8_t xcvrID);
extern void Terminal_SetSerial(Terminal_T * p_terminal, void * p_serial);
extern void Terminal_ConfigBaudRate(const Terminal_T * p_terminal, uint32_t baudRate);

extern char Terminal_RecvChar(const Terminal_T * p_terminal);
extern void Terminal_SendChar(const Terminal_T * p_terminal, char txChar);
extern void Terminal_SendString(const Terminal_T * p_terminal, const char * p_str);
extern void Terminal_SendString_Prefixed(const Terminal_T * p_terminal, const char * p_str, uint8_t length);
extern bool Terminal_GetIsKeyPressed(const Terminal_T * p_terminal);
extern void Terminal_SendNum(const Terminal_T * p_terminal, int32_t number);
extern void Terminal_SendEsc(const Terminal_T * p_terminal);
extern void Terminal_SendCrlf(const Terminal_T * p_terminal);
extern void Terminal_SendClear(const Terminal_T * p_terminal);
extern bool Terminal_PollEsc(const Terminal_T * p_terminal);
extern bool Terminal_ProcCmdline(Terminal_T * p_terminal);
extern Terminal_Status_T Terminal_ParseCmdline(Terminal_T * p_terminal);

#endif
