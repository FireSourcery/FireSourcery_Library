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
	@brief	Serial text support functions for terminal ui
	@version V0
 */
/******************************************************************************/
#include "Terminal.h"

#include "Peripheral/Serial/Serial.h"

#include <stdio.h>


//
//static inline bool IsXcvrSet(Terminal_T * p_terminal)
//{
//#ifdef CONFIG_SHELL_XCVR_ENABLE
//	return (p_terminal->Xcvr.p_Xcvr != 0U); //todo sub Xcvr.p_Xcvr
//#elif defined(CONFIG_SHELL_XCVR_SERIAL)

//#endif
//}

void Terminal_Init(Terminal_T * p_terminal)
{
	p_terminal->CursorIndex = 0;
}

#ifdef CONFIG_SHELL_XCVR_ENABLE
void Terminal_SetXcvr(Terminal_T * p_terminal, uint8_t xcvrID)
{
	Xcvr_Init(&p_terminal->Xcvr, xcvrID);
}
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
void Terminal_SetSerial(Terminal_T * p_terminal, void * p_serial)
{
	if(p_terminal->p_Serial != 0U)
	{
		p_terminal->p_Serial = p_serial;
	}
}
#endif

void Terminal_ConfigBaudRate(const Terminal_T * p_terminal, uint32_t baudRate)
{
	if (baudRate != 0U)
	{
#ifdef CONFIG_SHELL_XCVR_ENABLE
		//if (Xcvr_CheckValid(&p_terminal->Xcvr){}
		Xcvr_ConfigBaudRate(&p_terminal->Xcvr, baudRate);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
		if(p_terminal->p_Serial != 0U)
		{
		Serial_ConfigBaudRate(p_terminal->p_Serial, baudRate);
		}
#endif
	}
}

char Terminal_RecvChar(const Terminal_T * p_terminal)
{
	uint8_t rxChar = 0U;
#ifdef CONFIG_SHELL_XCVR_ENABLE
	Xcvr_Rx(&p_terminal->Xcvr, &rxChar, 1U);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	Serial_RecvChar(p_terminal->p_Serial, &rxChar);
#endif
	return rxChar;
}

void Terminal_SendChar(const Terminal_T * p_terminal, char txChar)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
	Xcvr_Tx(&p_terminal->Xcvr, &txChar, 1U);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	Serial_SendChar(p_terminal->p_Serial, txChar);
#endif
}

void Terminal_SendString(const Terminal_T * p_terminal, const char * p_str)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
	Xcvr_Tx(&p_terminal->Xcvr, (const uint8_t *)p_str, strlen(p_str));
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	Serial_SendString(p_terminal->p_Serial, (const uint8_t *)p_str, strlen(p_str));
#endif
}

void Terminal_SendString_Prefixed(const Terminal_T * p_terminal, const char * p_str, uint8_t length)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
	Xcvr_Tx(&p_terminal->Xcvr, (const uint8_t *)p_str, length);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	Serial_SendString(p_terminal->p_Serial, (const uint8_t *)p_str, length);
#endif
}

bool Terminal_GetIsKeyPressed(const Terminal_T * p_terminal)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
	return ((Xcvr_GetRxFullCount(&p_terminal->Xcvr) == 0U) ? false : true);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
	return ((Serial_GetRxFullCount(p_terminal->p_Serial) == 0U) ? false : true);
#endif
}

/*
 * Build Cmdline
 */
bool Terminal_ProcCmdline(Terminal_T * p_terminal) //read from terminal
{
	bool isComplete = false;
	char cmdChar;

	if (Terminal_GetIsKeyPressed(p_terminal))
	{
		cmdChar = Terminal_RecvChar(p_terminal);

		if ((cmdChar > 31U && cmdChar < 127U) && (p_terminal->CursorIndex < CMDLINE_CHAR_MAX - 2U)) /* printable char */
		{
			Terminal_SendChar(p_terminal, cmdChar);
			p_terminal->Cmdline[p_terminal->CursorIndex] = cmdChar;
			p_terminal->CursorIndex++;
		}
		else if ((cmdChar == '\b' || cmdChar == KEY_DEL) && (p_terminal->CursorIndex > 0U)) /* backspace key */  //ch == '^H' || ch == '^?'
		{
			Terminal_SendString(p_terminal, "\b \b");
			p_terminal->CursorIndex--;
		}
		else if (cmdChar == '\n' || cmdChar == '\r')
		{
			Terminal_SendString(p_terminal, "\r\n");
			p_terminal->Cmdline[p_terminal->CursorIndex] = '\0';
			p_terminal->CursorIndex = 0U;
			isComplete = true;
		}
//#ifdef CONFIG_SHELL_ARROW_KEYS_ENABLE
//		else if (cmdChar == '\e')
//		{
//			cmdChar =  Terminal_RecvChar(p_terminal);
//			if (cmdChar == '[')
//			{
//				cmdChar = Terminal_RecvChar(p_terminal);
//			}
//		}
//#endif
//		else if(cmdChar == '/0' || cmdChar == 0xFF)
//		{
//			Terminal_SendString(p_terminal, "\a");
//		}
		else
		{
			Terminal_SendString(p_terminal, "\a"); //beep
		}
	}

	return isComplete;
}

Terminal_Status_T Terminal_ParseCmdline(Terminal_T * p_terminal)
{
    char * p_cmdline = &p_terminal->Cmdline[0U];
    uint8_t argc = 0U;
	bool isDelimiter = true; /* at least one delimiter is encountered, can accept next char as argV */
	Terminal_Status_T status = TERMINAL_SUCCESS;

	for (uint8_t iChar = 0U; iChar < CMDLINE_CHAR_MAX; iChar++)
	{
		if (p_cmdline[iChar] != '\0')
		{
			if (p_cmdline[iChar] == ' ')
			{
				p_cmdline[iChar] = '\0';
				isDelimiter = true;
			}
			else if (isDelimiter)
			{
				if (argc < CMDLINE_ARG_MAX)
				{
					p_terminal->ArgV[argc] = &p_cmdline[iChar];
					isDelimiter = false;
					argc++;
				}
				else
				{
					status = TERMINAL_PARSER_FAIL;
				}
			}
		}
	}

	p_terminal->ArgC = argc;

	return status;
}

void Terminal_SendNum(const Terminal_T * p_terminal, int32_t number)
{
	char numStr[16U];
	snprintf(numStr, 16U, "%d", (int)number);
	Terminal_SendString(p_terminal, numStr);
}

void Terminal_SendEsc(const Terminal_T * p_terminal)
{
	Terminal_SendChar(p_terminal, 0x1BU); /* 'ESC' */
	Terminal_SendChar(p_terminal, 0x5BU); /* '[' */
}

void Terminal_SendCrlf(const Terminal_T * p_terminal)
{
	Terminal_SendChar(p_terminal, 0x0DU); /* 'CR' */
	Terminal_SendChar(p_terminal, 0x0AU); /* 'LF' */
}

void Terminal_SendClear(const Terminal_T * p_terminal)
{
	Terminal_SendEsc(p_terminal);
	Terminal_SendChar(p_terminal, 0x32U); /* '2' */
	Terminal_SendChar(p_terminal, 0x4AU); /* 'J' */
}

bool Terminal_PollEsc(const Terminal_T * p_terminal)
{
	return (Terminal_RecvChar(p_terminal) == KEY_ESC) ? true : false;
}

//bool Terminal_CheckCmdlineEsc(const Terminal_T * p_terminal)
//{
//	return (Terminal_PeekChar(p_terminal) == KEY_ESC) ? true : false;
//}
