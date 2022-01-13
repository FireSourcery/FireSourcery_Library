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
#ifndef TERMINAL_H
#define TERMINAL_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> //for snprintf

//each terminal instance fixed to this max
#ifndef CMDLINE_ARG_MAX
#define CMDLINE_ARG_MAX		5 //including cmd string
#endif

#ifndef CMDLINE_CHAR_MAX
#define CMDLINE_CHAR_MAX	50
#endif

//#ifdef CONFIG_SHELL_TERMINAL_CONNECT_SERIAL
	#include "Peripheral/Serial/Serial.h"
//	typedef Serial_T Terminal_Connect_T;
//#else
//   typedef void Terminal_Connect_T; // for shell using various connections
//   typedef enum
//   {
//	    TERMINAL_CONNECT_ID_SERIAL,
//   } Terminal_ConnectId_T;
//#endif

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
	/*
	 * TxRx string functions must be supplied by the user application.
	 */
//#ifdef CONFIG_TERMINAL_CONNECT_SERIAL
    Serial_T * p_Serial;
//#elif CONFIG_TERMINAL_CONNECT_STREAM
    //stream
//#elif CONFIG_TERMINAL_CONNECT_PRIMITIVE

    //uint8_t  BufferIn[]
    //uint8_t  BufferOut[]

//#endif

	  char Cmdline[CMDLINE_CHAR_MAX]; /*!< Cmdline buffer */
	  char * (ArgV[CMDLINE_ARG_MAX]); /*!<  */
	  uint8_t ArgC;
	  uint8_t CursorIndex;
}
Terminal_T;

//#ifdef CONFIG_SHELL_USE_SERIAL

static inline char Terminal_RecvChar(const Terminal_T * p_terminal)
{
	char rxChar = 0;
	Serial_RecvChar(p_terminal->p_Serial, &rxChar);
	return rxChar;
}

static inline void Terminal_SendChar(const Terminal_T * p_terminal, char txChar)
{
	Serial_SendChar(p_terminal->p_Serial, txChar); /* Send char */
}

static inline void Terminal_SendString(const Terminal_T * p_terminal, const char * p_str)
{
	const char * p_char = p_str;

	while (*p_char != '\0')
	{
		Terminal_SendChar(p_terminal, *p_char); //serial Serial_SendString
		p_char++;
	}
}

//GetIs
static inline bool Terminal_GetIsKeyPressed(const Terminal_T * p_terminal)
{
	return ((Serial_GetRxFullCount(p_terminal->p_Serial) == 0U) ? false : true);
}
//#endif

//static inline char Terminal_ProcWriteStream(const Terminal_T * p_terminal, const char * p_str, length)
//{
////	p_terminal->Buffer[] = p_str[];
//}
static inline void Terminal_SendNum(const Terminal_T * p_terminal, int32_t number)
{
	char str[16U];
	uint8_t i;

	snprintf(str, 16U, "%d", (int)number);

	while (str[i] != '\0')
	{
		Terminal_SendChar(p_terminal, str[i]); // check error
		i++;
	}
}

static inline void Terminal_SendEsc(const Terminal_T * p_terminal)
{
	Terminal_SendChar(p_terminal, 0x1BU); /* 'ESC' */
	Terminal_SendChar(p_terminal, 0x5BU); /* '[' */
}

static inline void Terminal_SendCrlf(const Terminal_T * p_terminal)
{
	Terminal_SendChar(p_terminal, 0x0DU); /* 'CR' */
	Terminal_SendChar(p_terminal, 0x0AU); /* 'LF' */
}

static inline void Terminal_SendClear(const Terminal_T * p_terminal)
{
	Terminal_SendCtrl_ESC(p_terminal);
	Terminal_SendChar(p_terminal, 0x32U); /* '2' */
	Terminal_SendChar(p_terminal, 0x4AU); /* 'J' */
}

static inline bool Terminal_PollCmdlineEsc(const Terminal_T * p_terminal)
{
	return (Terminal_RecvChar(p_terminal) == KEY_ESC) ? true : false;
}

//static inline bool Terminal_CheckCmdlineEsc(const Terminal_T * p_terminal)
//{
//	return (Terminal_PeekChar(p_terminal) == KEY_ESC) ? true : false;
//}

static inline bool Terminal_ProcCmdline(Terminal_T * p_terminal) //read from terminal
{
	bool isComplete = false;
	char ch;

	if (Terminal_GetIsKeyPressed(p_terminal))
	{
		ch = Terminal_RecvChar(p_terminal);

		if ((ch > 31U && ch < 127U) && (p_terminal->CursorIndex < CMDLINE_CHAR_MAX - 2U)) /* printable char */
		{
			Terminal_SendChar(p_terminal, ch);
			p_terminal->Cmdline[p_terminal->CursorIndex] = ch;
			p_terminal->CursorIndex++;
		}
		else if ((ch == '\b' || ch == KEY_DEL) && (p_terminal->CursorIndex > 0U)) /* backspace key */  //ch == '^H' || ch == '^?'
		{
			Terminal_SendString(p_terminal, "\b \b");
			p_terminal->CursorIndex--;
		}
		else if (ch == '\n' || ch == '\r')
		{
			Terminal_SendString(p_terminal, "\r\n");
			p_terminal->Cmdline[p_terminal->CursorIndex] = '\0';
			p_terminal->CursorIndex = 0;
			isComplete = true;
		}
//		else if (ch == '/0' || ch == 0xFF)
//		{
//
//		}
//			#ifdef CONFIG_SHELL_ARROW_KEYS_ENABLE
//			else if (ch == '\e')
//			{
//				// Serial_ReadChar(p_terminal->p_Serial,ch);
//				Shell_ReadChar(&ch);
//				if (ch == '[')
//				{
//					// Serial_ReadChar(p_terminal->p_Serial,ch);
//					Shell_ReadChar(&ch);
//					// Serial_ReadChar(p_terminal->p_Serial,ch);
//				}
//			}
//			#endif
//			else if (ch == '\0')
//			{
//
//			}
		else
		{
			Terminal_SendString(p_terminal, "\a"); //beep
		}
	}

	return isComplete;
}

static inline void Terminal_ParseCmdline(Terminal_T * p_terminal)
{
    char * p_cmdline = &p_terminal->Cmdline[0U];
    uint8_t argc = 0U;
	bool isArgv = true; //start each iteration assuming current char is a arg char, check if its not first

	while (*p_cmdline != '\0')
	{
		if (*p_cmdline == ' ')
		{
			*p_cmdline = '\0';
			isArgv = true; //next char is Argv or another ' '
		}
		else if (isArgv)
		{
			if (argc < CMDLINE_ARG_MAX)
			{
				p_terminal->ArgV[argc] = p_cmdline;
				isArgv = false;
				argc++;
			}
			else
			{
				//error
			}
		}
		p_cmdline++;
	}

	p_terminal->ArgC = argc;
}


static inline uint8_t Terminal_GetCmdlineArgC(const Terminal_T * p_terminal) //read from terminal
{
	return p_terminal->ArgC;
}

static inline char** Terminal_GetPtrCmdlineArgV(const Terminal_T * p_terminal) //read from terminal
{
	return p_terminal->ArgV;
}

static inline char* Terminal_GetCmdlineArgV(const Terminal_T * p_terminal, uint8_t varIndex) //read from terminal
{
	return p_terminal->ArgV[varIndex];
}

//extern char Terminal_RecvChar(Terminal_T * p_terminal);
//extern void Terminal_SendChar(char ch);
//extern void Terminal_SendString(const char *p_str);
//extern uint8_t Terminal_GetCmdlineArgC(Terminal_T * p_terminal);
//extern char** Terminal_GetCmdlineArgV(Terminal_T * p_terminal);
//extern bool Terminal_PollKeyPressed(Terminal_T * p_terminal);
//extern void Terminal_SendCtrl_ESCPrefix(Terminal_T * p_terminal);
//extern void Terminal_SendCtrl_CRLF(Terminal_T * p_terminal);
//extern void Terminal_Clear(Terminal_T * p_terminal);

//extern void Terminal_Init(Terminal_T * p_terminal, void * p_connect);
//extern bool Terminal_ProcCmdline(Terminal_T * p_terminal);
//extern void Terminal_ParseCmdline(Terminal_T * p_terminal);
//extern bool Terminal_WaitCmdline(Terminal_T * p_terminal);

#endif
