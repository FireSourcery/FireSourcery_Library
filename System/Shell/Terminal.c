/*
 * Serial text support functions for terminal ui
 */

#include "Terminal.h"

#include "Peripheral/Serial/Serial.h"

#include <stdio.h>

void Terminal_Init(Terminal_T * p_terminal, void * p_connect)
{
	//#ifdef CONFIG_SHELL_USE_SERIAL
	p_terminal->p_Serial = p_connect;
	//#endif

	p_terminal->CursorIndex = 0;
}

bool Terminal_ProcCmdline(Terminal_T * p_terminal) //read from terminal
{
	bool isComplete = false;
	char ch;

	if (Terminal_PollKeyPressed(p_terminal))
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

void Terminal_ParseCmdline(Terminal_T * p_terminal)
{
    char * p_cmdline = p_terminal->Cmdline;
    uint8_t argc = 0U;
	bool isArgv = true;

	while (*p_cmdline != '\0')
	{
		if (isArgv)
		{
			isArgv = false;
			p_terminal->ArgV[argc] = p_cmdline;
			argc++;
		}
		else
		{
			if (*p_cmdline == ' ')
			{
				isArgv = true; //next char is Argv
				*p_cmdline = '\0';
			}
		}
		p_cmdline++;
	}

	p_terminal->ArgC = argc;
}


