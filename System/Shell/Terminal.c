/*
 * Serial text support functions for terminal ui
 */

#include "Terminal.h"

#include "Peripheral/Serial/Serial.h"

#define KEY_ESC (0x1BU)
#define KEY_DEL (0x7FU)

#ifndef CMDLINE_ARGC_MAX
#define CMDLINE_ARGC_MAX	5 //including cmd string
#endif
#ifndef CMDLINE_CHAR_MAX
#define CMDLINE_CHAR_MAX	50
#endif

typedef struct
{
	/*
	 * TxRx string functions must be supplied by the user application.
	 */
//#ifdef CONFIG_SHELL_USE_SERIAL
    Serial_T * p_Serial;
//#endif
#ifdef CONFIG_SHELL_USE_FUNCTION_POINTER
     char (* ReadChar)(void);
     void (* WriteChar)(char ch);
     void WriteString(char * p_str);
     bool PollKeyPressed(void);
#endif

	char Cmdline[CMDLINE_CHAR_MAX]; /*!<  */
	char *p_Argv[CMDLINE_ARGC_MAX]; /*!<  */
	uint8_t Argc;

	uint8_t CursorIndex;
} Terminal_T;

Terminal_T MainTerminal;

void Terminal_Init(Serial_T *p_serial)
{
	MainTerminal.p_Serial = p_serial;
	MainTerminal.CursorIndex = 0;
}

char Terminal_RecvChar(void)
{
  return Serial_RecvChar(MainTerminal.p_Serial);
}

void Terminal_SendChar(char ch)
{
	Serial_SendChar(MainTerminal.p_Serial, ch); /* Send char */
}

void Terminal_SendString(const char *p_str)
{
	uint8_t *p_ch = p_str;

	while (*p_ch != 0x00U)
	{
		Serial_SendChar(*p_ch);
		p_ch++;
	}
}

bool Terminal_PollKeyPressed(void)
{
  return  ((Serial_GetAvailableRx(MainTerminal.p_Serial) == 0U) ? false : true);
}

bool Terminal_ProcCmdline(void) //read from terminal
{
	char ch = Terminal_RecvChar();

	bool isComplete = false;

	if ((ch > 31U && ch < 127U) && (MainTerminal.CursorIndex < CMDLINE_CHAR_MAX - 2U))  /* printable char */
	{
		Terminal_SendChar(ch);
		MainTerminal.Cmdline[MainTerminal.CursorIndex] = ch;
		MainTerminal.CursorIndex++;
	}
	else if ((ch == '\b' || ch == 127U) && (MainTerminal.CursorIndex > 0U))  /* backspace key */  //ch == '^H' || ch == '^?'
	{
		Terminal_SendString("\b \b");
		MainTerminal.CursorIndex--;
	}
	else if (ch == '\n' || ch == '\r')
	{
		Terminal_SendString("\r\n");
		MainTerminal.Cmdline[MainTerminal.CursorIndex] = '\0';
		MainTerminal.CursorIndex = 0;
		isComplete = true;
	}
//			#ifdef CONFIG_SHELL_ARROW_KEYS_ENABLE
//			else if (ch == '\e')
//			{
//				// Serial_ReadChar(MainTerminal.p_Serial,ch);
//				Shell_ReadChar(&ch);
//				if (ch == '[')
//				{
//					// Serial_ReadChar(MainTerminal.p_Serial,ch);
//					Shell_ReadChar(&ch);
//					// Serial_ReadChar(MainTerminal.p_Serial,ch);
//				}
//			}
//			#endif
//			else if (ch == '\0')
//			{
//
//			}
	else
	{
		Terminal_SendString("\a"); //beep
	}

	return isComplete;
}

void Terminal_ParseCmdline(void)
{
    char * p_cmdline = MainTerminal.Cmdline;
    uint8_t argc = 0U;
	bool isArgv = true;

	while (*p_cmdline != '\0')
	{
		if (isArgv)
		{
			isArgv = false;
			MainTerminal.p_Argv[argc] = p_cmdline;
			argc++;
		}
		else if (*p_cmdline == ' ')
		{
			isArgv = true; //next char is Argv
			*p_cmdline = 0U;
		}
		p_cmdline++;
	}

	MainTerminal.Argc = argc;
}


uint8_t Terminal_GetCmdlineArgC(void) //read from terminal
{
	return MainTerminal.Argc;
}

//char* Terminal_GetCmdlineArgV(uint8_t varIndex) //read from terminal
//{
//	return MainTerminal.p_Argv[varIndex];
//}

char ** Terminal_GetCmdlineArgV(void) //read from terminal
{
	return MainTerminal.p_Argv;
}

bool Terminal_WaitCmdline(void)
{
	bool isEscape;
	char ch = Terminal_RecvChar();

	if (ch == 27) //esc key
	{
		isEscape = true;
	}
	else
	{
		isEscape = false;
	}

	return isEscape;
}


void Terminal_SendCtrl_ESCPrefix(void)
{
	Terminal_SendChar(0x1BU); /* 'ESC' */
	Terminal_SendChar(0x5BU); /* '[' */
}

void Terminal_SendCtrl_CRLF(void)
{
	Terminal_SendChar(0x0DU); /* 'CR' */
	Terminal_SendChar(0x0AU); /* 'LF' */
}

void Terminal_Clear(void)
{
	Terminal_SendCtrl_ESCPrefix();
	Terminal_SendChar(0x32U); /* '2' */
	Terminal_SendChar(0x4AU); /* 'J' */
}

