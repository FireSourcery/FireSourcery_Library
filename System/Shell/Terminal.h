#ifndef TERMINAL_H
#define TERMINAL_H



#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> //for snprintf

//each terminal instance fixed to this max
#ifndef CMDLINE_ARGC_MAX
#define CMDLINE_ARGC_MAX	5 //including cmd string
#endif
#ifndef CMDLINE_CHAR_MAX
#define CMDLINE_CHAR_MAX	50
#endif

#ifdef CONFIG_SHELL_TERMINAL_CONNECT_SERIAL
	#include "Peripheral/Serial/Serial.h"
	typedef Serial_T Terminal_Connect_T;
#else
   typedef void Terminal_Connect_T; // for shell using various connections
   typedef enum
   {
	    TERMINAL_CONNECT_ID_SERIAL,
   } Terminal_ConnectId_T;
#endif



#define KEY_ESC (0x1BU)
#define KEY_DEL (0x7FU)

typedef struct
{
	/*
	 * TxRx string functions must be supplied by the user application.
	 */
//#ifdef CONFIG_SHELL_USE_SERIAL
    Serial_T * p_Serial;
//#endif

//    Terminal_Connect_T * p_Connect;
//    Terminal_ConnectId_T ConnectId;

#ifdef CONFIG_SHELL_USE_FUNCTION_POINTER
     char (* ReadChar)(Terminal_T * p_terminal);
     void (* WriteChar)(char ch);
     void WriteString(char * p_str);
     bool PollKeyPressed(Terminal_T * p_terminal);
#endif

	volatile char Cmdline[CMDLINE_CHAR_MAX]; /*!< Cmdline buffer */
	volatile char * (ArgV[CMDLINE_ARGC_MAX]); /*!<  */
	volatile uint8_t ArgC;
	volatile uint8_t CursorIndex;
}
Terminal_T;

//#ifdef CONFIG_SHELL_USE_SERIAL

static inline char Terminal_RecvChar(const Terminal_T * p_terminal)
{
  return Serial_RecvChar(p_terminal->p_Serial);
}

static inline void Terminal_SendChar(const Terminal_T * p_terminal, char ch)
{
	Serial_SendChar(p_terminal->p_Serial, ch); /* Send char */
}

static inline void Terminal_SendString(const Terminal_T * p_terminal, const char * p_str)
{
	const char * p_ch = p_str;

	while (*p_ch != '\0')
	{
		Terminal_SendChar(p_terminal, *p_ch); //serial Serial_SendString
		p_ch++;
	}
}

static inline bool Terminal_PollKeyPressed(const Terminal_T * p_terminal)
{
	return ((Serial_GetAvailableRx(p_terminal->p_Serial) == 0U) ? false : true);
}
//#endif

static inline void Terminal_SendNum(const Terminal_T * p_terminal, int32_t number)
{
	char str[16U];
	uint8_t i;

	snprintf(str, 16U, "%d", number);  //replace with inttostring function?

	while (str[i] != '\0')
	{
		Terminal_SendChar(p_terminal, str[i]); // check error
		i++;
	}
}

static inline void Terminal_SendCtrl_ESC(const Terminal_T * p_terminal)
{
	Terminal_SendChar(p_terminal, 0x1BU); /* 'ESC' */
	Terminal_SendChar(p_terminal, 0x5BU); /* '[' */
}

static inline void Terminal_SendCtrl_CRLF(const Terminal_T * p_terminal)
{
	Terminal_SendChar(p_terminal, 0x0DU); /* 'CR' */
	Terminal_SendChar(p_terminal, 0x0AU); /* 'LF' */
}

static inline void Terminal_Clear(const Terminal_T * p_terminal)
{
	Terminal_SendCtrl_ESC(p_terminal);
	Terminal_SendChar(p_terminal, 0x32U); /* '2' */
	Terminal_SendChar(p_terminal, 0x4AU); /* 'J' */
}

static inline bool Terminal_PollCmdlineEsc(const Terminal_T * p_terminal)
{
	return (Terminal_RecvChar(p_terminal) == KEY_ESC) ? true : false;
}

static inline uint8_t Terminal_GetCmdlineArgC(const Terminal_T * p_terminal) //read from terminal
{
	return p_terminal->ArgC;
}

static inline char** Terminal_GetCmdlineArgV(const Terminal_T * p_terminal) //read from terminal
{
	return p_terminal->ArgV;
}

static inline char* Terminal_GetCmdlineArgVIndex(const Terminal_T * p_terminal, uint8_t varIndex) //read from terminal
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

extern void Terminal_Init(Terminal_T * p_terminal, void * p_connect);
extern bool Terminal_ProcCmdline(Terminal_T * p_terminal);
extern void Terminal_ParseCmdline(Terminal_T * p_terminal);
extern bool Terminal_WaitCmdline(Terminal_T * p_terminal);

#endif
