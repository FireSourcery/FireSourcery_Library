#ifndef TERMINAL_H
#define TERMINAL_H

#include "Peripheral/Serial/Serial.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
	TERMINAL_CMDLINE_PARSE_SUCESS,
	TERMINAL_CMDLINE_INVALID_CMD, //invalid format
	TERMINAL_CMDLINE_INVALID_ARGS,
} Terminal_CmdlineStatus_T;


extern char Terminal_RecvChar(void);
extern void Terminal_SendChar(char ch);
extern void Terminal_SendString(const char *p_str);

extern void Terminal_Init(Serial_T *p_serial);
extern bool Terminal_Proc(void);

extern bool Terminal_PollKeyPressed(void);
extern void Terminal_SendCtrl_ESCPrefix(void);
extern void Terminal_SendCtrl_CRLF(void);
extern void Terminal_Clear(void);

extern bool Terminal_ProcCmdline(void);
extern void Terminal_ParseCmdline(void);
extern uint8_t Terminal_GetCmdlineArgC(void);
extern char** Terminal_GetCmdlineArgV(void);
extern bool Terminal_WaitCmdline(void);

#endif
