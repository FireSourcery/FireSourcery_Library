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
    @file    Terminal.h
    @author FireSourcery
    @brief    text io functions for terminal ui
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

/*
    All terminal instances will be instantiated to these sizes
*/
#ifndef CMDLINE_ARG_MAX
    #define CMDLINE_ARG_MAX        5U /* including cmd string */
#endif

#ifndef CMDLINE_CHAR_MAX
    #define CMDLINE_CHAR_MAX    50U
#endif

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

static inline void Terminal_Init(Terminal_T * p_terminal) { p_terminal->CursorIndex = 0; }
static inline void Terminal_Reset(Terminal_T * p_terminal) { p_terminal->CursorIndex = 0; }

static inline char Terminal_RecvChar(const Terminal_T * p_terminal)
{
    uint8_t rxChar = 0U;
#ifdef CONFIG_SHELL_XCVR_ENABLE
    Xcvr_Rx(&p_terminal->Xcvr, &rxChar, 1U);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Serial_RecvByte(p_terminal->p_Serial, &rxChar);
#endif
    return rxChar;
}

static inline void Terminal_SendChar(const Terminal_T * p_terminal, char txChar)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    Xcvr_Tx(&p_terminal->Xcvr, (uint8_t *)(&txChar), 1U);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Serial_SendByte(p_terminal->p_Serial, txChar);
#endif
}

/* Null Terminated */
/* compiler optimize strlen() of passed string literal? */
static inline void Terminal_SendString(const Terminal_T * p_terminal, const char * p_str)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    Xcvr_Tx(&p_terminal->Xcvr, (const uint8_t *)p_str, strlen(p_str));
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Serial_SendN(p_terminal->p_Serial, (const uint8_t *)p_str, strlen(p_str));
#endif
}

/* Length-Prefixed */
static inline void Terminal_SendString_Len(const Terminal_T * p_terminal, const char * p_str, uint8_t length)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    Xcvr_Tx(&p_terminal->Xcvr, (const uint8_t *)p_str, length);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Serial_SendN(p_terminal->p_Serial, (const uint8_t *)p_str, length);
#endif
}

static inline bool Terminal_GetIsKeyPressed(const Terminal_T * p_terminal)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    return ((Xcvr_GetRxFullCount(&p_terminal->Xcvr) == 0U) ? false : true);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    return ((Serial_GetRxFullCount(p_terminal->p_Serial) == 0U) ? false : true);
#endif
}

/* Experimental */
static inline uint8_t * Terminal_AcquireTxBuffer(const Terminal_T * p_terminal)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    return Xcvr_AcquireTxBuffer(&p_terminal->Xcvr);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    return Serial_AcquireTxBuffer(p_terminal->p_Serial);
#endif
}

static inline void Terminal_ReleaseTxBuffer(const Terminal_T * p_terminal, size_t writeSize)
{
#ifdef CONFIG_SHELL_XCVR_ENABLE
    Xcvr_ReleaseTxBuffer(&p_terminal->Xcvr, writeSize);
#elif defined(CONFIG_SHELL_XCVR_SERIAL)
    Serial_ReleaseTxBuffer(p_terminal->p_Serial, writeSize);
#endif
}

static inline uint8_t Terminal_GetCmdlineArgC(const Terminal_T * p_terminal) { return p_terminal->ArgC; }
static inline char ** Terminal_GetCmdlineArgV(Terminal_T * p_terminal) { return p_terminal->ArgV; }
static inline char * Terminal_GetCmdlineVar(const Terminal_T * p_terminal, uint8_t varIndex) { return p_terminal->ArgV[varIndex]; }

// extern void Terminal_Init(Terminal_T * p_terminal);
// extern char Terminal_RecvChar(const Terminal_T * p_terminal);
// extern void Terminal_SendChar(const Terminal_T * p_terminal, char txChar);
// extern void Terminal_SendString(const Terminal_T * p_terminal, const char * p_str);
// extern void Terminal_SendString_Len(const Terminal_T * p_terminal, const char * p_str, uint8_t length);
// extern bool Terminal_GetIsKeyPressed(const Terminal_T * p_terminal);

extern bool Terminal_ProcCmdline(Terminal_T * p_terminal);
extern Terminal_Status_T Terminal_ParseCmdline(Terminal_T * p_terminal);

extern void Terminal_SendNum(const Terminal_T * p_terminal, int32_t number);
extern void Terminal_SendEsc(const Terminal_T * p_terminal);
extern void Terminal_SendCrlf(const Terminal_T * p_terminal);
extern void Terminal_SendClear(const Terminal_T * p_terminal);
extern bool Terminal_PollEsc(const Terminal_T * p_terminal);

extern bool Terminal_SetXcvr(Terminal_T * p_terminal, uint8_t xcvrID);
extern void Terminal_SetSerial(Terminal_T * p_terminal, void * p_serial);
extern void Terminal_ConfigBaudRate(const Terminal_T * p_terminal, uint32_t baudRate);

#endif
