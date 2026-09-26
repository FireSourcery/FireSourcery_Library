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
    @file   UART.c
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "UART.h"

#if !defined(UART_ENTER_CRITICAL) && !defined(UART_EXIT_CRITICAL)
#define _ENTER_CRITICAL(local,...) local
#define _EXIT_CRITICAL(local,...) local
#else
    #define _ENTER_CRITICAL(local,...)   __VA_ARGS__
    #define _EXIT_CRITICAL(local,...)    __VA_ARGS__
#endif

/*
    Single threaded buffer read/write only need disable channel ISR
    if interrupt occurs after checking sw buffer, it will run to completion,
    sw write occur in between hw read/write
    Multithread disable all interrupts for entire duration
    Enter critical section before checking sw buffer
*/
static inline void EnterCriticalTx(UART_T * p_uart) { _ENTER_CRITICAL(HAL_UART_DisableTxInterrupt(p_uart->P_HAL_UART), UART_ENTER_CRITICAL(p_uart)); }
/* EnableTxInterrupt determine by checking of buffer */
static inline void ExitCriticalTx(UART_T * p_uart) { _EXIT_CRITICAL((void)p_uart, UART_EXIT_CRITICAL(p_uart)); }
static inline void EnterCriticalRx(UART_T * p_uart) { _ENTER_CRITICAL(HAL_UART_DisableRxInterrupt(p_uart->P_HAL_UART), UART_ENTER_CRITICAL(p_uart)); }
static inline void ExitCriticalRx(UART_T * p_uart) { _EXIT_CRITICAL(HAL_UART_EnableRxInterrupt(p_uart->P_HAL_UART), UART_EXIT_CRITICAL(p_uart)); }


/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void UART_Init(UART_T * p_uart)
{
    HAL_UART_Init(p_uart->P_HAL_UART);
    HAL_UART_WriteTxSwitch(p_uart->P_HAL_UART, true);
    HAL_UART_WriteRxSwitch(p_uart->P_HAL_UART, true);
    RingT_Clear(RING_T_ARGS(p_uart->TX_RING));
    RingT_Clear(RING_T_ARGS(p_uart->RX_RING));
    UART_EnableRxIsr(p_uart);
    UART_EnableTxIsr(p_uart);
}

void UART_Deinit(UART_T * p_uart)
{
    HAL_UART_Deinit(p_uart->P_HAL_UART);
}

bool UART_ConfigBaudRate(UART_T * p_uart, uint32_t baudRate)
{
    bool isSuccess;

    HAL_UART_WriteTxSwitch(p_uart->P_HAL_UART, false);
    HAL_UART_WriteRxSwitch(p_uart->P_HAL_UART, false);

    isSuccess = HAL_UART_ConfigBaudRate(p_uart->P_HAL_UART, baudRate);

    HAL_UART_WriteTxSwitch(p_uart->P_HAL_UART, true);
    HAL_UART_WriteRxSwitch(p_uart->P_HAL_UART, true);

    return isSuccess;
}

bool UART_SendByte(UART_T * p_uart, uint8_t txChar)
{
    bool isSuccess;

    EnterCriticalTx(p_uart);
    isSuccess = RingT_PushBack(RING_T_ARGS(p_uart->TX_RING), (void *)&txChar);
    if (isSuccess == true) { HAL_UART_EnableTxInterrupt(p_uart->P_HAL_UART); }
    ExitCriticalTx(p_uart);

    return isSuccess;
}

bool UART_RecvByte(UART_T * p_uart, uint8_t * p_rxChar)
{
    bool isSuccess;

    EnterCriticalRx(p_uart);
    isSuccess = RingT_PopFront(RING_T_ARGS(p_uart->RX_RING), (void *)p_rxChar);
    ExitCriticalRx(p_uart);

    return isSuccess;
}

/*
    Send as many as fit. May clip.
*/
size_t UART_SendMax(UART_T * p_uart, const uint8_t * p_srcBuffer, size_t srcSize)
{
    size_t charCount;

    EnterCriticalTx(p_uart);
    charCount = RingT_PushBackMax(RING_T_ARGS(p_uart->TX_RING), p_srcBuffer, srcSize);
    if (charCount > 0U) { HAL_UART_EnableTxInterrupt(p_uart->P_HAL_UART); }
    ExitCriticalTx(p_uart);

    return charCount;
}

size_t UART_RecvMax(UART_T * p_uart, uint8_t * p_destBuffer, size_t destSize)
{
    size_t charCount;

    EnterCriticalRx(p_uart);
    charCount = RingT_PopFrontMax(RING_T_ARGS(p_uart->RX_RING), p_destBuffer, destSize);
    ExitCriticalRx(p_uart);

    return charCount;
}

bool UART_SendN(UART_T * p_uart, const uint8_t * p_srcBuffer, size_t length)
{
    bool status;

    EnterCriticalTx(p_uart);
    status = RingT_PushBackArray(RING_T_ARGS(p_uart->TX_RING), p_srcBuffer, length);
    if (status == true) { HAL_UART_EnableTxInterrupt(p_uart->P_HAL_UART); }
    ExitCriticalTx(p_uart);

    return status;
}

/*
    Rx only if length had been reached
*/
bool UART_RecvN(UART_T * p_uart, uint8_t * p_destBuffer, size_t length)
{
    bool status;

    EnterCriticalRx(p_uart);
    status = RingT_PopFrontArray(RING_T_ARGS(p_uart->RX_RING), p_destBuffer, length);
    ExitCriticalRx(p_uart);

    return status;
}

void UART_FlushBuffers(UART_T * p_uart)
{
    RingT_Clear(RING_T_ARGS(p_uart->TX_RING));
    RingT_Clear(RING_T_ARGS(p_uart->RX_RING));
}
