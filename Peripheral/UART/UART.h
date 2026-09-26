#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   UART.h
    @author FireSourcery
    @brief  Buffered byte stream over a UART.
*/
/******************************************************************************/
#include "HAL_UART.h"
#include "Framework/Ring/RingT.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    The peer sends whenever it likes, so bytes arrive unbidden and the ring absorbs
    the rate mismatch between the wire and the polling thread. Both ISRs move
    min(available, space) bytes: each count only grows from the other side, so one
    snapshot bounds the whole loop and the body needs no per byte check.
*/
/******************************************************************************/
typedef const struct UART
{
    HAL_UART_T * P_HAL_UART;
    RingT_T RX_RING;
    RingT_T TX_RING;
}
UART_T;

/******************************************************************************/
/*!
    ISRs
*/
/******************************************************************************/
/*
    Rx data reg/fifo full ISR, receive from hw to software buffer
*/
static inline void UART_RxData_ISR(UART_T * p_uart)
{
    size_t hwCount = HAL_UART_ReadRxFullCount(p_uart->P_HAL_UART);
    size_t space   = RingT_GetEmptyCount(RING_T_ARGS(p_uart->RX_RING));
    size_t count   = (hwCount < space) ? hwCount : space;

    for (size_t i = 0U; i < count; i++)
    {
        uint8_t rxChar = HAL_UART_ReadRxChar(p_uart->P_HAL_UART);
        _RingT_PushBack(RING_T_ARGS(p_uart->RX_RING), (void *)&rxChar);
    }

    /* Bytes left with nowhere to go. Only then stop the flood: the restart poll waits on overrun */
    if (space < hwCount) { HAL_UART_DisableRxInterrupt(p_uart->P_HAL_UART); }
}

/*
    Tx data reg/fifo empty ISR, transmit from software buffer to hw
*/
static inline void UART_TxData_ISR(UART_T * p_uart)
{
    size_t hwSpace = HAL_UART_ReadTxEmptyCount(p_uart->P_HAL_UART);
    size_t pending = RingT_GetFullCount(RING_T_ARGS(p_uart->TX_RING));
    size_t count   = (hwSpace < pending) ? hwSpace : pending;

    for (size_t i = 0U; i < count; i++)
    {
        uint8_t txChar;
        _RingT_PopFront(RING_T_ARGS(p_uart->TX_RING), (void *)&txChar);
        HAL_UART_WriteTxChar(p_uart->P_HAL_UART, txChar);
    }

    /* Ring drained. The last byte is in the shifter and needs no further service */
    if (count == pending) { HAL_UART_DisableTxInterrupt(p_uart->P_HAL_UART); }
}

/*
    Poll and restart Rx ISR if overrun occurred and software buffer has space
*/
static inline bool UART_PollRestartRxIsr(const UART_T * p_uart)
{
    bool isOverrun = false;
    /* Continue waiting for buffer read, before restarting interrupts, if buffer is full */
    if ((HAL_UART_ReadRxOverrun(p_uart->P_HAL_UART) == true) && (RingT_IsFull(RING_T_ARGS(p_uart->RX_RING)) == false))
    {
        HAL_UART_ClearRxErrors(p_uart->P_HAL_UART);
        HAL_UART_EnableRxInterrupt(p_uart->P_HAL_UART);
        isOverrun = true;
    }
    return isOverrun;
}


/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline size_t UART_GetRxFullCount(const UART_T * p_uart)      { return RingT_GetFullCount(RING_T_ARGS(p_uart->RX_RING)); }
static inline size_t UART_GetTxEmptyCount(const UART_T * p_uart)     { return RingT_GetEmptyCount(RING_T_ARGS(p_uart->TX_RING)); }
static inline void UART_EnableTxIsr(const UART_T * p_uart)           { HAL_UART_EnableTxInterrupt(p_uart->P_HAL_UART); }
static inline void UART_DisableTxIsr(const UART_T * p_uart)          { HAL_UART_DisableTxInterrupt(p_uart->P_HAL_UART); }
static inline void UART_EnableRxIsr(const UART_T * p_uart)           { HAL_UART_EnableRxInterrupt(p_uart->P_HAL_UART); }
static inline void UART_DisableRxIsr(const UART_T * p_uart)          { HAL_UART_DisableRxInterrupt(p_uart->P_HAL_UART); }

/* Check for Overrun */
static inline bool UART_IsRxOverrun(const UART_T * p_uart)
{
    return ((HAL_UART_ReadRxOverrun(p_uart->P_HAL_UART) == true) && (RingT_IsFull(RING_T_ARGS(p_uart->RX_RING)) == false));
}

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void UART_Init(UART_T * p_uart);
extern void UART_Deinit(UART_T * p_uart);
extern bool UART_ConfigBaudRate(UART_T * p_uart, uint32_t baudRate);
extern bool UART_SendByte(UART_T * p_uart, uint8_t txChar);
extern bool UART_RecvByte(UART_T * p_uart, uint8_t * p_rxChar);
extern size_t UART_SendMax(UART_T * p_uart, const uint8_t * p_srcBuffer, size_t bufferSize);
extern size_t UART_RecvMax(UART_T * p_uart, uint8_t * p_destBuffer, size_t bufferSize);
extern bool UART_SendN(UART_T * p_uart, const uint8_t * p_srcBuffer, size_t length);
extern bool UART_RecvN(UART_T * p_uart, uint8_t * p_destBuffer, size_t length);
extern void UART_FlushBuffers(UART_T * p_uart);
