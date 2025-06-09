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
    @file   Serial.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "HAL_Serial.h"
#include "Config.h"
#include "Utility/Ring/Ring.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct Serial
{
    HAL_Serial_T * P_HAL_SERIAL;
    Ring_Context_T RX_RING; /* todo update to const handler */
    Ring_Context_T TX_RING;
}
Serial_T;

#define SERIAL_INIT(p_Hal, p_TxBuffer, TxBufferSize, p_RxBuffer, RxBufferSize)  \
{                                                                               \
    .P_HAL_SERIAL = p_Hal,                                                      \
    .TX_RING = RING_CONTEXT_INIT(sizeof(uint8_t), TxBufferSize, p_TxRingState),         \
    .RX_RING = RING_CONTEXT_INIT(sizeof(uint8_t), RxBufferSize, p_RxRingState),         \
}

#define SERIAL_ALLOC(p_Hal, TxBufferSize, RxBufferSize)    \
{                                                          \
    .P_HAL_SERIAL = p_Hal,                                          \
    .TX_RING = RING_CONTEXT_ALLOC(sizeof(uint8_t), TxBufferSize),     \
    .RX_RING = RING_CONTEXT_ALLOC(sizeof(uint8_t), RxBufferSize),     \
}

//todo with Ring_Context_T

/******************************************************************************/
/*!
    ISRs
*/
/******************************************************************************/
/*
    Rx data reg/fifo full ISR, receive from hw to software buffer
*/
static inline void Serial_RxData_ISR(Serial_T * p_serial)
{
    uint8_t rxChar;

    while (HAL_Serial_ReadRxFullCount(p_serial->P_HAL_SERIAL) > 0U) /* Rx until hw buffer is empty */
    {
        if (Ring_IsFull(p_serial->RX_RING.P_STATE) == true) /* Rx until software buffer is full */
        {
            /* if buffer stays full, disable irq to prevent blocking lower priority threads. user must restart rx irq */
            HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL);
            break;
        }
        else
        {
            rxChar = HAL_Serial_ReadRxChar(p_serial->P_HAL_SERIAL);
            Ring_Enqueue(p_serial->RX_RING.P_STATE, &rxChar);
        }
    }
}

/*
    Tx data reg/fifo empty ISR, transmit from software buffer to hw
    Alternatively, HAL_Serial_ReadTxFullCount < CONFIG_HAL_SERIAL_FIFO_SIZE
*/
static inline void Serial_TxData_ISR(Serial_T * p_serial)
{
    uint8_t txChar;

    while (HAL_Serial_ReadTxEmptyCount(p_serial->P_HAL_SERIAL) > 0U) /* Tx until hw buffer is full */
    {
        if (Ring_IsEmpty(p_serial->TX_RING.P_STATE) == true) /* Tx until software buffer is empty */
        {
            HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL);
            break;
        }
        else
        {
            Ring_Dequeue(p_serial->TX_RING.P_STATE, &txChar);
            HAL_Serial_WriteTxChar(p_serial->P_HAL_SERIAL, txChar);
        }
    }
}

static inline bool Serial_PollRestartRxIsr(const Serial_T * p_serial)
{
    bool isOverrun = false;
    /* Continue waiting for buffer read, before restarting interrupts, if buffer is full */
    if ((HAL_Serial_ReadRxOverrun(p_serial->P_HAL_SERIAL) == true) && (Ring_IsFull(p_serial->RX_RING.P_STATE) == false))
    {
        HAL_Serial_ClearRxErrors(p_serial->P_HAL_SERIAL);
        HAL_Serial_EnableRxInterrupt(p_serial->P_HAL_SERIAL);
        isOverrun = true;
    }
    return isOverrun;
}


/******************************************************************************/
/*
    Query
*/
/******************************************************************************/
static inline size_t Serial_GetRxFullCount(const Serial_T * p_serial)   { return Ring_GetFullCount(p_serial->RX_RING.P_STATE); }
static inline size_t Serial_GetTxEmptyCount(const Serial_T * p_serial)  { return Ring_GetEmptyCount(p_serial->TX_RING.P_STATE); }
static inline void Serial_EnableTxIsr(const Serial_T * p_serial)        { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
static inline void Serial_DisableTxIsr(const Serial_T * p_serial)       { HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL); }
static inline void Serial_EnableRxIsr(const Serial_T * p_serial)        { HAL_Serial_EnableRxInterrupt(p_serial->P_HAL_SERIAL); }
static inline void Serial_DisableRxIsr(const Serial_T * p_serial)       { HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL); }

/* Check for Overrun */
// static inline bool Serial_IsRxOverrun(const Serial_T * p_serial)
// {
//     return ((HAL_Serial_ReadRxOverrun(p_serial->CONST.P_HAL_SERIAL) == true) || (Ring_IsFull(&p_serial->RX_RING) == true));
// }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void Serial_Init(Serial_T * p_serial);
extern void Serial_Deinit(Serial_T * p_serial);
extern bool Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate);
extern bool Serial_SendByte(Serial_T * p_serial, uint8_t txChar);
extern bool Serial_RecvByte(Serial_T * p_serial, uint8_t * p_rxChar);
extern size_t Serial_SendMax(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t bufferSize);
extern size_t Serial_RecvMax(Serial_T * p_serial, uint8_t * p_destBuffer, size_t bufferSize);
extern bool Serial_SendN(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
extern bool Serial_RecvN(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);

extern bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
extern size_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);

