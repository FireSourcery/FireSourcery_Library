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
#include "Framework/Ring/RingT.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct Serial
{
    HAL_Serial_T * P_HAL_SERIAL;
    RingT_T RX_RING;
    RingT_T TX_RING;
}
Serial_T;



/*
    Serial rings are byte rings: the unit is what HAL_Serial_ReadRxChar returns.
    Buffer sizes are in bytes and must each be a power of 2 (RING_TYPE_INIT static_asserts).

    SERIAL_INIT  caller supplies the two Ring_State_T allocations
    SERIAL_ALLOC allocates them inline - file scope only, the compound literals
                 inside RING_STATE_ALLOC need static storage duration
*/
#define SERIAL_INIT(p_Hal, p_TxState, TxBufferSize, p_RxState, RxBufferSize)    \
{                                                                               \
    .P_HAL_SERIAL = p_Hal,                                                      \
    .TX_RING = RING_T_INIT(sizeof(uint8_t), TxBufferSize, p_TxState),           \
    .RX_RING = RING_T_INIT(sizeof(uint8_t), RxBufferSize, p_RxState),           \
}

#define SERIAL_ALLOC(p_Hal, TxBufferSize, RxBufferSize)    \
{                                                          \
    .P_HAL_SERIAL = p_Hal,                                            \
    .TX_RING = RING_T_ALLOC(sizeof(uint8_t), TxBufferSize),     \
    .RX_RING = RING_T_ALLOC(sizeof(uint8_t), RxBufferSize),     \
}

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
        /* Space must be checked before the hw read: a char read out of the hw fifo cannot be put back */
        if (RingT_IsFull(RING_T_ARGS(p_serial->RX_RING)) == true) /* Rx until software buffer is full */
        {
            /* if buffer stays full, disable irq to prevent blocking lower priority threads. user must restart rx irq */
            HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL);
            break;
        }
        else
        {
            rxChar = HAL_Serial_ReadRxChar(p_serial->P_HAL_SERIAL);
            _RingT_PushBack(RING_T_ARGS(p_serial->RX_RING), (void *)&rxChar); /* unchecked: space established above */
        }
    }
}

/*
    Tx data reg/fifo empty ISR, transmit from software buffer to hw
    Alternatively, HAL_Serial_ReadTxFullCount < HAL_SERIAL_FIFO_SIZE
*/
static inline void Serial_TxData_ISR(Serial_T * p_serial)
{
    uint8_t txChar;

    while (HAL_Serial_ReadTxEmptyCount(p_serial->P_HAL_SERIAL) > 0U) /* Tx until hw buffer is full */
    {
        /* Pop reports the empty case, so the hw write is reached only with a char in hand */
        if (RingT_PopFront(RING_T_ARGS(p_serial->TX_RING), (void *)&txChar) == false) /* Tx until software buffer is empty */
        {
            HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL);
            break;
        }
        else
        {
            HAL_Serial_WriteTxChar(p_serial->P_HAL_SERIAL, txChar);
        }
    }
}

static inline bool Serial_PollRestartRxIsr(const Serial_T * p_serial)
{
    bool isOverrun = false;
    /* Continue waiting for buffer read, before restarting interrupts, if buffer is full */
    if ((HAL_Serial_ReadRxOverrun(p_serial->P_HAL_SERIAL) == true) && (RingT_IsFull(RING_T_ARGS(p_serial->RX_RING)) == false))
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
static inline size_t Serial_GetRxFullCount(const Serial_T * p_serial)   { return RingT_GetFullCount(RING_T_ARGS(p_serial->RX_RING)); }
static inline size_t Serial_GetTxEmptyCount(const Serial_T * p_serial)  { return RingT_GetEmptyCount(RING_T_ARGS(p_serial->TX_RING)); }
static inline void Serial_EnableTxIsr(const Serial_T * p_serial)        { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
static inline void Serial_DisableTxIsr(const Serial_T * p_serial)       { HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL); }
static inline void Serial_EnableRxIsr(const Serial_T * p_serial)        { HAL_Serial_EnableRxInterrupt(p_serial->P_HAL_SERIAL); }
static inline void Serial_DisableRxIsr(const Serial_T * p_serial)       { HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL); }

/* Check for Overrun */
// static inline bool Serial_IsRxOverrun(const Serial_T * p_serial)
// {
//     return ((HAL_Serial_ReadRxOverrun(p_serial->P_HAL_SERIAL) == true) || (RingT_IsFull(RING_T_ARGS(p_serial->RX_RING)) == true));
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
extern void Serial_FlushBuffers(Serial_T * p_serial);
// extern char Serial_GetChar(Serial_T * p_serial);
// extern bool Serial_SendCharString(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);

// extern bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
// extern size_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);

