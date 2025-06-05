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
    @file   HAL_Serial.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef SERIAL_H
#define SERIAL_H

#include "HAL_Serial.h"
#include "Config.h"
#include "Utility/Ring/Ring.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct Serial
{
    HAL_Serial_T * const P_HAL_SERIAL;
    const Ring_Context_T RX_RING; /* todo update to const handler */
    const Ring_Context_T TX_RING;
}
Serial_T;

// #define SERIAL_INIT(p_Hal, p_TxBuffer, TxBufferSize, p_RxBuffer, RxBufferSize)  \
// {                                                                               \
//     .P_HAL_SERIAL = p_Hal,                                         \
//     .TX_RING = RING_INIT(p_TxBuffer, sizeof(uint8_t), TxBufferSize),         \
//     .RX_RING = RING_INIT(p_RxBuffer, sizeof(uint8_t), RxBufferSize),         \
// }

#define SERIAL_ALLOC(p_Hal, TxBufferSize, RxBufferSize)    \
{                                                          \
    .P_HAL_SERIAL = p_Hal,                                          \
    .TX_RING = RING_CONTEXT_ALLOC(sizeof(uint8_t), TxBufferSize),     \
    .RX_RING = RING_CONTEXT_ALLOC(sizeof(uint8_t), RxBufferSize),     \
}

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

/*
    Extern
*/
extern void Serial_RxData_ISR(Serial_T * p_serial);
extern void Serial_TxData_ISR(Serial_T * p_serial);
extern bool Serial_PollRestartRxIsr(const Serial_T * p_serial);

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


#endif
