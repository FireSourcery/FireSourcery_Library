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
    @file   HAL_UART.h
    @author FireSourcery
    @brief  Dependency inversion. Selects the platform HAL_UART.
*/
/******************************************************************************/
#ifndef HAL_UART_H
#define HAL_UART_H

#include "Peripheral/HAL/HAL_Peripheral.h"
#include <stddef.h>

// static inline void HAL_UART_WriteTxChar(HAL_UART_T * p_hal, uint8_t txChar);
// static inline uint8_t HAL_UART_ReadRxChar(const HAL_UART_T * p_hal);

// static inline bool _HAL_UART_ReadIsTxRegEmpty(const HAL_UART_T * p_hal);
// static inline bool _HAL_UART_ReadIsRxRegFull(const HAL_UART_T * p_hal);
// static inline uint8_t HAL_UART_ReadTxEmptyCount(const HAL_UART_T * p_hal);
// static inline uint8_t HAL_UART_ReadRxFullCount(const HAL_UART_T * p_hal);

// static inline bool HAL_UART_ReadRxOverrun(HAL_UART_T * p_hal);
// static inline void HAL_UART_ClearRxErrors(HAL_UART_T * p_hal);

// static inline void HAL_UART_EnableTxInterrupt(HAL_UART_T * p_hal);
// static inline void HAL_UART_DisableTxInterrupt(HAL_UART_T * p_hal);
// static inline void HAL_UART_EnableRxInterrupt(HAL_UART_T * p_hal);
// static inline void HAL_UART_DisableRxInterrupt(HAL_UART_T * p_hal);

// static inline void HAL_UART_WriteTxSwitch(HAL_UART_T * p_hal, bool enable);
// static inline void HAL_UART_WriteRxSwitch(HAL_UART_T * p_hal, bool enable);
// static inline bool HAL_UART_ConfigBaudRate(HAL_UART_T * p_hal, uint32_t baudRate_Bps);

// static inline void HAL_UART_Init(HAL_UART_T * p_hal);
// static inline void HAL_UART_Deinit(HAL_UART_T * p_hal);

#include HAL_PERIPHERAL_PATH(HAL_UART.h)

/******************************************************************************/
/*!
    Register level helpers, common to every platform.

    The counts only grow from the hardware's side, so one snapshot bounds the whole
    loop: min(count, length) needs no per char flag check and no fifo size macro.
*/
/******************************************************************************/
static inline bool HAL_UART_SendChar(HAL_UART_T * p_hal, uint8_t txChar)
{
    bool isNotFull = (HAL_UART_ReadTxEmptyCount(p_hal) > 0U);
    if (isNotFull == true) { HAL_UART_WriteTxChar(p_hal, txChar); }
    return isNotFull;
}

static inline bool HAL_UART_RecvChar(HAL_UART_T * p_hal, uint8_t * p_rxChar)
{
    bool isNotEmpty = (HAL_UART_ReadRxFullCount(p_hal) > 0U);
    if (isNotEmpty == true) { *p_rxChar = HAL_UART_ReadRxChar(p_hal); }
    return isNotEmpty;
}

/*! @return chars written to the hw buffer */
static inline size_t HAL_UART_Send(HAL_UART_T * p_hal, const uint8_t * p_srcBuffer, size_t length)
{
    size_t hwSpace = HAL_UART_ReadTxEmptyCount(p_hal);
    size_t count   = (hwSpace < length) ? hwSpace : length;

    for (size_t i = 0U; i < count; i++) { HAL_UART_WriteTxChar(p_hal, p_srcBuffer[i]); }

    return count;
}

/*! @return chars read out of the hw buffer */
static inline size_t HAL_UART_Recv(HAL_UART_T * p_hal, uint8_t * p_destBuffer, size_t length)
{
    size_t hwCount = HAL_UART_ReadRxFullCount(p_hal);
    size_t count   = (hwCount < length) ? hwCount : length;

    for (size_t i = 0U; i < count; i++) { p_destBuffer[i] = HAL_UART_ReadRxChar(p_hal); }

    return count;
}

#endif
