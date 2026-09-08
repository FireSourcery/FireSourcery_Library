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
#ifndef HAL_SERIAL_H
#define HAL_SERIAL_H

#include <stddef.h>

#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_Serial.h)

// typedef struct HAL_Serial HAL_Serial_T;

// static void HAL_Serial_WriteTxChar(HAL_Serial_T * p_hal, uint8_t txChar);
// static inline uint8_t HAL_Serial_ReadRxChar(const HAL_Serial_T * p_hal) {}
// static inline uint8_t HAL_Serial_ReadTxEmptyCount(const HAL_Serial_T * p_hal) {}
// static inline uint8_t HAL_Serial_ReadRxFullCount(const HAL_Serial_T * p_hal) {}
// static inline bool HAL_Serial_ReadRxOverrun(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_ClearRxErrors(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_EnableTxInterrupt(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_DisableTxInterrupt(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_EnableRxInterrupt(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_DisableRxInterrupt(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_WriteTxSwitch(HAL_Serial_T * p_hal, bool enable) {}
// static inline void HAL_Serial_WriteRxSwitch(HAL_Serial_T * p_hal, bool enable) {}
// static inline void HAL_Serial_ConfigBaudRate(HAL_Serial_T * p_hal, uint32_t baudRate) {}
// static inline void HAL_Serial_Init(HAL_Serial_T * p_hal) {}
// static inline void HAL_Serial_Deinit(HAL_Serial_T * p_hal) {}
/*

*/
static inline bool HAL_UART_SendChar(HAL_Serial_T * p_hal, const uint8_t txchar)
{
    bool isNotFull = (HAL_Serial_ReadTxEmptyCount(p_hal) > 0U);
    if (isNotFull == true) { HAL_Serial_WriteTxChar(p_hal, txchar); }
    return isNotFull;
}

/*

*/
static inline bool HAL_UART_RecvChar(HAL_Serial_T * p_hal, uint8_t * p_rxChar)
{
    bool isNotEmpty = (HAL_Serial_ReadRxFullCount(p_hal) > 0U);
    if (isNotEmpty == true) { *p_rxChar = HAL_Serial_ReadRxChar(p_hal); }
    return isNotEmpty;
}

static inline uint8_t HAL_UART_GetLoopCount(size_t length)
{
#ifdef SERIAL_HW_FIFO_DISABLE
    (void)length;
    return 1U;
#else
    return length;
#endif
}

static inline size_t HAL_UART_Send(HAL_Serial_T * p_hal, const uint8_t * p_srcBuffer, size_t length)
{
    size_t charCount;
    for (charCount = 0U; charCount < HAL_UART_GetLoopCount(length); charCount++)
    {
        if (HAL_UART_SendChar(p_hal, p_srcBuffer[charCount]) == false) { break; }
    }
    return charCount;
}

static inline size_t HAL_UART_Recv(HAL_Serial_T * p_hal, uint8_t * p_destBuffer, size_t length)
{
    size_t charCount;
    for (charCount = 0U; charCount < HAL_UART_GetLoopCount(length); charCount++)
    {
        if (HAL_UART_RecvChar(p_hal, &p_destBuffer[charCount]) == false) { break; }
    }
    return charCount;
}

#endif
