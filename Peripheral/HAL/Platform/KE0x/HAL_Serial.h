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
    @version V0
*/
/******************************************************************************/
#ifndef HAL_SERIAL_PLATFORM_H
#define HAL_SERIAL_PLATFORM_H

#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_SERIAL_CLOCK_SOURCE_FREQ
#define HAL_SERIAL_CLOCK_SOURCE_FREQ (CPU_FREQ / 2U)
#endif

#define HAL_SERIAL_FIFO_SIZE 0U

typedef UART_Type HAL_Serial_T;

static inline void HAL_Serial_WriteTxChar(HAL_Serial_T * p_hal, uint8_t txChar) { *((volatile uint8_t *)&(p_hal->D)) = txChar; }
static inline uint8_t HAL_Serial_ReadRxChar(const HAL_Serial_T * p_hal)         { return (uint8_t)p_hal->D; }

static inline bool _HAL_Serial_ReadIsTxRegEmpty(const HAL_Serial_T * p_hal)     { return ((p_hal->S1 & UART_S1_TDRE_MASK) != 0U); }
static inline bool _HAL_Serial_ReadIsRxRegFull(const HAL_Serial_T * p_hal)      { return ((p_hal->S1 & UART_S1_RDRF_MASK) != 0U); }
static inline uint8_t HAL_Serial_ReadTxEmptyCount(const HAL_Serial_T * p_hal)   { return (uint8_t)_HAL_Serial_ReadIsTxRegEmpty(p_hal); }
static inline uint8_t HAL_Serial_ReadRxFullCount(const HAL_Serial_T * p_hal)    { return (uint8_t)_HAL_Serial_ReadIsRxRegFull(p_hal); }

static inline bool HAL_Serial_ReadRxOverrun(HAL_Serial_T * p_hal) { return ((p_hal->S1 & UART_S1_OR_MASK) != 0U); }
static inline void HAL_Serial_ClearRxErrors(HAL_Serial_T * p_hal) { (void)p_hal; }

static inline void HAL_Serial_EnableTxInterrupt(HAL_Serial_T * p_hal)   { p_hal->C2 |= UART_C2_TIE_MASK; }
static inline void HAL_Serial_DisableTxInterrupt(HAL_Serial_T * p_hal)  { p_hal->C2 &= ~UART_C2_TIE_MASK; }
static inline void HAL_Serial_EnableRxInterrupt(HAL_Serial_T * p_hal)   { p_hal->C2 |= UART_C2_RIE_MASK; }
static inline void HAL_Serial_DisableRxInterrupt(HAL_Serial_T * p_hal)  { p_hal->C2 &= ~UART_C2_RIE_MASK; }

static inline void HAL_Serial_WriteTxSwitch(HAL_Serial_T * p_hal, bool enable)
{
    p_hal->C2 = (p_hal->C2 & ~UART_C2_TE_MASK) | ((enable ? 1UL : 0UL) << UART_C2_TE_SHIFT);
    while(((p_hal->C2 & UART_C2_TE_MASK) != 0U) != enable) {}    /* Wait for the register write operation to complete */
}

static inline void HAL_Serial_WriteRxSwitch(HAL_Serial_T * p_hal, bool enable)
{
    p_hal->C2 = (p_hal->C2 & ~UART_C2_RE_MASK) | ((enable ? 1UL : 0UL) << UART_C2_RE_SHIFT);
    while(((p_hal->C2 & UART_C2_RE_MASK) != 0U) != enable) {}    /* Wait for the register write operation to complete */
}

static inline bool HAL_Serial_ConfigBaudRate(HAL_Serial_T * p_hal, uint32_t baudRate_Bps)
{
    uint32_t sbr = 0;
    uint32_t baudDiff = 0;
    uint8_t oldCtrl;
    bool isSuccess;

    /* Calculate the baud rate modulo divisor, sbr*/
    sbr = HAL_SERIAL_CLOCK_SOURCE_FREQ / (baudRate_Bps * 16U);
    /* set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate */
    if(sbr == 0U) { sbr = 1U; }

    /* Calculate the baud rate based on the temporary SBR values */
    baudDiff = (HAL_SERIAL_CLOCK_SOURCE_FREQ / (sbr * 16U)) - baudRate_Bps;

    /* Select the better value between sbr and (sbr + 1) */
    if (baudDiff > (baudRate_Bps - (HAL_SERIAL_CLOCK_SOURCE_FREQ / (16U * (sbr + 1U)))))
    {
        baudDiff = baudRate_Bps - (HAL_SERIAL_CLOCK_SOURCE_FREQ / (16U * (sbr + 1U)));
        sbr++;
    }

    /* next, check to see if actual baud rate is within 3% of desired baud rate based on the calculate SBR value */
    if (baudDiff < ((baudRate_Bps / 100U) * 3U))
    {
        /* Store C2 before disable Tx and Rx */
        oldCtrl = p_hal->C2;

        /* Disable UART TX RX before setting. */
        p_hal->C2 &= ~((uint8_t)UART_C2_TE_MASK | (uint8_t)UART_C2_RE_MASK);

        /* Write the sbr value to the BDH and BDL registers*/
        p_hal->BDH = (p_hal->BDH & ~(uint8_t)UART_BDH_SBR_MASK) | (uint8_t)(sbr >> 8U);
        p_hal->BDL = (uint8_t)sbr;

        /* Restore C2. */
        p_hal->C2 = oldCtrl;

        isSuccess = true;
    }
    else
    {
        /* Unacceptable baud rate difference of more than 3%*/
        isSuccess = false;
    }

    return isSuccess;
}

static inline void HAL_Serial_Init(HAL_Serial_T * p_hal)
{
    (void)p_hal;
    // HAL_Serial_ConfigBaudRate(p_hal, 9600U);
}

static inline void HAL_Serial_Deinit(HAL_Serial_T * p_hal)
{
    HAL_Serial_DisableTxInterrupt(p_hal);
    HAL_Serial_DisableRxInterrupt(p_hal);
    HAL_Serial_WriteTxSwitch(p_hal, false);
    HAL_Serial_WriteRxSwitch(p_hal, false);
}

#endif
