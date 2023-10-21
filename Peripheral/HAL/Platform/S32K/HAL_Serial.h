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

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

#ifndef CONFIG_HAL_SERIAL_FIFO_SIZE
    #define CONFIG_HAL_SERIAL_FIFO_SIZE 4U
#endif

#ifndef CONFIG_HAL_SERIAL_BASE_FREQ
    #define CONFIG_HAL_SERIAL_BASE_FREQ 48000000U
#endif

typedef LPUART_Type HAL_Serial_T; /* Peripheral register map provided by chip manufacturer */

static inline void HAL_Serial_WriteTxChar(HAL_Serial_T * p_hal, uint8_t txChar)     { *((volatile uint8_t *)&(p_hal->DATA)) = txChar; }
static inline uint8_t HAL_Serial_ReadRxChar(const HAL_Serial_T * p_hal)             { return ((uint8_t)p_hal->DATA); }
static inline bool HAL_Serial_ReadIsTxRegEmpty(const HAL_Serial_T * p_hal)             { return ((p_hal->STAT & LPUART_STAT_TDRE_MASK) != 0U); }
static inline bool HAL_Serial_ReadIsRxRegFull(const HAL_Serial_T * p_hal)             { return ((p_hal->STAT & LPUART_STAT_RDRF_MASK) != 0U); }
static inline uint8_t _HAL_Serial_ReadTxFifoFullCount(const HAL_Serial_T * p_hal)     { return ((p_hal->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT); }
static inline uint8_t HAL_Serial_ReadRxFifoFullCount(const HAL_Serial_T * p_hal)     { return ((p_hal->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT); }
static inline uint8_t HAL_Serial_ReadTxFifoEmptyCount(const HAL_Serial_T * p_hal)     { return (CONFIG_HAL_SERIAL_FIFO_SIZE - _HAL_Serial_ReadTxFifoFullCount(p_hal)); }
//todo change back to hw fifo
static inline uint8_t HAL_Serial_ReadTxEmptyCount(const HAL_Serial_T * p_hal)     { return HAL_Serial_ReadIsTxRegEmpty(p_hal); }
static inline uint8_t HAL_Serial_ReadRxFullCount(const HAL_Serial_T * p_hal)     { return HAL_Serial_ReadIsRxRegFull(p_hal); }
static inline bool HAL_Serial_ReadRxOverrun(HAL_Serial_T * p_hal)         { return ((p_hal->STAT & LPUART_STAT_OR_MASK) != 0U); }
static inline void HAL_Serial_ClearRxErrors(HAL_Serial_T * p_hal)         { p_hal->STAT = 0xC01FC000U; } //FEATURE_LPUART_STAT_REG_FLAGS_MASK;
static inline void HAL_Serial_EnableTxInterrupt(HAL_Serial_T * p_hal)     { p_hal->CTRL |= LPUART_CTRL_TIE_MASK; }
static inline void HAL_Serial_DisableTxInterrupt(HAL_Serial_T * p_hal)     { p_hal->CTRL &= ~LPUART_CTRL_TIE_MASK; }
static inline void HAL_Serial_EnableRxInterrupt(HAL_Serial_T * p_hal)     { p_hal->CTRL |= LPUART_CTRL_RIE_MASK; }
static inline void HAL_Serial_DisableRxInterrupt(HAL_Serial_T * p_hal)     { p_hal->CTRL &= ~LPUART_CTRL_RIE_MASK; }

static inline void HAL_Serial_WriteTxSwitch(HAL_Serial_T * p_hal, bool enable)
{
    p_hal->CTRL = (p_hal->CTRL & ~LPUART_CTRL_TE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_TE_SHIFT);
    while (((p_hal->CTRL & LPUART_CTRL_TE_MASK) != 0U) != enable) {}    /* Wait for the register write operation to complete */
}

static inline void HAL_Serial_WriteRxSwitch(HAL_Serial_T * p_hal, bool enable)
{
    p_hal->CTRL = (p_hal->CTRL & ~LPUART_CTRL_RE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_RE_SHIFT);
    while(((p_hal->CTRL & LPUART_CTRL_RE_MASK) != 0U) != enable) {}    /* Wait for the register write operation to complete */
}

static inline void HAL_Serial_ConfigBaudRate(HAL_Serial_T * p_hal, uint32_t baudRate)
{
    const uint32_t UART_BASE_FREQ = CONFIG_HAL_SERIAL_BASE_FREQ;

    uint16_t sbr, sbrTemp;
    uint32_t osr, tempDiff, calculatedBaud, baudDiff, maxOsr;

    HAL_Serial_WriteTxSwitch(p_hal, false);
    HAL_Serial_WriteRxSwitch(p_hal, false);

    /* This lpuart instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, osr is typically hard-set to 16 in other lpuart instantiations
     * First calculate the baud rate using the minimum OSR possible (4) */
    osr = 4U;
    sbr = (uint16_t) (UART_BASE_FREQ / (baudRate * osr));
    calculatedBaud = (UART_BASE_FREQ / (osr * sbr));

    if (calculatedBaud > baudRate)
    {
        baudDiff = calculatedBaud - baudRate;
    }
    else
    {
        baudDiff = baudRate - calculatedBaud;
    }

    /* find maximum osr */
    maxOsr = UART_BASE_FREQ / baudRate;
    if (maxOsr > 32U)
    {
        maxOsr = 32U;
    }

    /* loop to find the best osr value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of osr */
    if (maxOsr >= 5U)
    {
        for (uint16_t iOsr = 5U; iOsr <= maxOsr; iOsr++)
        {
            /* calculate the temporary sbr value   */
            sbrTemp = (uint16_t) (UART_BASE_FREQ / (baudRate * iOsr));
            /* calculate the baud rate based on the temporary osr and sbr values */
            calculatedBaud = (UART_BASE_FREQ / (iOsr * sbrTemp));

            if (calculatedBaud > baudRate)
            {
                tempDiff = calculatedBaud - baudRate;
            }
            else
            {
                tempDiff = baudRate - calculatedBaud;
            }

            if (tempDiff <= baudDiff)
            {
                baudDiff = tempDiff;
                osr = iOsr; /* update and store the best osr value calculated */
                sbr = sbrTemp; /* update store the best sbr value calculated */
            }
        }
    }

    /* Check if osr is between 4x and 7x oversampling. If so, then "BOTHEDGE" sampling must be turned on */
    if (osr < 8U)
    {
        p_hal->BAUD |= LPUART_BAUD_BOTHEDGE_MASK;
    }

    /* program the osr value (bit value is one less than actual value) */
    p_hal->BAUD = (p_hal->BAUD & ~(LPUART_BAUD_OSR_MASK)) | LPUART_BAUD_OSR((osr - 1U));

    /* write the sbr value to the BAUD registers */
    /* Removed the shift operation as the SBR field position is zero; shifting with 0 violates MISRA */
    p_hal->BAUD = (p_hal->BAUD & ~(LPUART_BAUD_SBR_MASK)) | (sbr & LPUART_BAUD_SBR_MASK);

    HAL_Serial_WriteTxSwitch(p_hal, true);
    HAL_Serial_WriteRxSwitch(p_hal, true);
}

static inline void HAL_Serial_Init(HAL_Serial_T * p_hal)
{
    p_hal->STAT = 0xC01FC000U; /* Clear the error/interrupt flags */ //FEATURE_LPUART_STAT_REG_FLAGS_MASK;
//    p_hal->CTRL = 0x00000000;   /* Reset all features/interrupts by default */
    p_hal->FIFO = 0x0003C000U;// | LPUART_FIFO_RXFE_MASK | LPUART_FIFO_TXFE_MASK;
//    p_hal->WATER = LPUART_WATER_RXWATER(0x03U) | LPUART_WATER_TXWATER(0x01U);
    HAL_Serial_ConfigBaudRate(p_hal, 9600U);
}

static inline void HAL_Serial_Deinit(HAL_Serial_T * p_hal)
{
    HAL_Serial_DisableTxInterrupt(p_hal);
    HAL_Serial_DisableRxInterrupt(p_hal);
    HAL_Serial_WriteTxSwitch(p_hal, false);
    HAL_Serial_WriteRxSwitch(p_hal, false);
}

#endif
