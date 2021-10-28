/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@file 	HAL_Serial.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_SERIAL_S32K_H
#define HAL_SERIAL_S32K_H

#include "External/S32K142/include/S32K142.h"
//#include "SDK/platform/drivers/inc/interrupt_manager.h"
//#include "SDK/platform/drivers/inc/lpuart_driver.h"

#include <stdint.h>
#include <stdbool.h>

typedef LPUART_Type HAL_Serial_T;

static inline void HAL_Serial_WriteTxChar(HAL_Serial_T * p_uartRegMap, uint8_t txChar)
{
	*((volatile uint8_t*)&(p_uartRegMap->DATA)) = txChar;
}

static inline uint8_t HAL_Serial_ReadRxChar(const HAL_Serial_T * p_uartRegMap)
{
	return (uint8_t)p_uartRegMap->DATA;
}

static inline bool HAL_Serial_ReadIsTxRegEmpty(const HAL_Serial_T * p_uartRegMap)
{
	return ((p_uartRegMap->STAT & LPUART_STAT_TDRE_MASK) != 0U) ? true : false;
}

static inline uint8_t HAL_Serial_ReadTxFifoEmpty(const HAL_Serial_T * p_uartRegMap)
{
#ifndef CONFIG_HAL_SERIAL_FIFO_SIZE
	#define CONFIG_HAL_SERIAL_FIFO_SIZE 4U
#endif
	uint32_t count = (p_uartRegMap->WATER & LPUART_WATER_TXCOUNT_MASK) >> LPUART_WATER_TXCOUNT_SHIFT;
	return (CONFIG_HAL_SERIAL_FIFO_SIZE - count);
}
static inline uint8_t HAL_Serial_ReadTxEmptyCount(const HAL_Serial_T * p_uartRegMap)
{
	return HAL_Serial_ReadTxFifoEmpty(p_uartRegMap);
}

static inline bool HAL_Serial_ReadIsRxRegFull(const HAL_Serial_T * p_uartRegMap)
{
	return ((p_uartRegMap->STAT & LPUART_STAT_RDRF_MASK) != 0U) ? true : false;
}

static inline uint8_t HAL_Serial_ReadRxFifoFull(const HAL_Serial_T * p_uartRegMap)
{
	uint32_t count = (p_uartRegMap->WATER & LPUART_WATER_RXCOUNT_MASK) >> LPUART_WATER_RXCOUNT_SHIFT;
	return (count);
}

static inline uint8_t HAL_Serial_ReadRxFullCount(const HAL_Serial_T * p_uartRegMap)
{
	return HAL_Serial_ReadRxFifoFull(p_uartRegMap);
}

static inline bool HAL_Serial_ReadRxOverrun(HAL_Serial_T * p_uartRegMap)
{
	return ((p_uartRegMap->STAT & LPUART_STAT_OR_MASK) != 0U) ? true : false;
}

static inline void HAL_Serial_ClearRxErrors(HAL_Serial_T * p_uartRegMap)
{
	p_uartRegMap->STAT = 0xC01FC000U; //FEATURE_LPUART_STAT_REG_FLAGS_MASK;
}

static inline void HAL_Serial_EnableTxInterrupt(HAL_Serial_T * p_uartRegMap)
{
	p_uartRegMap->CTRL |= LPUART_CTRL_TIE_MASK;
}

static inline void HAL_Serial_DisableTxInterrupt(HAL_Serial_T * p_uartRegMap)
{
	p_uartRegMap->CTRL &= ~LPUART_CTRL_TIE_MASK;
}

static inline void HAL_Serial_EnableRxInterrupt(HAL_Serial_T * p_uartRegMap)
{
	p_uartRegMap->CTRL |= LPUART_CTRL_RIE_MASK;
}

static inline void HAL_Serial_DisableRxInterrupt(HAL_Serial_T * p_uartRegMap)
{
	p_uartRegMap->CTRL &= ~LPUART_CTRL_RIE_MASK;
}

static inline void HAL_Serial_WriteTxSwitch(HAL_Serial_T * p_uartRegMap, bool enable)
{
	p_uartRegMap->CTRL = (p_uartRegMap->CTRL & ~LPUART_CTRL_TE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_TE_SHIFT);
	while ((bool)((p_uartRegMap->CTRL & LPUART_CTRL_TE_MASK) != 0U) != enable) {}	/* Wait for the register write operation to complete */
}

static inline void HAL_Serial_WriteRxSwitch(HAL_Serial_T * p_uartRegMap, bool enable)
{
	p_uartRegMap->CTRL = (p_uartRegMap->CTRL & ~LPUART_CTRL_RE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_RE_SHIFT);
    while((bool)((p_uartRegMap->CTRL & LPUART_CTRL_RE_MASK) != 0U) != enable) {}    /* Wait for the register write operation to complete */
}

static inline void HAL_Serial_ConfigBaudRate(HAL_Serial_T * p_uartRegMap, uint32_t baudRate)
{
	#define CONFIG_HAL_SERIAL_BASE_FREQ 48000000U
	const uint32_t UART_BASE_FREQ = CONFIG_HAL_SERIAL_BASE_FREQ;

	uint16_t sbr, sbrTemp;
	uint32_t osr, tempDiff, calculatedBaud, baudDiff, maxOsr;

	//disable/enable transmitters
	HAL_Serial_WriteTxSwitch(p_uartRegMap, false);
	HAL_Serial_WriteRxSwitch(p_uartRegMap, false);

	/* This lpuart instantiation uses a slightly different baud rate calculation
	 * The idea is to use the best OSR (over-sampling rate) possible
	 * Note, osr is typically hard-set to 16 in other lpuart instantiations
	 * First calculate the baud rate using the minimum OSR possible (4) */
	osr = 4;
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
		p_uartRegMap->BAUD |= LPUART_BAUD_BOTHEDGE_MASK;
	}

	/* program the osr value (bit value is one less than actual value) */
	p_uartRegMap->BAUD = (p_uartRegMap->BAUD & ~(LPUART_BAUD_OSR_MASK)) | LPUART_BAUD_OSR((osr - 1U));

	/* write the sbr value to the BAUD registers */
	/* Removed the shift operation as the SBR field position is zero; shifting with 0 violates MISRA */
	p_uartRegMap->BAUD = (p_uartRegMap->BAUD & ~(LPUART_BAUD_SBR_MASK)) | (sbr & LPUART_BAUD_SBR_MASK);

	HAL_Serial_WriteTxSwitch(p_uartRegMap, true);
	HAL_Serial_WriteRxSwitch(p_uartRegMap, true);
}

static inline void HAL_Serial_Init(HAL_Serial_T * p_uartRegMap)
{
    /* Clear the error/interrupt flags */
	p_uartRegMap->STAT = 0xC01FC000U; //FEATURE_LPUART_STAT_REG_FLAGS_MASK;
//	p_uartRegMap->CTRL = 0x00000000;   /* Reset all features/interrupts by default */
	p_uartRegMap->FIFO = 0x0003C000U | LPUART_FIFO_RXFE_MASK | LPUART_FIFO_TXFE_MASK;
	p_uartRegMap->WATER = LPUART_WATER_RXWATER(0x03U) | LPUART_WATER_TXWATER(0x01U);
    HAL_Serial_ConfigBaudRate(p_uartRegMap, 9600U);
}

static inline void HAL_Serial_Deinit(HAL_Serial_T * p_uartRegMap)
{
	HAL_Serial_DisableTxInterrupt(p_uartRegMap);
	HAL_Serial_DisableRxInterrupt(p_uartRegMap);
	HAL_Serial_WriteTxSwitch(p_uartRegMap, false);
	HAL_Serial_WriteRxSwitch(p_uartRegMap, false);
}

#endif
