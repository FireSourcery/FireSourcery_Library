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
#include "SDK/platform/drivers/inc/interrupt_manager.h"
//#include "SDK/platform/drivers/inc/lpuart_driver.h"

#include <stdint.h>
#include <stdbool.h>

typedef LPUART_Type HAL_Serial_T;


static inline void HAL_Serial_WriteTxChar(HAL_Serial_T *p_uartRegMap, uint8_t txChar)
{
	*((volatile uint8_t*) &(p_uartRegMap->DATA)) = txChar;
}

static inline uint8_t HAL_Serial_ReadRxChar(const HAL_Serial_T *p_uartRegMap)
{
	return (uint8_t)p_uartRegMap->DATA;
}

static inline bool HAL_Serial_ReadTxRegEmpty(const HAL_Serial_T *p_uartRegMap)
{
	return ((p_uartRegMap->FIFO | LPUART_FIFO_TXEMPT_MASK) != 0U) ? true : false;
}

static inline bool HAL_Serial_ReadRxRegFull(const HAL_Serial_T *p_uartRegMap)
{
	return ((p_uartRegMap->FIFO | LPUART_FIFO_RXEMPT_MASK) != 0U) ? false : true;
}

static inline void HAL_Serial_EnableTxInterrupt(HAL_Serial_T *p_uartRegMap)
{
	p_uartRegMap->CTRL |= LPUART_CTRL_TIE_MASK;
}

static inline void HAL_Serial_DisableTxInterrupt(HAL_Serial_T *p_uartRegMap)
{
	p_uartRegMap->CTRL &= ~LPUART_CTRL_TIE_MASK;
}

static inline void HAL_Serial_WriteTxSwitch(HAL_Serial_T *p_uartRegMap, bool enable)
{
	p_uartRegMap->CTRL = (p_uartRegMap->CTRL & ~LPUART_CTRL_TE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_TE_SHIFT);
    /* Wait for the register write operation to complete */
    while((bool)((p_uartRegMap->CTRL & LPUART_CTRL_TE_MASK) != 0U) != enable) {}
}

static inline void HAL_Serial_EnableRxInterrupt(HAL_Serial_T *p_uartRegMap)
{
	p_uartRegMap->CTRL |= LPUART_CTRL_RIE_MASK;
}

static inline void HAL_Serial_DisableRxInterrupt(HAL_Serial_T *p_uartRegMap)
{
	p_uartRegMap->CTRL &= ~LPUART_CTRL_RIE_MASK;
}

//static inline void HAL_Serial_ReadIsTxInterruptFlag(HAL_Serial_T *p_uartRegMap)
//{
//	p_uartRegMap->STAT[TDRE]
//}

static inline void HAL_Serial_WriteRxSwitch(HAL_Serial_T *p_uartRegMap, bool enable)
{
	p_uartRegMap->CTRL = (p_uartRegMap->CTRL & ~LPUART_CTRL_RE_MASK) | ((enable ? 1UL : 0UL) << LPUART_CTRL_RE_SHIFT);
    /* Wait for the register write operation to complete */
    while((bool)((p_uartRegMap->CTRL & LPUART_CTRL_RE_MASK) != 0U) != enable) {}
}

static inline void HAL_Serial_ConfigBaudRate(HAL_Serial_T *p_uartRegMap, uint32_t baudRate)
{
	const uint32_t UART_BASE_FREQ = 48000000U;

	uint16_t sbr, sbrTemp, i;
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
		for (i = 5U; i <= maxOsr; i++)
		{
			/* calculate the temporary sbr value   */
			sbrTemp = (uint16_t) (UART_BASE_FREQ / (baudRate * i));
			/* calculate the baud rate based on the temporary osr and sbr values */
			calculatedBaud = (UART_BASE_FREQ / (i * sbrTemp));

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
				osr = i; /* update and store the best osr value calculated */
				sbr = sbrTemp; /* update store the best sbr value calculated */
			}
		}
	}
	/* Check if osr is between 4x and 7x oversampling.
	 * If so, then "BOTHEDGE" sampling must be turned on */
	if (osr < 8U)
	{
		p_uartRegMap->BAUD |= LPUART_BAUD_BOTHEDGE_MASK;
	}

	/* program the osr value (bit value is one less than actual value) */
	uint32_t baudRegValTemp;

	baudRegValTemp = p_uartRegMap->BAUD;
	baudRegValTemp &= ~(LPUART_BAUD_OSR_MASK);
	baudRegValTemp |= LPUART_BAUD_OSR((osr - 1U));
	p_uartRegMap->BAUD = baudRegValTemp;

	/* write the sbr value to the BAUD registers */

	baudRegValTemp = p_uartRegMap->BAUD;
	baudRegValTemp &= ~(LPUART_BAUD_SBR_MASK);
	/* Removed the shift operation as the SBR field position is zero; shifting with 0 violates MISRA */
	baudRegValTemp |= sbr & LPUART_BAUD_SBR_MASK;
	p_uartRegMap->BAUD = baudRegValTemp;

	HAL_Serial_WriteTxSwitch(p_uartRegMap, true);
	HAL_Serial_WriteRxSwitch(p_uartRegMap, true);
}

static inline void HAL_Serial_Init(HAL_Serial_T *p_uartRegMap)
{
//	lpuart_state_t lpUartState0;
//
//	const lpuart_user_config_t lpuart_0_InitConfig0 = {
//	  .transferType = LPUART_USING_INTERRUPTS,
//	  .baudRate = 9600UL,
//	  .parityMode = LPUART_PARITY_DISABLED,
//	  .stopBitCount = LPUART_ONE_STOP_BIT,
//	  .bitCountPerChar = LPUART_8_BITS_PER_CHAR,
//	  .rxDMAChannel = 0UL,
//	  .txDMAChannel = 0UL
//	};
//
//	const lpuart_user_config_t lpuart_0_InitConfig1 = {
//	  .transferType = LPUART_USING_INTERRUPTS,
//	  .baudRate = 115200UL,
//	  .parityMode = LPUART_PARITY_DISABLED,
//	  .stopBitCount = LPUART_ONE_STOP_BIT,
//	  .bitCountPerChar = LPUART_8_BITS_PER_CHAR,
//	  .rxDMAChannel = 0UL,
//	  .txDMAChannel = 0UL
//	};
	//	LPUART_DRV_Init(1U, &lpUartState0, &lpuart_0_InitConfig0);

    /* Set the default oversampling ratio (16) and baud-rate divider (4) */
//	p_uartRegMap->BAUD = ((uint32_t)((FEATURE_LPUART_DEFAULT_OSR << LPUART_BAUD_OSR_SHIFT) |  (FEATURE_LPUART_DEFAULT_SBR << LPUART_BAUD_SBR_SHIFT)));
	p_uartRegMap->BAUD = ((uint32_t)((0x0FUL << LPUART_BAUD_OSR_SHIFT) |  (0x04UL << LPUART_BAUD_SBR_SHIFT)));

    /* Clear the error/interrupt flags */
	p_uartRegMap->STAT = 0xC01FC000U; //FEATURE_LPUART_STAT_REG_FLAGS_MASK;
    /* Reset all features/interrupts by default */
	p_uartRegMap->CTRL = 0x00000000;
    /* Reset match addresses */
	p_uartRegMap->MATCH = 0x00000000;
    /* Reset IrDA modem features */
	p_uartRegMap->MODIR = 0x00000000;
    /* Reset FIFO feature */
	p_uartRegMap->FIFO = 0x0003C000U; //FEATURE_LPUART_FIFO_RESET_MASK;
    /* Reset FIFO Watermark values */
	p_uartRegMap->WATER = 0x00000000;

    HAL_Serial_ConfigBaudRate(p_uartRegMap, 9600U);

    /* config 8-bit (M=0) or 9-bits (M=1) */
    p_uartRegMap->CTRL = (p_uartRegMap->CTRL & ~LPUART_CTRL_M_MASK) | (8U << LPUART_CTRL_M_SHIFT);
    /* clear M10 to make sure not 10-bit mode */
    p_uartRegMap->BAUD &= ~LPUART_BAUD_M10_MASK;

    p_uartRegMap->BAUD = (p_uartRegMap->BAUD & ~LPUART_BAUD_SBNS_MASK) | ((uint32_t)1U << LPUART_BAUD_SBNS_SHIFT);

    if(p_uartRegMap == LPUART0)
    {
//    	INT_SYS_SetPriority(LPUART0_RxTx_IRQn, 15U);
    	INT_SYS_EnableIRQ(LPUART0_RxTx_IRQn);
    }
    else if (p_uartRegMap == LPUART1)
    {
//    	INT_SYS_SetPriority(LPUART1_RxTx_IRQn, 15U);
    	INT_SYS_EnableIRQ(LPUART1_RxTx_IRQn);
    }


}

#endif
