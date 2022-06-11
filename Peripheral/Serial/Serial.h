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
#ifndef SERIAL_H
#define SERIAL_H

#include "HAL_Serial.h"
#include "Config.h"
#include "Utility/Ring/Ring.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct Serial_Config_Tag
{
	HAL_Serial_T * const P_HAL_SERIAL;
}
Serial_Config_T;

typedef struct Serial_Tag
{
	const Serial_Config_T CONFIG;
	Ring_T RxRing;
	Ring_T TxRing;
	//  volatile Serial_Status_T TxStatus;		/*!< Status of last driver transmit operation */
	//  volatile Serial_Status_T RxStatus;		/*!< Status of last driver receive operation */
	//  Serial_TransferMode_T TransferMode;		/*!< interrupt/dma mode */
#if CONFIG_SERIAL_DMA_ENABLE
	uint8_t RxDmaChannel;                /*!< DMA channel number for DMA-based rx. */
	uint8_t TxDmaChannel;                /*!< DMA channel number for DMA-based tx. */
#endif
}
Serial_T;

#define SERIAL_INIT(p_Hal, p_TxBuffer, TxBufferSize, p_RxBuffer, RxBufferSize)		\
{																					\
	.CONFIG = {.P_HAL_SERIAL = p_Hal, },											\
	.TxRing = RING_INIT(p_TxBuffer, TxBufferSize, 1U, 0U),							\
	.RxRing = RING_INIT(p_RxBuffer, RxBufferSize, 1U, 0U),							\
}

/*
	Rx data reg/fifo full ISR, receive from hw to software buffer
*/
static inline void Serial_RxData_ISR(Serial_T * p_serial)
{
	uint8_t rxChar;

	while(HAL_Serial_ReadRxFullCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U) /* Rx until hw buffer is empty */
	{
		if(Ring_GetIsFull(&p_serial->RxRing) == true) /* Rx until software buffer is full */
		{
			/* if buffer stays full, disable irq to prevent blocking lower priority threads. user must restart rx irq */
			HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			break;
		}
		else
		{
			rxChar = HAL_Serial_ReadRxChar(p_serial->CONFIG.P_HAL_SERIAL);
			Ring_Enqueue(&p_serial->RxRing, &rxChar);
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

	while(HAL_Serial_ReadTxEmptyCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U) /* Tx until hw buffer is full */
	{
		if(Ring_GetIsEmpty(&p_serial->TxRing) == true) /* Tx until software buffer is empty */
		{
			HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			break;
		}
		else
		{
			Ring_Dequeue(&p_serial->TxRing, &txChar);
			HAL_Serial_WriteTxChar(p_serial->CONFIG.P_HAL_SERIAL, txChar);
		}
	}
}

static inline void Serial_PollRestartRxIsr(const Serial_T * p_serial)
{
	if((HAL_Serial_ReadRxOverrun(p_serial->CONFIG.P_HAL_SERIAL) == true) && (Ring_GetIsFull(&p_serial->RxRing) == false))
	{
		HAL_Serial_ClearRxErrors(p_serial->CONFIG.P_HAL_SERIAL);
		HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	}
}

//todo hw fifo buffer
/* UNTESTED */
static inline void Serial_PollRxData(Serial_T * p_serial)
{
	HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	Serial_RxData_ISR(p_serial);
	HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
}

static inline void Serial_PollTxData(Serial_T * p_serial)
{
	HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	Serial_TxData_ISR(p_serial);

	if(Ring_GetIsEmpty(&p_serial->TxRing) == false)
	{
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	}
}

static inline void Serial_FlushBuffers(Serial_T * p_serial)
{
	Ring_Clear(&p_serial->TxRing);
	Ring_Clear(&p_serial->RxRing);
}

static inline size_t Serial_GetRxFullCount(Serial_T * p_serial) 	{ return Ring_GetFullCount(&p_serial->RxRing); }
static inline size_t Serial_GetTxEmptyCount(Serial_T * p_serial) 	{ return Ring_GetEmptyCount(&p_serial->TxRing); }
static inline void Serial_EnableTxIsr(const Serial_T * p_serial) 	{ HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }
static inline void Serial_DisableTxIsr(const Serial_T * p_serial) 	{ HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }
static inline void Serial_EnableRx(const Serial_T * p_serial) 		{ HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }
static inline void Serial_DisableRx(const Serial_T * p_serial) 		{ HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }

extern void Serial_Init(Serial_T * p_serial);
extern void Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate);
extern bool Serial_SendByte(Serial_T * p_serial, uint8_t txChar);
extern bool Serial_RecvByte(Serial_T * p_serial, uint8_t * p_rxChar);
extern size_t Serial_SendMax(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t bufferSize);
extern size_t Serial_RecvMax(Serial_T * p_serial, uint8_t * p_destBuffer, size_t bufferSize);
extern bool Serial_SendN(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
extern bool Serial_RecvN(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);
	extern bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
	extern size_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);
extern uint8_t * Serial_AcquireTxBuffer(Serial_T * p_serial);
extern void Serial_ReleaseTxBuffer(Serial_T * p_serial, size_t writeSize);

#endif
