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
#include "Utility/Queue/Queue.h"

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
	Queue_T RxQueue;
	Queue_T TxQueue;
	//	uin32t_t BaudRate;
	//#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	//	uint8_t TxMutex;
	//	uint8_t RxMutex;
	//#endif
	//	void (* RxCallback)(void *); 	/*!< Callback on Rx */
	//	void * RxCallbackData; 			/*!< Receive callback parameter pointer.*/
	//	void (* TxCallback)(void *); 	/*!< Callback on Tx */
	//	void * TxCallbackData; 			/*!< Transmit callback parameter pointer.*/
	//  volatile Serial_Status_T TxStatus;		/*!< Status of last driver transmit operation */
	//  volatile Serial_Status_T RxStatus;		/*!< Status of last driver receive operation */
	//  Serial_TransferMode_T TransferMode;		/*!< interrupt/dma mode */
#if CONFIG_SERIAL_DMA_ENABLE
	uint8_t RxDMAChannel;                /*!< DMA channel number for DMA-based rx. */
	uint8_t TxDMAChannel;                /*!< DMA channel number for DMA-based tx. */
#endif
}
Serial_T;

#define SERIAL_DEFINE(p_Hal, p_TxBuffer, TxBufferSize, p_RxBuffer, RxBufferSize)	\
{																	\
	.CONFIG = {.P_HAL_SERIAL = p_Hal, },							\
	.TxQueue = QUEUE_DEFINE(p_TxBuffer, TxBufferSize, 1U, 0U),		\
	.RxQueue = QUEUE_DEFINE(p_RxBuffer, RxBufferSize, 1U, 0U),		\
}

/*
	Rx data reg/fifo full ISR, receive from hw to software buffer
*/
static inline void Serial_RxData_ISR(Serial_T * p_serial)
{
	uint8_t rxChar;

	while(HAL_Serial_ReadRxFullCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U) /* Rx until hw buffer is empty */
	{
		if(Queue_GetIsFull(&p_serial->RxQueue) == true) /* Rx until software buffer is full */
		{
			/* if buffer stays full, disable irq to prevent blocking lower priority threads. user must restart rx irq */
			HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			break;
		}
		else
		{
			rxChar = HAL_Serial_ReadRxChar(p_serial->CONFIG.P_HAL_SERIAL);
			Queue_Enqueue(&p_serial->RxQueue, &rxChar);
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
		if(Queue_GetIsEmpty(&p_serial->TxQueue) == true) /* Tx until software buffer is empty */
		{
			HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			break;
		}
		else
		{
			Queue_Dequeue(&p_serial->TxQueue, &txChar);
			HAL_Serial_WriteTxChar(p_serial->CONFIG.P_HAL_SERIAL, txChar);
		}
	}
}

static inline void Serial_PollRestartRxIsr(const Serial_T * p_serial)
{
	if((HAL_Serial_ReadRxOverrun(p_serial->CONFIG.P_HAL_SERIAL) == true) && (Queue_GetIsFull(&p_serial->RxQueue) == false))
	{
		HAL_Serial_ClearRxErrors(p_serial->CONFIG.P_HAL_SERIAL);
		HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	}
}

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

	if(Queue_GetIsEmpty(&p_serial->TxQueue) == false)
	{
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	}
}

static inline void Serial_FlushBuffers(Serial_T * p_serial)
{
	Queue_Clear(&p_serial->TxQueue);
	Queue_Clear(&p_serial->RxQueue);
}

static inline uint32_t Serial_GetRxFullCount(Serial_T * p_serial) 	{ return Queue_GetFullCount(&p_serial->RxQueue); }
static inline uint32_t Serial_GetTxEmptyCount(Serial_T * p_serial) 	{ return Queue_GetEmptyCount(&p_serial->TxQueue); }
static inline void Serial_EnableTxIsr(const Serial_T * p_serial) 	{ HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }
static inline void Serial_DisableTxIsr(const Serial_T * p_serial) 	{ HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }
static inline void Serial_EnableRx(const Serial_T * p_serial) 		{ HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }
static inline void Serial_DisableRx(const Serial_T * p_serial) 		{ HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL); }

extern void Serial_Init(Serial_T * p_serial);
extern void Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate);
extern bool Serial_SendChar(Serial_T * p_serial, uint8_t txChar);
extern bool Serial_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar);
extern uint32_t Serial_SendBuffer(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t bufferSize);
extern uint32_t Serial_RecvBuffer(Serial_T * p_serial, uint8_t * p_destBuffer, size_t bufferSize);
extern bool Serial_SendN(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
extern bool Serial_RecvN(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);
extern bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length);
extern uint32_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length);
extern uint8_t * Serial_AcquireTxBuffer(Serial_T * p_serial);
extern void Serial_ReleaseTxBuffer(Serial_T * p_serial, size_t writeSize);

#endif
