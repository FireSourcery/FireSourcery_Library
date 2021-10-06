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

#include "System/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
	HAL_Serial_T * const P_HAL_SERIAL;
	//	bool HasTxHarwareFIFO;
	//	bool HasRxHarwareFIFO;
	const Queue_Config_T RX_QUEUE;
	const Queue_Config_T TX_QUEUE;

//#ifdef CONFIG_SERIAL_NO_QUEUE
	volatile uint8_t * const P_TX_BUFFER;
	volatile uint8_t * const P_RX_BUFFER;
	const uint32_t TX_BUFFER_SIZE;
	const uint32_t RX_BUFFER_SIZE;
//#endif

} Serial_Config_T;

typedef struct
{
	const Serial_Config_T CONFIG;
	Queue_T RxQueue;
	Queue_T TxQueue;

//#ifdef CONFIG_SERIAL_NO_QUEUE
	volatile uint32_t TxBufferHead; // Write to buffer head. Tx from buffer tail
	volatile uint32_t TxBufferTail;
	volatile uint32_t RxBufferHead; // Rx to buffer head. Read from buffer tail.
	volatile uint32_t RxBufferTail;
//#endif

#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	uint8_t TxMutex;
	uint8_t RxMutex;
#endif

//	volatile bool IsTxBusy;         /*!< True if there is an active transmit. */
//	volatile bool IsRxBusy;         /*!< True if there is an active receive. */
//	void (* RxCallback)(void *); 	/*!< Callback on Rx */
//	void * RxCallbackData; 			/*!< Receive callback parameter pointer.*/
//	void (* TxCallback)(void *); 	/*!< Callback on Tx */
//	void * TxCallbackData; 			/*!< Transmit callback parameter pointer.*/
//  volatile bool RxComplete;
//  volatile bool TxComplete;
//  volatile Serial_Status_T TxStatus;		/*!< Status of last driver transmit operation */
//  volatile Serial_Status_T RxStatus;		/*!< Status of last driver receive operation */
//  Serial_TransferMode_T TransferMode;		/*!< interrupt/dma mode */

#if CONFIG_SERIAL_DMA_ENABLE
    uint8_t RxDMAChannel;                /*!< DMA channel number for DMA-based rx. */
    uint8_t TxDMAChannel;                /*!< DMA channel number for DMA-based tx. */
#endif
} Serial_T;

static inline bool IsTxBufferEmpty(Serial_T * p_serial)	{return (p_serial->TxBufferHead == p_serial->TxBufferTail);};
static inline bool IsRxBufferFull(Serial_T * p_serial)	{return ((p_serial->RxBufferHead + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE == p_serial->RxBufferTail);};

/*
 * Rx data reg/fifo full ISR, receive from hw to software buffer
 */
static inline void Serial_RxData_ISR(Serial_T * p_serial)
{
	uint32_t rxBufferHeadNext;

	while (HAL_Serial_ReadRxRegFull(p_serial->CONFIG.P_HAL_SERIAL))
	{
		rxBufferHeadNext = (p_serial->RxBufferHead + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE;

		if (rxBufferHeadNext == p_serial->RxBufferTail) //Rx until software buffer is full
		{
			//if buffer stays full, need to disable irq to prevent blocking lower prio threads. user must restart rx irq
			HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			break;
		}
		else
		{
			p_serial->CONFIG.P_RX_BUFFER[p_serial->RxBufferHead] = HAL_Serial_ReadRxChar(p_serial->CONFIG.P_HAL_SERIAL);
			p_serial->RxBufferHead = rxBufferHeadNext;
		}
	}
}

/*
 * Tx data reg/fifo empty ISR, transmit from software buffer to hw
 */
static inline void Serial_TxData_ISR(Serial_T * p_serial)
{
	while (HAL_Serial_ReadTxRegEmpty(p_serial->CONFIG.P_HAL_SERIAL)) //todo empty set to less than fifo size, currently 0, write 1 char only
	{
		if (p_serial->TxBufferHead == p_serial->TxBufferTail)  //Tx until software buffer is empty
		{
			HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			break;
		}
		else
		{
			HAL_Serial_WriteTxChar(p_serial->CONFIG.P_HAL_SERIAL, p_serial->CONFIG.P_TX_BUFFER[p_serial->TxBufferTail]);
			p_serial->TxBufferTail = (p_serial->TxBufferTail + 1U) % p_serial->CONFIG.TX_BUFFER_SIZE;
		}
	}
}

static inline void Serial_PollRestartRxIsr(const Serial_T * p_serial)
{
	if (HAL_Serial_ReadRxRegFull(p_serial->CONFIG.P_HAL_SERIAL) == true) //todo use threshhold, if full is > 0
	{
		HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	}
}

static inline uint32_t Serial_GetRxFull(Serial_T * p_serial)
{
	return (p_serial->CONFIG.RX_BUFFER_SIZE + p_serial->RxBufferHead - p_serial->RxBufferTail) % p_serial->CONFIG.RX_BUFFER_SIZE;
}

static inline uint32_t Serial_GetTxEmpty(Serial_T * p_serial)
{
	if (p_serial->TxBufferHead < p_serial->TxBufferTail)
	{
		return p_serial->TxBufferTail - p_serial->TxBufferHead - 1U;
	}
	else
	{
		return p_serial->CONFIG.TX_BUFFER_SIZE - p_serial->TxBufferHead + p_serial->TxBufferTail;
	}
}

static inline void Serial_EnableTx(const Serial_T * p_serial)
{
	HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
}

static inline void Serial_DisableTx(const Serial_T * p_serial)
{
	HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
}

static inline void Serial_EnableRx(const Serial_T * p_serial)
{
	HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
}

static inline void Serial_DisableRx(const Serial_T * p_serial)
{
	HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
}

extern bool Serial_SendChar(Serial_T * p_serial, uint8_t txChar);
extern bool Serial_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar);
extern uint32_t Serial_SendBuffer(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t bufferSize);
extern uint32_t Serial_RecvBuffer(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t bufferSize);
extern bool Serial_SendString(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t length);
extern bool Serial_RecvString(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t length);

extern bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t length);
extern uint32_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t length);

extern void Serial_SetBaudRate(Serial_T * p_serial, uint32_t baudRate);
extern void Serial_Init(Serial_T * p_serial);

#endif

