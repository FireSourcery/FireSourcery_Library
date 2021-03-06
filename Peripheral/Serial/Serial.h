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

#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED)
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	extern inline void Critical_Enter(void *);
	extern inline void Critical_Exit(void *);
#else

#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	HAL_Serial_T * p_HAL_Serial;

	//	bool HasTxHarwareFIFO;
	//	bool HasRxHarwareFIFO;

	// software buffer
	uint8_t * p_TxBuffer;	/*!< */
	uint8_t * p_RxBuffer;	/*!< */
	uint32_t TxBufferSize;
	uint32_t RxBufferSize;

	volatile uint32_t TxBufferHead; //head is end of buffer queue,
	volatile uint32_t TxBufferTail;
	volatile uint32_t RxBufferHead;
	volatile uint32_t RxBufferTail;

#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	uint8_t TxSemaphore;
	uint8_t RxSemaphore;
#endif

//	volatile bool IsTxBusy;            /*!< True if there is an active transmit. */
//	volatile bool IsRxBusy;            /*!< True if there is an active receive. */
//
//	void (* RxCallback)(void *); 	/*!< Callback on Rx */
//	void * RxCallbackData; 			/*!< Receive callback parameter pointer.*/
//	void (* TxCallback)(void *); 	/*!< Callback on Tx */
//	void * TxCallbackData; 			/*!< Transmit callback parameter pointer.*/
//
//    Serial_TransferMode_T TransferMode;	 /*!< interrupt/dma mode */
//
//    bool RxComplete;
//    bool TxComplete;
//
//    volatile Serial_Status_T TxStatus;	/*!< Status of last driver transmit operation */
//    volatile Serial_Status_T RxStatus;	/*!< Status of last driver receive operation */

#if CONFIG_SERIAL_DMA_ENABLE
    uint8_t RxDMAChannel;                /*!< DMA channel number for DMA-based rx. */
    uint8_t TxDMAChannel;                /*!< DMA channel number for DMA-based tx. */
#endif
} Serial_T;

typedef const struct
{
	HAL_Serial_T * const P_HAL_SERIAL;
	//	bool HasTxHarwareFIFO;
	//	bool HasRxHarwareFIFO;
	uint8_t * const P_TX_BUFFER;
	uint8_t * const P_RX_BUFFER;
	const uint32_t TX_BUFFER_SIZE;
	const uint32_t RX_BUFFER_SIZE;
} Serial_Init_T;

//static inline bool IsTxBufferEmpty(Serial_T * p_serial) 	{return (p_serial->TxBufferHead == p_serial->TxBufferTail);};
//static inline bool IsRxBufferFull(Serial_T * p_serial) 	{return (p_serial->RxBufferHead == p_serial->RxBufferTail);};

static inline uint32_t Serial_GetAvailableRx(Serial_T * p_serial)
{
	return (p_serial->RxBufferSize + p_serial->RxBufferHead - p_serial->RxBufferTail) % p_serial->RxBufferSize;
}

static inline uint32_t Serial_GetAvailableTx(Serial_T * p_serial)
{
	if (p_serial->TxBufferHead >= p_serial->TxBufferTail)
	{
		return p_serial->TxBufferSize - 1 - p_serial->TxBufferHead + p_serial->TxBufferTail;
	}
	else
	{
		return p_serial->TxBufferTail - p_serial->TxBufferHead - 1;
	}
}

static inline void Serial_EnableTx(Serial_T * p_serial)
{
	HAL_Serial_EnableTxInterrupt(p_serial->p_HAL_Serial);
}

static inline void Serial_DisableTx(Serial_T * p_serial)
{
	HAL_Serial_DisableTxInterrupt(p_serial->p_HAL_Serial);
}

static inline void Serial_EnableRx(Serial_T * p_serial)
{
	HAL_Serial_EnableRxInterrupt(p_serial->p_HAL_Serial);
}

static inline void Serial_DisableRx(Serial_T * p_serial)
{
	HAL_Serial_DisableRxInterrupt(p_serial->p_HAL_Serial);
}

extern uint8_t Serial_ReadChar(Serial_T * p_serial);
extern bool Serial_ReadString(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t length);
extern bool Serial_WriteChar(Serial_T * p_serial, uint8_t txChar);
extern bool Serial_WriteString(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t length);
extern void Serial_SetBaudRate(Serial_T * p_serial, uint32_t baudRate);

extern void Serial_Init
(
	Serial_T * p_serial,
	Serial_Init_T * p_serialInit
);

#endif

//typedef union
//{
//	uint8_t Byte;
//	struct
//	{
//		bool OverRun	:1;
//		bool Parity		:1;
//	};
//} Serial_Error_T;
