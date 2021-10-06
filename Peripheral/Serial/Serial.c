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
	@file 	Serial.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Serial.h"

#include "Config.h"

#include "System/Queue/Queue.h"

#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED)
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	extern inline void Critical_Enter(void * p_mutex);
	extern inline void Critical_Exit(void * p_mutex);
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * Single threaded buffer read/write only need disable channel ISR
 * if interrupt occurs after checking sw buffer, it will run to completion,
 * sw write occur in between hw read/write
 */
static inline void EnterCriticalTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void ExitCriticalTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void EnterCriticalRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void ExitCriticalRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

/*
 * Multithread must lock before checking sw buffer
 */
static inline bool AquireMutexTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	return Critical_AquireMutex(&p_serial->TxMutex);
#else
	return true;
#endif
}

static inline void ReleaseMutexTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	Critical_ReleaseMutex(&p_serial->TxMutex);
#endif
}

static inline bool AquireMutexRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	return Critical_AquireMutex(&p_serial->RxMutex);
#else
	return true;
#endif
}

static inline void ReleaseMutexRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_MULTITHREADED_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_MULTITHREADED_USER_DEFINED)
	Critical_ReleaseMutex(&p_serial->RxMutex);
#endif
}

static inline bool Hw_SendChar(Serial_T * p_serial, uint8_t txChar)
{
	bool isSuccess;

	if (HAL_Serial_ReadTxRegEmpty(p_serial->CONFIG.P_HAL_SERIAL) == true)
	{
		HAL_Serial_WriteTxChar(p_serial->CONFIG.P_HAL_SERIAL, txChar);
		isSuccess = true;
	}
	else
	{
		isSuccess = false;
	}

	return isSuccess;
}

static inline bool Hw_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess;

	if (HAL_Serial_ReadRxRegFull(p_serial->CONFIG.P_HAL_SERIAL) == true)
	{
		*p_rxChar = HAL_Serial_ReadRxChar(p_serial->CONFIG.P_HAL_SERIAL);
		isSuccess = true;
	}
	else
	{
		isSuccess = false;
	}

	return isSuccess;
}

static uint32_t Hw_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t length)
{
#ifdef CONFIG_SERIAL_NO_HW_FIFO
	return (Hw_SendChar(p_serial, p_srcBuffer[charCount]) == true) ? 1U : 0U;
#else
	uint32_t charCount;

	for (charCount = 0; charCount < length; charCount++)
	{
		if (Hw_SendChar(p_serial, p_srcBuffer[charCount]) == false)
		{
			break;
		}
	}

	return charCount;
#endif
}

static uint32_t Hw_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t length)
{
#ifdef CONFIG_SERIAL_NO_HW_FIFO
	return (Hw_RecvChar(p_serial, p_destBuffer[charCount]) == true) ? 1U : 0U;
#else
	uint32_t charCount;

	for (charCount = 0; charCount < length; charCount++)
	{
		if (Hw_RecvChar(p_serial, p_destBuffer[charCount]) == false)
		{
			break;
		}
	}

	return charCount;
#endif
}


//bool Serial_SendChar(Serial_T * p_serial, uint8_t txChar)
//{
//	bool isSuccess;
//
//	if (AquireMutexTx(p_serial))
//	{
//		if (Queue_GetIsEmpty(&p_serial->TxQueue))
//		{
//			isSuccess = Hw_SendChar(p_serial, txChar);
//		}
//		else
//		{
//			EnterCriticalTx(p_serial);
//
//			isSuccess = Queue_Enqueue(&p_serial->TxQueue, txChar);
//
//			if (isSuccess == true)
//			{
//				HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
//			}
//
//			ExitCriticalTx(p_serial);
//		}
//
//		ReleaseMutexTx(p_serial);
//	}
//
//	return isSuccess;
//}


void Serial_Init(Serial_T * p_serial)
{
	p_serial->TxBufferHead 	= 0;
	p_serial->TxBufferTail 	= 0;
	p_serial->RxBufferHead 	= 0;
	p_serial->RxBufferTail 	= 0;

	HAL_Serial_Init(p_serial->CONFIG.P_HAL_SERIAL);
	Serial_EnableRx(p_serial);
}

bool Serial_SendChar(Serial_T * p_serial, uint8_t txChar)
{
	uint32_t txBufferHeadNext;
	bool isSuccess;

	AquireMutexTx(p_serial);
	if (p_serial->TxBufferHead == p_serial->TxBufferTail)
	{
		isSuccess = Hw_SendChar(p_serial, txChar);
	}
	else
	{
		EnterCriticalTx(p_serial);
		txBufferHeadNext = (p_serial->TxBufferHead + 1U) % p_serial->CONFIG.TX_BUFFER_SIZE;

		if (txBufferHeadNext == p_serial->TxBufferTail)
		{
			isSuccess = false;
		}
		else
		{
			p_serial->CONFIG.P_TX_BUFFER[p_serial->TxBufferHead] = txChar;
			p_serial->TxBufferHead = txBufferHeadNext;
			HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			isSuccess = true;
		}
		ExitCriticalTx(p_serial);
	}
	ReleaseMutexTx(p_serial);

	return isSuccess;
}

bool Serial_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess;

	if(p_serial->RxBufferHead == p_serial->RxBufferTail)
	{
		isSuccess = Hw_RecvChar(p_serial, p_rxChar);
	}
	else
	{
		EnterCriticalRx(p_serial);
		*p_rxChar = p_serial->CONFIG.P_RX_BUFFER[p_serial->RxBufferTail];
		p_serial->RxBufferTail = (p_serial->RxBufferTail + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE;
		isSuccess = true;
		ExitCriticalRx(p_serial);
	}

	return isSuccess;
}

//send immediate if fit in hardware fifo
uint32_t Serial_SendBytes(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t bufferSize)
{
	uint32_t charCount;
	uint32_t txBufferHeadNext;

//	if (p_serial->RxBufferHead == p_serial->RxBufferTail) && bufferSize < get hw empty
//	{
//		Hw_Send(p_serial, p_srcBuffer, bufferSize);
//	}
//	else
	{
		EnterCriticalTx(p_serial);
		for (charCount = 0; charCount < bufferSize; charCount++)
		{
			txBufferHeadNext = (p_serial->TxBufferHead + 1U) % p_serial->CONFIG.TX_BUFFER_SIZE;
			if (txBufferHeadNext == p_serial->TxBufferTail)
			{
				break;
			}
			else
			{
				p_serial->CONFIG.P_TX_BUFFER[p_serial->TxBufferHead] = p_srcBuffer[charCount];
				p_serial->TxBufferHead = txBufferHeadNext;
			}
		}
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
		ExitCriticalTx(p_serial);
	}

	return charCount;
}

uint32_t Serial_RecvBytes(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t bufferSize)
{
	uint32_t charCount;

	if (p_serial->RxBufferHead == p_serial->RxBufferTail)
	{
		charCount = Hw_Recv(p_serial, p_destBuffer, bufferSize);
	}
	else
	{
		EnterCriticalRx(p_serial);
		for (charCount = 0U; charCount < bufferSize; charCount++)
		{
			if (p_serial->RxBufferHead == p_serial->RxBufferTail)
			{
				break;
			}
			else
			{
				p_destBuffer[charCount] = p_serial->CONFIG.P_RX_BUFFER[p_serial->RxBufferTail];
				p_serial->RxBufferTail = (p_serial->RxBufferTail + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE;
			}
		}
		ExitCriticalRx(p_serial);
	}

	return charCount;
}

bool Serial_SendString(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t length)
{
	bool status;

	if (Serial_GetTxEmpty(p_serial) < length)
	{
		status = false;
	}
	else
	{
		EnterCriticalTx(p_serial);
		for (uint32_t srcBufferIndex = 0U; srcBufferIndex < length; srcBufferIndex++)
		{
			p_serial->CONFIG.P_TX_BUFFER[p_serial->TxBufferHead] = p_srcBuffer[srcBufferIndex];
			p_serial->TxBufferHead = (p_serial->TxBufferHead + 1U) % p_serial->CONFIG.TX_BUFFER_SIZE;
		}
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
		ExitCriticalTx(p_serial);

		status = true;
	}

	return status;
}

bool Serial_RecvString(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t length)
{
	bool status;

	if (Serial_GetRxFull(p_serial) < length)
	{
		status = false;
	}
	else
	{
		EnterCriticalRx(p_serial);
		for (uint32_t destBufferIndex = 0U; destBufferIndex < length; destBufferIndex++)
		{
			p_destBuffer[destBufferIndex] = p_serial->CONFIG.P_RX_BUFFER[p_serial->RxBufferTail];
			p_serial->RxBufferTail = (p_serial->RxBufferTail + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE;
		}
		ExitCriticalRx(p_serial);

		status = true;
	}

	return status;
}

bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, uint32_t length)
{
	return Serial_SendString(p_serial, p_srcBuffer, length);
}

uint32_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, uint32_t length)
{
	return Serial_RecvBytes(p_serial, p_destBuffer, length);
}

void Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate)
{
	HAL_Serial_ConfigBaudRate(p_serial->CONFIG.P_HAL_SERIAL, baudRate);
}

