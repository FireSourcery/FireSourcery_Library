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

#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED)
	#include "System/Critical/Critical.h"
#elif defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	extern inline void Critical_Enter(void);
	extern inline void Critical_Exit(void);
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * Single threaded buffer read/write only need disable channel ISR
 * if interrupt occurs after checking sw buffer, it will run to completion,
 * sw write occur in between hw read/write
 */
static inline void EnterCriticalLocalTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void ExitCriticalLocalTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void EnterCriticalLocalRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void ExitCriticalLocalRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void EnterCriticalGlobal(void)
{
#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	Critical_Enter();
#endif
}

static inline void ExitCriticalGlobal(void)
{
#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	Critical_Exit();
#endif
}

/*
 * Multithread must enter critical section before checking sw buffer
 */
static inline bool EnterCriticalCommonTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	#if  defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
		return Critical_AquireMutex(&p_serial->TxMutex);
	#elif defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
		EnterCriticalGlobal(); 	return true;
	#endif
#else
	return true;
#endif
}

static inline void ExitCriticalCommonTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	#if  defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
		Critical_ReleaseMutex(&p_serial->TxMutex);
	#elif defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
		ExitCriticalGlobal();
	#endif
#endif
}

static inline bool EnterCriticalCommonRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	#if  defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
		return Critical_AquireMutex(&p_serial->RxMutex);
	#elif defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
		EnterCriticalGlobal(); 	return true;
	#endif
#else
	return true;
#endif
}

static inline void ExitCriticalCommonRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_CRITICAL_LIBRARY_DEFINED) || defined(CONFIG_SERIAL_CRITICAL_USER_DEFINED)
	#if  defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
		Critical_ReleaseMutex(&p_serial->RxMutex);
	#elif defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
		ExitCriticalGlobal();
	#endif
#endif
}

static inline bool Hw_SendChar(Serial_T * p_serial, const uint8_t txchar)
{
	bool isSuccess;
	if (HAL_Serial_ReadTxEmptyCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U)
	{
		HAL_Serial_WriteTxChar(p_serial->CONFIG.P_HAL_SERIAL, txchar);
		isSuccess = true;
	}
	else
	{
		isSuccess = false;
	}
	return isSuccess;
}

/*
	Tx need not disable interrupt after checking empty, hw buffer can only decrease.
*/
static inline size_t Hw_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
#ifdef CONFIG_SERIAL_NO_HW_FIFO
	(void)length
	return (Hw_SendChar(p_serial, p_srcBuffer[charCount]) == false) ? 1U : 0U;
#else
	size_t charCount;

	for (charCount = 0U; charCount < length; charCount++)
	{
		if (Hw_SendChar(p_serial, p_srcBuffer[charCount]) == false)
		{
			break;
		}
	}

	return charCount;
#endif
}

/*
 * Rx must prevent interrupt after checking full, hw buffer can increase.
 */
static inline bool Hw_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess;
	if (HAL_Serial_ReadRxFullCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U)
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

static inline size_t Hw_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
#ifdef CONFIG_SERIAL_NO_HW_FIFO
	(void)length
	return (Hw_RecvChar(p_serial, &p_destBuffer[charCount]) == false) ? 1U : 0U;
#else
	size_t charCount;

	for (charCount = 0U; charCount < length; charCount++)
	{
		if (Hw_RecvChar(p_serial, &p_destBuffer[charCount]) == false)
		{
			break;
		}
	}

	return charCount;
#endif
}

#ifdef CONFIG_SERIAL_QUEUE_LIBRARY
void Serial_Init(Serial_T * p_serial)
{
	HAL_Serial_Init(p_serial->CONFIG.P_HAL_SERIAL);
	Queue_Init(&p_serial->TxQueue);
	Queue_Init(&p_serial->RxQueue);
	Serial_EnableRx(p_serial);
}

bool Serial_SendChar(Serial_T * p_serial, uint8_t txChar)
{
	bool isSuccess = false;

	if (EnterCriticalCommonTx(p_serial) == true)
	{
		if (Queue_GetIsEmpty(&p_serial->TxQueue) == true)
		{	//if Tx interrupt occurs here, no data will transfer from sw queue to hw buffer,
			isSuccess = Hw_SendChar(p_serial, txChar);
		}
		else
		{
			EnterCriticalLocalTx(p_serial);
			isSuccess = Queue_Enqueue(&p_serial->TxQueue, &txChar);
			if (isSuccess == true)
			{
				HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			}
			ExitCriticalLocalTx(p_serial);
		}
		ExitCriticalCommonTx(p_serial);
	}

	return isSuccess;
}

bool Serial_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess = false;

	if (EnterCriticalCommonRx(p_serial) == true)
	{
		if (Queue_GetIsEmpty(&p_serial->RxQueue) == true)
		{ //if rx interrupt occurs after checking software buffer, it will run to completion.
			EnterCriticalLocalRx(p_serial);
			isSuccess = Hw_RecvChar(p_serial, p_rxChar);
			ExitCriticalLocalRx(p_serial);
		}
		else
		{
			EnterCriticalLocalRx(p_serial);
			isSuccess = Queue_Dequeue(&p_serial->RxQueue, p_rxChar);
			ExitCriticalLocalRx(p_serial);
		}
		ExitCriticalCommonRx(p_serial);
	}

	return isSuccess;
}

//send immediate if fits in hardware fifo
uint32_t Serial_SendBytes(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t srcSize)
{
	uint32_t charCount = 0U;

	if (EnterCriticalCommonTx(p_serial) == true)
	{
		if (Queue_GetIsEmpty(&p_serial->TxQueue) == true)
		{
			charCount += Hw_Send(p_serial, p_srcBuffer, srcSize);
		}

		if (charCount < srcSize)
		{
			EnterCriticalLocalTx(p_serial);
			charCount += Queue_EnqueueMax(&p_serial->RxQueue, p_srcBuffer, srcSize - charCount);
			HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			ExitCriticalLocalTx(p_serial);
		}
		ExitCriticalCommonTx(p_serial);
	}

	return charCount;
}

uint32_t Serial_RecvBytes(Serial_T * p_serial, uint8_t * p_destBuffer, size_t destSize)
{
	uint32_t charCount = 0U;

	if (EnterCriticalCommonRx(p_serial) == true)
	{
		if (Queue_GetIsEmpty(&p_serial->RxQueue) == true)
		{
			EnterCriticalLocalRx(p_serial);
			charCount += Hw_Recv(p_serial, p_destBuffer, destSize);
			ExitCriticalLocalRx(p_serial);
		}
		else
		{
			EnterCriticalLocalRx(p_serial);
			charCount += Queue_DequeueMax(&p_serial->RxQueue, p_destBuffer, destSize);
			ExitCriticalLocalRx(p_serial);
		}
		ExitCriticalCommonRx(p_serial);
	}

	return charCount;
}

bool Serial_SendString(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
	bool status = false;

	if (EnterCriticalCommonTx(p_serial) == true)
	{
		if (Queue_GetEmptyCount(&p_serial->TxQueue) >= length)
		{
			EnterCriticalLocalTx(p_serial);
			Queue_EnqueueN(&p_serial->TxQueue, p_srcBuffer, length);
			HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			ExitCriticalLocalTx(p_serial);
			status = true;
		}
	}

	return status;
}

bool Serial_RecvString(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
	bool status = false;

	if (EnterCriticalCommonRx(p_serial) == true)
	{
		if (Queue_GetFullCount(&p_serial->RxQueue) >= length)
		{
			EnterCriticalLocalRx(p_serial);
			Queue_DequeueN(&p_serial->RxQueue, p_destBuffer, length);
			ExitCriticalLocalRx(p_serial);
			status = true;
		}
	}

	return status;
}

#elif defined(CONFIG_SERIAL_QUEUE_INTERNAL)
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

	if (EnterCriticalCommonTx(p_serial))
	{
		if (p_serial->TxBufferHead == p_serial->TxBufferTail)
		{
			isSuccess = Hw_SendChar(p_serial, txChar);
		}
		else
		{
			EnterCriticalLocalTx(p_serial);
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
			ExitCriticalLocalTx(p_serial);
		}
		ExitCriticalCommonTx(p_serial);
	}

	return isSuccess;
}

bool Serial_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess;

	if (p_serial->RxBufferHead == p_serial->RxBufferTail)
	{
		isSuccess = Hw_RecvChar(p_serial, p_rxChar);
	}
	else
	{
		EnterCriticalLocalRx(p_serial);
		*p_rxChar = p_serial->CONFIG.P_RX_BUFFER[p_serial->RxBufferTail];
		p_serial->RxBufferTail = (p_serial->RxBufferTail + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE;
		isSuccess = true;
		ExitCriticalLocalRx(p_serial);
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
		EnterCriticalLocalTx(p_serial);
		for (charCount = 0U; charCount < bufferSize; charCount++)
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
		ExitCriticalLocalTx(p_serial);
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
		EnterCriticalLocalRx(p_serial);
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
		ExitCriticalLocalRx(p_serial);
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
		EnterCriticalLocalTx(p_serial);
		for (uint32_t srcBufferIndex = 0U; srcBufferIndex < length; srcBufferIndex++)
		{
			p_serial->CONFIG.P_TX_BUFFER[p_serial->TxBufferHead] = p_srcBuffer[srcBufferIndex];
			p_serial->TxBufferHead = (p_serial->TxBufferHead + 1U) % p_serial->CONFIG.TX_BUFFER_SIZE;
		}
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
		ExitCriticalLocalTx(p_serial);

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
		EnterCriticalLocalRx(p_serial);
		for (uint32_t destBufferIndex = 0U; destBufferIndex < length; destBufferIndex++)
		{
			p_destBuffer[destBufferIndex] = p_serial->CONFIG.P_RX_BUFFER[p_serial->RxBufferTail];
			p_serial->RxBufferTail = (p_serial->RxBufferTail + 1U) % p_serial->CONFIG.RX_BUFFER_SIZE;
		}
		ExitCriticalLocalRx(p_serial);

		status = true;
	}

	return status;
}
#endif

bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
	return Serial_SendString(p_serial, p_srcBuffer, length);
}

uint32_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
	return Serial_RecvBytes(p_serial, p_destBuffer, length);
}

void Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate)
{
	HAL_Serial_ConfigBaudRate(p_serial->CONFIG.P_HAL_SERIAL, baudRate);
}
