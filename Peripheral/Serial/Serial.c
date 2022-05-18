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
#include "HAL_Serial.h"

#include "Config.h"
#include "Utility/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

/*
	Single threaded buffer read/write only need disable channel ISR
	if interrupt occurs after checking sw buffer, it will run to completion,
	sw write occur in between hw read/write
*/
static inline void EnterCriticalSerialTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_DisableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void ExitCriticalSerialTx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void EnterCriticalSerialRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_DisableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

static inline void ExitCriticalSerialRx(Serial_T * p_serial)
{
#if defined(CONFIG_SERIAL_SINGLE_THREADED)
	HAL_Serial_EnableRxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
#endif
}

/*
	Multithread must enter critical section before checking sw buffer
*/

/*
	Always disable all interrupts for entire duration
*/
static inline void EnterCriticalGlobal(void)
{
#if defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL) || defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
	Critical_Enter();
#endif
}

static inline void ExitCriticalGlobal(void)
{
#if defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL) || defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
	Critical_Exit();
#endif
}

/*
	Flexible selection between mutex and critical
	UNTESTED
*/
static inline bool AcquireCriticalGlobalTx(Serial_T * p_serial)
{
#if		defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
	return Critical_AquireMutex(&p_serial->TxMutex);
#elif 	defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
	(void)p_serial;
	Critical_Enter(); 	return true;
#elif 	defined(CONFIG_SERIAL_SINGLE_THREADED)
	(void)p_serial;
	return true;
#endif
}

static inline void ReleaseCriticalGlobalTx(Serial_T * p_serial)
{
#if		defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
	Critical_ReleaseMutex(&p_serial->TxMutex);
#elif 	defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
	(void)p_serial;
	Critical_Exit();
#elif 	defined(CONFIG_SERIAL_SINGLE_THREADED)
	(void)p_serial;
#endif
}

static inline bool AcquireCriticalGlobalRx(Serial_T * p_serial)
{
#if		defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
	return Critical_AquireMutex(&p_serial->RxMutex);
#elif 	defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
	(void)p_serial;
	Critical_Enter(); 	return true;
#elif 	defined(CONFIG_SERIAL_SINGLE_THREADED)
	(void)p_serial;
	return true;
#endif
}

static inline void ReleaseCriticalGlobalRx(Serial_T * p_serial)
{
#if		defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
	Critical_ReleaseMutex(&p_serial->RxMutex);
#elif	defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
	(void)p_serial;
	Critical_Exit();
#elif 	defined(CONFIG_SERIAL_SINGLE_THREADED)
	(void)p_serial;
#endif
}

/*
	Tx need not disable interrupt after checking empty, hw buffer can only decrease.
*/
static inline bool Hw_SendChar(Serial_T * p_serial, const uint8_t txchar)
{
	bool isSuccess;
	if(HAL_Serial_ReadTxEmptyCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U)
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
	Rx must prevent interrupt after checking full, hw buffer can increase.
 */
static inline bool Hw_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess;
	if(HAL_Serial_ReadRxFullCount(p_serial->CONFIG.P_HAL_SERIAL) > 0U)
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

static inline uint8_t Hw_GetLoopCount(size_t length)
{
#ifdef CONFIG_SERIAL_HW_FIFO_DISABLE
	(void)length;
	return 1U;
#else
	return length;
#endif
}

static inline size_t Hw_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
	size_t charCount;

	for(charCount = 0U; charCount < Hw_GetLoopCount(length); charCount++)
	{
		if(Hw_SendChar(p_serial, p_srcBuffer[charCount]) == false)
		{
			break;
		}
	}

	return charCount;
}

static inline size_t Hw_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
	size_t charCount;

	for(charCount = 0U; charCount < Hw_GetLoopCount(length); charCount++)
	{
		if(Hw_RecvChar(p_serial, &p_destBuffer[charCount]) == false)
		{
			break;
		}
	}

	return charCount;
}

/******************************************************************************/
/*!
	Public
*/
/******************************************************************************/
void Serial_Init(Serial_T * p_serial)
{
	HAL_Serial_Init(p_serial->CONFIG.P_HAL_SERIAL);
	Queue_Init(&p_serial->TxQueue);
	Queue_Init(&p_serial->RxQueue);
	Serial_EnableRx(p_serial);
}

void Serial_Deinit(Serial_T * p_serial)
{
	HAL_Serial_Deinit(p_serial->CONFIG.P_HAL_SERIAL);
}

void Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate) //Serial_InitBaudRate
{
	HAL_Serial_ConfigBaudRate(p_serial->CONFIG.P_HAL_SERIAL, baudRate);
}

bool Serial_SendChar(Serial_T * p_serial, uint8_t txChar)
{
	bool isSuccess = false;

	EnterCriticalGlobal();
	//write directly to hw fifo/reg todo
	//	if (Queue_GetIsEmpty(&p_serial->TxQueue) == true) && HAL_Serial_GetIsActive == false
	//	{	//if Tx interrupt occurs here, no data will transfer from sw queue to hw buffer,
	//		isSuccess = Hw_SendChar(p_serial, txChar);
	//	}
	//	else
	//	{
	EnterCriticalSerialTx(p_serial);
	isSuccess = Queue_Enqueue(&p_serial->TxQueue, &txChar);
	if(isSuccess == true)
	{
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
	}
	ExitCriticalSerialTx(p_serial);
	//	}
	ExitCriticalGlobal();

	return isSuccess;
}

bool Serial_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
	bool isSuccess = false;

	EnterCriticalGlobal();
	//	if (Queue_GetIsEmpty(&p_serial->RxQueue) == true)
	//	{	//if rx interrupt occurs after checking software buffer, it will run to completion.
	//		EnterCriticalSerialRx(p_serial);
	//		isSuccess = Hw_RecvChar(p_serial, p_rxChar);
	//		ExitCriticalSerialRx(p_serial);
	//	}
	//	else
	//	{
	EnterCriticalSerialRx(p_serial);
	isSuccess = Queue_Dequeue(&p_serial->RxQueue, p_rxChar);
	ExitCriticalSerialRx(p_serial);
	//	}
	ExitCriticalGlobal();

	return isSuccess;
}

/*
	calling function must check to avoid meta data collision
*/
uint8_t Serial_GetChar(Serial_T * p_serial)
{
	uint8_t rxChar;

	if(Serial_RecvChar(p_serial, &rxChar) == false)
	{
		rxChar = 0xFFU;
	}

	return rxChar;
}


/*
	Send max available, if > than buffer size. May clip.
*/
uint32_t Serial_SendMax(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t srcSize)
{
	uint32_t charCount = 0U;

	if(AcquireCriticalGlobalTx(p_serial) == true)
	{
		//send immediate if fits in hardware fifo
		//		if (Queue_GetIsEmpty(&p_serial->TxQueue) == true)
		//		{
		//			charCount += Hw_Send(p_serial, p_srcBuffer, srcSize);
		//		}

		//		if (charCount < srcSize)
		//		{
		EnterCriticalSerialTx(p_serial);
		charCount += Queue_EnqueueMax(&p_serial->RxQueue, p_srcBuffer, srcSize - charCount);
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
		ExitCriticalSerialTx(p_serial);
		//		}
		ReleaseCriticalGlobalTx(p_serial);
	}

	return charCount;
}

uint32_t Serial_RecvMax(Serial_T * p_serial, uint8_t * p_destBuffer, size_t destSize)
{
	uint32_t charCount = 0U;

	if(AcquireCriticalGlobalRx(p_serial) == true)
	{
		//		if (Queue_GetIsEmpty(&p_serial->RxQueue) == true)
		//		{
		//			EnterCriticalSerialRx(p_serial);
		//			charCount += Hw_Recv(p_serial, p_destBuffer, destSize);
		//			ExitCriticalSerialRx(p_serial);
		//		}
		//		else
		//		{
		EnterCriticalSerialRx(p_serial);
		charCount += Queue_DequeueMax(&p_serial->RxQueue, p_destBuffer, destSize);
		ExitCriticalSerialRx(p_serial);
		//		}
		ReleaseCriticalGlobalRx(p_serial);
	}

	return charCount;
}

bool Serial_SendN(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
	bool status = false;

	if(AcquireCriticalGlobalTx(p_serial) == true)
	{
		if(Queue_GetEmptyCount(&p_serial->TxQueue) >= length)
		{
			EnterCriticalSerialTx(p_serial);
			Queue_EnqueueN(&p_serial->TxQueue, p_srcBuffer, length);
			HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
			ExitCriticalSerialTx(p_serial);
			status = true;
		}
		ReleaseCriticalGlobalTx(p_serial);
	}

	return status;
}

/*
	Rx only if length had been reached
*/
bool Serial_RecvN(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
	bool status = false;

	if(AcquireCriticalGlobalRx(p_serial) == true)
	{
		if(Queue_GetFullCount(&p_serial->RxQueue) >= length)
		{
			EnterCriticalSerialRx(p_serial);
			Queue_DequeueN(&p_serial->RxQueue, p_destBuffer, length);
			ExitCriticalSerialRx(p_serial);
			status = true;
		}
		ReleaseCriticalGlobalRx(p_serial);
	}

	return status;
}

bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
	return Serial_SendN(p_serial, p_srcBuffer, length);
}

uint32_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
	return Serial_RecvMax(p_serial, p_destBuffer, length);
}

/*
	Passes buffer to upper layer to avoid double buffer write.
	Must flush buffer to start at 0 index.
*/
uint8_t * Serial_AcquireTxBuffer(Serial_T * p_serial)
{
	uint8_t * p_buffer = 0U;

	if(AcquireCriticalGlobalTx(p_serial) == true)
	{
		EnterCriticalSerialTx(p_serial);
		p_buffer = Queue_AcquireBuffer(&p_serial->TxQueue);
	}

	return p_buffer;
}

void Serial_ReleaseTxBuffer(Serial_T * p_serial, size_t writeSize)
{
	Queue_ReleaseBuffer(&p_serial->TxQueue, writeSize);
	ExitCriticalSerialTx(p_serial);
	ReleaseCriticalGlobalTx(p_serial);
}

/* UNTESTED */
bool Serial_SendCharString(Serial_T * p_serial, const uint8_t * p_srcBuffer)
{
	bool status = false;

	const uint8_t * p_char = p_srcBuffer;

	if(AcquireCriticalGlobalTx(p_serial) == true)
	{
		EnterCriticalSerialTx(p_serial);
		while(*p_char != '\0')
		{
			status = Queue_Enqueue(&p_serial->TxQueue, p_char);
			if(status == false)
			{
				break;
			}
			p_char++;
		}
		HAL_Serial_EnableTxInterrupt(p_serial->CONFIG.P_HAL_SERIAL);
		ExitCriticalSerialTx(p_serial);

		ReleaseCriticalGlobalTx(p_serial);
	}

	return status;
}