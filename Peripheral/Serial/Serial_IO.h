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
	@file 	Serial_IO.h
	@author FireSoucery
	@brief 	module functions must be placed into corresponding user app threads
	@version V0
*/
/******************************************************************************/
#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include "Serial.h"
#include "HAL_Serial.h"

#include <stdint.h>
#include <stdbool.h>

//Rx data reg full ISR, enqueue data, head is end of queue
static inline void Serial_RxData_IO(Serial_T *p_serial)
{
//	uint32_t rxBufferHeadNext = (p_serial->RxBufferHead + 1) % p_serial->RxBufferSize;

//	if (rxBufferHeadNext == p_serial->RxBufferTail) return;
//
//	p_serial->HAL_Serial_ReadChar(&p_serial->p_RxBuffer[p_serial->RxBufferHead]);
//	p_serial->RxBufferHead = rxBufferHeadNext;

	uint32_t rxBufferHeadNext;

	while (HAL_Serial_ReadRxRegFull(p_serial->p_HAL_Serial))
	{
		rxBufferHeadNext = (p_serial->RxBufferHead + 1) % p_serial->RxBufferSize;

		if (rxBufferHeadNext == p_serial->RxBufferTail)
		{
			//if buffer stays full, need to disable irq to prevent blocking main loop
			//user must restart rx irq
			HAL_Serial_DisableRxInterrupt(p_serial->p_HAL_Serial);
			break;
		}
		else
		{
			p_serial->p_RxBuffer[p_serial->RxBufferHead] = HAL_Serial_ReadChar(p_serial->p_HAL_Serial);
			p_serial->RxBufferHead = rxBufferHeadNext;
		}
	}
}

//Tx data reg empty ISR
static inline void Serial_TxData_IO(Serial_T * p_serial)
{
//	p_serial->HAL_Serial_WriteChar(p_serial->p_TxBuffer[p_serial->TxBufferTail]);
//	p_serial->TxBufferTail = (p_serial->TxBufferTail + 1) % p_serial->TxBufferSize;
//
//	if (p_serial->TxBufferHead == p_serial->TxBufferTail) p_serial->HAL_Serial_DisableTxInterrupt();

	while(HAL_Serial_ReadTxDataRegEmpty(p_serial->p_HAL_Serial))
	{
		HAL_Serial_WriteChar(p_serial->p_HAL_Serial, p_serial->p_TxBuffer[p_serial->TxBufferTail]);
		p_serial->TxBufferTail = (p_serial->TxBufferTail + 1) % p_serial->TxBufferSize;

		if (p_serial->TxBufferHead == p_serial->TxBufferTail)
		{
			HAL_Serial_DisableTxInterrupt(p_serial->p_HAL_Serial);
			break;
		}
	}
}

static inline void Serial_RestartRx_IO(Serial_T *p_serial)
{
	Serial_EnableRx(p_serial);
}

static inline void  Serial_TxComplete_IO(Serial_T * p_serial)
{

}

static inline void  Serial_Error_IO(Serial_T * p_serial)
{
  //HAL_Serial_GetError( );
}


#endif /* SERIAL_IO_H */
