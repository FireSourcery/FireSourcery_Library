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
	@file 	DataLink.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "DataLink.h"
#include "Config.h"

#include <stdint.h>
#include <stdbool.h>


void DataLink_Init(DataLink_T * p_dataLink)
{

//
//	DataLink_EnableRx(p_dataLink);
}


bool DataLink_SendChar(DataLink_T * p_dataLink, uint8_t txChar)
{
	bool isSuccess = false;

	switch(p_dataLink->Type)
	{
		case DATA_LINK_SERIAL:	isSuccess = Serial_SendChar(p_dataLink->p_Serial, txChar);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		 I2C_RecvChar(p_dataLink->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		 CanBus_RecvChar(p_dataLink->p_Serial);		break;
		default: break;
	}

	return isSuccess;
}

bool DataLink_RecvChar(DataLink_T * p_dataLink, uint8_t * p_rxChar)
{
	bool isSuccess = false;

	switch(p_dataLink->Type)
	{
		case DATA_LINK_SERIAL:	isSuccess = Serial_RecvChar(p_dataLink->p_Serial, p_rxChar);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		isSuccess = I2C_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		isSuccess = CanBus_RecvChar(p_protocol->p_Serial);		break;
		default: break;
	}

	return isSuccess;
}


//send immediate if fit in hardware fifo
uint32_t DataLink_SendBytes(DataLink_T * p_dataLink, const uint8_t * p_srcBuffer, size_t srcSize)
{
	uint32_t charCount;



	return charCount;
}

uint32_t DataLink_RecvBytes(DataLink_T * p_dataLink, uint8_t * p_destBuffer, size_t destSize)
{
	uint32_t charCount;


	return charCount;
}

bool DataLink_SendString(DataLink_T * p_dataLink, const uint8_t * p_src, size_t length)
{
	bool isSuccess = false;

	switch(p_dataLink->Type)
	{
		case DATA_LINK_SERIAL:	isSuccess = Serial_SendString(p_dataLink->p_Serial, p_src, length);		break;
//		case DATA_LINK_I2C:		 I2C_RecvChar(p_dataLink->p_Serial);		break;
//		case DATA_LINK_CAN:		 CanBus_RecvChar(p_dataLink->p_Serial);		break;
		default: break;
	}

	return isSuccess;

}

bool DataLink_RecvString(DataLink_T * p_dataLink, uint8_t * p_destBuffer, size_t length)
{
	bool status = false;

	return status;
}


bool DataLink_Send(DataLink_T * p_dataLink, const uint8_t * p_srcBuffer, size_t length)
{
	return DataLink_SendString(p_dataLink, p_srcBuffer, length);
}

uint32_t DataLink_Recv(DataLink_T * p_dataLink, uint8_t * p_destBuffer, size_t length)
{
	return DataLink_RecvBytes(p_dataLink, p_destBuffer, length);
}
//
//void DataLink_ConfigBaudRate(DataLink_T * p_dataLink, uint32_t baudRate)
//{
//	HAL_DataLink_ConfigBaudRate(p_dataLink->CONFIG.P_HAL_SERIAL, baudRate);
//}


static inline void DataLink_Poll(const DataLink_T * p_dataLink)
{
//	DataLink_PollRestartRxIsr(p_Serial)

}

static inline uint32_t DataLink_GetRxFullCount(DataLink_T * p_dataLink)
{

}

static inline uint32_t DataLink_GetTxEmptyCount(DataLink_T * p_dataLink)
{

}


static inline void DataLink_EnableTx(const DataLink_T * p_dataLink)
{

}

static inline void DataLink_DisableTx(const DataLink_T * p_dataLink)
{

}

static inline void DataLink_EnableRx(const DataLink_T * p_dataLink)
{

}

static inline void DataLink_DisableRx(const DataLink_T * p_dataLink)
{

}
