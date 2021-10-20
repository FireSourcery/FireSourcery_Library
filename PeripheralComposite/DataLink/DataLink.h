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
	@file 	HAL_DataLink.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef DATA_LINK_H
#define DATA_LINK_H

#include "Config.h"

#include "Peripheral/Serial/Serial.h"
#include "System/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{

}
DataLink_Config_T;

typedef enum
{
	DATA_LINK_SERIAL,
	DATA_LINK_I2C,
	DATA_LINK_SPI,
	DATA_LINK_CAN,
}
DataLink_Type_T;


/*
 *
 */
typedef struct
{
	union
	{
		void * p_DataLink;
		Serial_T * p_Serial;
	};
	DataLink_Type_T Type;
}
DataLink_T;

#define DATA_LINK_CONFIG(p_DataLink, Type)		\
{												\
												\
}





extern bool DataLink_SendChar(DataLink_T * p_dataLink, uint8_t txChar);
extern bool DataLink_RecvChar(DataLink_T * p_dataLink, uint8_t * p_rxChar);
extern uint32_t DataLink_SendBytes(DataLink_T * p_dataLink, const uint8_t * p_srcBuffer, size_t bufferSize);
extern uint32_t DataLink_RecvBytes(DataLink_T * p_dataLink, uint8_t * p_destBuffer, size_t bufferSize);
extern bool DataLink_SendString(DataLink_T * p_dataLink, const uint8_t * p_srcBuffer, size_t length);
extern bool DataLink_RecvString(DataLink_T * p_dataLink, uint8_t * p_destBuffer, size_t length);

extern bool DataLink_Send(DataLink_T * p_dataLink, const uint8_t * p_srcBuffer, size_t length);
extern uint32_t DataLink_Recv(DataLink_T * p_dataLink, uint8_t * p_destBuffer, size_t length);

//extern void DataLink_ConfigBaudRate(DataLink_T * p_dataLink, uint32_t baudRate);
extern void DataLink_Init(DataLink_T * p_dataLink);

#endif

