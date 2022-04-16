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
	@file 	HAL_Transceiver.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef XCVR_H
#define XCVR_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum
{
	XCVR_TYPE_SERIAL,
	XCVR_TYPE_I2C,
	XCVR_TYPE_SPI,
}
Xcvr_Type_T;

typedef const struct
{
	Xcvr_Type_T Type;
	void 		* p_Context;
	void 		(* Init)		(void * p_context);
//	bool 		(* SendChar)	(void * p_context, uint8_t txChar);
//	bool 		(* RecvChar)	(void * p_context, uint8_t * p_rxChar);
//	uint32_t 	(* SendBytes)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t bufferSize);
//	uint32_t 	(* RecvBytes)	(void * p_context, uint8_t * p_destBuffer, 			size_t bufferSize);
//	bool 		(* SendString)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
//	bool 		(* RecvString)	(void * p_context, uint8_t * p_destBuffer, 			size_t length);

	uint32_t 	(* Recv) (void * p_context, uint8_t * p_destBuffer, 		size_t bufferSize);
	bool 		(* Send) (void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
}
Transceiver_Interface_T;

typedef const struct
{
	//todo table
//	Transceiver_Interface_T Interfaces[];
	void ** P_XCVR_TABLE;
	uint8_t XCVR_COUNT;

}
Transceiver_Config_T;

typedef const struct
{
}
Transceiver_Table_T;

/*
 *
 */
typedef const struct
{
	Transceiver_Config_T CONFIG;
//	void * p_Struct;
//	Xcvr_Type_T Type;
}
Transceiver_T;

//bool Transceiver_Validate(const Transceiver_T * p_xcvr, void * target)
//{
//	for (uint8_t iXcvr = 0; iXcvr < p_xcvr->CONFIG.XCVR_COUNT; iXcvr++)
//	{
//		if (target == p_xcvr->CONFIG.P_XCVR_TABLE[iXcvr])
//		{
//			p_xcvr->p_Struct = target;
//		}
//	}
//}

#define XCVR_CONFIG(p_context, Init, Send, Recv)		\
{													\
	.p_Context	= p_context							\
}

//void Transceiver_Init(const Transceiver_T * p_xcvr)	{p_xcvr->Init(p_xcvr->p_Context);}
//bool Transceiver_SendChar(const Transceiver_T * p_xcvr, uint8_t txChar) 		{return p_xcvr->SendChar(p_xcvr->p_Context, txChar);}
//bool Transceiver_RecvChar(const Transceiver_T * p_xcvr, uint8_t * p_rxChar)	{return p_xcvr->RecvChar(p_xcvr->p_Context, p_rxChar);}
//uint32_t Transceiver_SendBytes(const Transceiver_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize)	{p_xcvr->SendBytes(p_xcvr->p_Context, p_srcBuffer, srcSize);}
//uint32_t Transceiver_RecvBytes(const Transceiver_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)		{p_xcvr->RecvBytes(p_xcvr->p_Context, p_destBuffer, destSize);}
//
//bool Transceiver_SendString(const Transceiver_T * p_xcvr, const uint8_t * p_src, size_t length)				{p_xcvr->SendString(p_xcvr->p_Context, p_src, length);}
//bool Transceiver_RecvString(const Transceiver_T * p_xcvr, uint8_t * p_dest, size_t length)

//bool Transceiver_Send(const Transceiver_T * p_xcvr, const uint8_t * p_srcBuffer, size_t length)
//{
//	switch(p_xcvr->Type)
//	{
//		case XCVR_TYPE_CAN:
//			CanMessage_Set();
//			CanBus_SendMessage();
//	}
//
//	return Transceiver_SendString(p_xcvr, p_srcBuffer, length);
//}


uint32_t Transceiver_Recv(const Transceiver_T * p_xcvr, uint8_t * p_destBuffer, size_t length, uint16_t param)
{
	uint32_t rxSize;

//	switch(p_xcvr->Type)
//	{
//		case XCVR_TYPE_CAN:
////			rxSize = (CanBus_PollRecvMessage(p_xcvr, p_destBuffer, length, param) == true) ? CanBus_GetRxMessageLength(p_xcvr, param) : 0U;
//			break;
//		case XCVR_TYPE_SERIAL:
//			rxSize = Serial_RecvBytes(p_xcvr, p_destBuffer, length);
//			break;
//		default: break;
//	}

	return rxSize;
}



//void Transceiver_ConfigBaudRate(const Transceiver_T * p_xcvr, uint32_t baudRate)
//{
//	HAL_Transceiver_ConfigBaudRate(p_xcvr->CONFIG.P_HAL_SERIAL, baudRate);
//}

static inline void Transceiver_Poll(const Transceiver_T * p_xcvr)
{
//	Transceiver_PollRestartRxIsr(p_Serial)

}

static inline uint32_t Transceiver_GetRxFullCount(const Transceiver_T * p_xcvr)
{

}

static inline uint32_t Transceiver_GetTxEmptyCount(const Transceiver_T * p_xcvr)
{

}

static inline void Transceiver_EnableTx(const Transceiver_T * p_xcvr){}
static inline void Transceiver_DisableTx(const Transceiver_T * p_xcvr){}
static inline void Transceiver_EnableRx(const Transceiver_T * p_xcvr){}
static inline void Transceiver_DisableRx(const Transceiver_T * p_xcvr){}

#endif

