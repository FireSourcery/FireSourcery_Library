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
#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include "Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


typedef const struct
{
	void * p_Context;
	void 		(* Init)		(void * p_context);
	bool 		(* SendChar)	(void * p_context, uint8_t txChar);
	bool 		(* RecvChar)	(void * p_context, uint8_t * p_rxChar);
	uint32_t 	(* SendBytes)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t bufferSize);
	uint32_t 	(* RecvBytes)	(void * p_context, uint8_t * p_destBuffer, 			size_t bufferSize);
	bool 		(* SendString)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
	bool 		(* RecvString)	(void * p_context, uint8_t * p_destBuffer, 			size_t length);


//	bool (*HAL_CanBus_GetRxBufferFullFlag)(void);
//	bool (*HAL_CanBus_GetTxBufferEmptyFlag)(void);
//	void (*HAL_CanBus_EnableTxBufferEmptyInterrupt)(void);
//	void (*HAL_CanBus_EnableRxBufferFullInterrupt)(void);
//	void (*HAL_CanBus_DisableTxBufferEmptyInterrupt)(void);
//	void (*HAL_CanBus_DisableRxBufferFullInterrupt)(void);
//	void (*HAL_CanBus_ClearRxBufferFullFlag)(void);
//	void (*HAL_CanBus_ReadRxMessageBuffer)(CAN_Frame_t * p_rxFrame);
//	void (*HAL_CanBus_WriteTxMessageBuffer)(CAN_Frame_t * p_rxFrame);

//	void (*HAL_CanBus_SetBaudRate)(uint32_t);

	//	void (*HAL_CanBus_RxCompleteCallback)(void);
	//	void (*HAL_CanBus_TxCompleteCallback)(void);
}
Transceiver_Interface_T;

//void CanBus_Init
//(
//	CanBus_T * p_can,
////	bool (*getRxBufferFullFlag)(void),
////	bool (*getTxBufferEmptyFlag)(void),
////	void (*enableTxBufferEmptyInterrupt)(void),
////	void (*enableRxBufferFullInterrupt)(void),
////	void (*ensableTxBufferEmptyInterrupt)(void),
////	void (*disableRxBufferFullInterrupt)(void),
////	void (*clearRxBufferFullFlag)(void),
////	void (*readRxMessageBuffer)(CAN_Frame_t * p_rxFrame),
////	void (*writeTxMessageBuffer)(CAN_Frame_t * p_rxFrame),
////	void (*rxCompleteCallback)(void),
////	void (*txCompleteCallback)(void),
////	void (*setBaudRate)(uint32_t),
////	uint32_t baudRate
//)
//{
////	HAL_CanBus_GetRxBufferFullFlag 				= getRxBufferFullFlag;
////	HAL_CanBus_GetTxBufferEmptyFlag 			= getTxBufferEmptyFlag;
////	HAL_CanBus_EnableTxBufferEmptyInterrupt 	= enableTxBufferEmptyInterrupt;
////	HAL_CanBus_EnableRxBufferFullInterrupt 		= enableRxBufferFullInterrupt;
////	HAL_CanBus_DisableTxBufferEmptyInterrupt 	= ensableTxBufferEmptyInterrupt;
////	HAL_CanBus_DisableRxBufferFullInterrupt 	= disableRxBufferFullInterrupt;
////	HAL_CanBus_ClearRxBufferFullFlag 			= clearRxBufferFullFlag;
////	HAL_CanBus_ReadRxMessageBuffer 				= readRxMessageBuffer;
////	HAL_CanBus_WriteTxMessageBuffer 			= writeTxMessageBuffer;
////	HAL_CanBus_RxCompleteCallback 				= rxCompleteCallback;
////	HAL_CanBus_TxCompleteCallback 				= txCompleteCallback;
////	HAL_CanBus_SetBaudRate 						= setBaudRate;
////
////	HAL_CanBus_SetBaudRate(baudRate);
//}

typedef const struct
{
//	Transceiver_Interface_T Interfaces[];
	void ** P_TRANSCEIVER_TABLE;
	uint8_t TRANSCEIVER_COUNT;
}
Transceiver_Config_T;



/*
 *
 */
typedef const struct
{
	Transceiver_Config_T CONFIG;
	void * p_Struct;
}
Transceiver_T;

//bool Transceiver_Validate(const Transceiver_T * p_xcvr, void * target)
//{
//	for (uint8_t iXcvr = 0; iXcvr < p_xcvr->CONFIG.TRANSCEIVER_COUNT; iXcvr++)
//	{
//		if (target == p_xcvr->CONFIG.P_TRANSCEIVER_TABLE[iXcvr])
//		{
//			p_xcvr->p_Struct = target;
//		}
//	}
//}



#define TRANSCEIVER_CONFIG(p_context, Init, SendChar, RecvChar, SendBytes, RecvBytes, SendString, RecvString)		\
{													\
	.p_Context	= p_context											\
}

//void Transceiver_Init(const Transceiver_T * p_xcvr)	{p_xcvr->Init(p_xcvr->p_Context);}
//bool Transceiver_SendChar(const Transceiver_T * p_xcvr, uint8_t txChar) 		{return p_xcvr->SendChar(p_xcvr->p_Context, txChar);}
//bool Transceiver_RecvChar(const Transceiver_T * p_xcvr, uint8_t * p_rxChar)	{return p_xcvr->RecvChar(p_xcvr->p_Context, p_rxChar);}
//uint32_t Transceiver_SendBytes(const Transceiver_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize)	{p_xcvr->SendBytes(p_xcvr->p_Context, p_srcBuffer, srcSize);}
//uint32_t Transceiver_RecvBytes(const Transceiver_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)		{p_xcvr->RecvBytes(p_xcvr->p_Context, p_destBuffer, destSize);}
//
//bool Transceiver_SendString(const Transceiver_T * p_xcvr, const uint8_t * p_src, size_t length)				{p_xcvr->SendString(p_xcvr->p_Context, p_src, length);}
//bool Transceiver_RecvString(const Transceiver_T * p_xcvr, uint8_t * p_dest, size_t length)					{p_xcvr->RecvString(p_xcvr->p_Context, p_dest, length);}


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


//uint32_t Transceiver_Recv(const Transceiver_T * p_xcvr, uint8_t * p_destBuffer, size_t length)
//{
//	return Transceiver_RecvBytes(p_xcvr, p_destBuffer, length);
//}



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

