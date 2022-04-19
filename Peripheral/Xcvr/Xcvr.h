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
	@file 	HAL_Xcvr.h
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

typedef const struct
{
	void * P_XCVRS;
	uint8_t XCVR_COUNT;
}
Xcvr_Table_T;

/*
 *
 */
//typedef enum
//{
//	XCVR_TYPE_SERIAL,
//	XCVR_TYPE_I2C,
//	XCVR_TYPE_SPI,
//}
//Xcvr_Type_T;
//
//typedef const struct
//{
//	//todo table
////	Xcvr_Interface_T Interfaces[];
//	void ** P_XCVR_TABLE;
//	uint8_t XCVR_COUNT;
//
//}
//Xcvr_Config_T;
//typedef const struct
//{
//	Xcvr_Config_T CONFIG;
////	void * p_Struct;
////	Xcvr_Type_T Type;
//}
//Xcvr_T;

//typedef   struct
//{
//	Xcvr_Interface_T * P_XCVR_TABLE;
//	uint8_t XCVR_COUNT;
//	Xcvr_Interface_T * P_XCVR_DEFAULT;


//	Xcvr_Interface_T * p_Xcvr;
//}
//Xcvr_T;

typedef const struct
{
	void 		* P_CONTEXT;
	void 		(* INIT)		(void * p_context);
//	bool 		(* SendChar)	(void * p_context, uint8_t txChar);
//	bool 		(* RecvChar)	(void * p_context, uint8_t * p_rxChar);
//	uint32_t 	(* SendBytes)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t bufferSize);
//	uint32_t 	(* RecvBytes)	(void * p_context, uint8_t * p_destBuffer, 			size_t bufferSize);
//	bool 		(* SendString)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
//	bool 		(* RecvString)	(void * p_context, uint8_t * p_destBuffer, 			size_t length);
	uint32_t 	(* RECV) (void * p_context, uint8_t * p_destBuffer, 		size_t bufferSize);
	bool 		(* SEND) (void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
}
Xcvr_T;


#define XCVR_CONFIG(p_Context, Init, Send, Recv)	\
{													\
	.P_CONTEXT	= p_context							\
	.INIT		= Init								\
	.SEND		= Send								\
	.RECV		= Recv								\
}



static inline bool Xcvr_Send(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t length)
{
	return p_xcvr->SEND(p_xcvr->P_CONTEXT, p_srcBuffer, length);
}

static inline uint32_t Xcvr_Recv(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t length)
{
	return p_xcvr->RECV(p_xcvr->P_CONTEXT, p_destBuffer, length);
}

//bool Xcvr_Validate(const Xcvr_T * p_xcvr, void * target)
//{
//	for (uint8_t iXcvr = 0; iXcvr < p_xcvr->CONFIG.XCVR_COUNT; iXcvr++)
//	{
//		if (target == p_xcvr->CONFIG.P_XCVR_TABLE[iXcvr])
//		{
//			p_xcvr->p_Struct = target;
//		}
//	}
//}
//
//static inline bool Xcvr_Table_CheckValid(const Xcvr_Table_T * p_table, const Xcvr_T * p_xcvr)
//{
//	bool isValid = false;
//
//	for (uint8_t iXcvr = 0; iXcvr < p_table->XCVR_COUNT; iXcvr++)
//	{
//		if (p_xcvr == p_table->XCVR_COUNT[iXcvr])
//		{
//			isValid = true;
//			break;
//		}
//	}
//}

static inline void Xcvr_Init(const Xcvr_T * p_xcvr)
{
//	if (Xcvr_Table_CheckValid(p_table,  p_xcvr))
//	{
//		p_xcvr->INIT(p_xcvr->P_CONTEXT);
//	}
//	else
//	{
//		p_xcvr.p_xcvr = p_table.default
//		p_xcvr->INIT(p_xcvr->P_CONTEXT);
//	}
}

//static inline Xcvr_T * Xcvr_Table_GetPtr(const Xcvr_Table_T * p_table, uint8_t index)
//{
//	return (index < p_table->XCVR_COUNT) ? p_table->P_XCVRS[index] : 0U;
//}




//bool Xcvr_SendChar(const Xcvr_T * p_xcvr, uint8_t txChar) 									{return p_xcvr->SendChar(p_xcvr->p_Context, txChar);}
//bool Xcvr_RecvChar(const Xcvr_T * p_xcvr, uint8_t * p_rxChar)									{return p_xcvr->RecvChar(p_xcvr->p_Context, p_rxChar);}
//uint32_t Xcvr_SendBytes(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize)	{p_xcvr->SendBytes(p_xcvr->p_Context, p_srcBuffer, srcSize);}
//uint32_t Xcvr_RecvBytes(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)		{p_xcvr->RecvBytes(p_xcvr->p_Context, p_destBuffer, destSize);}
//bool Xcvr_SendString(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)				{p_xcvr->SendString(p_xcvr->p_Context, p_src, length);}
//bool Xcvr_RecvString(const Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length)

//void Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate)
//{
//	HAL_Xcvr_ConfigBaudRate(p_xcvr->CONFIG.P_HAL_SERIAL, baudRate);
//}

//static inline void Xcvr_Poll(const Xcvr_T * p_xcvr)
//{
////	Xcvr_PollRestartRxIsr(p_Serial)
//}

//static inline uint32_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr)
//{
//
//}
//
//static inline uint32_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr)
//{
//
//}
//
//static inline void Xcvr_EnableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_EnableRx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableRx(const Xcvr_T * p_xcvr){}

#endif

