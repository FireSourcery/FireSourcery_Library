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
	@file 	Xcvr.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef XCVR_H
#define XCVR_H

#include "Config.h"
#include "Peripheral/Serial/Serial.h"
#include "Utility/Queue/Queue.h"

#include <stdint.h>
#include <stdbool.h>
//#include <stddef.h>

typedef enum
{
	XCVR_TYPE_SERIAL,
	XCVR_TYPE_I2C,
	XCVR_TYPE_SPI,
	XCVR_TYPE_VIRTUAL,
}
Xcvr_Type_T;

typedef const struct
{
	void * P_XCVR;
	Xcvr_Type_T TYPE;
}
Xcvr_Xcvr_T;

#define XCVR_XCVR_CONFIG(p_Xcvr, Type)		\
{											\
	.P_XCVR 	= (void *) p_Xcvr, 			\
	.TYPE 		= Type,						\
}

typedef const struct
{
	const Xcvr_Xcvr_T * const P_XCVR_TABLE;
	const uint8_t XCVR_COUNT;
}
Xcvr_Config_T;

typedef struct
{
	const Xcvr_Config_T CONFIG;
	void * p_Xcvr;
	Xcvr_Type_T Type;
	/* only 1 nvm param is needed, handle by outside module */
}
Xcvr_T;

#define XCVR_CONFIG(p_XcvrTable, Count)		\
{											\
	.CONFIG =								\
	{  										\
		.P_XCVR_TABLE	= p_XcvrTable,		\
		.XCVR_COUNT		= Count,			\
	},										\
}

extern bool Xcvr_SetXcvr(Xcvr_T * p_xcvr, uint8_t xcvrIndex);

//bool Xcvr_SendChar(const Xcvr_T * p_xcvr, uint8_t txChar) 									{return p_xcvr->SendChar(p_xcvr->p_Context, txChar);}
//bool Xcvr_RecvChar(const Xcvr_T * p_xcvr, uint8_t * p_rxChar)									{return p_xcvr->RecvChar(p_xcvr->p_Context, p_rxChar);}
//uint32_t Xcvr_SendBytes(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize)	{p_xcvr->SendBytes(p_xcvr->p_Context, p_srcBuffer, srcSize);}
//uint32_t Xcvr_RecvBytes(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)		{p_xcvr->RecvBytes(p_xcvr->p_Context, p_destBuffer, destSize);}
//bool Xcvr_SendString(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)				{p_xcvr->SendString(p_xcvr->p_Context, p_src, length);}
//bool Xcvr_RecvString(const Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length)



//static inline void Xcvr_Proc(const Xcvr_T * p_xcvr)
//{
//}



//static inline void Xcvr_EnableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_EnableRx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableRx(const Xcvr_T * p_xcvr){}

#endif
//typedef const struct
//{
//	void 		(* INIT)		(void * p_context);
////	bool 		(* SendChar)	(void * p_context, uint8_t txChar);
////	bool 		(* RecvChar)	(void * p_context, uint8_t * p_rxChar);
////	uint32_t 	(* SendBytes)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t bufferSize);
////	uint32_t 	(* RecvBytes)	(void * p_context, uint8_t * p_destBuffer, 			size_t bufferSize);
////	bool 		(* SendString)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
////	bool 		(* RecvString)	(void * p_context, uint8_t * p_destBuffer, 			size_t length);
//	uint32_t 	(* RECV) (void * p_context, uint8_t * p_destBuffer, 		size_t bufferSize);
//	bool 		(* SEND) (void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
//}
//Xcvr_T;
//return p_xcvr->RECV(p_xcvr->p_Xcvr, p_destBuffer, length);
//return p_xcvr->SEND(p_xcvr->p_Xcvr, p_srcBuffer, length);
