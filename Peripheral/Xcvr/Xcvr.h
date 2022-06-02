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

// #if defined(XCVR_SERIAL_ENABLE)
#include "Peripheral/Serial/Serial.h"
// #endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum Xcvr_Type_Tag
{
	XCVR_TYPE_SERIAL,
	XCVR_TYPE_I2C,
	XCVR_TYPE_SPI,
	XCVR_TYPE_VIRTUAL,
	XCVR_TYPE_INTERFACE,
}
Xcvr_Type_T;

typedef const struct Xcvr_Interface_Tag
{
	void 		(* INIT)		(void * p_context);
	bool 		(* SEND_BYTE)	(void * p_context, uint8_t txChar);
	bool 		(* RECV_BYTE)	(void * p_context, uint8_t * p_rxChar);
	uint32_t 	(* SEND_MAX)	(void * p_context, const uint8_t * p_srcBuffer, 	size_t bufferSize);
	uint32_t 	(* RECV_MAX)	(void * p_context, uint8_t * p_destBuffer, 			size_t bufferSize);
	bool 		(* SEND_N)		(void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
	bool 		(* RECV_N)		(void * p_context, uint8_t * p_destBuffer, 			size_t length);
	// uint32_t 	(* RECV) (void * p_context, uint8_t * p_destBuffer, 		size_t bufferSize);
	// bool 		(* SEND) (void * p_context, const uint8_t * p_srcBuffer, 	size_t length);
}
Xcvr_Interface_T;

typedef const struct Xcvr_Xcvr_Tag
{
	void * P_XCVR; /* Xcvr context or hal */
	Xcvr_Type_T TYPE;
	Xcvr_Interface_T * P_INTERFACE; /* if applicable */
}
Xcvr_Xcvr_T;

#define XCVR_XCVR_DEFINE(p_Xcvr, Type)		\
{											\
	.P_XCVR 	= (void *)p_Xcvr, 			\
	.TYPE 		= Type,						\
}

typedef const struct Xcvr_Config_Tag
{
	const Xcvr_Xcvr_T * const P_XCVR_TABLE;
	const uint8_t XCVR_COUNT;
}
Xcvr_Config_T;

typedef struct Xcvr_Tag
{
	const Xcvr_Config_T CONFIG;
// Xcvr_Xcvr_T * p_Xcvr;

	void * p_Xcvr;
	Xcvr_Type_T Type; /* NvM XcvrId param handle by outside module */
}
Xcvr_T;

#define XCVR_DEFINE(p_XcvrTable, Count)		\
{											\
	.CONFIG =								\
	{  										\
		.P_XCVR_TABLE	= p_XcvrTable,		\
		.XCVR_COUNT		= Count,			\
	},										\
}

extern void Xcvr_Init(Xcvr_T * p_xcvr, uint8_t xcvrIndex);
extern bool Xcvr_SetXcvr(Xcvr_T * p_xcvr, uint8_t xcvrIndex);
extern bool Xcvr_CheckIsSet(const Xcvr_T * p_xcvr, uint8_t xcvrIndex);
extern bool Xcvr_CheckValid(const Xcvr_T * p_xcvr, void * p_target);
extern void Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate);
extern bool Xcvr_Tx(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t length);
extern uint32_t Xcvr_Rx(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t length);
extern uint32_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr);
extern uint32_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr);
extern uint8_t * Xcvr_AcquireTxBuffer(const Xcvr_T * p_xcvr);
extern void Xcvr_ReleaseTxBuffer(const Xcvr_T * p_xcvr, size_t writeSize);

#endif



