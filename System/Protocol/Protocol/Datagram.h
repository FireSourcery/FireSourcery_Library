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
	@file 	Datagram.h
	@author FireSoucery
	@brief	Datagram mode, continuous transmission
	@version V0
*/
/******************************************************************************/
#ifndef DATAGRAM_H
#define DATAGRAM_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
//	DATAGRAM_STATE_WAIT_RX_BYTE_1,
//	DATAGRAM_STATE_WAIT_RX_PACKET,
//	DATAGRAM_STATE_WAIT_PROCESS,
	DATAGRAM_STATE_WAIT_SIGNAL,
	DATAGRAM_STATE_ACTIVE,
	DATAGRAM_STATE_INACTIVE,
}
Datagram_State_T;


typedef enum
{
	DATAGRAM_TX_MODE_ON_SIGNAL,
	DATAGRAM_TX_MODE_PERIODIC,
}
Datagram_TxMode_T;


typedef struct
{
	volatile void * p_Var;
	uint8_t VarSize;
}
Datagram_Entry_T;

//typedef uint8_t datagram_varid_t;

typedef struct
{
	//config
	Datagram_Entry_T * const P_VAR_TABLE;
	const uint8_t VAR_TABLE_LENGTH; 	// Var Count Max
	const uint8_t VAR_SIZE_MAX; 		// Prevent unbounded read
	uint8_t * const P_RX_BUFFER; 		// Keep track of initial configuration
	uint8_t * const P_TX_BUFFER; 		// Separate from protocol
	uint8_t * const P_NONVOLATILE_BUFFER; 	//store var if non volatile write


//	void (*const BUILD_HEADER) (void * p_txPacketHeader, const void * p_datagram);
	//	void (*const BUILD_HEADER) (const void * p_txPacket, uint8_t dataLength, P_RX_CMD );

	void (*const BuildHeader)(const void * p_datagram);

	void (*const CheckSignal)(const void * p_datagram);

	uint8_t TxDataSizeMax;		//packet payload size
	uint8_t TxDataSizeActive;

	uint8_t ActiveVarIndex;
	uint8_t ActiveVarByteIndex;

	uint8_t PresetCycles;

	void (*const OnCycle)(void * p_datagram);
	void (*const OnComplete)(void * p_datagram);

	volatile Datagram_State_T State;
	volatile uint8_t DatagramVarCount; //active

	volatile uint32_t WaitForSignal;

	const uint32_t ReturnCode;


	const uint8_t HeaderSize;
	volatile bool DatagramModeEnable;  //periodic / on signal

}
Datagram_T;

#endif
