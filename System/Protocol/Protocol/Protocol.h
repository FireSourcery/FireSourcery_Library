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
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef PROTOCOL_G_H
#define PROTOCOL_G_H

#include "Datagram.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Flash/Flash.h"

#include <stdint.h>
#include <stdbool.h>

/*
	Protocol_Req_T
 */
typedef uint32_t protocolg_reqid_t;

typedef enum
{
	PROTOCOL_REQ_STANDARD,
//	PROTOCOL_REQ_FAST,
//	PROTOCOL_REQ_EXT, 		//passes user defined P_PROCESS_CONTEXT
//	PROTOCOL_REQ_FLASH, 	//passes special control context
	PROTOCOL_REQ_DATAGRAM,	//passes special control context
	PROTOCOL_REQ_CONFIG,	//passes special control context
}
Protocol_ReqType_T;

typedef enum
{
	PROTOCOL_REQ_STATUS_WAIT,
	PROTOCOL_REQ_STATUS_EXTEND_TIMER,
	PROTOCOL_REQ_STATUS_COMPLETE,	//exit nonblocking wait state upon reception
	PROTOCOL_REQ_STATUS_AWAIT_RX,	//rx new packet
	PROTOCOL_REQ_STATUS_AWAIT_SYNC,	//wait for static ack nack
}
Protocol_ReqStatus_T;

typedef void (*Protocol_ReqFastReadWrite_T) (volatile void * p_appInterface, volatile uint8_t * p_txPacket, volatile size_t * p_txSize, const volatile uint8_t * p_rxPacket, size_t rxSize);

typedef struct
{
	volatile void * p_SubState;
	volatile void * p_AppInterface;
	volatile uint8_t * p_TxPacket;
	volatile size_t * p_TxSize;
	const volatile uint8_t * p_RxPacket;
	size_t RxSize;
	uint32_t Option; //stop, datagram address
}
Protocol_ReqExtProcessArgs_T;

typedef Protocol_ReqStatus_T (*Protocol_ReqExtFunction_T)	(Protocol_ReqExtProcessArgs_T args); //typedef Protocol_ReqExtStatus_T (*Protocol_ReqExtFunction_T)	(volatile void * p_subState, volatile void * p_appInterface, volatile uint8_t * p_txPacket, volatile size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

/*
 * static string sync
 * Ack Nack using fixed strings
 * Dynamic ack nack string must use process
 */
typedef const struct
{
	const bool USE_TX_ACK_REQ;
	const bool USE_TX_RETRANSMIT_ON_NACK;
	const bool USE_WAIT_RX_ACK_COMPLETE;
//	const uint32_t WAIT_RX_ACK_TIMEOUT;
	const uint8_t WAIT_RX_NACK_REPEAT; //stay in wait for ack state
}
Protocol_ReqSync_T;

/*
	for wait process state, and template/format behavior
	process context support, flash, datagram, user provide
 */
typedef const struct
{
	const Protocol_ReqExtFunction_T PROCESS;
//	const uint32_t TIMEOUT;
}
Protocol_ReqExtProcess_T;

typedef const struct
{
	const protocolg_reqid_t 			ID;
	const Protocol_ReqType_T 			TYPE; 				/* pass special options, module cmd code */
	const Protocol_ReqFastReadWrite_T 	FAST;				/* Handles simple read write */
	const Protocol_ReqSync_T 			* const P_SYNC;		/* with static ack nack */
	const Protocol_ReqExtProcess_T		* const P_EXT;		/* Wait, loop, dynamic ack nack and additional process contest */
//	const uint32_t TIMEOUT;
}
Protocol_ReqEntry_T;

#define PROTOCOL_G_REQ_ENTRY(ReqId, ReqType, ReqFast, p_ReqSync, p_ReqExt) { (.ID = ReqId), (.TYPE = ReqType), (.FAST_READ_WRITE = ReqFast), (.P_SYNC = p_ReqSync), (.P_EXT = p_ReqExt) }

typedef enum
{
	PROTOCOL_REQ_STATE_WAIT_RX_REQ,
	PROTOCOL_REQ_STATE_WAIT_RX_DATA,
	PROTOCOL_REQ_STATE_WAIT_RX_SYNC,
	PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL,
	PROTOCOL_REQ_STATE_WAIT_PROCESS,
	PROTOCOL_REQ_STATE_INACTIVE,
}
Protocol_ReqState_T;


typedef enum
{
	PROTOCOL_RX_STATUS_WAIT,
	PROTOCOL_RX_STATUS_SUCCESS,
	PROTOCOL_RX_STATUS_ERROR_REQ,
	PROTOCOL_RX_STATUS_ERROR_DATA,
	PROTOCOL_RX_STATUS_ACK,
	PROTOCOL_RX_STATUS_NACK,
}
Protocol_RxStatus_T;

typedef enum
{
	PROTOCOL_RX_STATE_WAIT_BYTE_1,
	PROTOCOL_RX_STATE_WAIT_PACKET,
	PROTOCOL_RX_STATE_WAIT_REQ,
	PROTOCOL_RX_STATE_INACTIVE,
}
Protocol_RxState_T;

//typedef struct
//{
//	volatile void * p_ProtocolState;
//	protocolg_reqid_t * p_ReqId;
//	const uint8_t * p_RxPacket;
//	size_t RxCount;
//}
//Protocol_RxArgs_T;

typedef enum
{
	PROTOCOL_TX_SYNC_ACK_REQ,
	PROTOCOL_TX_SYNC_NACK_TIMEOUT,
	PROTOCOL_TX_SYNC_NACK_DATA_ERROR,
	PROTOCOL_TX_SYNC_NACK_REQ_ID,
}
Protocol_TxSyncId_T;

/*
 * Packet Format Specs
 */
typedef const struct
{
	/* required to identify rx cmd */
	const uint32_t RX_TIMEOUT;			//reset if packet is not complete
//	const uint32_t TIME_OUT_BYTE;		//reset if byte is not received
	const uint8_t RX_LENGTH_MIN;
	const uint8_t RX_LENGTH_MAX;

	/*
	 * User provide function to determine completion of rx packet. returns reqid upon completion.
	 */
	Protocol_RxStatus_T 	(*const PARSE_RX_REQ) 	(volatile void * p_subState, protocolg_reqid_t * p_reqId, const volatile void * p_rxPacket, uint8_t rxCount);
	void 					(*const BUILD_TX_SYNC) 	(volatile void * p_subState, volatile uint8_t * p_txPacket, volatile size_t * p_txSize, Protocol_TxSyncId_T txId);
	void 					(*const RESET_SUBSTATE)	(volatile void * p_subState);

	const Protocol_ReqEntry_T * const P_REQ_TABLE;
	const uint8_t REQ_TABLE_LENGTH;
	const uint32_t REQ_TIMEOUT; //per function loop.

	/* Optional */
	const uint32_t RX_START_ID; //set to 0x00 or 0xff by default for bus idle state
	const uint32_t RX_END_ID;
//	const uint8_t START_ID_LENGTH; multichar start/end id
//	const uint8_t END_ID_LENGTH;
	const bool ENCODED;	//encoded data, non encoded use time out only, past first char, no meta chars
}
Protocol_Specs_T;


/*

 */
typedef const struct
{
	const volatile uint32_t * const P_TIMER;
	const uint8_t PACKET_BUFFER_LENGTH; // must be greater than RX_LENGTH_MAX
	volatile uint8_t * const P_RX_PACKET_BUFFER;
	volatile uint8_t * const P_TX_PACKET_BUFFER;
	volatile void * const P_SUBSTATE_BUFFER; 	// track separate from app interface for user convineice
	void * const P_APP_INTERFACE;		// user map to app data interface

//	Flash_T * const P_FLASH;
	const union
	{
		const struct
		{
			const uint32_t BAUD_RATE_DEFAULT;
		};
		const struct
		{

		};
	};
}
Protocol_Config_T;

typedef struct Protocol_Tag
{
	//compile time consts config
	const Protocol_Config_T CONFIG;

	//run time config
	const Protocol_Specs_T * p_Specs;
	Serial_T * p_Port;
	Datagram_T Datagram;

	//proc variables
//	volatile Protocol_State_T State;

	volatile Protocol_RxState_T		RxState;
	volatile Protocol_RxStatus_T 	RxStatus;
	volatile Protocol_ReqState_T 	ReqState;
	volatile Protocol_ReqStatus_T 	ReqStatus;

	volatile uint32_t TimeStart;
	volatile size_t RxIndex;
	volatile size_t TxLength;
	volatile uint8_t NackCount;

	volatile protocolg_reqid_t ReqIdActive;
	volatile Protocol_ReqEntry_T * p_ReqActive;
	//	Protocol_Control_T Control;
}
Protocol_T;

//extern void Protocol_Init			(Protocol_T * p_protocol);
extern void Protocol_Init_Void	(Protocol_T * p_protocol);
extern void Protocol_SetSpecs	(Protocol_T * p_protocol, const Protocol_Specs_T * p_specs);
extern void Protocol_SetPort	(Protocol_T * p_protocol, void * p_transceiver);
extern void Protocol_Slave_Proc	(Protocol_T * p_protocol);

#endif
