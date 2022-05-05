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
	@file	Protocol.h
	@author FireSoucery
	@brief	Simple general configurable protocol
	@version V0
*/
/******************************************************************************/
#ifndef PROTOCOL_H 
#define PROTOCOL_H

#include "Config.h"
//#include "Datagram.h"

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	#include "Peripheral/Xcvr/Xcvr.h"
#else
	#include "Peripheral/Serial/Serial.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/*
	Protocol_Req_T
 */
typedef uint32_t protocolg_reqid_t;

typedef void (*Protocol_ReqFastReadWrite_T)(void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

/*
 * Common req from child protocol to supported general protocol control
 * predefined behaviors
 */
typedef enum
{
	PROTOCOL_REQ_CODE_WAIT_PROCESS, 		//wait process
//	PROTOCOL_REQ_CODE_WAIT_PROCESS_PAUSE_RX,
PROTOCOL_REQ_CODE_COMPLETE,				//exit nonblocking wait state upon reception
PROTOCOL_REQ_CODE_AWAIT_RX_DATA,		//rx new packet
PROTOCOL_REQ_CODE_AWAIT_RX_SYNC,		//wait for static ack nack
PROTOCOL_REQ_CODE_TX_DATA,
PROTOCOL_REQ_CODE_TX_ACK,
PROTOCOL_REQ_CODE_TX_NACK,
PROTOCOL_REQ_CODE_EXTEND_TIMER,
//	PROTOCOL_REQ_CODE_DATAGRAM_CONFIG,
}
Protocol_ReqCode_T;

typedef Protocol_ReqCode_T(*Protocol_ReqExtFunction_T)	(void * p_subState, void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);
//typedef Protocol_ReqCode_T (*Protocol_ReqExtFunction_T)	(const Protocol_Interface_T * args, size_t * p_txSize, size_t rxSize);
//typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)	(Datagram_T * p_datagram, uint8_t * p_txPacket,  size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

typedef void (*Protocol_DatagramInit_T)	(void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

/*
 * static string sync
 * Ack Nack using fixed strings
 * Dynamic ack nack string must use process
 *
 * per req sync. does not includ timeout, and user req initiated, as they do not need to be
 */
typedef const struct
{
	const bool USE_TX_ACK_REQ;				// Tx Ack on Rx Req
	const bool USE_WAIT_RX_ACK_COMPLETE; 	// Wait on Req Complete

	//do these need to be per req?
	const uint8_t WAIT_RX_NACK_REPEAT;	//repeat tx on rx nack
	const uint8_t TX_NACK_REPEAT; 		//tx nack on rx error
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
	const Protocol_ReqFastReadWrite_T 	FAST;				/* Stateless req, simple read write */
	const Protocol_ReqSync_T * const P_SYNC;		/* Stateless ack nack */
	const Protocol_ReqExtProcess_T * const P_EXT;		/* Extended req, saving state data. - Support wait, loop, dynamic ack/nack and additional processes */
//	const uint32_t TIMEOUT;
}
Protocol_ReqEntry_T;

#define PROTOCOL_REQ_ENTRY(ReqId, ReqType, ReqFast, p_ReqSync, p_ReqExt) { (.ID = ReqId), (.TYPE = ReqType), (.FAST_READ_WRITE = ReqFast), (.P_SYNC = p_ReqSync), (.P_EXT = p_ReqExt) }



/*
 * User implement, communicate to module, validity and id of packet
 */
typedef enum
{
	PROTOCOL_RX_CODE_WAIT_PACKET,
	//	PROTOCOL_RX_CODE_WAIT_PACKET_LENGTH,
	//	PROTOCOL_RX_CODE_WAIT_PACKET_DATA,
	PROTOCOL_RX_CODE_REQ_ID,
	PROTOCOL_RX_CODE_ERROR_PACKET,
	PROTOCOL_RX_CODE_ERROR_PACKET_ECC,
	PROTOCOL_RX_CODE_DATA,
	PROTOCOL_RX_CODE_ACK,
	PROTOCOL_RX_CODE_NACK,
	PROTOCOL_RX_CODE_ABORT,
	//	PROTOCOL_RX_CODE_REQ_VAR,
	//	PROTOCOL_RX_CODE_REQ_DATAGRAM,
	//	PROTOCOL_RX_CODE_REQ_FLASH,
	//	PROTOCOL_RX_CODE_REQ_ID_DATA, //continue req
}
Protocol_RxCode_T;

typedef enum
{
	PROTOCOL_TX_SYNC_ACK_REQ,
	PROTOCOL_TX_SYNC_NACK_TIMEOUT,
	PROTOCOL_TX_SYNC_NACK_PACKET_ERROR,
	PROTOCOL_TX_SYNC_NACK_PACKET_ECC_ERROR,
	PROTOCOL_TX_SYNC_NACK_REQ_ID,
	PROTOCOL_TX_SYNC_ACK_DATA,		//initiated from user req
	PROTOCOL_TX_SYNC_NACK_DATA,
}
Protocol_TxSyncId_T;


/*
 * Packet Format Specs
 */
typedef const struct
{
	/* required to identify rx cmd */
	const uint32_t RX_TIMEOUT;			//reset if packet is not complete
//	const uint32_t RX_TIMEOUT_BYTE;		//reset if byte is not received
	const uint8_t RX_LENGTH_MIN;
	const uint8_t RX_LENGTH_MAX;

	/*
	 * User provide function to determine completion of rx packet. returns reqid upon completion.
	 */
	Protocol_RxCode_T(* const PARSE_RX)(void * p_subState, protocolg_reqid_t * p_reqId, const void * p_rxPacket, uint8_t rxCount);
	void 				(* const BUILD_TX_SYNC)(void * p_subState, uint8_t * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId);
	void 				(* const RESET_SUBSTATE)(void * p_subState);

	//	BuildTx_T * P_BUILD_TX_TABLE;

	const Protocol_ReqEntry_T * const P_REQ_TABLE;
	const uint8_t REQ_TABLE_LENGTH;
	const uint32_t REQ_TIMEOUT; //per function loop.

	/* Optional */
	const uint32_t RX_START_ID; //set to 0x00 or 0xff by default for bus idle state
	const uint32_t RX_END_ID;
	//	const uint8_t START_ID_LENGTH; multichar start/end id
	//	const uint8_t END_ID_LENGTH;
	const bool ENCODED;	//encoded data, non encoded use time out only, past first char, no meta chars

//	bool USE_RX_PAUSE_ON_REQ;
	const uint32_t BAUD_RATE_DEFAULT;
}
Protocol_Specs_T;

typedef enum
{
	PROTOCOL_RX_STATE_WAIT_BYTE_1,
	PROTOCOL_RX_STATE_WAIT_PACKET,
	PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
	PROTOCOL_RX_STATE_INACTIVE,
}
Protocol_RxState_T;

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

typedef struct __attribute__((aligned(4U)))
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	uint8_t XcvrId;
#else
	Serial_T * p_Serial;
#endif
	uint8_t SpecsId;
	bool IsEnable; 	/* enable on start up */
}
Protocol_Params_T;

//typedef const struct
//{
//	void * const P_APP_CONTEXT;
//	void * const P_SUBSTATE_BUFFER;
//	uint8_t * const P_TX_PACKET_BUFFER;
//	size_t * const P_TX_LENGTH;
//	const uint8_t * const P_RX_PACKET_BUFFER;
//	const size_t * const P_RX_INDEX;
//}
//Protocol_Interface_T;

/*

 */
typedef const struct
{
	uint8_t * const P_RX_PACKET_BUFFER;
	uint8_t * const P_TX_PACKET_BUFFER;
	const uint8_t PACKET_BUFFER_LENGTH; 	// must be greater than RX_LENGTH_MAX
	void * const P_APP_CONTEXT;				// user app context
	void * const P_SUBSTATE_BUFFER; 		// child protocol control variables, may be seperate from app_interface, must be largest enough to hold substate context from specs
	const Protocol_Specs_T * const * const P_SPECS_TABLE; /* bound and verify specs selection, set to 1 if fixed to 1 */
	const uint8_t SPECS_COUNT;
	const volatile uint32_t * const P_TIMER;
	const Protocol_Params_T * const P_PARAMS; //address of params in nvm
}
Protocol_Config_T;

#if defined(CONFIG_PROTOCOL_XCVR_ENABLE)
#define PROTOCOL_CONFIG_XCVR(p_XcvrTable, TableLength) .Xcvr = XCVR_CONFIG(p_XcvrTable, TableLength)
#else
#define PROTOCOL_CONFIG_XCVR(p_XcvrTable, TableLength)
#endif

#define PROTOCOL_CONFIG(p_RxBuffer, p_TxBuffer, PacketLengthMax, p_AppContext, p_SubstateBuffer, p_SpecsTable, SpecsCount, p_XcvrTable, TableLength, p_Timer, p_Params)	\
{																\
	.CONFIG = 													\
	{															\
		.P_RX_PACKET_BUFFER 	= p_RxBuffer,					\
		.P_TX_PACKET_BUFFER 	= p_TxBuffer,					\
		.PACKET_BUFFER_LENGTH 	= PacketLengthMax,				\
		.P_APP_CONTEXT 			= p_AppContext,					\
		.P_SUBSTATE_BUFFER 		= p_SubstateBuffer,				\
		.P_SPECS_TABLE 			= p_SpecsTable,					\
		.SPECS_COUNT 			= SpecsCount,					\
		.P_TIMER 				= p_Timer,						\
		.P_PARAMS 				= p_Params,						\
	},															\
	PROTOCOL_CONFIG_XCVR(p_XcvrTable, TableLength),	 			\
}

typedef struct Protocol_Tag
{
	const Protocol_Config_T CONFIG;		//compile time consts config
	Protocol_Params_T Params;			//run time config
//	Datagram_T Datagram;

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	Xcvr_T Xcvr;
#endif
	const Protocol_Specs_T * p_Specs;
 
	size_t RxIndex;
	size_t TxLength;
	//	 volatile bool ReqRxSemaphore;
	Protocol_RxState_T		RxState;
	Protocol_RxCode_T 		RxCode;
	Protocol_ReqState_T 	ReqState;
	Protocol_ReqCode_T 		ReqCode;
	uint32_t ReqTimeStart;
	uint32_t RxTimeStart;
	uint8_t RxNackCount;
	uint8_t TxNackCount;
	protocolg_reqid_t ReqIdActive;
	Protocol_ReqEntry_T * p_ReqActive;
	//	Protocol_Control_T Control;
}
Protocol_T;

extern void Protocol_Init(Protocol_T * p_protocol);
extern void Protocol_Slave_Proc(Protocol_T * p_protocol);
extern void Protocol_ConfigBaudRate(Protocol_T * p_protocol, uint32_t baudRate);
extern void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t specsId);
extern void Protocol_SetSpecs_Ptr(Protocol_T * p_protocol, const Protocol_Specs_T * p_specs);
extern void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId);
extern void Protocol_SetXcvr_Ptr(Protocol_T * p_protocol, void * p_transceiver); 
extern bool Protocol_Enable(Protocol_T * p_protocol);
extern void Protocol_Disable(Protocol_T * p_protocol);

#endif
