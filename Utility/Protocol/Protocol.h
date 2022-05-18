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



/******************************************************************************/
/*!
	Rx Parser
	User provide function to determine completion of rx packet.
	returns reqid code and sets pointer upon completion.
*/
/******************************************************************************/
typedef uint8_t protocol_reqid_t;

/*
	User functions return status - communicate module handled behaviors
*/
typedef enum Protocol_RxCode_Tag
{
	PROTOCOL_RX_CODE_WAIT_PACKET,
	PROTOCOL_RX_CODE_WAIT_PACKET_HEADER = PROTOCOL_RX_CODE_WAIT_PACKET,
	PROTOCOL_RX_CODE_WAIT_PACKET_PAYLOAD,
	PROTOCOL_RX_CODE_REQ_ID_SUCCESS,
	PROTOCOL_RX_CODE_COMPLETE = PROTOCOL_RX_CODE_REQ_ID_SUCCESS,

	PROTOCOL_RX_CODE_ERROR_PACKET,
	PROTOCOL_RX_CODE_ERROR_PACKET_DATA = PROTOCOL_RX_CODE_ERROR_PACKET,

	// PROTOCOL_RX_CODE_DATA_SUCCESS,
	PROTOCOL_RX_CODE_REQ_CONTINUE, /* Continue stateful ReqExt */

	/*  */
	PROTOCOL_RX_CODE_ACK,
	PROTOCOL_RX_CODE_NACK,
	PROTOCOL_RX_CODE_ABORT,
	//	PROTOCOL_RX_CODE_REQ_VAR,
	//	PROTOCOL_RX_CODE_REQ_DATAGRAM,
	//	PROTOCOL_RX_CODE_REQ_FLASH,
	PROTOCOL_RX_CODE_RESP_DATA_SUCCESS,
}
Protocol_RxCode_T;

typedef size_t (*Protocol_BuildTx_T)(uint8_t * p_txPacket, const void * p_appInterface);
typedef Protocol_RxCode_T (*Protocol_ParseRx_T)(void * p_appInterface, const uint8_t * p_rxPacket); //Protocol_RxCode_T?

// typedef size_t(*Protocol_Ext_BuildTx_T)(void * p_subState, uint8_t * p_txPacket, const void * p_appInterface);
// typedef void (*Protocol_Ext_ParseRx_T)(void * p_subState, void * p_appInterface, const uint8_t * p_rxPacket);
// typedef void (*Protocol_Ext_Proc_T)(void * p_subState);


typedef Protocol_RxCode_T(*Protocol_ProcRx_T)(protocol_reqid_t * p_reqId, size_t * p_rxRemaining, const void * p_rxPacket, size_t rxCount);
//seprate? check proc loop
typedef Protocol_RxCode_T(*Protocol_ParseRxLength_T)(size_t * p_rxRemaining, const void * p_rxPacket, size_t rxCount);
typedef Protocol_RxCode_T(*Protocol_ParseRxId_T)(protocol_reqid_t * p_reqId, const void * p_rxPacket, size_t rxCount);
/******************************************************************************/
/*!
	Req - interface for Rx and Tx packets
*/
/******************************************************************************/
/*
	Stateless req, simple read write
*/
typedef void (*Protocol_ReqFast_T)(void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);


/*
	Common Req from child protocol to supported general protocol control
	predefined behaviors
*/
typedef enum Protocol_ReqCode_Tag
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


/*
	Extended stateful req - Support wait, loop, dynamic ack/nack and additional processes
	for wait process state, and template/format behavior
	process context support, flash, datagram, user provide
*/
typedef Protocol_ReqCode_T (*Protocol_ReqExt_T)	(void * p_subState, void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

//typedef Protocol_ReqCode_T (*Protocol_ReqExtFunction_T)	(const Protocol_Interface_T * args, size_t * p_txSize, size_t rxSize);
//typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)	(Datagram_T * p_datagram, uint8_t * p_txPacket,  size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);
// typedef void (*Protocol_DatagramInit_T)	(void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

typedef void (*Protocol_ReqExtReset_T)(void * p_subState);


/*
	Static string sync - Ack Nack Protocol_BuildTxSync_T
	Dynamic ack nack string implemented using Protocol_ReqExtFunction_T
	per req sync. does not includ timeout, and user req initiated, as they do not need to be
*/
// typedef const struct
// {
// 	// const bool TX_ACK;	// Tx Ack on Rx Req Sucess
// 	// const bool RX_ACK; 	// Wait for Ack on Req Complete, Tx Response
// 	// //do these need to be per req?
// 	// const uint8_t RX_NACK_REPEAT;	//Number of repeat TxPacket on Rx nack
// 	// const uint8_t TX_NACK_REPEAT; 	//Number of repeat Tx nack on RxPacket error
// }
// Protocol_ReqSync_T;
typedef const union Protocol_ReqSync_Tag
{
	struct
	{
		uint32_t TX_ACK 		: 1U;
		uint32_t RX_ACK 		: 1U;
		uint32_t NACK_REPEAT 	: 3U;
		// uint32_t RX_NACK_REPEAT : 3U;
		// uint32_t TX_NACK_REPEAT : 3U;
	};
	uint32_t ID;
}
Protocol_ReqSync_T;

// /*
// 	Protocol_Req_T
// */
// typedef const struct Protocol_Req_Tag
// {
// 	const protocol_reqid_t 		ID;
// 	const Protocol_ReqFast_T 	FAST;
// 	const Protocol_ReqSync_T  	SYNC;	/* Stateless ack nack */
// 	const Protocol_ReqExt_T  	EXT;
// //	const uint32_t 	TIMEOUT; /* overwrite common timeout */
// }
// Protocol_Req_T;

// #define PROTOCOL_CONFIG_SYNC_ID(UseTxAck, UseRxAck, NackRepeat) { (NackRepeat << 2U) | (UseRxAck << 1U) | (UseTxAck) }
// #define PROTOCOL_CONFIG_REQ(ReqId, ReqFast, ReqExt, ReqSyncId) { .ID = ReqId, .FAST = (Protocol_ReqFast_T)ReqFast, .EXT = (Protocol_ReqExt_T)ReqExt, .SYNC = { .ID = ReqSyncId, }, }


/*
	Protocol_Req_T
*/
typedef const struct Protocol_Req_Tag
{
	const protocol_reqid_t 			ID;
	const Protocol_BuildTx_T 		BUILD_REQ;
	// const Protocol_ParseRx_T 		PARSE_RESP;
	const Protocol_ReqSync_T  		SYNC;	/* Stateless ack nack */
	// Protocol_Cmdr_GetRespLength_T ?
//	const uint32_t 	TIMEOUT; /* overwrite common timeout */
}
Protocol_Req_T;

#define CONFIG_PROTOCOL_SYNC_ID(UseTxAck, UseRxAck, NackRepeat) { (NackRepeat << 2U) | (UseRxAck << 1U) | (UseTxAck) }
// #define CONFIG_PROTOCOL_REQ(ReqId, BuildReq, ParseResp, ReqSyncId) {.ID = ReqId, .BUILD_REQ = (Protocol_BuildTx_T)BuildReq, .PARSE_RESP = (Protocol_ParseRx_T)ParseResp, .SYNC = { .ID = ReqSyncId, }, }
#define CONFIG_PROTOCOL_REQ(ReqId, BuildReq, ReqSyncId) {.ID = ReqId, .BUILD_REQ = (Protocol_BuildTx_T)BuildReq, .SYNC = { .ID = ReqSyncId, }, }



/******************************************************************************/
/*!
	Tx Sync
	User supply function to build Tx Ack, Nack, etc
*/
/******************************************************************************/
typedef enum Protocol_TxSyncId_Tag
{
	PROTOCOL_TX_SYNC_ACK_REQ_ID,
	PROTOCOL_TX_SYNC_NACK_REQ_ID,
	PROTOCOL_TX_SYNC_NACK_TIMEOUT,
	PROTOCOL_TX_SYNC_NACK_PACKET_ERROR,
	// PROTOCOL_TX_SYNC_NACK_PACKET_ECC_ERROR,

	PROTOCOL_TX_SYNC_ACK_DATA,
	PROTOCOL_TX_SYNC_NACK_DATA,
	PROTOCOL_TX_SYNC_ABORT,
}
Protocol_TxSyncId_T;

typedef void (* const Protocol_BuildTxSync_T)(uint8_t * p_txPacket, uint8_t * p_txSize, Protocol_TxSyncId_T txId);


/*
	Packet Format Specs
*/
typedef const struct Protocol_Specs_Tag
{
	/* required to identify rx cmd */
	const uint32_t RX_TIMEOUT;			//reset cumulative per packet, until user reset
	const uint32_t RX_TIMEOUT_BYTE;		//reset per byte

	const uint32_t RX_TIMEOUT_RX;
	const uint32_t RX_TIMEOUT_REQ;
//	const uint32_t RX_TIMEOUT_PACKET;

	// const uint32_t RX_HEADER_LENGTH; fixed header length known to include contain data length value
	const uint8_t RX_LENGTH_MIN;	/* Rx Header Length, Rx this many bytes before calling PARSE_RX */
	const uint8_t RX_LENGTH_MAX;

	const Protocol_ProcRx_T PROC_RX;				/* Parse for RxReqId and RxRemaining, and check data  */
	const Protocol_BuildTxSync_T BUILD_TX_SYNC;		/* Build Sync Packets */

	const Protocol_Req_T * const P_REQ_TABLE;
	const uint8_t REQ_TABLE_LENGTH;
	const uint32_t REQ_TIMEOUT; 					/* checked for stateful Req only */
	const Protocol_ReqExtReset_T REQ_EXT_RESET; 	/* Alternatively, handle in req by user */

	/* Cmdr Side */
	// const Protocol_Req_T * const P_CMDR_REQ_TABLE;
	// const uint8_t CMDR_REQ_TABLE_LENGTH;
	const Protocol_ParseRx_T PARSE_RX; /* Cmdr pass packet to interface */

	// Protocol_ReqSync_T SYNC; //default statless sync / timeout nack

	/* Optional */
	const uint32_t RX_START_ID; /* set to 0x00 or 0xff by default for bus idle state */
	const uint32_t RX_END_ID;
	const bool ENCODED;	/* TODO Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */
	const uint32_t BAUD_RATE_DEFAULT;
}
Protocol_Specs_T;

typedef enum Protocol_RxState_Tag
{
	PROTOCOL_RX_STATE_WAIT_BYTE_1,
	PROTOCOL_RX_STATE_WAIT_PACKET,
	PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
	PROTOCOL_RX_STATE_INACTIVE,
}
Protocol_RxState_T;

typedef enum Protocol_ReqState_Tag
{
	PROTOCOL_REQ_STATE_WAIT_RX_REQ,
	PROTOCOL_REQ_STATE_WAIT_RX_DATA, //wait rx continue
	PROTOCOL_REQ_STATE_WAIT_RX_SYNC,
	PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL,
	PROTOCOL_REQ_STATE_WAIT_PROCESS,
	PROTOCOL_REQ_STATE_INACTIVE,
}
Protocol_ReqState_T;

typedef struct __attribute__((aligned(4U))) Protocol_Params_Tag
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	uint8_t XcvrId;
#else
	Serial_T * p_Serial;
#endif
	uint8_t SpecsId;
	bool IsEnableOnInit; 	/* enable on start up */
}
Protocol_Params_T;

// pass arguments as contiguous memory?
// typedef const struct
// {
// 	void * const P_APP_INTERFACE;
// 	void * const P_SUBSTATE_BUFFER;
// 	uint8_t * const P_TX_PACKET_BUFFER;
// 	const uint8_t * const P_RX_PACKET_BUFFER;
// 	//size_t * const P_TX_LENGTH;
// 	//const size_t * const P_RX_INDEX;
// }
// Protocol_Interface_T;

/*

*/
typedef const struct Protocol_Config_Tag
{
	uint8_t * const P_RX_PACKET_BUFFER;
	uint8_t * const P_TX_PACKET_BUFFER;
	const uint8_t PACKET_BUFFER_LENGTH; 	// must be greater than RX_LENGTH_MAX
	void * const P_APP_INTERFACE;			// user app context
	void * const P_SUBSTATE_BUFFER; 		// child protocol control variables, may be seperate from app_interface, must be largest enough to hold substate context from specs
	const Protocol_Specs_T * const * const P_SPECS_TABLE; /* bound and verify specs selection, set to 1 if fixed to 1 */
	const uint8_t SPECS_COUNT;
	const volatile uint32_t * const P_TIMER;
	const Protocol_Params_T * const P_PARAMS; //address of params in nvm
}
Protocol_Config_T;

#if defined(CONFIG_PROTOCOL_XCVR_ENABLE)
#define PROTOCOL_CONFIG_XCVR(p_XcvrTable, TableLength) .Xcvr = XCVR_CONFIG(p_XcvrTable, TableLength),
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
		.P_APP_INTERFACE 			= p_AppContext,					\
		.P_SUBSTATE_BUFFER 		= p_SubstateBuffer,				\
		.P_SPECS_TABLE 			= p_SpecsTable,					\
		.SPECS_COUNT 			= SpecsCount,					\
		.P_TIMER 				= p_Timer,						\
		.P_PARAMS 				= p_Params,						\
	},															\
	PROTOCOL_CONFIG_XCVR(p_XcvrTable, TableLength)	 			\
}

typedef struct Protocol_Tag
{
	const Protocol_Config_T CONFIG;
	Protocol_Params_T Params;
	//	Datagram_T Datagram; //configurable broadcast

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	Xcvr_T Xcvr;
#endif
	const Protocol_Specs_T * p_Specs;

	size_t RxIndex; //rxcount
	size_t RxRemaining;
	protocol_reqid_t RxReqId;

	size_t TxLength;
	Protocol_RxState_T		RxState;
	Protocol_RxCode_T 		RxCode;
	Protocol_ReqState_T 	ReqState;
	Protocol_ReqCode_T 		ReqCode;
	uint32_t ReqTimeStart;
	uint32_t RxTimeStart;

	uint8_t RxNackCount; //todo combine
	uint8_t TxNackCount;

	protocol_reqid_t ReqIdActive;
	Protocol_Req_T * p_ReqActive;
	//	 volatile bool ReqRxSemaphore;
	//	Protocol_Interface_T Interface;
}
Protocol_T;

extern void Protocol_Init(Protocol_T * p_protocol);
extern void Protocol_Slave_Proc(Protocol_T * p_protocol);
extern void Protocol_ConfigBaudRate(Protocol_T * p_protocol, uint32_t baudRate);
extern void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t specsId);
extern void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId);
// extern void Protocol_SetSpecs_Ptr(Protocol_T * p_protocol, const Protocol_Specs_T * p_specs);
// extern void Protocol_SetXcvr_Ptr(Protocol_T * p_protocol, void * p_transceiver);
extern bool Protocol_Enable(Protocol_T * p_protocol);
extern void Protocol_Disable(Protocol_T * p_protocol);
extern void Protocol_EnableOnInit(Protocol_T * p_protocol);
extern void Protocol_DisableOnInit(Protocol_T * p_protocol);

#endif
