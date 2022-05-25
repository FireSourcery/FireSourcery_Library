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

#if defined(CONFIG_PROTOCOL_XCVR_ENABLE)
#include "Peripheral/Xcvr/Xcvr.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/*!
	Rx Parser
	User provide function to determine completion of rx packet.
	returns reqid code and sets pointer upon completion.
*/
/******************************************************************************/
typedef uint8_t protocol_reqid_t; /* Index into Req Function Table P_REQ_TABLE */

/*
	User functions return status - communicate module handled behaviors
*/
typedef enum Protocol_RxCode_Tag
{
	PROTOCOL_RX_CODE_WAIT_PACKET, 	/* Wait Build Rx */
	PROTOCOL_RX_CODE_ERROR, 		/* Error Req Packet Checksum/CRC */
	PROTOCOL_RX_CODE_COMPLETE, 		/* Req/ReqExt Packet Complete Success*/

	/* Sync */
	PROTOCOL_RX_CODE_ACK,
	PROTOCOL_RX_CODE_NACK,
	PROTOCOL_RX_CODE_ABORT,

	/* Special Context Functions */
	//	PROTOCOL_RX_CODE_REQ_VAR,
	//	PROTOCOL_RX_CODE_REQ_DATAGRAM,
	//	PROTOCOL_RX_CODE_REQ_FLASH,
}
Protocol_RxCode_T;

typedef Protocol_RxCode_T(*Protocol_ParseRxMeta_T)(protocol_reqid_t * p_reqId, size_t * p_rxRemaining, const void * p_rxPacket, size_t rxCount);
// seprate?
// typedef Protocol_RxCode_T(*Protocol_ParseRxLength_T)(size_t * p_rxRemaining, const void * p_rxPacket, size_t rxCount);
// typedef Protocol_RxCode_T(*Protocol_ParseRxId_T)(protocol_reqid_t * p_reqId, const void * p_rxPacket, size_t rxCount);

/******************************************************************************/
/*!
	Cmdr side check packet only
*/
/******************************************************************************/
typedef Protocol_RxCode_T(*Protocol_CheckPacket_T)(const uint8_t * p_rxPacket);

/******************************************************************************/
/*!
	Req - interface for Rx and Tx packets
	buffer once, compared to shared parse rx to packet interface, then call proc app interface and tx
*/
/******************************************************************************/
/* Stateless req, simple read write. */
typedef void (*Protocol_ProcReqResp_T)(void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

/*
	Extended stateful req - Support wait, loop, dynamic ack/nack and additional processes
	for wait process state, and template/format behavior
	process context support, flash, datagram, user provide
*/

/* Common Req from child protocol to supported general protocol control, predefined behaviors */
typedef enum Protocol_ReqCode_Tag
{
	PROTOCOL_REQ_CODE_WAIT_PROCESS, 		/* Wait ReqExt processing */
	PROTOCOL_REQ_CODE_COMPLETE,				/* Exit nonblocking wait processing state upon reception */
	PROTOCOL_REQ_CODE_AWAIT_RX_REQ_EXT,		/* Expecting Rx new packet */
	PROTOCOL_REQ_CODE_AWAIT_RX_SYNC,		/* Expecting static ack nack */
	PROTOCOL_REQ_CODE_TX_DATA,
	PROTOCOL_REQ_CODE_TX_ACK,
	PROTOCOL_REQ_CODE_TX_NACK,
	PROTOCOL_REQ_CODE_WAIT_PROCESS_EXTEND_TIMER,
	//	PROTOCOL_REQ_CODE_DATAGRAM_CONFIG,
}
Protocol_ReqCode_T;

/* Stateless req function. */
typedef Protocol_ReqCode_T (*Protocol_ProcReqRespExt_T)	(void * p_subState, void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

typedef void (*Protocol_ResetExt_T)(void * p_subState);

/*
	Static string sync - Ack Nack Protocol_BuildTxSync_T
	Dynamic ack nack string implemented using Protocol_ReqExtFunction_T
	per req sync. does not includ timeout, and user req initiated, as they do not need to be
*/
typedef const union Protocol_ReqSync_Tag
{
	struct
	{
		uint32_t TX_ACK 		: 1U; 	/* Tx Ack on Rx Req Sucess */
		uint32_t RX_ACK 		: 1U; 	/* Wait for Ack on Req Complete, Tx Response */
		uint32_t NACK_REPEAT 	: 3U; 	/* Number of repeat TxPacket on Rx Nack, Tx Nack on RxPacket error */
	};
	uint32_t ID;
}
Protocol_ReqSync_T;

#define PROTOCOL_SYNC_ID_DISABLE (0U)
#define PROTOCOL_SYNC_ID_DEFINE(UseTxAck, UseRxAck, NackRepeat) { (NackRepeat << 2U) | (UseRxAck << 1U) | (UseTxAck) }

/* Protocol_Req_T */
typedef const struct Protocol_Req_Tag
{
	const protocol_reqid_t 				ID;
	const Protocol_ProcReqResp_T 		PROC;
	const Protocol_ProcReqRespExt_T  	PROC_EXT;
	const Protocol_ReqSync_T  			SYNC;		/* Stateless ack nack */
	// Protocol_Cmdr_GetRespLength_T?
//	const uint32_t 	TIMEOUT; /* overwrite common timeout */
}
Protocol_Req_T;

#define PROTOCOL_REQ_DEFINE(ReqId, ProcReqResp, ProcExt, ReqSyncId) { .ID = (protocol_reqid_t)ReqId, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqRespExt_T)ProcExt, .SYNC = { .ID = ReqSyncId, }, }

// typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)	(Datagram_T * p_datagram, uint8_t * p_txPacket,  size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);
// typedef void (*Protocol_DatagramInit_T) (void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

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
	PROTOCOL_TX_SYNC_ABORT,
	PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT,
	PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT,
	PROTOCOL_TX_SYNC_NACK_PACKET_ERROR,
	PROTOCOL_TX_SYNC_ACK_REQ_EXT,
	PROTOCOL_TX_SYNC_NACK_REQ_EXT,
}
Protocol_TxSyncId_T;

typedef void (* const Protocol_BuildTxSync_T)(uint8_t * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId);

/*
	Packet Format Specs
*/
typedef const struct Protocol_Specs_Tag
{
	const uint8_t RX_LENGTH_MIN;			/* Rx Header Length, Rx this many bytes before calling PARSE_RX */
	const uint8_t RX_LENGTH_MAX;
	const uint32_t RX_TIMEOUT;				/* Reset cumulative per packet, until user reset RX_TIMEOUT_PACKET */
	// const uint32_t RX_TIMEOUT_BYTE;	 	/* Reset per byte */

	const void * const P_REQ_TABLE; 				/* Protocol_Req_T or Protocol_Cmdr_Req_T */
	const uint8_t REQ_TABLE_LENGTH;
	const uint32_t REQ_TIMEOUT; 					/* checked for stateful Req only */
	const Protocol_ResetExt_T REQ_EXT_RESET; 		/* Alternatively, handle in req by user */

	const Protocol_BuildTxSync_T BUILD_TX_SYNC;		/* Build Sync Packets */

	/* Controller side only */
	const Protocol_ParseRxMeta_T PARSE_RX_META;		/* Parse for RxReqId and RxRemaining, and check data */

	/* Cmdr side only */
	const Protocol_CheckPacket_T CHECK_PACKET;

	/* Optional */
	const uint32_t RX_START_ID; 	/* set to 0x00 or 0xff by default for bus idle state */
	const uint32_t RX_END_ID;
	const bool ENCODED;				/* TODO Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */
	const uint32_t BAUD_RATE_DEFAULT;
	Protocol_ReqSync_T SYNC;  		/* Rx Packet Nack, default statless sync, timeout nack */
	// const uint32_t RX_HEADER_LENGTH; fixed header length known to include contain data length value
}
Protocol_Specs_T;

typedef enum Protocol_RxState_Tag
{
	PROTOCOL_RX_STATE_INACTIVE,
	PROTOCOL_RX_STATE_WAIT_BYTE_1,
	PROTOCOL_RX_STATE_WAIT_PACKET,
	PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
}
Protocol_RxState_T;

typedef enum Protocol_ReqState_Tag
{
	PROTOCOL_REQ_STATE_INACTIVE,
	PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID,
	PROTOCOL_REQ_STATE_WAIT_RX_REQ_EXT, /* Wait for next ReqExt packet in stateful routine */
	PROTOCOL_REQ_STATE_WAIT_RX_SYNC,
	PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL,
	PROTOCOL_REQ_STATE_WAIT_PROCESS,
}
Protocol_ReqState_T;

typedef struct __attribute__((aligned(4U))) Protocol_Params_Tag
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	uint8_t XcvrId;
#endif
	uint8_t SpecsId;
	uint32_t RxLostTime;
	bool IsEnableOnInit; 	/* enable on start up */
}
Protocol_Params_T;

/*

*/
typedef const struct Protocol_Config_Tag
{
	uint8_t * const P_RX_PACKET_BUFFER;
	uint8_t * const P_TX_PACKET_BUFFER;
	const uint8_t PACKET_BUFFER_LENGTH; 		/* Must be greater than Specs RX_LENGTH_MAX */
	void * const P_APP_INTERFACE;			 	/* User app context for packet processing */
	void * const P_SUBSTATE_BUFFER; 			/* Child protocol control variables, may be seperate from app_interface, must be largest enough to hold substate context from specs */
	const Protocol_Specs_T * const * const PP_SPECS_TABLE; 	/* bound and verify specs selection */
	const uint8_t SPECS_COUNT;
	const volatile uint32_t * const P_TIMER;
	const Protocol_Params_T * const P_PARAMS;
}
Protocol_Config_T;

#if defined(CONFIG_PROTOCOL_XCVR_ENABLE)
#define _PROTOCOL_XCVR_DEFINE(p_XcvrTable, TableLength) .Xcvr = XCVR_DEFINE(p_XcvrTable, TableLength),
#else
#define _PROTOCOL_XCVR_DEFINE(p_XcvrTable, TableLength)
#endif

#define PROTOCOL_DEFINE(p_RxBuffer, p_TxBuffer, PacketBufferLength, p_AppInterface, p_SubstateBuffer, p_SpecsTable, SpecsCount, p_XcvrTable, XcvrCount, p_Timer, p_Params)	\
{																\
	.CONFIG = 													\
	{															\
		.P_RX_PACKET_BUFFER 	= p_RxBuffer,					\
		.P_TX_PACKET_BUFFER 	= p_TxBuffer,					\
		.PACKET_BUFFER_LENGTH 	= PacketBufferLength,			\
		.P_APP_INTERFACE 		= p_AppInterface,				\
		.P_SUBSTATE_BUFFER 		= p_SubstateBuffer,				\
		.PP_SPECS_TABLE 		= p_SpecsTable,					\
		.SPECS_COUNT 			= SpecsCount,					\
		.P_TIMER 				= p_Timer,						\
		.P_PARAMS 				= p_Params,						\
	},															\
	_PROTOCOL_XCVR_DEFINE(p_XcvrTable, XcvrCount)	 			\
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

	size_t RxIndex; 				/* AKA RxCount */
	size_t RxRemaining; 			/* Alternatively use parse for total rxpacket length */
	Protocol_RxCode_T RxCode;		/* Save RxCode as it contrains Sync Ids, handles Sync Packets */
	Protocol_RxState_T RxState;
	uint32_t RxTimeStart;

	Protocol_ReqState_T ReqState;
	protocol_reqid_t ReqIdActive;
	Protocol_Req_T * p_ReqActive;	/* Protocol_Req_T or Protocol_Cmdr_Req_T */
	uint32_t ReqTimeStart;			/* Set on Req Start and Complete */

	size_t TxLength;
	uint8_t NackCount;
}
Protocol_T;

extern void Protocol_Init(Protocol_T * p_protocol);

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
extern void Protocol_Proc(Protocol_T * p_protocol);
extern void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate);
extern void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId);
#endif
extern bool Protocol_CheckRxLost(Protocol_T * p_protocol);

extern void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t specsId);
extern bool Protocol_Enable(Protocol_T * p_protocol);
extern void Protocol_Disable(Protocol_T * p_protocol);
extern void Protocol_EnableOnInit(Protocol_T * p_protocol);
extern void Protocol_DisableOnInit(Protocol_T * p_protocol);

#endif

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
// Protocol_ArgsInterface_T;
