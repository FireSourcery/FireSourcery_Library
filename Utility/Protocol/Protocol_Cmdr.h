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
#ifndef PROTOCOL_CMDR_H
#define PROTOCOL_CMDR_H

#include "Protocol.h"

/*
	User provide functions to convert between packet format and appInterface format
	variable length reponse?
*/
typedef void (*Protocol_Cmdr_BuildReq_T)(uint8_t * p_txPacket, size_t * p_txLength, size_t * p_rxRespLength, const void * p_appInterface);
typedef void (*Protocol_Cmdr_ParseResp_T)(void * p_appInterface, const uint8_t * p_rxPacket); /* If p_rxPacket contains length. Include uint8_t RxCount in signiture otherwise. */
typedef Protocol_ReqCode_T (*Protocol_Cmdr_BuildReqExt_T)(void * p_subState, uint8_t * p_txPacket, size_t * p_txLength, size_t * p_rxRespLength, const void * p_appInterface);
typedef Protocol_ReqCode_T (*Protocol_Cmdr_ParseRespExt_T)(void * p_subState, void * p_appInterface, const uint8_t * p_rxPacket);

/* Alternatively, One function handle all cases, if not using function table */
// typedef void (*Protocol_Cmdr_BuildReq_T)(uint8_t * p_txPacket, size_t * p_txLength, size_t * p_rxRemainig, const void * p_appInterface, protocol_reqid_t reqId);

/*
	Protocol_Cmdr_Req_T
*/
typedef const struct Protocol_Cmdr_Req_Tag
{
	const protocol_reqid_t 				ID;
	const Protocol_Cmdr_BuildReq_T 		BUILD_REQ;
	const Protocol_Cmdr_ParseResp_T 	PARSE_RESP;
	const Protocol_ReqSync_T  			SYNC;	/* Stateless ack nack */
//	const uint32_t 	TIMEOUT; /* overwrite common timeout */
}
Protocol_Cmdr_Req_T;

#define PROTOCOL_CMDR_REQ_DEFINE(ReqId, BuildReq, ParseResp, ReqSyncId) { .ID = (protocol_reqid_t)ReqId, .BUILD_REQ = (Protocol_Cmdr_BuildReq_T)BuildReq, .PARSE_RESP = (Protocol_Cmdr_ParseResp_T)ParseResp, .SYNC = { .ID = ReqSyncId, }, }

/*
	User app handle Tx
*/
static inline size_t Protocol_Cmdr_GetReqLength(Protocol_T * p_protocol) { return p_protocol->TxLength; }
static inline size_t Protocol_Cmdr_GetRespRemaining(Protocol_T * p_protocol) { return p_protocol->RxRemaining; }

/*
	extern
*/
extern bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
extern size_t _Protocol_Cmdr_BuildTxReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
extern bool _Protocol_Cmdr_PollTimeout(Protocol_T * p_protocol);
extern bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol);

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
/*
	Tx Packet using Protocol Xcvr
*/
extern void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
extern void Protocol_Cmdr_ProcRx(Protocol_T * p_protocol);
#endif

extern bool Protocol_Cmdr_CheckTxIdle(Protocol_T * p_protocol);

#endif