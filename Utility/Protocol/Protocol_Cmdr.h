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
*/
/* If RxPacket contains length */
// typedef void (*Protocol_Cmdr_BuildReq_T)(uint8_t * p_txPacket, size_t * p_txLength, size_t * p_rxRemainig, const void * p_appInterface);
// typedef size_t (*Protocol_Cmdr_BuildReq_T)(uint8_t * p_txPacket, const void * p_appInterface);
// typedef void (*Protocol_Cmdr_GetRespLength_T)(const void * p_appInterface, protocol_reqid_t id); //skip checking resp packet, if available, return 255 for variable return length?
// typedef void (*Protocol_Cmdr_ParseResp_T)(void * p_appInterface, const uint8_t * p_rxPacket); //uint8_t RxCount, include length in signiture if, length is not included in packet
/* One function handle all cases, alternatively use function table */
// typedef void (*Protocol_Cmdr_BuildReq_T)(uint8_t * p_txPacket, size_t * p_txLength, size_t * p_rxRemainig, const void * p_appInterface, protocol_reqid_t reqId);



/*
	Tx Packet using Protocol Xcvr
*/
static inline void _Protocol_Cmdr_TxPacket(Protocol_T * p_protocol)
{

}

/*
	User app handle Tx
*/
// static inline uint8_t * Protocol_Cmdr_GetPtrTxPacket(Protocol_T * p_protocol) { return p_protocol->CONFIG.P_TX_PACKET_BUFFER; }
static inline size_t Protocol_Cmdr_GetReqLength(Protocol_T * p_protocol) { return p_protocol->TxLength; }
static inline size_t Protocol_Cmdr_GetRespRemaining(Protocol_T * p_protocol) { return p_protocol->RxRemaining; }


/*
	extern
*/
extern bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
extern bool _Protocol_Cmdr_StartReq_Resp(Protocol_T * p_protocol, protocol_reqid_t cmdId, size_t respLength);
extern void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
extern void Protocol_Cmdr_StartReq_Resp(Protocol_T * p_protocol, protocol_reqid_t cmdId, size_t respLength);
extern Protocol_RxCode_T _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol);
extern Protocol_RxCode_T Protocol_Cmdr_ProcRx(Protocol_T * p_protocol);

#endif

