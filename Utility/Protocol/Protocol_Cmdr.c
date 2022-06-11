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
	@file 	Protocol.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Protocol_Cmdr.h"


/*
	3 Implied priority levels

	cannot distinguish StartReq_Background 	begins only if PROTOCOL_REQ_STATE_INACTIVE
	StartReq				begins if no sync is active
	StartReq_Overwrite 		begins always, overwrite wait for sync

	Resp validated using ParseRxMeta, independent from Req_GetRespLength

	sync reqs wait for resp, overwrite with _Protocol_Cmdr_StartReq_Overwrite
	no sync reqs, handle RespPacket idependently

	alternatively, resp Queue, match RespLength
*/
static bool StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	Protocol_Req_T * p_req = _Protocol_SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, cmdId);
	bool isSucess = p_req != 0U;

	if(isSucess == true)
	{
		p_protocol->p_Specs->CMDR_BUILD_TX_REQ(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_APP_INTERFACE, cmdId);

		// p_req->CMDR_BUILD_REQ(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_APP_INTERFACE);
		//if resp length is known, and wait sync, PROTOCOL_RX_STATE_WAIT_PACKET, skip wait byte one

		if((p_req->PROC_EXT != 0U) || (p_req->SYNC.RX_ACK == true))
		{
			p_protocol->p_ReqActive = p_req;
			p_protocol->ReqIdActive = cmdId;
		}

		if(p_req->PROC_EXT != 0U)
		{
			// if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
			if(p_req->SYNC.RX_ACK == true) 	{ p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC; }
			else 							{ p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_EXT; }

		}
		else if(p_req->SYNC.RX_ACK == true) 	{ p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL; }
		else 									{ p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID; }

		//set inactive or wait id, cannot distinguish, without queue count. only ditinguish sync wait and non sync
		p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;
	}

	return isSucess;
}

/* Does not start state if sync is active */
// bool _Protocol_Cmdr_StartReq_Background(Protocol_T * p_protocol, protocol_reqid_t cmdId)
// {
// 	return (p_protocol->ReqState == PROTOCOL_REQ_STATE_INACTIVE) ? StartReq(p_protocol, cmdId) : false;
// }

// /* Does not start if ReqState WAIT_SYNC WAIT_EXT */
bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return ((p_protocol->ReqState == PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID)) ? StartReq(p_protocol, cmdId) : false;
	//(p_protocol->ReqState == PROTOCOL_REQ_STATE_INACTIVE) ||
}

/*
	overwrite sync wait
	Tx new Req before Rx Reponse to previous Req
	Prioir Resp will be discarded
*/
bool _Protocol_Cmdr_StartReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return StartReq(p_protocol, cmdId);
}

/*
	No overwrite existing
*/
void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	if(_Protocol_Cmdr_StartReq(p_protocol, cmdId) == true) { Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength); }
}

void Protocol_Cmdr_StartReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	if(_Protocol_Cmdr_StartReq_Overwrite(p_protocol, cmdId) == true) { Xcvr_TxN(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength); }
}

bool Protocol_Cmdr_CheckTxIdle(Protocol_T * p_protocol)
{
	return (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Params.WatchdogTime); //(p_protocol->ReqState == PROTOCOL_REQ_STATE_INACTIVE) &&
}


// /*
// 	Commander Mode, sequential Cmd/Rx
// 	non blocking, single threaded only

// 	Resp handled independent of Req if Sync is not used.
// 	i.e. Resp recieved with no prior Req, or out of sequence Req, will be parsed using contents of Resp packet
// */
// void Protocol_Cmdr_Proc(Protocol_T * p_protocol)
// {
// 	Protocol_Proc(p_protocol);

// 	//optionally, match txreq count vs rx proc count to determine inactive
// }

// size_t _Protocol_Cmdr_BuildTxReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
// {
// 	return (_Protocol_Cmdr_StartReq(p_protocol, cmdId) == true) ? p_protocol->TxLength : 0U;
// }

// size_t _Protocol_Cmdr_BuildTxReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId)
// {
// 	return (StartReq(p_protocol, cmdId) == true) ? p_protocol->TxLength : 0U;
// }

// /*!
// 	@return true if time out
// */
// bool _Protocol_Cmdr_PollTimeout(Protocol_T * p_protocol)
// {
// 	bool isTimeout = (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT);
// 	if(isTimeout == true) { p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE; }
// 	return isTimeout;
// }

// /*!
// 	@return true if CRC/Checksum matches
// 	Using known resp length.

// 	Alternatively,
// 		BuildRxPacket(p_protocol), handles out of sequence packets
// 		Resp queue
// */
// bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol)
// {
// 	// p_protocol->RxCode = p_protocol->p_Specs->CHECK_PACKET(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->ReqIdActive);
// 	//change to handle with build
// 	// BuildRxPacket(p_protocol);
// 	// if Req return == ReqIdActive,
// 	bool isSuccess = false;

// 	if(p_protocol->RxCode == PROTOCOL_RX_CODE_PACKET_COMPLETE)
// 	{
// 		((Protocol_Cmdr_Req_T *)p_protocol->p_CmdrReqActive)->PARSE_RESP(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_RX_PACKET_BUFFER);
// 		// parse packet may need to check packet seqeunce correctness p_protocol->ReqIdActive

// 		// Proc ReqResp
// 		// p_protocol->p_CmdrReqActive->PROC
// 		// (
// 		// 	p_protocol->CONFIG.P_APP_INTERFACE,
// 		// 	p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
// 		// 	p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
// 		// );

// 		// p_protocol->TimeStart = *p_protocol->CONFIG.P_TIMER; //restart watch tx idle
// 		p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
// 		isSuccess = true;
// 	}
// 	else if(p_protocol->RxCode == PROTOCOL_RX_CODE_PACKET_ERROR)
// 	{
// 		//todo nack
// 		// if (p_protocol->NackCount < p_protocol->p_Specs->SYNC.NACK_REPEAT)
// 		// {
// 		// 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
// 		// }
// 		// else
// 		{
// 			p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
// 		}
// 	}

// 	return isSuccess;
// }
