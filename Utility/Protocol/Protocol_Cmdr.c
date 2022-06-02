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

static const Protocol_Cmdr_Req_T * SearchReqTable(Protocol_Cmdr_Req_T * p_reqTable, size_t tableLength, protocol_reqid_t id)
{
	const Protocol_Cmdr_Req_T * p_req = 0U;
	for(uint8_t iChar = 0U; iChar < tableLength; iChar++) { if(p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; } }
	return p_req;
}

static bool StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	bool isSucess = false;
	// Protocol_Cmdr_Req_T * p_reqActive;	/* todo pointer type */
	p_protocol->p_ReqActive = (void *)SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, cmdId);

	if(p_protocol->p_ReqActive != 0U)
	{
		((Protocol_Cmdr_Req_T *)p_protocol->p_ReqActive)->BUILD_REQ
		(
			p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, &p_protocol->RxRemaining, p_protocol->CONFIG.P_APP_INTERFACE
		);

		//sync reqs wait for resp, overwrite with _Protocol_Cmdr_StartReq_Overwrite
		//no sync reqs, handle RespPacket idependently
		if(((Protocol_Cmdr_Req_T *)p_protocol->p_ReqActive)->SYNC.ID != 0U)
		{
			p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID; //cmdr req states? //PROTOCOL_REQ_STATE_WAIT_RX_REQ_EXT
			p_protocol->ReqIdActive = cmdId;

			//todo set p_protocol->RxRemaining == resp or ack or 0?
		}
		else
		{
			p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE; //PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID
		}

		p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;

		isSucess = true;
	}

	return isSucess;
}

/* Does not start state if sync is active */
bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return (p_protocol->ReqState == PROTOCOL_REQ_STATE_INACTIVE) ? StartReq(p_protocol, cmdId) : false; //PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID
}

size_t _Protocol_Cmdr_BuildTxReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return (_Protocol_Cmdr_StartReq(p_protocol, cmdId) == true) ? p_protocol->TxLength : 0U;
}


// tod resp Queue or priority overwrite status? overwrite status still invalidate active req response

/*
	overwrite sync wait
	Tx new Req before Rx Reponse to previous Req
	Prioir Resp will be discarded
*/
bool _Protocol_Cmdr_StartReq_Overwrite(Protocol_T *p_protocol, protocol_reqid_t cmdId)
{
	return StartReq(p_protocol, cmdId);
}

size_t _Protocol_Cmdr_BuildTxReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return (StartReq(p_protocol, cmdId) == true) ? p_protocol->TxLength : 0U;
}

/*!
	@return true if time out
*/
bool _Protocol_Cmdr_PollTimeout(Protocol_T * p_protocol)
{
	bool isTimeout = (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT);
	if(isTimeout == true) { p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE; }
	return isTimeout;
}

/*!
	@return true if CRC/Checksum matches
	Using known resp length.

	Alternatively,
		BuildRxPacket(p_protocol), handles out of sequence packets
		Resp queue
*/
bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol)
{
	p_protocol->RxCode = p_protocol->p_Specs->CHECK_PACKET(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->ReqIdActive);
	//change to handle with build
	// BuildRxPacket(p_protocol);
	// if Req return == ReqIdActive,
	bool isSuccess = false;

	if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE)
	{
		((Protocol_Cmdr_Req_T *)p_protocol->p_ReqActive)->PARSE_RESP(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_RX_PACKET_BUFFER);
		// parse packet may need to check packet seqeunce correctness p_protocol->ReqIdActive

		// Proc ReqResp
		// p_protocol->p_ReqActive->PROC
		// (
		// 	p_protocol->CONFIG.P_APP_INTERFACE,
		// 	p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
		// 	p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
		// );

		// p_protocol->TimeStart = *p_protocol->CONFIG.P_TIMER; //restart watch tx idle
		p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
		isSuccess = true;
	}
	else if(p_protocol->RxCode == PROTOCOL_RX_CODE_ERROR)
	{
		//todo nack
		// if (p_protocol->NackCount < p_protocol->p_Specs->SYNC.NACK_REPEAT)
		// {
		// 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
		// }
		// else
		{
			p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
		}
	}

	return isSuccess;
}


#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
/*
	Master Mode, sequential Cmd/Rx
	non blocking, single threaded only

	Resp handled independent of Req if Sync is not used.
	i.e. Resp recieved with no prior Req, or out of sequence Req, will be parsed using contents of Resp packet
*/
void Protocol_Cmdr_ProcRx(Protocol_T * p_protocol)
{
	// _Protocol_ProcRxState(p_protocol);
	// _Protocol_ProcReqState(p_protocol);
	// if SYNC.respId == true
	// if  PARSE_RX_META Req return CmdrReqRespId == ReqIdActive,
	//proc parse resp
	// proc resp indepedent of activereq
}

/*
	No overwrite existing
*/
void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	if(_Protocol_Cmdr_StartReq(p_protocol, cmdId) == true)
	{
		Xcvr_TxPacket(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
	}
}
#endif

bool Protocol_Cmdr_CheckTxIdle(Protocol_T * p_protocol)
{
	return (p_protocol->RxState == PROTOCOL_RX_STATE_INACTIVE) && (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Params.WatchdogTime);
}