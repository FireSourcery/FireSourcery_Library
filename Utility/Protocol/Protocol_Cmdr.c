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

	for(uint8_t iChar = 0U; iChar < tableLength; iChar++)
	{
		if(p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; }
	}

	return p_req;
}

bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	bool isSucess = false;

	if(p_protocol->RxState == PROTOCOL_RX_STATE_INACTIVE)
	{
		p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
		p_protocol->ReqIdActive = cmdId;
		p_protocol->RxTimeStart = *p_protocol->CONFIG.P_TIMER;
		p_protocol->p_ReqActive = SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, cmdId);

		if(p_protocol->p_ReqActive != 0U)
		{
			((Protocol_Cmdr_Req_T *)p_protocol->p_ReqActive)->BUILD_REQ(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, &p_protocol->RxRemaining, p_protocol->CONFIG.P_APP_INTERFACE);

			//alternatively
			// p_protocol->TxLength = p_req->BUILD_REQ(p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->CONFIG.P_APP_INTERFACE);
			// p_protocol->RxRemaining = p_req->GET_RESP_LENGTH(p_protocol->CONFIG.P_APP_INTERFACE, cmdId);
			isSucess = true;
		}
	}

	return isSucess;
}

/* return true if time out */
bool _Protocol_Cmdr_PollTimeout(Protocol_T * p_protocol)
{
	bool isTimeout = (*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->p_Specs->RX_TIMEOUT);
	if(isTimeout == true) { p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE; }
	return isTimeout;
}

/*!
	@return Protocol_RxCode_T PROTOCOL_RX_CODE_RESP_DATA_SUCCESS or PROTOCOL_RX_CODE_ERROR_PACKET
*/
bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol)
{
	// Protocol_RxCode_T rxStatus = p_protocol->p_Specs->CHECK_PACKET(p_protocol->CONFIG.P_RX_PACKET_BUFFER);

	// if(rxStatus == PROTOCOL_RX_CODE_RESP_DATA_SUCCESS)
	// {
	// 	((Protocol_Cmdr_Req_T *)p_protocol->p_ReqActive)->PARSE_RESP(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_RX_PACKET_BUFFER);
	// 	p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
	// }

	// return rxStatus;
	p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
	return PROTOCOL_RX_CODE_COMPLETE;
}

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
/*
	Master Mode, sequential Cmd/Rx
	non blocking, single threaded only
*/
void Protocol_Cmdr_ProcRx(Protocol_T * p_protocol)
{
	// Protocol_RxCode_T rxStatus = 0U ;

	switch(p_protocol->RxState)
	{
		case PROTOCOL_RX_STATE_INACTIVE: break;

		// case PROTOCOL_STATE_SEND_CMD: break;

		case PROTOCOL_RX_STATE_WAIT_PACKET:
			if(_Protocol_Cmdr_CheckTimeout(p_protocol) == false)  /* No need to check for overflow if using millis */
			{

				if(Xcvr_RxPacket(&p_protocol->Xcvr, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxRemaining) == true)
				{
					//stateless proc immediately
					if(_Protocol_Cmdr_ParseResp(p_protocol) == PROTOCOL_RX_CODE_ERROR)
					{
						// if use nack resend
					}
				}

				//stateful wait use Req States
				// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL;

			}
			else
			{
				//send txtimeout
			}

			break;

		// case PROTOCOL_STATE_CMD_RESPONSE:
		case PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL: break;
		default: break;
	}

	// switch(p_protocol->ReqState) //todo stateful
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


// bool _Protocol_Cmdr_StartReq_Resp(Protocol_T * p_protocol, protocol_reqid_t cmdId, size_t respLength)
// {
// 	bool isSucess = _Protocol_Cmdr_StartReq(p_protocol, cmdId);
// 	if(isSucess == true) { p_protocol->RxRemaining = respLength; }
// 	return isSucess;
// }

// void Protocol_Cmdr_StartReq_Resp(Protocol_T * p_protocol, protocol_reqid_t cmdId, size_t respLength)
// {
// 	if(_Protocol_Cmdr_StartReq_Resp(p_protocol, cmdId, respLength) == true)
// 	{
// 		_Protocol_TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
// 	}
// }