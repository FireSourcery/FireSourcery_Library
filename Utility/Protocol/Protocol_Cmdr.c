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
// #include <string.h>

bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	bool isSucess = false;
	Protocol_Req_T * p_req;

	if(p_protocol->RxState == PROTOCOL_RX_STATE_INACTIVE)
	{
		p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
		p_protocol->ReqIdActive = cmdId;
		p_protocol->RxTimeStart = *p_protocol->CONFIG.P_TIMER;

		p_req = _Protocol_SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, cmdId);

		if(p_req != 0U)
		{
			// p_req->BUILD_REQ(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, &p_protocol->RxRemaining, p_protocol->CONFIG.P_APP_INTERFACE);

			//alternatively
			p_protocol->TxLength = p_req->BUILD_REQ(p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->CONFIG.P_APP_INTERFACE);
			// p_protocol->RxRemaining = p_req->GET_RESP_LENGTH(p_protocol->CONFIG.P_APP_INTERFACE, );
			isSucess = true;
		}
	}

	return isSucess;
}

bool _Protocol_Cmdr_StartReq_Resp(Protocol_T * p_protocol, protocol_reqid_t cmdId, size_t respLength)
{
	bool isSucess = _Protocol_Cmdr_StartReq(p_protocol, cmdId);
	if(isSucess == true) { p_protocol->RxRemaining = respLength; }
	return isSucess;
}
/* return true if time out */
bool _Protocol_Cmdr_CheckTimeout(Protocol_T * p_protocol)
{
	return (*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->p_Specs->RX_TIMEOUT);
}

Protocol_RxCode_T _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol)
{
	Protocol_RxCode_T rxStatus = p_protocol->p_Specs->PARSE_RX(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_RX_PACKET_BUFFER);
	if(rxStatus == PROTOCOL_RX_CODE_RESP_DATA_SUCCESS) { p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE; }
	return rxStatus;
}



/*
	Master Mode, sequential Cmd/Rx
	non blocking, single threaded only,
*/
void Protocol_Cmdr_ProcRx(Protocol_T * p_protocol)
{
	// Protocol_RxCode_T rxStatus = 0U ;

	switch(p_protocol->RxState)
	{
		case PROTOCOL_RX_STATE_INACTIVE: break;

		// case PROTOCOL_STATE_SEND_CMD: break;

		case PROTOCOL_RX_STATE_WAIT_PACKET:
			if(*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
			{
				// if(BuildRxPacket(p_protocol) == PROTOCOL_RX_CODE_RESP_DATA_SUCCESS)  //todo //if(CheckIsRxPacketComplete(p_protocol) == true)
				// {

					//stateful wait
					// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL;

					//stateless can proc right away
					// if(_Protocol_Cmdr_ParseResp(p_protocol) == PROTOCOL_RX_CODE_ERROR_PACKET_DATA)
					// {
					// 	// if use nack resend
					// }
				// }
			}
			else
			{
				p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
				// p_protocol->State = PROTOCOL_STATE_SEND_CMD;
				//send txtimeout
			}

			break;

		// case PROTOCOL_STATE_CMD_RESPONSE:
		case PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL:

			break;


		default: break;
	}

		// switch(p_protocol->ReqState) //todo stateful

}

/*
	No overwrite existing
*/
void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	if(_Protocol_Cmdr_StartReq( p_protocol, cmdId) == true)
	{
		_Protocol_TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
	}
}

void Protocol_Cmdr_StartReq_Resp(Protocol_T * p_protocol, protocol_reqid_t cmdId, size_t respLength)
{
	if(_Protocol_Cmdr_StartReq_Resp( p_protocol, cmdId, respLength) == true)
	{
		_Protocol_TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
	}
}