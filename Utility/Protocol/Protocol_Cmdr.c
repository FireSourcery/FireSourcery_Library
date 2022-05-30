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

		//todo if p_protocol->RxRemaining > 0U || p_ReqActive.SYNC
		p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
		p_protocol->ReqIdActive = cmdId;
		p_protocol->ReqTimeStart = *p_protocol->CONFIG.P_TIMER;

		isSucess = true;
	}

	return isSucess;
}

bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return (p_protocol->RxState == PROTOCOL_RX_STATE_INACTIVE) ? StartReq(p_protocol, cmdId) : false;
}

size_t _Protocol_Cmdr_BuildTxReq(Protocol_T * p_protocol, protocol_reqid_t cmdId)
{
	return (_Protocol_Cmdr_StartReq(p_protocol, cmdId) == true) ? p_protocol->TxLength : 0U;
}

/*
	Tx new Req before Rx Reponse to previous Req
	Prioir Resp will be discarded
*/
//todo move overwrite to req table?
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
	if(isTimeout == true) { p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE; }
	return isTimeout;
}

/*!
	@return true if CRC/Checksum matches
*/
bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol)
{
	Protocol_RxCode_T rxStatus = p_protocol->p_Specs->CHECK_PACKET(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->ReqIdActive); //change to crc only?
	bool isSuccess = false;

	if(rxStatus == PROTOCOL_RX_CODE_COMPLETE)
	{
		((Protocol_Cmdr_Req_T *)p_protocol->p_ReqActive)->PARSE_RESP(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_RX_PACKET_BUFFER); // parse packet may need to check packet seqeunce correctness p_protocol->ReqIdActive
		// p_protocol->TimeStart = *p_protocol->CONFIG.P_TIMER; //restart watch tx idle
		p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
		isSuccess = true;
	}
	else if(rxStatus == PROTOCOL_RX_CODE_ERROR)
	{
		//todo nack
		// if (p_protocol->NackCount < p_protocol->p_Specs->SYNC.NACK_REPEAT)
		// {
		// 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
		// }
		// else
		{
			p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
		}
	}

	return isSuccess;
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
					/* Stateless proc immediately */
					if(_Protocol_Cmdr_ParseResp(p_protocol) == PROTOCOL_RX_CODE_ERROR)
					{
						// if use nack resend
					}
				}

				/* Stateful wait use Req States */
				// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL;

			}
			else
			{
				//send txtimeout
				// ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_ID);
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

bool Protocol_Cmdr_CheckTxIdle(Protocol_T * p_protocol)
{
	return (p_protocol->RxState == PROTOCOL_RX_STATE_INACTIVE) && (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Params.RxLostTime);
}