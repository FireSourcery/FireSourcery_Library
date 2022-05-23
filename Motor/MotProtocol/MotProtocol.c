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
	@file 	MotPacket.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotProtocol.h"

#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*!
	Common functions mapping directly to Protocol Specs
*/
/******************************************************************************/
void MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId)
{
	MotPacket_HeaderId_T syncId;

	switch(txId)
	{
		case PROTOCOL_TX_SYNC_ACK_REQ_ID:			syncId = MOTPROTOCOL_SYNC_ACK; 		break;
		case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:	syncId = MOTPROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_NACK_REQ_ID:			syncId = MOTPROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_ACK_DATA:				syncId = MOTPROTOCOL_SYNC_ACK;		break;
		case PROTOCOL_TX_SYNC_NACK_DATA:			syncId = MOTPROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_ABORT:				syncId = MOTPROTOCOL_SYNC_ABORT;	break;
		default: *p_txSize = 0U; syncId = 0xFFU; break;
	}

	*p_txSize = MotPacket_Sync_Build(p_txPacket, syncId);
}

void MotProtocol_ResetExt(MotProtocol_Substate_T * p_subState)
{
	p_subState->StateId = 0U;
}



/******************************************************************************/
/*!
	Ctrl side
*/
/******************************************************************************/
/******************************************************************************/
/*! RxParser */
/******************************************************************************/
Protocol_RxCode_T MotProtocol_ParseRxMeta(protocol_reqid_t * p_reqId, size_t * p_rxRemaining, const MotPacket_T * p_rxPacket, size_t rxCount)
{
	Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_WAIT_PACKET;

	if(rxCount >= p_rxPacket->Header.Length + sizeof(MotPacket_Header_T)) // if(rxCount >= 3U) && (rxCount >= p_rxPacket->Header.Length + sizeof(MotPacket_Header_T))
	{
		*p_reqId = p_rxPacket->Header.TypeId;
		rxCode = (MotPacket_CheckChecksum(p_rxPacket)) ? PROTOCOL_RX_CODE_COMPLETE : PROTOCOL_RX_CODE_ERROR;

		// optionally further refine cmd
		// if(rxCount >= p_rxPacket->Header.Length + sizeof(MotPacket_Header_T))
		// {
		// 	switch(p_rxPacket->Header.TypeId)
		// 	{
		// 		case MOTPROTOCOL_CMD_MONITOR_TYPE: break;
		// 		case MOTPROTOCOL_CMD_CONTROL_TYPE: break;
		// 		default:  break;
		// 	}
		// 	*p_reqId = p_rxPacket->Header.TypeId;
		// 	rxCode = (Packet_CalcChecksum(p_rxPacket) == p_rxPacket->Header.Crc) ? PROTOCOL_RX_CODE_REQ_ID_SUCCESS : PROTOCOL_RX_CODE_ERROR_PACKET_DATA;
		// }
	}
	else if(rxCount >= 3U) /* Move this to protocol module handle, if header length defined */
	{
		*p_rxRemaining = p_rxPacket->Header.Length + sizeof(MotPacket_Header_T) - rxCount;
		rxCode = PROTOCOL_RX_CODE_WAIT_PACKET_REMAINING;
	}
	else if(rxCount >= MOTPROTOCOL_PACKET_LENGTH_MIN)
	{
		switch(p_rxPacket->Header.TypeId)
		{
			case MOTPROTOCOL_STOP_MOTORS:	rxCode = PROTOCOL_RX_CODE_COMPLETE; *p_reqId = MOTPROTOCOL_STOP_MOTORS;		break;
			case MOTPROTOCOL_PING:			rxCode = PROTOCOL_RX_CODE_COMPLETE; *p_reqId = MOTPROTOCOL_PING;			break;
			case MOTPROTOCOL_SYNC_ACK:		rxCode = PROTOCOL_RX_CODE_ACK; 				break;
			case MOTPROTOCOL_SYNC_NACK:		rxCode = PROTOCOL_RX_CODE_NACK; 			break;
			case MOTPROTOCOL_SYNC_ABORT:	rxCode = PROTOCOL_RX_CODE_ABORT; 			break;
			default: break;
		}
	}

	return rxCode;
}


/******************************************************************************/
/*!
	Using Packet Interface
	Packet Interface map to Protocol Module
	Protocol Module function types
*/
/******************************************************************************/

/******************************************************************************/
/*!
	Cmdr side
*/
/******************************************************************************/
//cmdr side uses PROTOCOL_RX_CODE_RESP_DATA_SUCCESS
Protocol_RxCode_T MotProtocol_CheckPacket(const MotPacket_T * p_rxPacket) //general version might need include rxlength
{
	return (MotPacket_CheckChecksum(p_rxPacket) == true) ? PROTOCOL_RX_CODE_COMPLETE : PROTOCOL_RX_CODE_ERROR;
}


/*
	Note: casting uint_t return type to size_t
	althought map to size_t, calling function in context to read return as int8_t

	MotPacket_HeaderId_T is cast to uint8_t
*/
//include response length function in table
// static const Protocol_Req_T CMDR_REQ_TABLE[] =
// {
// 	PROTOCOL_REQ_DEFINE(MOTPROTOCOL_CMD_MONITOR_TYPE, 	MotPacket_ReqPacket_Monitor_Build, 	PROTOCOL_SYNC_ID_DISABLE),
// 	PROTOCOL_REQ_DEFINE(MOTPROTOCOL_CMD_CONTROL_TYPE, 	MotPacket_ReqPacket_Control_Build, 	PROTOCOL_SYNC_ID_DISABLE),
// };

// const Protocol_Specs_T MOT_PROTOCOL_CMDR_SPECS =
// {
// 	.RX_TIMEOUT 	= MOTPROTOCOL_TIMEOUT_MS,
// 	// const uint32_t RX_TIMEOUT_BYTE;		//reset per byte
// 	// const uint32_t RX_TIMEOUT_RX;
// 	// const uint32_t RX_TIMEOUT_REQ;
// 	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
// 	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

// 	.P_REQ_TABLE 		= &CMDR_REQ_TABLE[0U],
// 	.REQ_TABLE_LENGTH 	= sizeof(CMDR_REQ_TABLE)/sizeof(Protocol_Req_T),
// 	.REQ_EXT_RESET 		= 0U,
// 	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,
// 	.PARSE_RX 			= (Protocol_ParseRxPacket_T)Cmdr_ParseResp,
// 	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)BuildTxSync,

// 	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
// 	.RX_END_ID 		= 0x00U,
// 	.ENCODED 		= false,

// 	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
// };


/******************************************************************************/
/*!
	Ctrl side
*/
/******************************************************************************/


// void Ctrlr_ProcReqResp_Monitor(MotPacket_RespPacket_Monitor_T * p_txPacket, size_t * p_txSize, const MotPacket_ReqPacket_Monitor_T * p_rxPacket, size_t rxSize, MotPacket_Interface_T * p_interface)
// {
// 	*p_txSize = MotPacket_RespPacket_Monitor_Build(p_txPacket, p_rxPacket, p_interface);
// }


// static const Protocol_Req_T CTRLR_REQ_TABLE[] =
// {
// 	// PROTOCOL_CMDR_CONFIG_REQ(MOTPROTOCOL_CMD_MONITOR_TYPE, 	ControlBrake_BuildTxPacket, 	0U, 							PROTOCOL_SYNC_ID_DISABLE),
// 	// PROTOCOL_CMDR_CONFIG_REQ(MOTPROTOCOL_CMD_CONTROL_TYPE, 	MonitorSpeed_BuildTxPacket, 	MonitorSpeed_ParseRxPacket, 	PROTOCOL_SYNC_ID_DISABLE),
// };

// //common for cmdr side and ctrlr side?
// const Protocol_Specs_T MOT_PROTOCOL_CTRLR_SPECS =
// {
// 	.RX_TIMEOUT 	= MOTPROTOCOL_TIMEOUT_MS,
// 	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
// 	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

// 	// .PARSE_RX 			= (Protocol_ProcRx_T)RxPacket_Parse,
// 	// .BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)TxPacket_BuildSync,

// 	.P_REQ_TABLE 		= &CTRLR_REQ_TABLE[0U],
// 	.REQ_TABLE_LENGTH 	= sizeof(CTRLR_REQ_TABLE)/sizeof(Protocol_Req_T),
// 	.REQ_EXT_RESET 		= 0U,
// 	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,

// 	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
// 	.RX_END_ID 		= 0x00U,
// 	.ENCODED 		= false,

// 	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
// };