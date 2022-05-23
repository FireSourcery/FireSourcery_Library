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
		case PROTOCOL_TX_SYNC_ACK_REQ_ID:			syncId = MOT_PROTOCOL_SYNC_ACK; 		break;
		case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:	syncId = MOT_PROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_NACK_REQ_ID:			syncId = MOT_PROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_ACK_DATA:				syncId = MOT_PROTOCOL_SYNC_ACK;			break;
		case PROTOCOL_TX_SYNC_NACK_DATA:			syncId = MOT_PROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_ABORT:				syncId = MOT_PROTOCOL_SYNC_ABORT;		break;
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
	Ctrl side only
*/
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
		// 		case MOT_PROTOCOL_CMD_MONITOR_TYPE: *p_reqId =; break;
		// 		case MOT_PROTOCOL_CMD_CONTROL_TYPE: *p_reqId =; break;
		// 		default: *p_reqId = p_rxPacket->Header.TypeId; break;
		// 	}
		// }
	}
	else if(rxCount >= 3U) /* Move this to protocol module handle, if header length defined */
	{
		*p_rxRemaining = p_rxPacket->Header.Length + sizeof(MotPacket_Header_T) - rxCount;
		rxCode = PROTOCOL_RX_CODE_WAIT_PACKET_REMAINING;
	}
	else if(rxCount >= MOT_PACKET_LENGTH_MIN)
	{
		switch(p_rxPacket->Header.TypeId)
		{
			case MOT_PROTOCOL_STOP_MOTORS:	rxCode = PROTOCOL_RX_CODE_COMPLETE; *p_reqId = MOT_PROTOCOL_STOP_MOTORS;		break;
			case MOT_PROTOCOL_PING:			rxCode = PROTOCOL_RX_CODE_COMPLETE; *p_reqId = MOT_PROTOCOL_PING;			break;
			case MOT_PROTOCOL_SYNC_ACK:		rxCode = PROTOCOL_RX_CODE_ACK; 				break;
			case MOT_PROTOCOL_SYNC_NACK:	rxCode = PROTOCOL_RX_CODE_NACK; 			break;
			case MOT_PROTOCOL_SYNC_ABORT:	rxCode = PROTOCOL_RX_CODE_ABORT; 			break;
			default: break;
		}
	}

	return rxCode;
}

/******************************************************************************/
/*!
	Cmdr side only
*/
/******************************************************************************/
Protocol_RxCode_T MotProtocol_CheckPacket(const MotPacket_T * p_rxPacket) //general version might need include rxlength
{
	return (MotPacket_CheckChecksum(p_rxPacket) == true) ? PROTOCOL_RX_CODE_COMPLETE : PROTOCOL_RX_CODE_ERROR;
}
