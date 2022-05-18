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
	@file 	MotProtocol.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Mot_Protocol.h"
#include <stddef.h>
#include <string.h>


/******************************************************************************/
/*!
	Protocol Module Composition version,
	Packet Interface map to Protocol Module
	Protocol Module function types
*/
/******************************************************************************/

/******************************************************************************/
/*!
	Common
*/
/******************************************************************************/
#define SYNC_DISABLE_ID 0U

static void BuildTxSync(MotProtocol_SyncPacket_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId)
{
	MotProtocol_HeaderId_T syncId;

	switch(txId)
	{
		case PROTOCOL_TX_SYNC_ACK_REQ_ID:			syncId = MOTPROTOCOL_SYNC_ACK; 		break;
		case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:	syncId = MOTPROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_NACK_REQ_ID:			syncId = MOTPROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_ACK_DATA:				syncId = MOTPROTOCOL_SYNC_ACK;		break;
		case PROTOCOL_TX_SYNC_NACK_DATA:			syncId = MOTPROTOCOL_SYNC_NACK;		break;
		case PROTOCOL_TX_SYNC_ABORT:				syncId = MOTPROTOCOL_SYNC_ABORT;	break;
		default: *p_txSize = 0U; break;
	}

	*p_txSize = _MotProtocol_SyncPacket_Build(p_txPacket, syncId);
}

/******************************************************************************/
/*!
	Cmdr side
*/
/******************************************************************************/
// static void Mot_Protocol_Cmdr_BuildReqControl(MotProtocol_ReqPacket_Control_T * p_reqPacket, size_t * p_txLength, size_t * p_respLength, const MotProtocol_Interface_T * p_interface)
// {
// 	*p_txLength = MotProtocol_ReqPacket_Control_Build(p_reqPacket, p_interface);
// 	*p_respLength = MotProtocol_GetControlRespLength(p_interface->ReqControl.ControlId);
// }

// static void Mot_Protocol_Cmdr_BuildReqMonitor(MotProtocol_ReqPacket_Control_T * p_reqPacket, size_t * p_txLength, size_t * p_respLength, const MotProtocol_Interface_T * p_interface)
// {
// 	*p_txLength = MotProtocol_ReqPacket_Monitor_Build(p_reqPacket, p_interface);
// 	*p_respLength = MotProtocol_GetMonitorRespLength(p_interface->ReqMonitor.MonitorId);
// }

/* One function handle all cases, alternatively use function table */
// static void MotProtocol_Protocol_Cmdr_BuildReq(MotProtocol_Packet_T * p_reqPacket, size_t * p_txLength, size_t * p_respLength, const MotProtocol_Interface_T * p_interface, MotProtocol_HeaderId_T typeId)
// {
// 	*p_txLength = _MotProtocol_SyncPacket_Build(p_reqPacket, typeId);
// 	if(*p_txLength != 0U)
// 	{
// 		*p_respLength = 0U;
// 	}
// 	else
// 	{
// 		switch(typeId)
// 		{
// 			case MOTPROTOCOL_CMD_MONITOR_TYPE: 	 		break;
// 			case MOTPROTOCOL_CMD_CONTROL_TYPE: 			break;
// 		}
// 	}
// }

static Protocol_RxCode_T Cmdr_ParseResp(MotProtocol_Interface_T * p_interface, const MotProtocol_Packet_T * p_rxPacket) //general version may include rxlength
{
	return (MotProtocol_RespPacket_Parse(p_interface, p_rxPacket) == true) ? PROTOCOL_RX_CODE_RESP_DATA_SUCCESS : PROTOCOL_RX_CODE_ERROR_PACKET_DATA;
}

static const Protocol_Req_T CMDR_REQ_TABLE[] =
{
	CONFIG_PROTOCOL_REQ(MOTPROTOCOL_CMD_MONITOR_TYPE, 	MotProtocol_ReqPacket_Monitor_Build,  	SYNC_DISABLE_ID),
	CONFIG_PROTOCOL_REQ(MOTPROTOCOL_CMD_CONTROL_TYPE, 	MotProtocol_ReqPacket_Control_Build, 	SYNC_DISABLE_ID),
};

const Protocol_Specs_T MOT_PROTOCOL_CMDR_SPECS =
{
	.RX_TIMEOUT 	= MOTPROTOCOL_TIMEOUT_MS,
	// const uint32_t RX_TIMEOUT_BYTE;		//reset per byte
	// const uint32_t RX_TIMEOUT_RX;
	// const uint32_t RX_TIMEOUT_REQ;

	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

	.P_REQ_TABLE 		= &CMDR_REQ_TABLE[0U],
	.REQ_TABLE_LENGTH 	= sizeof(CMDR_REQ_TABLE)/sizeof(Protocol_Req_T),
	.REQ_EXT_RESET 		= 0U,
	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,
	.PARSE_RX 			= (Protocol_ParseRx_T)Cmdr_ParseResp,
	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)BuildTxSync,

	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
	.RX_END_ID 		= 0x00U,
	.ENCODED 		= false,

	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
};


/******************************************************************************/
/*!
	Ctrl side - builds packet only
*/
/******************************************************************************/
// void Mot_Protocol_ReqFast_Monitor(MotProtocol_RespPacket_Monitor_T * p_txPacket, size_t * p_txSize, const MotProtocol_ReqPacket_Monitor_T * p_rxPacket, size_t rxSize, const MotProtocol_Interface_T * p_interface)
// {
// 	*p_txSize = MotProtocol_RespPacket_Monitor_Build(p_txPacket, p_rxPacket, p_interface);
// }


static const Protocol_Req_T CTRLR_REQ_TABLE[] =
{
	// PROTOCOL_CMDR_CONFIG_REQ(MOTPROTOCOL_CMD_MONITOR_TYPE, 	ControlBrake_BuildTxPacket, 	0U, 							SYNC_DISABLE_ID),
	// PROTOCOL_CMDR_CONFIG_REQ(MOTPROTOCOL_CMD_CONTROL_TYPE, 	MonitorSpeed_BuildTxPacket, 	MonitorSpeed_ParseRxPacket, 	SYNC_DISABLE_ID),
};

//common for cmdr side and ctrlr side?
const Protocol_Specs_T MOT_PROTOCOL_CTRLR_SPECS =
{
	.RX_TIMEOUT 	= MOTPROTOCOL_TIMEOUT_MS,
	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

	// .PARSE_RX 			= (Protocol_ProcRx_T)RxPacket_Parse,
	// .BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)TxPacket_BuildSync,

	// .P_REQ_TABLE 		= &REQ_TABLE[0U],
	// .REQ_TABLE_LENGTH 	= sizeof(REQ_TABLE)/sizeof(Protocol_Cmdr_Req_T),
	.REQ_EXT_RESET 		= 0U,
	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,

	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
	.RX_END_ID 		= 0x00U,
	.ENCODED 		= false,

	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
};