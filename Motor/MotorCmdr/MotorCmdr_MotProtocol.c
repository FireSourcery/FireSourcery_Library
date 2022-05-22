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
#include "MotorCmdr.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include <stddef.h>
#include <string.h>


/* Alternatively, app provides interface */

/******************************************************************************/
/*!	Ping */
/******************************************************************************/
static void Ping_BuildReq(MotPacket_PingReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_PingReq_Build(p_txPacket);
	*p_respLength = MotPacket_PingReq_GetRespLength();
}

static void Ping_ParseResp(MotorCmdr_T * p_app, const MotPacket_PingResp_T * p_rxPacket)
{
	//  = MotPacket_PingResp_Parse(&p_app->Version[0], p_rxPacket);
}

/******************************************************************************/
/*!	Stop */
/******************************************************************************/
static void Stop_BuildReq(MotPacket_StopReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_StopReq_Build(p_txPacket);
	*p_respLength = MotPacket_StopReq_GetRespLength();
}

static void Stop_ParseResp(MotorCmdr_T * p_app, const MotPacket_StopResp_T * p_rxPacket)
{
	MotPacket_StopResp_Parse(&p_app->RespStatus, p_rxPacket);
}

/******************************************************************************/
/*!	Control */
/******************************************************************************/
static void Control_BuildReq(MotPacket_Req_Control_T * p_reqPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	/* App uses its own structure */
	switch(p_app->ControlIdActive)
	{
		case MOTPROTOCOL_CONTROL_THROTTLE:
			*p_txLength = MotPacket_Req_Control_BuildThrottle(p_reqPacket, p_app->MotorCmdValue);
			// MotPacket_GetControlThrottleRespLength( )
			break;
	}

	*p_respLength = MotPacket_GetControlRespLength(p_app->ControlIdActive);
}

static Protocol_RxCode_T Control_ParseResp(MotorCmdr_T * p_app, const MotPacket_Resp_Control_T * p_rxPacket)
{
	switch(p_app->ControlIdActive)
	{
		case MOTPROTOCOL_CONTROL_THROTTLE:
			break;
	}
}


/*

*/
static const Protocol_Cmdr_Req_T CMDR_REQ_TABLE[] =
{
	PROTOCOL_CMDR_REQ_DEFINE(MOTPROTOCOL_STOP_MOTORS, 		Stop_BuildReq, 		Stop_ParseResp, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOTPROTOCOL_PING, 				Ping_BuildReq, 		Ping_ParseResp, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOTPROTOCOL_CMD_CONTROL_TYPE, 	Control_BuildReq, 	Control_ParseResp, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOTPROTOCOL_CMD_MONITOR_TYPE, 	0U, 				0U, 				PROTOCOL_SYNC_ID_DISABLE),
};

const Protocol_Specs_T MOTOR_CMDR_MOT_PROTOCOL_SPECS =
{
	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

	.RX_TIMEOUT 	= MOTPROTOCOL_TIMEOUT_MS,
	//.RX_TIMEOUT_BYTE =  ,

	.P_REQ_TABLE 		= &CMDR_REQ_TABLE[0U],
	.REQ_TABLE_LENGTH 	= sizeof(CMDR_REQ_TABLE)/sizeof(Protocol_Cmdr_Req_T),
	.REQ_EXT_RESET 		= (Protocol_ResetExt_T)MotProtocol_ResetExt,
	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,

	.CHECK_PACKET 		= (Protocol_CheckPacket_T)MotProtocol_CheckPacket, //todo check

	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)MotProtocol_BuildTxSync,

	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
	.RX_END_ID 		= 0x00U,
	.ENCODED 		= false,
	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
};
