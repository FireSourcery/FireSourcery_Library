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

/*
	App Protcol Packet Interface
	Parse to App structure
*/

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
	MotPacket_PingResp_Parse(&p_app->Version[0], p_rxPacket);
}

/******************************************************************************/
/*!	Stop */
/******************************************************************************/
static void StopAll_BuildReq(MotPacket_StopReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_StopReq_Build(p_txPacket);
	*p_respLength = MotPacket_StopReq_GetRespLength();
}

static void StopAll_ParseResp(MotorCmdr_T * p_app, const MotPacket_StopResp_T * p_rxPacket)
{
	// p_app->RespStatus = MotPacket_StopResp_Parse(p_rxPacket);
}

/******************************************************************************/
/*!	Save NvMemory */
/******************************************************************************/
static void SaveNvm_BuildReq(MotPacket_SaveNvmReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_SaveNvmReq_Build(p_txPacket);
	*p_respLength = MotPacket_SaveNvmReq_GetRespLength();
}

static void SaveNvm_ParseResp(MotorCmdr_T * p_app, const MotPacket_SaveNvmResp_T * p_rxPacket)
{
	p_app->RespStatus = p_rxPacket->Header.Status;
}

/******************************************************************************/
/*!	Init Units */
/******************************************************************************/
static void InitUnits_BuildReq(MotPacket_InitUnitsReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_InitUnitsReq_Build(p_txPacket);
	*p_respLength = MotPacket_InitUnitsReq_GetRespLength();
}

static void InitUnits_ParseResp(MotorCmdr_T * p_app, const MotPacket_InitUnitsResp_T * p_rxPacket)
{
	// p_app->RespStatus =
	MotPacket_InitUnitsResp_Parse
	(
		&p_app->Units.SpeedFeedbackRef_Rpm, &p_app->Units.IMaxRef_Amp, &p_app->Units.VSupplyRef_Volts,
		&p_app->Units.VSupply_R1, &p_app->Units.VSupply_R2,
		&p_app->Units.VSense_R1, &p_app->Units.VSense_R2,
		&p_app->Units.VAcc_R1, &p_app->Units.VAcc_R2,
		p_rxPacket
	);
}


/******************************************************************************/
/*!	Control */
/******************************************************************************/
static void Control_BuildReq(MotPacket_ControlReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	switch(p_app->ControlIdActive)
	{
		case MOT_PROTOCOL_CONTROL_THROTTLE:				*p_txLength = MotPacket_ControlReq_Throttle_Build(p_txPacket, p_app->MotorCmdValue); 	break;
		case MOT_PROTOCOL_CONTROL_BRAKE: 				*p_txLength = MotPacket_ControlReq_Brake_Build(p_txPacket, p_app->MotorCmdValue); 		break;
		case MOT_PROTOCOL_CONTROL_RELEASE: 				*p_txLength = MotPacket_ControlReq_Release_Build(p_txPacket); 							break;
		case MOT_PROTOCOL_CONTROL_DIRECTION_FORWARD: 	*p_txLength = MotPacket_ControlReq_DirectionForward_Build(p_txPacket); 					break;
		case MOT_PROTOCOL_CONTROL_DIRECTION_REVERSE: 	*p_txLength = MotPacket_ControlReq_DirectionReverse_Build(p_txPacket); 					break;
		case MOT_PROTOCOL_CONTROL_DIRECTION_NEUTRAL: 	*p_txLength = MotPacket_ControlReq_DirectionNeutral_Build(p_txPacket); 					break;
		default: break;
	}

	*p_respLength = MotPacket_ControlReq_GetRespLength(p_app->ControlIdActive);
}

static void Control_ParseResp(MotorCmdr_T * p_app, const MotPacket_ControlResp_T * p_rxPacket)
{
	// p_app->RespStatus = p_rxPacket->Header.Status;
}

/******************************************************************************/
/*!	Monitor */
/******************************************************************************/
static void Monitor_BuildReq(MotPacket_MonitorReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	*p_txLength = MotPacket_MonitorReq_Build(p_txPacket, p_app->MonitorIdActive);
	*p_respLength = MotPacket_MonitorReq_GetRespLength(p_app->MonitorIdActive);
}

static void Monitor_ParseResp(MotorCmdr_T * p_app, const MotPacket_MonitorResp_T * p_rxPacket)
{
	// p_app->RespStatus =
	switch(p_app->MonitorIdActive) /* todo check monitor req must be active req */
	{
		case MOT_PROTOCOL_MONITOR_SPEED: MotPacket_MonitorResp_Speed_Parse(&p_app->Speed, p_rxPacket); 		break;
		case MOT_PROTOCOL_MONITOR_I_FOC:
			MotPacket_MonitorResp_IFoc_Parse(&p_app->Ia, &p_app->Ib, &p_app->Ic, &p_app->Ialpha, &p_app->Ibeta, &p_app->Id, &p_app->Iq, p_rxPacket);
			break;
		default: break;
	}
}

/******************************************************************************/
/*!	Write Immediate Var */
/******************************************************************************/
static void WriteImmediate_BuildReq(MotPacket_WriteImmediateReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	*p_txLength = MotPacket_WriteImmediateReq_Build(p_txPacket, p_app->MotorReadWriteVarId, p_app->MotorReadWriteVarValue);
	*p_respLength = MotPacket_WriteImmediateReq_GetRespLength();
}

static void WriteImmediate_ParseResp(MotorCmdr_T * p_app, const MotPacket_WriteImmediateResp_T * p_rxPacket)
{
	// p_app->RespStatus = p_rxPacket->Header.Status;
}

/******************************************************************************/
/*!	Read Immediate Var */
/******************************************************************************/
static void ReadImmediate_BuildReq(MotPacket_ReadImmediateReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	*p_txLength = MotPacket_ReadImmediateReq_Build(p_txPacket, p_app->MotorReadWriteVarId);
	*p_respLength = MotPacket_ReadImmediateReq_GetRespLength();
}

static void ReadImmediate_ParseResp(MotorCmdr_T * p_app, const MotPacket_ReadImmediateResp_T * p_rxPacket)
{
	// p_app->RespStatus = p_rxPacket->Header.Status;
	MotPacket_ReadImmediateResp_Parse(&p_app->MotorReadWriteVarValue, p_rxPacket);
	//pass in to more buffer?
}

/*

*/
static const Protocol_Cmdr_Req_T CMDR_REQ_TABLE[] =
{
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_STOP_ALL, 			StopAll_BuildReq, 			StopAll_ParseResp, 			PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_PING, 				Ping_BuildReq, 				Ping_ParseResp, 			PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_CMD_CONTROL_TYPE, 	Control_BuildReq, 			Control_ParseResp, 			PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_CMD_MONITOR_TYPE, 	Monitor_BuildReq, 			Monitor_ParseResp, 			PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_CMD_INIT_UNITS, 		InitUnits_BuildReq, 		InitUnits_ParseResp, 		PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_CMD_SAVE_NVM, 		SaveNvm_BuildReq, 			SaveNvm_ParseResp, 			PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_CMD_WRITE_IMMEDIATE, 	WriteImmediate_BuildReq, 	WriteImmediate_ParseResp, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_CMDR_REQ_DEFINE(MOT_PROTOCOL_CMD_READ_IMMEDIATE, 	ReadImmediate_BuildReq, 	ReadImmediate_ParseResp, 	PROTOCOL_SYNC_ID_DISABLE),

};

const Protocol_Specs_T MOTOR_CMDR_MOT_PROTOCOL_SPECS =
{
	.RX_LENGTH_MIN 	= MOT_PACKET_LENGTH_MIN,
	.RX_LENGTH_MAX 	= MOT_PACKET_LENGTH_MAX,
	// .RX_TIMEOUT 	= MOT_PROTOCOL_TIMEOUT_MS,
	//.RX_TIMEOUT_BYTE =  ,
	.CHECK_PACKET 	= (Protocol_CheckPacket_T)MotProtocol_CheckRxPacket, //todo check

	.P_REQ_TABLE 		= &CMDR_REQ_TABLE[0U],
	.REQ_TABLE_LENGTH 	= sizeof(CMDR_REQ_TABLE)/sizeof(Protocol_Cmdr_Req_T),
	.REQ_EXT_RESET 		= (Protocol_ResetExt_T)MotProtocol_ResetExt,
	.REQ_TIMEOUT		= MOT_PROTOCOL_TIMEOUT_MS,

	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)MotProtocol_BuildTxSync,

	.RX_START_ID 	= MOT_PACKET_START_BYTE,
	.RX_END_ID 		= 0x00U,
	.ENCODED 		= false,
	.BAUD_RATE_DEFAULT = MOT_PROTOCOL_BAUD_RATE_DEFAULT,
};
