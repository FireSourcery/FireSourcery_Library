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
#include "MotPacket.h"
#include <string.h>
#include <stddef.h>

/******************************************************************************/
/*!
	Common
*/
/******************************************************************************/
static uint16_t CalcChecksum(const uint8_t * p_src, uint8_t bytes)
{
	uint16_t checkSum = 0U;
	for(uint8_t iByte = 0U; iByte < bytes; iByte++) { checkSum += p_src[iByte]; }
	return checkSum;
}

static uint16_t Packet_CalcChecksum(const MotPacket_T * p_packet)
{
	uint16_t checkSum = 0U;
	checkSum += CalcChecksum((const uint8_t *)&p_packet->Header, sizeof(MotPacket_Header_T) - sizeof(uint16_t));
	checkSum += CalcChecksum(&p_packet->Payload[0U], p_packet->Header.Length);
	return checkSum;
}

bool MotPacket_CheckChecksum(const MotPacket_T * p_packet)
{
	return (Packet_CalcChecksum(p_packet) == p_packet->Header.Crc);
}

/*!
	@brief 	Set header and perform checksum. Common Req, Resp
	@return size of full packet. Header + Payload
*/
static uint8_t Packet_BuildHeader(MotPacket_T * p_packet, MotPacket_HeaderId_T headerId, uint8_t payloadLength)
{
	p_packet->Header.Start = MOTPROTOCOL_START_BYTE;
	p_packet->Header.TypeId = headerId;
	p_packet->Header.Length = payloadLength;
	p_packet->Header.Crc = Packet_CalcChecksum(p_packet);

	return payloadLength + sizeof(MotPacket_Header_T);
}

uint8_t _MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
	p_txPacket->Start = MOTPROTOCOL_START_BYTE;
	p_txPacket->SyncId = syncId;
	return 2U;
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
	return ((syncId == MOTPROTOCOL_STOP_MOTORS) || (syncId == MOTPROTOCOL_PING) || (syncId == MOTPROTOCOL_SYNC_ACK) || (syncId == MOTPROTOCOL_SYNC_NACK) || (syncId == MOTPROTOCOL_SYNC_ABORT)) ?
		_MotPacket_Sync_Build(p_txPacket, syncId) : 0U;
}

uint8_t MotPacket_GetControlRespLength(MotPacket_ControlId_T controlId)
{
	/* Status response only */
	(void)controlId;
	return 2U;

	/* Per Var Status */
	// switch(p_interface->ReqControl.ControlId)
	// {
	// 	case MOTPROTOCOL_CONTROL_STOP: 		*p_respLength = 1U; 												break;
	// 	case MOTPROTOCOL_CONTROL_THROTTLE: 	*p_respLength = sizeof(MotPacket_ReqPayload_Control_Throttle_T); 	break;
	// 	case MOTPROTOCOL_CONTROL_BRAKE: 	*p_respLength = sizeof(MotPacket_ReqPayload_Control_Brake_T); 		break;
	// 	default: break;
	// }
}

uint8_t MotPacket_GetMonitorRespLength(MotPacket_MonitorId_T monitorId)
{
	uint8_t respPayloadLength;

	switch(monitorId)
	{
		case MOTPROTOCOL_MONITOR_SPEED: 			respPayloadLength = sizeof(MotPacket_RespPayload_Monitor_Speed_T); 		break;
		case MOTPROTOCOL_MONITOR_I_PHASES: 			respPayloadLength = sizeof(MotPacket_RespPayload_Monitor_IPhases_T); 	break;
		case MOTPROTOCOL_MONITOR_ADC_BATCH_MSB: 	respPayloadLength = 16U; 	break;
		default: respPayloadLength = 0U; break;
	}

	return respPayloadLength;
}


/******************************************************************************/
/*!
	Build Packet Functions
	@return size of Packet(TxLength)

	Parse Packet Functions
	@return Packet Status, parse Req/Cmd Status??
*/
/******************************************************************************/

/******************************************************************************/
/*!
	Cmdr side function
	Use Build Packet Functions directly as inheritance

	Interface vesrion - directly map to Protocol module Table for compositiion
		when app uses same interface type included in MotPacket,
		alternatively, app provide function to map to alternate app interface
*/
/******************************************************************************/
/******************************************************************************/
/*!	Ping */
/******************************************************************************/
uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOTPROTOCOL_PING); }
uint8_t MotPacket_PingReq_GetRespLength(void) { return 0U; }
MotPacket_HeaderStatus_T MotPacket_PingResp_Parse(uint8_t * p_version, const MotPacket_PingResp_T * p_respPacket)
{
	memcpy(p_version, p_respPacket->Version.Bytes, 4U);
	return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Stop */
/******************************************************************************/
uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOTPROTOCOL_STOP_MOTORS); }
uint8_t MotPacket_StopReq_GetRespLength(void) { return 0U; }
MotPacket_HeaderStatus_T MotPacket_StopResp_Parse(MotPacket_StatusResp_Id_T * p_status, const MotPacket_StopResp_T * p_respPacket)
{
	*p_status = p_respPacket->Status.Id;
	return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Read Immediate */
/******************************************************************************/
uint8_t MotPacket_Req_ReadImmediate_Build(MotPacket_Req_ReadImmediate_T * p_reqPacket, uint16_t motVarId)
{
	p_reqPacket->CmdRead.MotVarId = motVarId;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_READ_IMMEDIATE, sizeof(MotPacket_ReqPayload_ReadImmediate_T));
}

/*!
	@return 4-byte value uint32_t or int32_t
*/
MotPacket_StatusResp_Id_T MotPacket_Resp_ReadImmediate_Parse(uint32_t * p_value, const MotPacket_Resp_ReadImmediate_T * p_respPacket)
{
	*p_value = p_respPacket->ReadVar.Value;
	return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Control Type */
/******************************************************************************/
uint8_t MotPacket_Req_Control_BuildThrottle(MotPacket_Req_Control_T * p_reqPacket, uint16_t throttleValue)
{
	p_reqPacket->CmdControl.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	p_reqPacket->CmdControl.ValueU16s[0U] = throttleValue;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_CONTROL_TYPE, sizeof(MotPacket_ReqPayload_Control_Throttle_T));
}

uint8_t MotPacket_Req_Control_BuildBrake(MotPacket_Req_Control_T * p_reqPacket, uint16_t brakeValue)
{
	p_reqPacket->CmdControl.ControlId = MOTPROTOCOL_CONTROL_BRAKE;
	p_reqPacket->CmdControl.ValueU16s[0U] = brakeValue;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_CONTROL_TYPE, sizeof(MotPacket_ReqPayload_Control_Brake_T));
}

// uint8_t MotPacket_Req_Control_Build(MotPacket_Req_Control_T * p_reqPacket, MotPacket_ControlId_T controlId, 15regs)
// {
// 	uint8_t packetLength;
// 	switch(controlId)
// 	{
// 		case MOTPROTOCOL_CONTROL_STOP: 		packetLength = 7U; 																		break;
// 		case MOTPROTOCOL_CONTROL_THROTTLE: 	packetLength = MotPacket_Req_Control_BuildThrottle(p_reqPacket, p_req->ValueU16s[0U]); 	break;
// 		case MOTPROTOCOL_CONTROL_BRAKE: 	packetLength = MotPacket_Req_Control_BuildBrake(p_reqPacket, p_req->ValueU16s[0U])		break;
// 		default: packetLength = 0U; break;
// 	}
// 	return packetLength;
// }


/******************************************************************************/
/*!	Monitor Type */
/******************************************************************************/
uint8_t MotPacket_Req_Monitor_BuildSpeed(MotPacket_Req_Monitor_T * p_reqPacket)
{
	p_reqPacket->CmdMonitor.MonitorId = MOTPROTOCOL_MONITOR_SPEED;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_ReqPayload_Monitor_T));
}

uint8_t MotPacket_Req_Monitor_Build(MotPacket_Req_Monitor_T * p_reqPacket, MotPacket_MonitorId_T monitorId)
{
	p_reqPacket->CmdMonitor.MonitorId = monitorId;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_ReqPayload_Monitor_T));
}

/*!
	@param[out] p_speed_Frac32 Q16.16
*/
MotPacket_StatusResp_Id_T MotPacket_Resp_Monitor_ParseSpeed(int32_t * p_speed_Frac32, const MotPacket_Resp_Monitor_T * p_respPacket)
{
	*p_speed_Frac32 = ((MotPacket_RespPayload_Monitor_Speed_T *)&p_respPacket->Values)->Speed;
	return p_respPacket->Header.Status;
}

MotPacket_StatusResp_Id_T MotPacket_Resp_Monitor_ParseIPhases(int16_t * p_ia_FracS16, int16_t * p_ib_FracS16, int16_t * p_ic_FracS16, const MotPacket_Resp_Monitor_T * p_respPacket)
{
	MotPacket_RespPayload_Monitor_IPhases_T * p_payload = (MotPacket_RespPayload_Monitor_IPhases_T *)&p_respPacket->Values;
	*p_ia_FracS16 = p_payload->PhaseA;
	*p_ib_FracS16 = p_payload->PhaseB;
	*p_ic_FracS16 = p_payload->PhaseC;
	return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!
	Ctrlr side Req function
*/
/******************************************************************************/
uint8_t MotPacket_PingResp_Build(MotPacket_T * p_respPacket)
{
	p_respPacket->Payload[0U] = MOTPROTOCOL_VERSION_BUGFIX;
	p_respPacket->Payload[1U] = MOTPROTOCOL_VERSION_MINOR;
	p_respPacket->Payload[2U] = MOTPROTOCOL_VERSION_MAJOR;
	p_respPacket->Payload[3U] = MOTPROTOCOL_VERSION_OPT;

	return Packet_BuildHeader(p_respPacket, MOTPROTOCOL_PING, 4U);
}

uint8_t MotPacket_Resp_Monitor_Speed_Build(MotPacket_Resp_Monitor_T * p_respPacket, int32_t speed)
{
	((MotPacket_RespPayload_Monitor_Speed_T *)&p_respPacket->Values)->Speed = speed;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_RespPayload_Monitor_Speed_T));
}

uint8_t MotPacket_Resp_Monitor_IPhases_Build(MotPacket_Resp_Monitor_T * p_respPacket, int16_t ia, int16_t ib, int16_t ic)
{
	p_respPacket->Values.S16s[0U] = ia;
	p_respPacket->Values.S16s[1U] = ib;
	p_respPacket->Values.S16s[2U] = ic;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_RespPayload_Monitor_Speed_T));
}

// MotPacket_StatusResp_Id_T MotPacket_Resp_Monitor_ParseId(MotPacket_MonitorId_T * p_monitorId, const MotPacket_Req_Monitor_T * p_reqPacket)
// {
// 	*p_monitorId = p_reqPacket->CmdMonitor.MonitorId;
// 	return p_reqPacket->Header.Status;
// }


/******************************************************************************/
/*!
	Interface verion
*/
/******************************************************************************/

/* only valid on set req, reads from set interface */
// uint8_t MotPacket_GetRespLength(const MotPacket_Interface_T * p_interface, MotPacket_HeaderId_T id)
// {
// 	uint8_t respPayloadLength;

// 	switch(id)
// 	{
// 		case MOTPROTOCOL_CMD_CONTROL_TYPE: respPayloadLength = MotPacket_GetControlRespLength(p_interface->ReqControl.ControlId); break;
// 		case MOTPROTOCOL_CMD_MONITOR_TYPE: respPayloadLength = MotPacket_GetControlRespLength(p_interface->ReqMonitor.MonitorId); break;
// 		default: respPayloadLength = 0U; break;
// 	}

// 	return respPayloadLength;
// }

/*
	Using Packet Interface
	Cmdr side parse Resp
	Ctrlr side parse Req
*/
/*
	Parse returns upto 16 2-byte values, 32 1-byte values
*/
// bool MotPacket_Rx_Parse(MotPacket_Interface_T * p_interface, const MotPacket_T * p_respPacket)
// {
// 	bool status = Packet_CheckChecksum(p_respPacket);
// 	if(status == true) { memcpy(p_interface, &p_respPacket->Payload, p_respPacket->Header.Length); }
// 	return status;
// }

// uint8_t MotPacket_InterfaceReq_ReadImmediate_Build(MotPacket_Req_ReadImmediate_T * p_reqPacket, const MotPacket_Interface_T * p_interface)
// {
// 	memcpy(&p_reqPacket->CmdRead, &p_interface->ReqReadImmediate, sizeof(MotPacket_ReqPayload_ReadImmediate_T));
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_READ_IMMEDIATE, sizeof(MotPacket_ReqPayload_ReadImmediate_T));
// }

// MotPacket_Status_T MotPacket_InterfaceResp_ReadImmediate_Parse(MotPacket_Interface_T * p_respInterface, const MotPacket_Interface_T * p_reqInterface, const MotPacket_Resp_ReadImmediate_T * p_respPacket)
// {
// 	switch(p_reqInterface->ReqReadImmediate.MotVarId)
// 	{
// 		case MOT_VAR_POLE_PAIRS: p_respInterface-> = p_respPacket->ReadVar; break;

// 	}
// }



/*
	Req Control (General)
	payloadLength determined by ControlId

	Req includes Id in Payload
*/
// uint8_t MotPacket_Req_Control_Build(MotPacket_Req_Control_T * p_reqPacket, const MotPacket_Interface_T * p_interface)
// {
// 	uint8_t payloadLength;
// 	switch(p_interface->ReqControl.ControlId)
// 	{
// 		case MOTPROTOCOL_CONTROL_STOP: 		payloadLength = 1U; 												break;
// 		case MOTPROTOCOL_CONTROL_THROTTLE: 	payloadLength = sizeof(MotPacket_ReqPayload_Control_Throttle_T); 	break;
// 		case MOTPROTOCOL_CONTROL_BRAKE: 	payloadLength = sizeof(MotPacket_ReqPayload_Control_Brake_T); 		break;
// 		default: payloadLength = 0U; break;
// 	}

// 	memcpy(&p_reqPacket->CmdControl, p_interface, payloadLength); //interface.req
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_CONTROL_TYPE, payloadLength);
// }


/*
	Req Monitor
*/
// uint8_t MotPacket_Req_Monitor_Build(MotPacket_Req_Monitor_T * p_reqPacket, const MotPacket_ReqInterface_T * p_interface)
// uint8_t MotPacket_Req_Monitor_Build(MotPacket_Req_Monitor_T * p_reqPacket, const MotPacket_Interface_T * p_interface)
// {
// 	// memcpy(&p_reqPacket->CmdMonitor, p_interface, 1U);
// 	p_reqPacket->CmdMonitor.MonitorId = p_interface->ReqMonitor.MonitorId;
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_ReqPayload_Monitor_T));
// }

// void MotPacket_Inteface_MonitorResp_Parse(MotPacket_Interface_T * p_respInterface, const MotPacket_Interface_T * p_reqInterface, const MotPacket_Resp_Monitor_T * p_respPacket)
// {
// 	switch(p_reqInterface->ReqMonitor.MonitorId)
// 	{
// 		case MOTPROTOCOL_MONITOR_SPEED: 			MotPacket_Resp_ParseMonitorSpeed(&p_respInterface->Speed.Speed, p_respInterface); 	break;
// 		case MOTPROTOCOL_MONITOR_I_PHASES: 				break;
// 		case MOTPROTOCOL_MONITOR_ADC_BATCH_MSB: 		break;
// 	}
// }



/******************************************************************************/
/* One function handle all cases, alternatively use typeId is function table index */
/******************************************************************************/
// uint8_t MotPacket_Req_Build(MotPacket_Req_Monitor_T * p_reqPacket, const MotPacket_Interface_T * p_interface, MotPacket_HeaderId_T typeId)
// {
// 	uint8_t txLength = MotPacket_Sync_Build((MotPacket_Sync_T  *)p_reqPacket, typeId);

// 	if(txLength == 0U)
// 	{
// 		switch(typeId)
// 		{
// 			case MOTPROTOCOL_CMD_MONITOR_TYPE: 	txLength = MotPacket_Req_Monitor_Build((MotPacket_Req_Monitor_T *)p_reqPacket, p_interface);	break;
// 			case MOTPROTOCOL_CMD_CONTROL_TYPE: 	txLength = MotPacket_Req_Control_Build((MotPacket_Req_Control_T *)p_reqPacket, p_interface);	break;
// 			default: break;
// 		}
// 	}

// 	return txLength;
// }



/* cmdr */

/* Resp does not include SubId in payload */
// uint8_t MotPacket_Resp_Monitor_Build(MotPacket_Resp_Monitor_T * p_respPacket, const MotPacket_Interface_T * p_interface, MotPacket_MonitorId_T monitorId)
// {
// 	uint8_t payloadLength = MotPacket_GetMonitorRespLength(monitorId);
// 	memcpy(&p_respPacket->Values, p_interface, payloadLength);

// 	p_respPacket->Values.U16s[0U] = p_interface.rx.focA;

// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, payloadLength);
// }

