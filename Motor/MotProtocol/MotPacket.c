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
	p_packet->Header.Start = MOT_PACKET_START_BYTE;
	p_packet->Header.TypeId = headerId;
	p_packet->Header.Length = payloadLength;
	p_packet->Header.Crc = Packet_CalcChecksum(p_packet);

	return payloadLength + sizeof(MotPacket_Header_T);
}

uint8_t _MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
	p_txPacket->Start = MOT_PACKET_START_BYTE;
	p_txPacket->SyncId = syncId;
	return 2U;
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
	return ((syncId == MOT_PROTOCOL_STOP_MOTORS) || (syncId == MOT_PROTOCOL_PING) || (syncId == MOT_PROTOCOL_SYNC_ACK) || (syncId == MOT_PROTOCOL_SYNC_NACK) || (syncId == MOT_PROTOCOL_SYNC_ABORT)) ?
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
	// 	case MOT_PROTOCOL_CONTROL_STOP: 		*p_respLength = 1U; 												break;
	// 	case MOT_PROTOCOL_CONTROL_THROTTLE: 	*p_respLength = sizeof(MotPacket_ControlReq_Throttle_Payload_T); 	break;
	// 	case MOT_PROTOCOL_CONTROL_BRAKE: 	*p_respLength = sizeof(MotPacket_ControlReq_Brake_Payload_T); 		break;
	// 	default: break;
	// }
}

uint8_t MotPacket_GetMonitorRespLength(MotPacket_MonitorId_T monitorId)
{
	uint8_t respPayloadLength;

	switch(monitorId)
	{
		case MOT_PROTOCOL_MONITOR_SPEED: 			respPayloadLength = sizeof(MotPacket_MonitorResp_Speed_Payload_T); 		break;
		case MOT_PROTOCOL_MONITOR_I_PHASES: 		respPayloadLength = sizeof(MotPacket_MonitorResp_IPhases_Payload_T); 	break;
		case MOT_PROTOCOL_MONITOR_ADC_BATCH_MSB: 	respPayloadLength = 16U; 	break;
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
	Cmdr side
*/
/******************************************************************************/
/******************************************************************************/
/*!	Ping */
/******************************************************************************/
uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOT_PROTOCOL_PING); }
uint8_t MotPacket_PingReq_GetRespLength(void) { return sizeof(MotPacket_PingResp_T); }
void MotPacket_PingResp_Parse(uint8_t * p_version, const MotPacket_PingResp_T * p_respPacket)
{
	memcpy(p_version, p_respPacket->Version.Bytes, 4U);
	// return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Stop */
/******************************************************************************/
uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOT_PROTOCOL_STOP_MOTORS); }
uint8_t MotPacket_StopReq_GetRespLength(void) { return sizeof(MotPacket_StopResp_T); }
void MotPacket_StopResp_Parse(MotPacket_StatusResp_Id_T * p_status, const MotPacket_StopResp_T * p_respPacket)
{
	*p_status = p_respPacket->Status.Id;
	// return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Read Immediate */
/******************************************************************************/
uint8_t MotPacket_ReadImmediateReq_Build(MotPacket_ReadImmediateReq_T * p_reqPacket, uint16_t motVarId)
{
	p_reqPacket->CmdRead.MotVarId = motVarId;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_READ_IMMEDIATE, sizeof(MotPacket_ReadImmediateReq_Payload_T));
}

/*!
	@return 4-byte value uint32_t or int32_t
*/
void MotPacket_ReadImmediateResp_Parse(uint32_t * p_value, const MotPacket_ReadImmediateResp_T * p_respPacket)
{
	*p_value = p_respPacket->ReadVar.Value;
	// *p_status = p_respPacket->ReadVar.Status;
	// return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Control Type */
/******************************************************************************/
uint8_t MotPacket_ControlReq_Throttle_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t throttleValue)
{
	p_reqPacket->CmdControl.ControlId = MOT_PROTOCOL_CONTROL_THROTTLE;
	p_reqPacket->CmdControl.ValueU16s[0U] = throttleValue;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Throttle_Payload_T));
}

uint8_t MotPacket_ControlReq_Brake_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t brakeValue)
{
	p_reqPacket->CmdControl.ControlId = MOT_PROTOCOL_CONTROL_BRAKE;
	p_reqPacket->CmdControl.ValueU16s[0U] = brakeValue;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Brake_Payload_T));
}

// uint8_t MotPacket_ControlReq_Build(MotPacket_ControlReq_T * p_reqPacket, MotPacket_ControlId_T controlId, 15regs)
// {
// 	uint8_t packetLength;
// 	switch(controlId)
// 	{
// 		case MOT_PROTOCOL_CONTROL_STOP: 		packetLength = 7U; 																		break;
// 		case MOT_PROTOCOL_CONTROL_THROTTLE: 	packetLength = MotPacket_ControlReq_Throttle_Build(p_reqPacket, p_req->ValueU16s[0U]); 	break;
// 		case MOT_PROTOCOL_CONTROL_BRAKE: 	packetLength = MotPacket_ControlReq_Brake_Build(p_reqPacket, p_req->ValueU16s[0U])		break;
// 		default: packetLength = 0U; break;
// 	}
// 	return packetLength;
// }


/******************************************************************************/
/*!	Monitor Type */
/******************************************************************************/
uint8_t MotPacket_MonitorReq_Speed_Build(MotPacket_MonitorReq_T * p_reqPacket)
{
	p_reqPacket->CmdMonitor.MonitorId = MOT_PROTOCOL_MONITOR_SPEED;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_MonitorReq_Payload_T));
}

uint8_t MotPacket_MonitorReq_Build(MotPacket_MonitorReq_T * p_reqPacket, MotPacket_MonitorId_T monitorId)
{
	p_reqPacket->CmdMonitor.MonitorId = monitorId;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_MonitorReq_Payload_T));
}

/*!
	@param[out] p_speed_Frac32 Q16.16
*/
void MotPacket_MonitorResp_Speed_Parse(int32_t * p_speed_Frac32, const MotPacket_MonitorResp_T * p_respPacket)
{
	*p_speed_Frac32 = ((MotPacket_MonitorResp_Speed_Payload_T *)&p_respPacket->Values)->Speed;
	// return p_respPacket->Header.Status;
}

void MotPacket_MonitorResp_IPhases_Parse(int16_t * p_ia_FracS16, int16_t * p_ib_FracS16, int16_t * p_ic_FracS16, const MotPacket_MonitorResp_T * p_respPacket)
{
	MotPacket_MonitorResp_IPhases_Payload_T * p_payload = (MotPacket_MonitorResp_IPhases_Payload_T *)&p_respPacket->Values;
	*p_ia_FracS16 = p_payload->PhaseA;
	*p_ib_FracS16 = p_payload->PhaseB;
	*p_ic_FracS16 = p_payload->PhaseC;
	// return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!
	Ctrlr side
*/
/******************************************************************************/
/******************************************************************************/
/*!	Ping Type */
/******************************************************************************/
uint8_t MotPacket_PingResp_Build(MotPacket_T * p_respPacket)
{
	p_respPacket->Payload[0U] = MOT_PROTOCOL_VERSION_BUGFIX;
	p_respPacket->Payload[1U] = MOT_PROTOCOL_VERSION_MINOR;
	p_respPacket->Payload[2U] = MOT_PROTOCOL_VERSION_MAJOR;
	p_respPacket->Payload[3U] = MOT_PROTOCOL_VERSION_OPT;

	return Packet_BuildHeader(p_respPacket, MOT_PROTOCOL_PING, 4U);
}

/******************************************************************************/
/*!	Control Type */
/******************************************************************************/
uint8_t MotPacket_ControlResp_MainStatus_Build(MotPacket_ControlResp_T * p_respPacket, MotPacket_StatusResp_Id_T status)
{
	p_respPacket->Status.MainStatus = status;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PROTOCOL_CMD_CONTROL_TYPE, sizeof(uint16_t));
}

/******************************************************************************/
/*!	Monitor Type */
/******************************************************************************/
uint8_t MotPacket_MonitorResp_Speed_Build(MotPacket_MonitorResp_T * p_respPacket, int32_t speed)
{
	((MotPacket_MonitorResp_Speed_Payload_T *)&p_respPacket->Values)->Speed = speed;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_MonitorResp_Speed_Payload_T));
}

uint8_t MotPacket_MonitorResp_IPhases_Build(MotPacket_MonitorResp_T * p_respPacket, int16_t ia, int16_t ib, int16_t ic)
{
	MotPacket_MonitorResp_IPhases_Payload_T * p_payload = (MotPacket_MonitorResp_IPhases_Payload_T *)&p_respPacket->Values;
	p_payload->PhaseA = ia;
	p_payload->PhaseB = ib;
	p_payload->PhaseC = ic;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_MonitorResp_Speed_Payload_T));
}

void MotPacket_MonitorResp_ParseId(MotPacket_MonitorId_T * p_monitorId, const MotPacket_MonitorReq_T * p_reqPacket)
{
	*p_monitorId = p_reqPacket->CmdMonitor.MonitorId;
	// return p_reqPacket->Header.Status;
}


/******************************************************************************/
/*!
	Interface verion - directly map to Protocol module Table for compositiion
		when app uses same interface type included in MotPacket,
		alternatively, app provide function to map to alternate app interface
*/
/******************************************************************************/

/* only valid on set req, reads from set interface */
// uint8_t MotPacket_GetRespLength(const MotPacket_Interface_T * p_interface, MotPacket_HeaderId_T id)
// {
// 	uint8_t respPayloadLength;

// 	switch(id)
// 	{
// 		case MOT_PROTOCOL_CMD_CONTROL_TYPE: respPayloadLength = MotPacket_GetControlRespLength(p_interface->ReqControl.ControlId); break;
// 		case MOT_PROTOCOL_CMD_MONITOR_TYPE: respPayloadLength = MotPacket_GetControlRespLength(p_interface->ReqMonitor.MonitorId); break;
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

// uint8_t MotPacket_InterfaceReq_ReadImmediate_Build(MotPacket_ReadImmediateReq_T * p_reqPacket, const MotPacket_Interface_T * p_interface)
// {
// 	memcpy(&p_reqPacket->CmdRead, &p_interface->ReqReadImmediate, sizeof(MotPacket_ReadImmediateReq_Payload_T));
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_READ_IMMEDIATE, sizeof(MotPacket_ReadImmediateReq_Payload_T));
// }

// MotPacket_Status_T MotPacket_InterfaceResp_ReadImmediate_Parse(MotPacket_Interface_T * p_respInterface, const MotPacket_Interface_T * p_reqInterface, const MotPacket_ReadImmediateResp_T * p_respPacket)
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
// uint8_t MotPacket_Req_Control_Build(MotPacket_ControlReq_T * p_reqPacket, const MotPacket_Interface_T * p_interface)
// {
// 	uint8_t payloadLength;
// 	switch(p_interface->ReqControl.ControlId)
// 	{
// 		case MOT_PROTOCOL_CONTROL_STOP: 		payloadLength = 1U; 												break;
// 		case MOT_PROTOCOL_CONTROL_THROTTLE: 	payloadLength = sizeof(MotPacket_ControlReq_Throttle_Payload_T); 	break;
// 		case MOT_PROTOCOL_CONTROL_BRAKE: 	payloadLength = sizeof(MotPacket_ControlReq_Brake_Payload_T); 		break;
// 		default: payloadLength = 0U; break;
// 	}

// 	memcpy(&p_reqPacket->CmdControl, p_interface, payloadLength); //interface.req
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_CONTROL_TYPE, payloadLength);
// }


/*
	Req Monitor
*/
// uint8_t MotPacket_MonitorReq_Build(MotPacket_MonitorReq_T * p_reqPacket, const MotPacket_ReqInterface_T * p_interface)
// uint8_t MotPacket_MonitorReq_Build(MotPacket_MonitorReq_T * p_reqPacket, const MotPacket_Interface_T * p_interface)
// {
// 	// memcpy(&p_reqPacket->CmdMonitor, p_interface, 1U);
// 	p_reqPacket->CmdMonitor.MonitorId = p_interface->ReqMonitor.MonitorId;
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PROTOCOL_CMD_MONITOR_TYPE, sizeof(MotPacket_MonitorReq_Payload_T));
// }

// void MotPacket_Inteface_MonitorResp_Parse(MotPacket_Interface_T * p_respInterface, const MotPacket_Interface_T * p_reqInterface, const MotPacket_MonitorResp_T * p_respPacket)
// {
// 	switch(p_reqInterface->ReqMonitor.MonitorId)
// 	{
// 		case MOT_PROTOCOL_MONITOR_SPEED: 			MotPacket_Resp_ParseMonitorSpeed(&p_respInterface->Speed.Speed, p_respInterface); 	break;
// 		case MOT_PROTOCOL_MONITOR_I_PHASES: 				break;
// 		case MOT_PROTOCOL_MONITOR_ADC_BATCH_MSB: 		break;
// 	}
// }



/******************************************************************************/
/* One function handle all cases, alternatively use typeId is function table index */
/******************************************************************************/
// uint8_t MotPacket_Req_Build(MotPacket_MonitorReq_T * p_reqPacket, const MotPacket_Interface_T * p_interface, MotPacket_HeaderId_T typeId)
// {
// 	uint8_t txLength = MotPacket_Sync_Build((MotPacket_Sync_T  *)p_reqPacket, typeId);

// 	if(txLength == 0U)
// 	{
// 		switch(typeId)
// 		{
// 			case MOT_PROTOCOL_CMD_MONITOR_TYPE: 	txLength = MotPacket_MonitorReq_Build((MotPacket_MonitorReq_T *)p_reqPacket, p_interface);	break;
// 			case MOT_PROTOCOL_CMD_CONTROL_TYPE: 	txLength = MotPacket_Req_Control_Build((MotPacket_ControlReq_T *)p_reqPacket, p_interface);	break;
// 			default: break;
// 		}
// 	}

// 	return txLength;
// }



/* cmdr */

/* Resp does not include SubId in payload */
// uint8_t MotPacket_Resp_Monitor_Build(MotPacket_MonitorResp_T * p_respPacket, const MotPacket_Interface_T * p_interface, MotPacket_MonitorId_T monitorId)
// {
// 	uint8_t payloadLength = MotPacket_GetMonitorRespLength(monitorId);
// 	memcpy(&p_respPacket->Values, p_interface, payloadLength);

// 	p_respPacket->Values.U16s[0U] = p_interface.rx.focA;

// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PROTOCOL_CMD_MONITOR_TYPE, payloadLength);
// }

