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
#include "MotProtocol.h"
#include <stddef.h>
#include <string.h>

/* MotProtocol_Packet */

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

static uint16_t Packet_CalcChecksum(const MotProtocol_Packet_T * p_packet)
{
	uint16_t checkSum = 0U;
	checkSum += CalcChecksum((const uint8_t *)&p_packet->Header, sizeof(MotProtocol_Header_T) - sizeof(uint16_t));
	checkSum += CalcChecksum(&p_packet->Payload[0U], p_packet->Header.Length);
	return checkSum;
}

static bool Packet_CheckChecksum(const MotProtocol_Packet_T * p_packet)
{
	return (Packet_CalcChecksum(p_packet) == p_packet->Header.Crc);
}

/*!
	@brief 	Set header and perform checksum. Common Req, Resp
	@return size of full packet. Header + Payload
*/
uint8_t MotProtocol_Packet_BuildHeader(MotProtocol_Packet_T * p_packet, MotProtocol_HeaderId_T id, uint8_t payloadLength)
{
	p_packet->Header.Start = MOTPROTOCOL_START_BYTE;
	p_packet->Header.TypeId = id;
	p_packet->Header.Length = payloadLength;
	p_packet->Header.Crc = Packet_CalcChecksum(p_packet);

	return payloadLength + sizeof(MotProtocol_Header_T);
}

uint8_t _MotProtocol_SyncPacket_Build(MotProtocol_SyncPacket_T * p_txPacket, MotProtocol_HeaderId_T syncId)
{
	p_txPacket->Start = MOTPROTOCOL_START_BYTE;
	p_txPacket->SyncId = syncId;
	return 2U;
}

uint8_t MotProtocol_SyncPacket_Build(MotProtocol_SyncPacket_T * p_txPacket, MotProtocol_HeaderId_T syncId)
{
	return ((syncId == MOTPROTOCOL_STOP_MOTOR) || (syncId == MOTPROTOCOL_PING) || (syncId == MOTPROTOCOL_SYNC_ACK) || (syncId == MOTPROTOCOL_SYNC_NACK) || (syncId == MOTPROTOCOL_SYNC_ABORT)) ?
		_MotProtocol_SyncPacket_Build(p_txPacket, syncId) : 0U;
}


uint8_t MotProtocol_GetControlRespLength(MotProtocol_ControlId_T controlId)
{
	/* Status response only */
	return 2U;

	/* Per Var Status */
	// switch(p_interface->ReqControl.ControlId)
	// {
	// 	case MOTPROTOCOL_CONTROL_STOP: 		*p_respLength = 1U; 												break;
	// 	case MOTPROTOCOL_CONTROL_THROTTLE: 	*p_respLength = sizeof(MotProtocol_ReqPayload_Control_Throttle_T); 	break;
	// 	case MOTPROTOCOL_CONTROL_BRAKE: 	*p_respLength = sizeof(MotProtocol_ReqPayload_Control_Brake_T); 	break;
	// 	default: break;
	// }
}

uint8_t MotProtocol_GetMonitorRespLength(MotProtocol_MonitorId_T id)
{
	uint8_t respPayloadLength;

	switch(id)
	{
		case MOTPROTOCOL_MONITOR_SPEED: 			respPayloadLength = sizeof(MotProtocol_RespPayload_Monitor_Speed_T); 	break;
		case MOTPROTOCOL_MONITOR_I_PHASES: 			respPayloadLength = sizeof(MotProtocol_RespPayload_Monitor_IPhases_T); 	break;
		case MOTPROTOCOL_MONITOR_ADC_BATCH_MSB: 	respPayloadLength = 16U; 	break;
		default: respPayloadLength = 0U; break;
	}

	return respPayloadLength;
}

/* only valid on set req, reads from set interface */
uint8_t MotProtocol_GetRespLength(const MotProtocol_Interface_T * p_interface, MotProtocol_HeaderId_T id)
{
	uint8_t respPayloadLength;

	switch(id)
	{
		case MOTPROTOCOL_CMD_CONTROL_TYPE: respPayloadLength = MotProtocol_GetControlRespLength(p_interface->ReqControl.ControlId); break;
		case MOTPROTOCOL_CMD_MONITOR_TYPE: respPayloadLength = MotProtocol_GetControlRespLength(p_interface->ReqMonitor.MonitorId); break;
		default: respPayloadLength = 0U; break;
	}

	return respPayloadLength;
}


/******************************************************************************/
/*!
	Cmdr side Req function
	Use Build Packet Functions directly as inheritance
	Or map to Protocol module for compositiion
*/
/******************************************************************************/

/******************************************************************************/
/*!
	Build Packet Functions
	Optionally map directly to Protocol Table,
	@return size of Packet (TxLength),
			althought map to size_t, calling function in context to read return as int8_t
*/
/******************************************************************************/

/******************************************************************************/
/*!	Control Type */
/******************************************************************************/
uint8_t MotProtocol_ReqPacket_Control_BuildThrottle(MotProtocol_ReqPacket_Control_T * p_reqPacket, uint16_t throttleValue)
{
	p_reqPacket->CmdControl.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	p_reqPacket->CmdControl.ValueU16s[0U] = throttleValue;
	return MotProtocol_Packet_BuildHeader(p_reqPacket, MOTPROTOCOL_CMD_CONTROL_TYPE, sizeof(MotProtocol_ReqPayload_Control_Throttle_T));
}

uint8_t MotProtocol_ReqPacket_Control_BuildBrake(MotProtocol_ReqPacket_Control_T * p_reqPacket, uint16_t brakeValue)
{
	p_reqPacket->CmdControl.ControlId = MOTPROTOCOL_CONTROL_BRAKE;
	p_reqPacket->CmdControl.ValueU16s[0U] = brakeValue;
	return MotProtocol_Packet_BuildHeader(p_reqPacket, MOTPROTOCOL_CMD_CONTROL_TYPE, sizeof(MotProtocol_ReqPayload_Control_Brake_T));
}

/*
	ReqPacket Control (General)
	payloadLength determined by ControlId

	ReqPacket includes Id in Payload
*/
uint8_t MotProtocol_ReqPacket_Control_Build(MotProtocol_ReqPacket_Control_T * p_reqPacket, const MotProtocol_Interface_T * p_interface)
{
	uint8_t payloadLength;
	switch(p_interface->ReqControl.ControlId)
	{
		case MOTPROTOCOL_CONTROL_STOP: 		payloadLength = 1U; 												break;
		case MOTPROTOCOL_CONTROL_THROTTLE: 	payloadLength = sizeof(MotProtocol_ReqPayload_Control_Throttle_T); 	break;
		case MOTPROTOCOL_CONTROL_BRAKE: 	payloadLength = sizeof(MotProtocol_ReqPayload_Control_Brake_T); 	break;
		default: break;
	}

	memcpy(&p_reqPacket->CmdControl, p_interface, payloadLength);
	return MotProtocol_Packet_BuildHeader(p_reqPacket, MOTPROTOCOL_CMD_CONTROL_TYPE, payloadLength);
}



/******************************************************************************/
/*!	Monitor Type */
/******************************************************************************/
uint8_t MotProtocol_ReqPacket_Monitor_BuildSpeed(MotProtocol_ReqPacket_Monitor_T * p_reqPacket)
{
	p_reqPacket->CmdMonitor.MonitorId = MOTPROTOCOL_MONITOR_SPEED;
	return MotProtocol_Packet_BuildHeader(p_reqPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotProtocol_ReqPayload_Monitor_T));
}

/*
	ReqPacket Monitor (General)
*/
uint8_t MotProtocol_ReqPacket_Monitor_Build(MotProtocol_ReqPacket_Monitor_T * p_reqPacket, const MotProtocol_Interface_T * p_interface)
{
	p_reqPacket->CmdMonitor.MonitorId = p_interface->ReqMonitor.MonitorId;
	// memcpy(&p_reqPacket->CmdMonitor, p_interface, 1U);
	return MotProtocol_Packet_BuildHeader(p_reqPacket, MOTPROTOCOL_CMD_MONITOR_TYPE, sizeof(MotProtocol_ReqPayload_Monitor_T));
}




/******************************************************************************/
/*!	Parse Response */
/******************************************************************************/
void MotProtocol_RespPacket_ParseMonitorSpeed(int32_t * p_speed_Frac16, const MotProtocol_RespPacket_Monitor_T * p_rxPacket)
{
	p_speed_Frac16 = ((MotProtocol_RespPayload_Monitor_Speed_T *)&p_rxPacket->Values)->Speed;
	// p_speed_Frac16 = p_rxPacket->Values.ValueU16s[0U];
}

// /*
// 	Parse returns upto 16 2-byte values, 32 1-byte values
// */
// static void MotProtocol_RespPacket_ParseMonitorIPhases(int16_t * p_Ia_Frac15, int16_t * p_Ib_Frac15, int16_t * p_Ic_Frac15, const MotProtocol_RespPacket_Monitor_T * p_rxPacket)
// {
// 	MotProtocol_RespPayload_Monitor_IPhases_T * p_payload = (MotProtocol_RespPayload_Monitor_IPhases_T *)&p_rxPacket->Values;

// 	p_Ia_Frac15 = p_payload->PhaseA;
// 	p_Ib_Frac15 = p_payload->PhaseB;
// 	p_Ic_Frac15 = p_payload->PhaseC;

// 	// p_Ia_Frac15 = p_rxPacket->Monitor.ValueS16s[0U];
// 	// p_Ib_Frac15 = p_rxPacket->Monitor.ValueS16s[1U];
// 	// p_Ic_Frac15 = p_rxPacket->Monitor.ValueS16s[2U];
// }

// static void MotProtocol_RespPacket_ParseMonitorIPhases(MotProtocol_Interface_T * p_interface, const MotProtocol_RespPacket_Monitor_T * p_rxPacket)
// {
// 	MotProtocol_RespPayload_Monitor_IPhases_T * p_payload = (MotProtocol_RespPayload_Monitor_IPhases_T *)&p_rxPacket->Values;
// 	p_interface->IPhases.PhaseA = p_payload->PhaseA;
// 	p_interface->IPhases.PhaseB = p_payload->PhaseB;
// 	p_interface->IPhases.PhaseC = p_payload->PhaseC;
// }

/*
	when app uses same interface type included in MotProtocol
	alternatively, app provide function to map to alternate app interface
*/
bool MotProtocol_RespPacket_Parse(MotProtocol_Interface_T * p_interface, const MotProtocol_Packet_T * p_rxPacket)
{
	bool status = Packet_CheckChecksum(p_rxPacket);
	if(status == true) { memcpy(p_interface, &p_rxPacket->Payload, p_rxPacket->Header.Length); }
	return status;
}




/******************************************************************************/
/*!
	Ctrlr side Req function
*/
/******************************************************************************/

/* RespPacket does not include id in payload */
uint8_t MotProtocol_RespPacket_Monitor_Build(MotProtocol_RespPacket_Monitor_T * p_respPacket, const MotProtocol_Interface_T * p_interface, MotProtocol_MonitorId_T id)
{
	uint8_t payloadLength = MotProtocol_GetMonitorRespLength(id);
	memcpy(&p_respPacket->Values, p_interface, payloadLength);
	return MotProtocol_Packet_BuildHeader(p_respPacket, id, payloadLength);
}


