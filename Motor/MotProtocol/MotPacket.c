/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
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
	checkSum += CalcChecksum(&p_packet->Payload[0U], p_packet->Header.TotalLength - sizeof(MotPacket_Header_T));
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
static uint8_t Packet_BuildHeader(MotPacket_T * p_packet, MotPacket_HeaderId_T headerId, uint8_t payloadLength, uint8_t status) //, uint8_t Opt[2U])
{
	p_packet->Header.Start = MOT_PACKET_START_BYTE;
	p_packet->Header.HeaderId = headerId;
	p_packet->Header.TotalLength = payloadLength + sizeof(MotPacket_Header_T);
	p_packet->Header.Status = status;
	p_packet->Header.Opt[0U] = 0U;
	p_packet->Header.Opt[1U] = 0U;
	p_packet->Header.Crc = Packet_CalcChecksum(p_packet);

	return p_packet->Header.TotalLength;
}

uint8_t _MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
	p_txPacket->Start = MOT_PACKET_START_BYTE;
	p_txPacket->SyncId = syncId;
	return 2U;
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
	return 	(
				(syncId == MOT_PACKET_STOP_ALL) || (syncId == MOT_PACKET_PING) ||
				(syncId == MOT_PACKET_SYNC_ACK) || (syncId == MOT_PACKET_SYNC_NACK) || (syncId == MOT_PACKET_SYNC_ABORT)
			) ?	_MotPacket_Sync_Build(p_txPacket, syncId) : 0U;
}

/******************************************************************************/
/*!
	Build Packet Functions
	@return size of Packet(TxLength)

	Parse Packet Functions
	@return Packet Header Status, parse Req/Cmd Status??
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
uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOT_PACKET_PING); }
// uint8_t MotPacket_PingReq_GetRespLength(void) { return sizeof(MotPacket_PingResp_T); }
void MotPacket_PingResp_Parse(uint8_t * p_version, const MotPacket_PingResp_T * p_respPacket)
{
	memcpy(p_version, &p_respPacket->VersionResp.Version[0U], 4U); //todo timestamp
	// return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Stop All */
/******************************************************************************/
uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOT_PACKET_STOP_ALL); }
// uint8_t MotPacket_StopReq_GetRespLength(void) { return sizeof(MotPacket_StopResp_T); }

// void MotPacket_StopResp_Parse(MotPacket_StatusResp_Id_T * p_status, const MotPacket_StopResp_T * p_respPacket)
// {
// 	*p_status = p_respPacket->Status.Id;
// 	// return p_respPacket->Header.Status;
// }

/******************************************************************************/
/*!	Save Nvm Params */
/******************************************************************************/
uint8_t MotPacket_SaveNvmReq_Build(MotPacket_SaveNvmReq_T * p_reqPacket)
{
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_SAVE_NVM, 0U, MOT_PACKET_HEADER_STATUS_OK);
}
// uint8_t MotPacket_SaveNvmReq_GetRespLength(void) { return sizeof(MotPacket_SaveNvmResp_T); }

/* returns NvMemory Write Status */
MotPacket_HeaderStatus_T MotPacket_SaveNvmResp_Parse(const MotPacket_SaveNvmResp_T * p_respPacket)
{
	return p_respPacket->Header.Status;
}

/******************************************************************************/
/*!	Read Var */
/******************************************************************************/
uint8_t MotPacket_ReadVarReq_Build(MotPacket_ReadVarReq_T * p_reqPacket, MotVarId_T motVarId)
{
	p_reqPacket->ReadReq.MotVarId = (uint16_t)motVarId;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVarReq_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_ReadVarReq_GetRespLength(void) { return sizeof(MotPacket_ReadVarResp_T); }

/*!
	@param[out] p_value 4-byte value uint32_t or int32_t
*/
MotPacket_HeaderStatus_T MotPacket_ReadVarResp_Parse(uint32_t * p_value, const MotPacket_ReadVarResp_T * p_respPacket)
{
	*p_value = p_respPacket->ReadResp.Value;
	return p_respPacket->Header.Status;	// *p_status = p_respPacket->ReadResp.Status;
}

/******************************************************************************/
/*!	Write Var */
/******************************************************************************/
uint8_t MotPacket_WriteVarReq_Build(MotPacket_WriteVarReq_T * p_reqPacket, MotVarId_T motVarId, uint32_t value)
{
	p_reqPacket->WriteReq.MotVarId = (uint16_t)motVarId;
	p_reqPacket->WriteReq.Value = value;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVarReq_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_WriteVarReq_GetRespLength(void) { return sizeof(MotPacket_WriteVarReq_T); }

MotPacket_HeaderStatus_T MotPacket_WriteVarResp_Parse(const MotPacket_WriteVarResp_T * p_respPacket)
{
	return p_respPacket->Header.Status;	// *p_status = p_respPacket->WriteResp.Status;
}

// /******************************************************************************/
// /*!	Read Var 16 */
// /******************************************************************************/
// uint8_t MotPacket_ReadVars16Req_Build
// (
// 	MotPacket_ReadVars16Req_T * p_respPacket,
	// void (* p_ReadVar)(void*),
// 	MotVarId_T motVarId, uint32_t value
// 	)
// {
// 	p_respPacket->ReadResp.Value = value;
// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVars16Req_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

// /******************************************************************************/
// /*!	Write Var 8 */
// /******************************************************************************/
// uint8_t MotPacket_WriteVars8Req_Build(MotPacket_WriteVars8Req_T * p_respPacket,  )
// {
// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVars8Req_Payload_T), status);
// }

/******************************************************************************/
/*!	Init Units */
/******************************************************************************/
// uint8_t MotPacket_InitUnitsReq_Build(MotPacket_InitUnitsReq_T * p_reqPacket)
// {
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_INIT_UNITS, 0U, MOT_PACKET_HEADER_STATUS_OK);
// }

// // uint8_t MotPacket_InitUnitsReq_GetRespLength(void) { return sizeof(MotPacket_InitUnitsResp_T); }

// void MotPacket_InitUnitsResp_Parse
// (
// 	uint16_t * p_speedRef_Rpm, uint16_t * p_iRef_Amp, uint16_t * p_vRef_Volts, /* Frac16 conversions */
// 	uint16_t * p_vSupply_R1, uint16_t * p_vSupply_R2,	/* Adcu <-> Volts conversions */
// 	uint16_t * p_vSense_R1, uint16_t * p_vSense_R2,
// 	uint16_t * p_vAcc_R1, uint16_t * p_vAcc_R2,
// 	const MotPacket_InitUnitsResp_T * p_respPacket
// )
// {
// 	// *p_speedRef_Rpm = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_iRef_Amp = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vRef_Volts = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vSupply_R1 = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vSupply_R2 = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vSense_R1 = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vSense_R2 = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vAcc_R1 = p_respPacket->InitUnitsResp.Data16s[0U];
// 	// *p_vAcc_R2 = p_respPacket->InitUnitsResp.Data16s[0U];

// 	// return p_respPacket->Header.Status;
// }


/******************************************************************************/
/*!	Control Type */
/******************************************************************************/
uint8_t MotPacket_ControlReq_Release_Build(MotPacket_ControlReq_T * p_reqPacket)
{
	p_reqPacket->ControlReq.ControlId = MOT_PACKET_CONTROL_RELEASE;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Release_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

uint8_t MotPacket_ControlReq_DirectionForward_Build(MotPacket_ControlReq_T * p_reqPacket)
{
	p_reqPacket->ControlReq.ControlId = MOT_PACKET_CONTROL_DIRECTION_FORWARD;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Direction_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

uint8_t MotPacket_ControlReq_DirectionReverse_Build(MotPacket_ControlReq_T * p_reqPacket)
{
	p_reqPacket->ControlReq.ControlId = MOT_PACKET_CONTROL_DIRECTION_REVERSE;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Direction_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

uint8_t MotPacket_ControlReq_DirectionNeutral_Build(MotPacket_ControlReq_T * p_reqPacket)
{
	p_reqPacket->ControlReq.ControlId = MOT_PACKET_CONTROL_DIRECTION_NEUTRAL;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Direction_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

uint8_t MotPacket_ControlReq_Throttle_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t throttleValue)
{
	p_reqPacket->ControlReq.ControlId = MOT_PACKET_CONTROL_THROTTLE;
	p_reqPacket->ControlReq.ValueU16s[0U] = throttleValue;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Throttle_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

uint8_t MotPacket_ControlReq_Brake_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t brakeValue)
{
	p_reqPacket->ControlReq.ControlId = MOT_PACKET_CONTROL_BRAKE;
	p_reqPacket->ControlReq.ValueU16s[0U] = brakeValue;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_CONTROL_TYPE, sizeof(MotPacket_ControlReq_Brake_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_ControlReq_Build(MotPacket_ControlReq_T * p_reqPacket, MotPacket_ControlId_T controlId, 15regs)
// {
// 	uint8_t packetLength;
// 	switch(controlId)
// 	{
// 		case MOT_PACKET_CONTROL_STOP: 		packetLength = 7U; 																		break;
// 		case MOT_PACKET_CONTROL_THROTTLE: 	packetLength = MotPacket_ControlReq_Throttle_Build(p_reqPacket, p_req->ValueU16s[0U]); 	break;
// 		case MOT_PACKET_CONTROL_BRAKE: 	packetLength = MotPacket_ControlReq_Brake_Build(p_reqPacket, p_req->ValueU16s[0U])		break;
// 		default: packetLength = 0U; break;
// 	}
// 	return packetLength;
// }

uint8_t MotPacket_ControlReq_GetRespLength(MotPacket_ControlId_T controlId)
{
	/* Status response only */
	(void)controlId;
	return 2U + sizeof(MotPacket_Header_T);

	/* Per Var Status */
	// switch(p_interface->ReqControl.ControlId)
	// {
	// 	case MOT_PACKET_CONTROL_STOP: 		 1U; 												break;
	// 	case MOT_PACKET_CONTROL_THROTTLE: 	 sizeof(MotPacket_ControlReq_Throttle_Payload_T); 	break;
	// 	case MOT_PACKET_CONTROL_BRAKE: 	 sizeof(MotPacket_ControlReq_Brake_Payload_T); 		break;
	// 	default: break;
	// }
}

/******************************************************************************/
/*!	Monitor Type */
/******************************************************************************/
uint8_t MotPacket_MonitorReq_Build(MotPacket_MonitorReq_T * p_reqPacket, MotPacket_MonitorId_T monitorId)
{
	p_reqPacket->MonitorReq.MonitorId = monitorId;
	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_MONITOR_TYPE, sizeof(MotPacket_MonitorReq_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_MonitorReq_Speed_Build(MotPacket_MonitorReq_T * p_reqPacket)
// {
// 	p_reqPacket->MonitorReq.MonitorId = MOT_PACKET_MONITOR_SPEED;
// 	return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_MONITOR_TYPE, sizeof(MotPacket_MonitorReq_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

uint8_t MotPacket_MonitorReq_GetRespLength(MotPacket_MonitorId_T monitorId)
{
	uint8_t respPayloadLength;

	switch(monitorId)
	{
		case MOT_PACKET_MONITOR_SPEED: 			respPayloadLength = sizeof(MotPacket_MonitorResp_Speed_Payload_T); 		break;
		case MOT_PACKET_MONITOR_I_FOC: 			respPayloadLength = sizeof(MotPacket_MonitorResp_IFoc_Payload_T); 		break;
		// case MOT_PACKET_MONITOR_ADC_BATCH_MSB: 	respPayloadLength = 16U; 	break;
		default: respPayloadLength = 0U; break;
	}

	return respPayloadLength + sizeof(MotPacket_Header_T);
}

/*!
	@param[out] p_speed_Fixed32 Q16.16
*/
void MotPacket_MonitorResp_Speed_Parse(int32_t * p_speed_Fixed32, const MotPacket_MonitorResp_T * p_respPacket)
{
	*p_speed_Fixed32 = ((MotPacket_MonitorResp_Speed_Payload_T *)&p_respPacket->MonitorResp)->Speed;
	// return p_respPacket->Header.Status;
}

void MotPacket_MonitorResp_IFoc_Parse
(
	int16_t * p_ia, int16_t * p_ib, int16_t * p_ic,
	int16_t * p_ialpha, int16_t * p_ibeta, int16_t * p_id, int16_t * p_iq,
	const MotPacket_MonitorResp_T * p_respPacket
)
{
	MotPacket_MonitorResp_IFoc_Payload_T * p_payload = (MotPacket_MonitorResp_IFoc_Payload_T *)&p_respPacket->MonitorResp;
	*p_ia = p_payload->Ia;	*p_ib = p_payload->Ib;	*p_ic = p_payload->Ic;
	*p_ialpha = p_payload->Ialpha;	*p_ibeta = p_payload->Ibeta; *p_id = p_payload->Id;	*p_iq = p_payload->Iq;
	// return p_respPacket->Header.Status;
}

// void MotPacket_MonitorResp_IPhases_Parse(int16_t * p_ia_FracS16, int16_t * p_ib_FracS16, int16_t * p_ic_FracS16, const MotPacket_MonitorResp_T * p_respPacket)
// {
// 	MotPacket_MonitorResp_IPhases_Payload_T * p_payload = (MotPacket_MonitorResp_IPhases_Payload_T *)&p_respPacket->MonitorResp;
// 	*p_ia_FracS16 = p_payload->PhaseA;
// 	*p_ib_FracS16 = p_payload->PhaseB;
// 	*p_ic_FracS16 = p_payload->PhaseC;
// 	// return p_respPacket->Header.Status;
// }

/******************************************************************************/
/*!
	Ctrlr side
*/
/******************************************************************************/
/******************************************************************************/
/*!	Ping Type */
/******************************************************************************/
//alternatively resp time stamp
uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket)
{
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VERSION, 0U, MOT_PACKET_HEADER_STATUS_OK);
}


/******************************************************************************/
/*!	Version Type */
/******************************************************************************/
uint8_t MotPacket_VersionResp_Build(MotPacket_VersionResp_T * p_respPacket)
{
	p_respPacket->VersionResp.Version[0U] = MOT_PACKET_VERSION_BUGFIX;
	p_respPacket->VersionResp.Version[1U] = MOT_PACKET_VERSION_MINOR;
	p_respPacket->VersionResp.Version[2U] = MOT_PACKET_VERSION_MAJOR;
	p_respPacket->VersionResp.Version[3U] = MOT_PACKET_VERSION_OPT;

	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

/******************************************************************************/
/*!	Stop Type */
/******************************************************************************/
uint8_t MotPacket_StopResp_Build(MotPacket_StopResp_T * p_respPacket)
{
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_STATUS, 0U, MOT_PACKET_HEADER_STATUS_OK);
}

/******************************************************************************/
/*!	Save Nvm Type */
/******************************************************************************/
uint8_t MotPacket_SaveNvmResp_Build(MotPacket_SaveNvmResp_T * p_respPacket, uint8_t status)
{
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_SAVE_NVM, 0U, status);
}

/******************************************************************************/
/*!	Read Var */
/******************************************************************************/
uint8_t MotPacket_ReadVarResp_Build(MotPacket_ReadVarResp_T * p_respPacket, uint32_t value)
{
	p_respPacket->ReadResp.Value = value;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVarReq_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

/******************************************************************************/
/*!	Write Var */
/******************************************************************************/
uint8_t MotPacket_WriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVarReq_Payload_T), status);
}

// /******************************************************************************/
// /*!	Read Var 16 */
// /******************************************************************************/
// uint8_t MotPacket_ReadVars16Resp_Build(MotPacket_ReadVars16Resp_T * p_respPacket, uint32_t value)
// {
// 	p_respPacket->ReadResp.Value = value;
// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVars16Req_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

// /******************************************************************************/
// /*!	Write Var 8 */
// /******************************************************************************/
// uint8_t MotPacket_WriteVars8Resp_Build(MotPacket_WriteVars8Resp_T * p_respPacket, MotPacket_HeaderStatus_T status)
// {
// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVars8Req_Payload_T), status);
// }

/******************************************************************************/
/*!	Control Type */
/******************************************************************************/
uint8_t MotPacket_ControlResp_MainStatus_Build(MotPacket_ControlResp_T * p_respPacket, uint16_t status)
{
	p_respPacket->ControlResp.MainStatus = status;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_CONTROL_TYPE, sizeof(uint16_t), MOT_PACKET_HEADER_STATUS_OK);
}

/******************************************************************************/
/*!	Monitor Type */
/******************************************************************************/
uint8_t MotPacket_MonitorResp_Speed_Build(MotPacket_MonitorResp_T * p_respPacket, int32_t speed)
{
	((MotPacket_MonitorResp_Speed_Payload_T *)&p_respPacket->MonitorResp)->Speed = speed;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_MONITOR_TYPE, sizeof(MotPacket_MonitorResp_Speed_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_MonitorResp_IPhases_Build(MotPacket_MonitorResp_T * p_respPacket, int16_t ia, int16_t ib, int16_t ic)
// {
// 	MotPacket_MonitorResp_IPhases_Payload_T * p_payload = (MotPacket_MonitorResp_IPhases_Payload_T *)&p_respPacket->MonitorResp;
// 	p_payload->PhaseA = ia;
// 	p_payload->PhaseB = ib;
// 	p_payload->PhaseC = ic;
// 	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_MONITOR_TYPE, sizeof(MotPacket_MonitorResp_Speed_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

uint8_t MotPacket_MonitorResp_IFoc_Build
(
	MotPacket_MonitorResp_T * p_respPacket,
	int16_t ia, int16_t ib, int16_t ic,
	int16_t ialpha, int16_t ibeta,
	int16_t id, int16_t iq
)
{
	MotPacket_MonitorResp_IFoc_Payload_T * p_payload = (MotPacket_MonitorResp_IFoc_Payload_T *)&p_respPacket->MonitorResp;
	p_payload->Ia = ia;	p_payload->Ib = ib;	p_payload->Ic = ic;
	p_payload->Ialpha = ialpha;	p_payload->Ibeta = ibeta;
	p_payload->Id = id;	p_payload->Iq = iq;
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_MONITOR_TYPE, sizeof(MotPacket_MonitorResp_IFoc_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

MotPacket_MonitorId_T MotPacket_MonitorReq_GetId(const MotPacket_MonitorReq_T * p_reqPacket)
{
	return p_reqPacket->MonitorReq.MonitorId;
}

MotPacket_HeaderId_T MotPacket_MonitorReq_Parse(MotPacket_MonitorId_T * p_monitorId, const MotPacket_MonitorReq_T * p_reqPacket)
{
	*p_monitorId = p_reqPacket->MonitorReq.MonitorId;
	return p_reqPacket->Header.Status;
}


/******************************************************************************/
/*!	Init Units Type */
/******************************************************************************/
uint8_t MotPacket_InitUnitsResp_Build
(
	MotPacket_InitUnitsResp_T * p_respPacket,
	uint16_t speedRef_Rpm, uint16_t iRef_Amp, uint16_t vRef_Volts, /* Frac16 conversions */
	uint16_t vSupply_R1, uint16_t vSupply_R2,	/* Adcu <-> Volts conversions */
	uint16_t vSense_R1, uint16_t vSense_R2,
	uint16_t vAcc_R1, uint16_t vAcc_R2
)
{
	p_respPacket->InitUnitsResp.U16s[0U] = speedRef_Rpm;
	p_respPacket->InitUnitsResp.U16s[0U] = iRef_Amp;
	p_respPacket->InitUnitsResp.U16s[0U] = vRef_Volts;
	p_respPacket->InitUnitsResp.U16s[0U] = vSupply_R1;
	p_respPacket->InitUnitsResp.U16s[0U] = vSupply_R2;
	p_respPacket->InitUnitsResp.U16s[0U] = vSense_R1;
	p_respPacket->InitUnitsResp.U16s[0U] = vSense_R2;
	p_respPacket->InitUnitsResp.U16s[0U] = vAcc_R1;
	p_respPacket->InitUnitsResp.U16s[0U] = vAcc_R2;

	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_INIT_UNITS, 18U, MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_InitUnitsResp_Build0();

// uint8_t MotPacket_InitUnitsResp_Build
// (
// 	MotProtocol_Substate_T * p_subState,
// 	MotPacket_InitUnitsResp_T * p_respPacket,
// 	uint16_t unitReference[16U]
// )
// {
// 	uint8_t txSize;

// 	switch(p_subState->StateId)
// 	{
// 		case 0U:
// 			// txSize = MotPacket_InitUnitsResp_Build0(p_respPacket, unitReference[0U], unitReference[1U], unitReference[2U]);
// 			p_subState->StateId = 1U;
// 			break;
// 	}

// 	return txSize;
// }


/******************************************************************************/
/*!	Read Data */
/******************************************************************************/
MotPacket_HeaderStatus_T MotPacket_ReadDataReq_Parse(uint32_t * p_addressStart, uint16_t * p_sizeBytes, const MotPacket_ReadDataReq_T * p_reqPacket)
{
	*p_addressStart = p_reqPacket->ReadDataReq.AddressStart;
	*p_sizeBytes = p_reqPacket->ReadDataReq.SizeBytes;
	return p_reqPacket->Header.Status;
}

uint8_t MotPacket_ReadDataResp_Build(MotPacket_ReadDataResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_DATA_MODE_READ, sizeof(MotPacket_ReadDataResp_Payload_T), status);
}

/******************************************************************************/
/*!	Write Data */
/******************************************************************************/
MotPacket_HeaderStatus_T MotPacket_WriteDataReq_Parse(uint32_t * p_addressStart, uint16_t * p_sizeBytes, const MotPacket_WriteDataReq_T * p_reqPacket)
{
	*p_addressStart = p_reqPacket->WriteDataReq.AddressStart;
	*p_sizeBytes = p_reqPacket->WriteDataReq.SizeBytes;
	return p_reqPacket->Header.Status;
}

uint8_t MotPacket_WriteDataResp_Build(MotPacket_WriteDataResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
	return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_DATA_MODE_WRITE, sizeof(MotPacket_WriteDataReq_Payload_T), status);
}

/******************************************************************************/
/*!	Data */
/******************************************************************************/
uint8_t MotPacket_DataPacket_Build(MotPacket_DataPacket_T * p_dataPacket, uint8_t * p_address, uint8_t sizeData)
{
	memcpy(&p_dataPacket->Data.Bytes[0U], p_address, sizeData);
	return Packet_BuildHeader((MotPacket_T *)p_dataPacket, MOT_PACKET_DATA_MODE_TYPE, sizeof(MotPacket_DataPacket_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

MotPacket_HeaderStatus_T MotPacket_DataPacket_Parse(const uint8_t ** pp_data, uint8_t * p_dataSize, const MotPacket_DataPacket_T * p_dataPacket)
{
	*pp_data = &p_dataPacket->Data.Bytes[0U];
	*p_dataSize = p_dataPacket->Header.TotalLength - sizeof(MotPacket_Header_T);
	return p_dataPacket->Header.Status;
}

// uint8_t MotPacket_DataPacket_ParseDataLength(const MotPacket_DataPacket_T * p_dataPacket)
// {
// 	return p_dataPacket->Header.TotalLength - sizeof(MotPacket_Header_T);
// }