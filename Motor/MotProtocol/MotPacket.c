/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotPacket.c
    @author FireSourcery
    @version V0
    @brief
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

static uint8_t BuildHeader(MotPacket_T * p_packet, MotPacket_HeaderId_T headerId, uint8_t payloadLength, uint8_t status)
{
    p_packet->Header.Start          = MOT_PACKET_START_BYTE;
    p_packet->Header.HeaderId       = headerId;
    p_packet->Header.TotalLength    = payloadLength + sizeof(MotPacket_Header_T);
    p_packet->Header.Status         = status;
    p_packet->Header.Crc            = Packet_CalcChecksum(p_packet);
    return p_packet->Header.TotalLength;
}

/*!
    @brief  Set header and build checksum. call last.
    @return size of full packet. Header + Payload
*/
static inline uint8_t Packet_BuildHeader(MotPacket_T * p_packet, MotPacket_HeaderId_T headerId, uint8_t payloadLength, uint8_t status)
{
    p_packet->Header.Immediate16 = 0U;
    return BuildHeader(p_packet, headerId, payloadLength, status);
}

static inline uint8_t Packet_BuildHeader_Imm(MotPacket_T * p_packet, MotPacket_HeaderId_T headerId, uint8_t payloadLength, uint8_t status, uint8_t imm0, uint8_t imm1)
{
    p_packet->Header.Immediate[0U] = imm0;
    p_packet->Header.Immediate[1U] = imm1;
    return BuildHeader(p_packet, headerId, payloadLength, status);
}

static inline uint8_t Packet_BuildHeader_Imm16(MotPacket_T * p_packet, MotPacket_HeaderId_T headerId, uint8_t payloadLength, uint8_t status, uint16_t imm16)
{
    p_packet->Header.Immediate16 = imm16;
    return BuildHeader(p_packet, headerId, payloadLength, status);
}

uint8_t _MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
    p_txPacket->Start = MOT_PACKET_START_BYTE;
    p_txPacket->SyncId = syncId;
    return sizeof(MotPacket_Sync_T);
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId)
{
    return ((syncId == MOT_PACKET_STOP_ALL) || (syncId == MOT_PACKET_PING) || (syncId == MOT_PACKET_SYNC_ACK) || (syncId == MOT_PACKET_SYNC_NACK) || (syncId == MOT_PACKET_SYNC_ABORT))
        ? _MotPacket_Sync_Build(p_txPacket, syncId) : 0U;
}

/******************************************************************************/
/*!
    Build Packet Functions @return size of Packet(TxLength)
    Parse Packet Functions @return Payload Parsed
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Ctrlr side
    Parse Req In, Build Resp Out
*/
/******************************************************************************/
/******************************************************************************/
/*! Ping Type */
/******************************************************************************/
uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket)
{
    return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_respPacket, MOT_PACKET_SYNC_ACK); //alternatively resp time stamp
}

/******************************************************************************/
/*! Stop Type */
/******************************************************************************/
uint8_t MotPacket_StopResp_Build(MotPacket_StopResp_T * p_respPacket) //may need stateful check todo
{
    // return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_STATUS, 0U, MOT_PACKET_HEADER_STATUS_OK);
    return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_respPacket, MOT_PACKET_SYNC_ACK);
}

/******************************************************************************/
/*! Version Type */
/******************************************************************************/
uint8_t MotPacket_VersionResp_Build(MotPacket_VersionResp_T * p_respPacket) // , uint32_t software, uint32_t flex)
{
    p_respPacket->VersionResp.Version[0U] = MOT_PACKET_VERSION_BUGFIX;
    p_respPacket->VersionResp.Version[1U] = MOT_PACKET_VERSION_MINOR;
    p_respPacket->VersionResp.Version[2U] = MOT_PACKET_VERSION_MAJOR;
    p_respPacket->VersionResp.Version[3U] = MOT_PACKET_VERSION_OPT;

    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

// uint8_t MotPacket_VersionResp_BuildExtended(MotPacket_VersionResp_T * p_respPacket, uint32_t software, uint32_t flex)
// {
//     p_respPacket->VersionResp.Version[0U] = MOT_PACKET_VERSION_BUGFIX;
//     p_respPacket->VersionResp.Version[1U] = MOT_PACKET_VERSION_MINOR;
//     p_respPacket->VersionResp.Version[2U] = MOT_PACKET_VERSION_MAJOR;
//     p_respPacket->VersionResp.Version[3U] = MOT_PACKET_VERSION_OPT;

//     // p_respPacket->VersionResp.Software[0U] = MotorController_User_GetLibraryVersionIndex(uint8_t charIndex);
//     // p_respPacket->VersionResp.Software[1U] = 0;
//     // p_respPacket->VersionResp.Software[2U] = 0;
//     // p_respPacket->VersionResp.Software[3U] = 0;

//     // p_respPacket->VersionResp.Flex0[0U] = MotorController_User_GetMainVersionIndex(p_mc, uint8_t charIndex);
//     // p_respPacket->VersionResp.Flex1[1U] = 0;
//     // p_respPacket->VersionResp.Flex2[2U] = 0;
//     // p_respPacket->VersionResp.Flex3[3U] = 0;

//     return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

/******************************************************************************/
/*! Save Nvm Type */
/******************************************************************************/
uint8_t MotPacket_SaveNvmResp_Build(MotPacket_SaveNvmResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_SAVE_NVM, 0U, status);
}

/******************************************************************************/
/*! Read Var */
/******************************************************************************/
uint16_t MotPacket_ReadVarReq_ParseVarId(const MotPacket_ReadVarReq_T * p_reqPacket) { return p_reqPacket->Header.Immediate16; }

uint8_t MotPacket_ReadVarResp_Build(MotPacket_ReadVarResp_T * p_respPacket, uint32_t value)
{
    p_respPacket->ReadVarResp.Value32 = value; /* Upper 2-Bytes is written 0, if uint16 value. */
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVarResp_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
}

/******************************************************************************/
/*! Write Var */
/******************************************************************************/
uint16_t MotPacket_WriteVarReq_ParseVarId(const MotPacket_WriteVarReq_T * p_reqPacket) { return p_reqPacket->Header.Immediate16; }
uint32_t MotPacket_WriteVarReq_ParseVarValue(const MotPacket_WriteVarReq_T * p_reqPacket) { p_reqPacket->WriteVarReq.Value32; }

uint8_t MotPacket_WriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, 0U, status);
}

/******************************************************************************/
/*! Read Var16s */
/******************************************************************************/
uint8_t MotPacket_ReadVars16Req_ParseVarIdsCount(const MotPacket_ReadVars16Req_T * p_reqPacket) { return MotPacket_ParsePayloadLength((MotPacket_T *)p_reqPacket) / sizeof(uint16_t); }
uint16_t MotPacket_ReadVars16Req_ParseVarId(const MotPacket_ReadVars16Req_T * p_reqPacket, uint8_t index) { return p_reqPacket->ReadVars16Req.MotVarIds[index]; }
/* avoids double buffer */
void MotPacket_ReadVars16Resp_BuildVarValue(MotPacket_ReadVars16Resp_T * p_respPacket, uint8_t index, uint16_t value) { p_respPacket->ReadVars16Resp.Value16[index] = value; }

uint8_t MotPacket_ReadVars16Resp_BuildHeader(MotPacket_ReadVars16Resp_T * p_respPacket, uint8_t varsCount)
{
    //IdCheckSum to guard against cmd mismatch
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VARS16, varsCount * sizeof(uint16_t), MOT_PACKET_HEADER_STATUS_OK);
}

/******************************************************************************/
/*! Write Var16s */
/******************************************************************************/
uint8_t MotPacket_WriteVars16Req_ParseVarIdsCount(const MotPacket_WriteVars16Req_T * p_reqPacket) { return MotPacket_ParsePayloadLength((MotPacket_T *)p_reqPacket) / sizeof(uint16_t) / 2U; }
uint16_t MotPacket_WriteVars16Req_ParseVarId(const MotPacket_WriteVars16Req_T * p_reqPacket, uint8_t index) { return p_reqPacket->WriteVars16Req.Vars[index].MotVarId; }
uint16_t MotPacket_WriteVars16Req_ParseVarValue(const MotPacket_WriteVars16Req_T * p_reqPacket, uint8_t index) { return p_reqPacket->WriteVars16Req.Vars[index].Value16; }

void MotPacket_WriteVars16Resp_BuildVarStatus(MotPacket_WriteVars16Resp_T * p_respPacket, uint8_t index, MotPacket_HeaderStatus_T status)
{

}

uint8_t MotPacket_WriteVars16Resp_BuildHeader(MotPacket_WriteVars16Resp_T * p_respPacket, MotPacket_HeaderStatus_T status, uint8_t varsCount)
{
    //IdCheckSum to guard against cmd mismatch
    // todo individual status
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VARS16, sizeof(MotPacket_WriteVars16Resp_Payload_T), status);
}

/******************************************************************************/
/*!
    Stateful Read/Write
*/
/******************************************************************************/
/******************************************************************************/
/*! Read Data */
/******************************************************************************/
void MotPacket_ReadDataReq_Parse(const MotPacket_ReadDataReq_T * p_reqPacket, uint32_t * p_addressStart, uint32_t * p_sizeBytes)
{
    *p_addressStart = p_reqPacket->ReadDataReq.AddressStart;
    *p_sizeBytes = p_reqPacket->ReadDataReq.SizeBytes;
}

uint32_t MotPacket_ReadDataReq_ParseAddress(const MotPacket_ReadDataReq_T * p_reqPacket)    { return p_reqPacket->ReadDataReq.AddressStart; }
uint32_t MotPacket_ReadDataReq_ParseSize(const MotPacket_ReadDataReq_T * p_reqPacket)       { return p_reqPacket->ReadDataReq.SizeBytes; }

uint8_t MotPacket_ReadDataResp_Build(MotPacket_ReadDataResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_DATA_MODE_READ, 0, status);
}

/******************************************************************************/
/*! Write Data */
/******************************************************************************/
void MotPacket_WriteDataReq_Parse(const MotPacket_WriteDataReq_T * p_reqPacket, uint32_t * p_addressStart, uint32_t * p_sizeBytes)
{
    *p_addressStart = p_reqPacket->WriteDataReq.AddressStart;
    *p_sizeBytes = p_reqPacket->WriteDataReq.SizeBytes;
}

uint32_t MotPacket_WriteDataReq_ParseAddress(const MotPacket_WriteDataReq_T * p_reqPacket)  { return p_reqPacket->WriteDataReq.AddressStart; }
uint32_t MotPacket_WriteDataReq_ParseSize(const MotPacket_WriteDataReq_T * p_reqPacket)     { return p_reqPacket->WriteDataReq.SizeBytes; }

uint8_t MotPacket_WriteDataResp_Build(MotPacket_WriteDataResp_T * p_respPacket, MotPacket_HeaderStatus_T status)
{
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_DATA_MODE_WRITE, 0, status);
}

/******************************************************************************/
/*! Data */
/******************************************************************************/
uint8_t MotPacket_Data_Build(MotPacket_DataReqResp_T * p_dataPacket, const uint8_t * p_address, uint8_t sizeData)
{
    memcpy(&p_dataPacket->Data.Bytes[0U], p_address, sizeData);
    return Packet_BuildHeader((MotPacket_T *)p_dataPacket, MOT_PACKET_DATA_MODE_DATA, sizeData, MOT_PACKET_HEADER_STATUS_OK);
}

const uint8_t * MotPacket_Data_ParsePtrPayload(const MotPacket_DataReqResp_T * p_dataPacket) { return &p_dataPacket->Data.Bytes[0U]; }
uint8_t MotPacket_Data_ParseSize(const MotPacket_DataReqResp_T * p_dataPacket) { return MotPacket_ParsePayloadLength((MotPacket_T *)p_dataPacket); }

/******************************************************************************/
/*! Ext Sequence Batch */
/******************************************************************************/
uint8_t MotPacket_BatchResp_Build(MotPacket_BatchResp_T * p_respPacket, MotPacket_ExtId_T id)
{
    p_respPacket->BatchResp;
    return Packet_BuildHeader_Imm((MotPacket_T *)p_respPacket, MOT_PACKET_EXT_CMD, sizeof(uint16_t), MOT_PACKET_HEADER_STATUS_OK, id, 0U);
}


/******************************************************************************/
/*!
    Cmdr side
    Build Req Out, Parse Resp In
*/
/******************************************************************************/
/******************************************************************************/
/*! Stop All */
/******************************************************************************/
// uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOT_PACKET_STOP_ALL); }
// // uint8_t MotPacket_StopReq_GetRespLength(void) { return sizeof(MotPacket_StopResp_T); }
// MotPacket_HeaderStatus_T MotPacket_StopResp_Parse(const MotPacket_StopResp_T * p_respPacket) { return p_respPacket->Header.Status; }

// /******************************************************************************/
// /*! Ping */
// /******************************************************************************/
// uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket) { return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_reqPacket, MOT_PACKET_PING); }
// // uint8_t MotPacket_PingReq_GetRespLength(void) { return sizeof(MotPacket_PingResp_T); }
// MotPacket_HeaderId_T MotPacket_PingResp_Parse(const MotPacket_PingResp_T * p_respPacket){ return p_respPacket->SyncId; }
// // todo timestamp

// /******************************************************************************/
// /*! Version */
// /******************************************************************************/
// uint8_t MotPacket_VersionReq_Build(MotPacket_VersionReq_T * p_reqPacket)
// {
//     return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_VERSION, 0, MOT_PACKET_HEADER_STATUS_OK);
// }
// uint8_t MotPacket_VersionReq_GetRespLength(void) { return sizeof(MotPacket_VersionResp_T); }
// uint32_t MotPacket_VersionResp_Parse(const MotPacket_VersionResp_T * p_respPacket) { return (*(uint32_t *)&p_respPacket->VersionResp.Version[0U]); }


// /******************************************************************************/
// /*! Save Nvm Params */
// /******************************************************************************/
// uint8_t MotPacket_SaveNvmReq_Build(MotPacket_SaveNvmReq_T * p_reqPacket)
// {
//     return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_SAVE_NVM, 0U, MOT_PACKET_HEADER_STATUS_OK);
// }
// uint8_t MotPacket_SaveNvmReq_GetRespLength(void) { return sizeof(MotPacket_SaveNvmResp_T); }
// /* returns NvMemory Write Status */
// MotPacket_HeaderStatus_T MotPacket_SaveNvmResp_Parse(const MotPacket_SaveNvmResp_T * p_respPacket) { return p_respPacket->Header.Status; }

// /******************************************************************************/
// /*! Read Var */
// /******************************************************************************/
// uint8_t MotPacket_ReadVarReq_Build(MotPacket_ReadVarReq_T * p_reqPacket, uint16_t motVarId)
// {
//     p_reqPacket->Header.Immediate16 = (uint16_t)motVarId;
//     return BuildHeaderOpt((MotPacket_T *)p_reqPacket, MOT_PACKET_READ_VAR, 0, MOT_PACKET_HEADER_STATUS_OK);
// }
// uint8_t MotPacket_ReadVarReq_GetRespLength(void) { return sizeof(MotPacket_ReadVarResp_T); }

// /*!
//     @param[out] p_value 4-byte value uint32_t or int32_t
// */
// MotPacket_HeaderStatus_T MotPacket_ReadVarResp_Parse(const MotPacket_ReadVarResp_T * p_respPacket, uint32_t * p_value)
// {
//     *p_value = p_respPacket->ReadVarResp.Value32;
//     return p_respPacket->Header.Status;    // *p_status = p_respPacket->ReadResp.Status;
// }

// /******************************************************************************/
// /*! Write Var */
// /******************************************************************************/
// uint8_t MotPacket_WriteVarReq_Build(MotPacket_WriteVarReq_T * p_reqPacket, uint16_t motVarId, uint32_t value)
// {
//     p_reqPacket->Header.MotVarId = (uint16_t)motVarId;


//     p_reqPacket->WriteVarReq.Value32 = value;
//     return BuildHeaderOpt((MotPacket_T *)p_reqPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVarReq_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

// // uint8_t MotPacket_WriteVarReq_GetRespLength(void) { return sizeof(MotPacket_WriteVarReq_T); }

// MotPacket_HeaderStatus_T MotPacket_WriteVarResp_Parse(const MotPacket_WriteVarResp_T * p_respPacket)
// {
//     return p_respPacket->Header.Status;    // *p_status = p_respPacket->WriteResp.Status;
// }

// /******************************************************************************/
// /*! Read Var16s */
// /******************************************************************************/
// uint8_t MotPacket_ReadVars16Req_Build
// (
//     MotPacket_ReadVars16Req_T * p_respPacket,
//     void (* p_ReadVar)(void*),
//     uint16_t motVarId, uint32_t value
//     )
// {
//     p_respPacket->ReadResp.Value = value;
//     return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVars16Req_Payload_T), MOT_PACKET_HEADER_STATUS_OK);
// }

// /******************************************************************************/
// /*! Write Var16s */
// /******************************************************************************/
// uint8_t MotPacket_WriteVars8Req_Build(MotPacket_WriteVars16Req_T * p_respPacket,  )
// {
//     // return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVars16Req_Payload_T), status);
// }


// /******************************************************************************/
// /*! Batch */
// /******************************************************************************/
// uint8_t MotPacket_ControlReq_Build(MotPacket_ControlReq_T * p_reqPacket, MotPacket_ExtId_T ExtId, 15regs)
// {
//     uint8_t packetLength;
//     switch(ExtId)
//     {
//         case MOT_PACKET_CONTROL_STOP:         packetLength = 7U;                                                                         break;
//         case MOT_PACKET_CONTROL_THROTTLE:     packetLength = MotPacket_ControlReq_Throttle_Build(p_reqPacket, p_req->ValueU16s[0U]);     break;
//         case MOT_PACKET_CONTROL_BRAKE:     packetLength = MotPacket_ControlReq_Brake_Build(p_reqPacket, p_req->ValueU16s[0U])        break;
//         default: packetLength = 0U; break;
//     }
//     return packetLength;
// }

// uint8_t MotPacket_ControlReq_GetRespLength(MotPacket_ExtId_T ExtId)
// {
//     /* Status response only */
//     (void)ExtId;
//     return 2U + sizeof(MotPacket_Header_T);

//     /* Per Var Status */
//     // switch(p_interface->ReqControl.ExtId)
//     // {
//     //     case MOT_PACKET_CONTROL_STOP:          1U;                                                 break;
//     //     case MOT_PACKET_CONTROL_THROTTLE:      sizeof(MotPacket_ControlReq_Throttle_Payload_T);     break;
//     //     case MOT_PACKET_CONTROL_BRAKE:      sizeof(MotPacket_ControlReq_Brake_Payload_T);         break;
//     //     default: break;
//     // }
// }

// uint8_t MotPacket_MonitorReq_GetRespLength(MotPacket_ExtId_T monitorId)
// {
//     uint8_t respPayloadLength;

//     switch(monitorId)
//     {
//         case MOT_PACKET_MONITOR_SPEED:             respPayloadLength = sizeof(MotPacket_MonitorResp_Speed_Payload_T);         break;
//         case MOT_PACKET_MONITOR_I_FOC:             respPayloadLength = sizeof(MotPacket_MonitorResp_IFoc_Payload_T);         break;
//         // case MOT_PACKET_MONITOR_ADC_BATCH_MSB:     respPayloadLength = 16U;     break;
//         default: respPayloadLength = 0U; break;
//     }

//     return respPayloadLength + sizeof(MotPacket_Header_T);
// }

// /*!
//     @param[out] p_speed_Fixed32 Q16.16
// */
// void MotPacket_MonitorResp_Speed_Parse(int32_t * p_speed_Fixed32, const MotPacket_MonitorResp_T * p_respPacket)
// {
//     *p_speed_Fixed32 = ((MotPacket_MonitorResp_Speed_Payload_T *)&p_respPacket->MonitorResp)->Speed;
//     // return p_respPacket->Header.Status;
// }

// void MotPacket_MonitorResp_IFoc_Parse
// (
//     int16_t * p_ia, int16_t * p_ib, int16_t * p_ic,
//     int16_t * p_ialpha, int16_t * p_ibeta, int16_t * p_id, int16_t * p_iq,
//     const MotPacket_MonitorResp_T * p_respPacket
// )
// {
//     MotPacket_MonitorResp_IFoc_Payload_T * p_payload = (MotPacket_MonitorResp_IFoc_Payload_T *)&p_respPacket->MonitorResp;
//     *p_ia = p_payload->Ia;    *p_ib = p_payload->Ib;    *p_ic = p_payload->Ic;
//     *p_ialpha = p_payload->Ialpha;    *p_ibeta = p_payload->Ibeta; *p_id = p_payload->Id;    *p_iq = p_payload->Iq;
//     // return p_respPacket->Header.Status;
// }

// void MotPacket_MonitorResp_IPhases_Parse(int16_t * p_ia_FracS16, int16_t * p_ib_FracS16, int16_t * p_ic_FracS16, const MotPacket_MonitorResp_T * p_respPacket)
// {
//     MotPacket_MonitorResp_IPhases_Payload_T * p_payload = (MotPacket_MonitorResp_IPhases_Payload_T *)&p_respPacket->MonitorResp;
//     *p_ia_FracS16 = p_payload->PhaseA;
//     *p_ib_FracS16 = p_payload->PhaseB;
//     *p_ic_FracS16 = p_payload->PhaseC;
//     // return p_respPacket->Header.Status;
// }

/******************************************************************************/
/*! Init Units */
/******************************************************************************/
// uint8_t MotPacket_InitUnitsReq_Build(MotPacket_InitUnitsReq_T * p_reqPacket)
// {
//     return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_INIT_UNITS, 0U, MOT_PACKET_HEADER_STATUS_OK);
// }

// // uint8_t MotPacket_InitUnitsReq_GetRespLength(void) { return sizeof(MotPacket_InitUnitsResp_T); }

// void MotPacket_InitUnitsResp_Parse
// (
//     uint16_t * p_speedRef_Rpm, uint16_t * p_iRef_Amp, uint16_t * p_vRef_Volts, /* Frac16 conversions */
//     uint16_t * p_vSupply_R1, uint16_t * p_vSupply_R2,    /* Adcu <-> Volts conversions */
//     uint16_t * p_vSense_R1, uint16_t * p_vSense_R2,
//     uint16_t * p_vAcc_R1, uint16_t * p_vAcc_R2,
//     const MotPacket_InitUnitsResp_T * p_respPacket
// )
// {
//     // *p_speedRef_Rpm = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_iRef_Amp = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vRef_Volts = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vSupply_R1 = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vSupply_R2 = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vSense_R1 = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vSense_R2 = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vAcc_R1 = p_respPacket->InitUnitsResp.Data16s[0U];
//     // *p_vAcc_R2 = p_respPacket->InitUnitsResp.Data16s[0U];

//     // return p_respPacket->Header.Status;
// }