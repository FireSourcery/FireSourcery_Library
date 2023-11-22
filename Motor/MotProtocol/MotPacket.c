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
    checkSum += CalcChecksum((uint8_t *)&p_packet->Header, sizeof(MotPacket_Header_T) - sizeof(p_packet->Header.Checksum));
    checkSum += CalcChecksum((uint8_t *)&p_packet->Payload[0U], p_packet->Header.Length - sizeof(MotPacket_Header_T));
    return checkSum;
}

static uint8_t BuildHeader(MotPacket_T * p_packet, MotPacket_Id_T id, uint8_t payloadLength)
{
    p_packet->Header.Start          = MOT_PACKET_START_BYTE;
    p_packet->Header.Id             = id;
    p_packet->Header.Length         = payloadLength + sizeof(MotPacket_Header_T);
    p_packet->Header.Checksum       = Packet_CalcChecksum(p_packet);
    return p_packet->Header.Length;
}

/*!
    @brief  Set header and build checksum. call last.
    @return size of full packet. Header + Payload
*/
static inline uint8_t Packet_BuildHeader(MotPacket_T * p_packet, MotPacket_Id_T headerId, uint8_t payloadLength)
{
    return BuildHeader(p_packet, headerId, payloadLength);
}

bool MotPacket_CheckChecksum(const MotPacket_T * p_packet)
{
    // static_assert(sizeof(p_packet->Header.Checksum) == sizeof(checksum_t));
    return ((checksum_t)Packet_CalcChecksum(p_packet) == p_packet->Header.Checksum);
}

uint8_t _MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId)
{
    p_txPacket->Start = MOT_PACKET_START_BYTE;
    p_txPacket->SyncId = syncId;
    return sizeof(MotPacket_Sync_T);
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId)
{
    return ((syncId == MOT_PACKET_PING) || (syncId == MOT_PACKET_SYNC_ACK) || (syncId == MOT_PACKET_SYNC_NACK) || (syncId == MOT_PACKET_SYNC_ABORT)) ?
        _MotPacket_Sync_Build(p_txPacket, syncId) : 0U;
}

// static inline void Packet_BuildInnerHeader(MotPacket_StatusResp_T * p_packet, uint16_t reqState, uint16_t status)
// {
//     p_packet->StatusResp.ReqState = reqState;
//     p_packet->StatusResp.Status = status;
// }

/******************************************************************************/
/*!
    Controller Side - Parse Req In, Build Resp Out
    Build Packet Functions
    @return size of Packet(TxLength)
*/
/******************************************************************************/
/******************************************************************************/
/*! Fixed Length */
/******************************************************************************/
/******************************************************************************/
/*! Ping */
/******************************************************************************/
uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket)
{
    return _MotPacket_Sync_Build((MotPacket_Sync_T *)p_respPacket, MOT_PACKET_SYNC_ACK);
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
uint8_t MotPacket_VersionResp_Build(MotPacket_VersionResp_T * p_respPacket, uint32_t library, uint32_t main, uint32_t board)
{
    p_respPacket->VersionResp.Version[0U] = MOT_PACKET_VERSION_BUGFIX;
    p_respPacket->VersionResp.Version[1U] = MOT_PACKET_VERSION_MINOR;
    p_respPacket->VersionResp.Version[2U] = MOT_PACKET_VERSION_MAJOR;
    p_respPacket->VersionResp.Version[3U] = MOT_PACKET_VERSION_OPT;
    p_respPacket->VersionResp.Library = library;
    p_respPacket->VersionResp.Main = main;
    p_respPacket->VersionResp.Board = board;

    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_Payload_T));
}

/******************************************************************************/
/*! Stop */
/******************************************************************************/
uint8_t MotPacket_StopResp_Build(MotPacket_StopResp_T * p_respPacket, uint16_t status)
{
    p_respPacket->StopResp.Status = status;
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_STOP_ALL, sizeof(MotPacket_StopResp_Payload_T));
}

/******************************************************************************/
/*! Call */
/******************************************************************************/
uint8_t MotPacket_CallResp_Build(MotPacket_CallResp_T * p_respPacket, uint16_t id, uint16_t status)
{
    p_respPacket->CallResp.Id = id;
    p_respPacket->CallResp.Status = status;
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_CALL, sizeof(MotPacket_CallResp_Payload_T));
}

/******************************************************************************/
/*! Variable Length */
/******************************************************************************/

//todo
// static uint16_t VarId_CalcChecksum(const MotPacket_T * p_packet)
// {
//     uint16_t checkSum = 0U;
//     checkSum += CalcChecksum((uint8_t *)&p_packet->Payload[0U], p_packet->Header.Length - sizeof(MotPacket_Header_T));
//     return checkSum;
// }
/******************************************************************************/
/*! ReadVars */
/******************************************************************************/
uint8_t MotPacket_VarReadReq_ParseVarIdCount(const MotPacket_VarReadReq_T * p_reqPacket)            { return (MotPacket_GetPayloadLength((MotPacket_T *)p_reqPacket) - 4U) / sizeof(uint16_t); }
uint16_t MotPacket_VarReadReq_ParseVarId(const MotPacket_VarReadReq_T * p_reqPacket, uint8_t index) { return p_reqPacket->VarReadReq.MotVarIds[index]; }

void MotPacket_VarReadResp_BuildVarValue(MotPacket_VarReadResp_T * p_respPacket, uint8_t index, uint16_t value) { p_respPacket->VarReadResp.Value16[index] = value; } /* pass index avoids double buffer */
void MotPacket_VarReadResp_BuildInnerHeader(MotPacket_VarReadResp_T * p_respPacket, uint16_t idChecksum, uint16_t status16)
{
    p_respPacket->VarReadResp.IdChecksum = idChecksum;
    p_respPacket->VarReadResp.Status16 = status16;
}
uint8_t MotPacket_VarReadResp_BuildHeader(MotPacket_VarReadResp_T * p_respPacket, uint8_t varsCount)
{
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VAR_READ, varsCount * sizeof(uint16_t) + 4U);
}

/******************************************************************************/
/*! WriteVars */
/******************************************************************************/
uint8_t MotPacket_VarWriteReq_ParseVarCount(const MotPacket_VarWriteReq_T * p_reqPacket)                    { return (MotPacket_GetPayloadLength((MotPacket_T *)p_reqPacket)  - 4U) / sizeof(uint16_t) / 2U; }
uint16_t MotPacket_VarWriteReq_ParseVarId(const MotPacket_VarWriteReq_T * p_reqPacket, uint8_t index)       { return p_reqPacket->VarWriteReq.Pairs[index].MotVarId; }
uint16_t MotPacket_VarWriteReq_ParseVarValue(const MotPacket_VarWriteReq_T * p_reqPacket, uint8_t index)    { return p_reqPacket->VarWriteReq.Pairs[index].Value16; }

void MotPacket_VarWriteResp_BuildVarStatus(MotPacket_VarWriteResp_T * p_respPacket, uint8_t index, uint16_t status) { p_respPacket->VarWriteResp.VarStatus[index] = status; }
void MotPacket_VarWriteResp_BuildInnerHeader(MotPacket_VarWriteResp_T * p_respPacket, uint16_t idChecksum, uint16_t status16)
{
    p_respPacket->VarWriteResp.IdChecksum = idChecksum;
    p_respPacket->VarWriteResp.Status16 = status16;
}
uint8_t MotPacket_VarWriteResp_BuildHeader(MotPacket_VarWriteResp_T * p_respPacket, uint8_t varsCount)
{
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_VAR_WRITE, varsCount * sizeof(uint8_t) + 4U);
}

/******************************************************************************/
/*!
    Stateful Read/Write
*/
/******************************************************************************/
/******************************************************************************/
/*! DataModeReq Read/Write Initial Common */
/******************************************************************************/
uint32_t MotPacket_DataModeReq_ParseAddress(const MotPacket_DataModeReq_T * p_reqPacket)  { return p_reqPacket->DataModeReq.AddressStart; }
uint32_t MotPacket_DataModeReq_ParseSize(const MotPacket_DataModeReq_T * p_reqPacket)     { return p_reqPacket->DataModeReq.SizeBytes; }
void MotPacket_DataModeReq_Parse(const MotPacket_DataModeReq_T * p_reqPacket, uint32_t * p_addressStart, uint32_t * p_sizeBytes)
{
    *p_addressStart = p_reqPacket->DataModeReq.AddressStart;
    *p_sizeBytes = p_reqPacket->DataModeReq.SizeBytes;
}

/******************************************************************************/
/*! ReadDataReq */
/******************************************************************************/
uint8_t MotPacket_DataModeReadResp_Build(MotPacket_DataModeResp_T * p_respPacket, uint16_t status)
{
    p_respPacket->DataModeResp.Status = status;
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_DATA_MODE_READ, 4U);
}

/******************************************************************************/
/*! WriteDataReq */
/******************************************************************************/
uint8_t MotPacket_DataModeWriteResp_Build(MotPacket_DataModeResp_T * p_respPacket, uint16_t status) // app status
{
    p_respPacket->DataModeResp.Status = status;
    return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_DATA_MODE_WRITE, 4U);
}

/******************************************************************************/
/*! Data */
/******************************************************************************/
void MotPacket_DataRead_BuildStatus(MotPacket_DataMode_T * p_dataPacket, uint16_t checksum, uint16_t sequence)
{
    p_dataPacket->DataMode.Checksum = checksum;
    p_dataPacket->DataMode.Sequence = sequence;
}

uint8_t MotPacket_DataRead_BuildData(MotPacket_DataMode_T * p_dataPacket, const uint8_t * p_address, uint8_t sizeData)
{
    memcpy(&p_dataPacket->DataMode.ByteData[0U], p_address, sizeData + 4U);
    return Packet_BuildHeader((MotPacket_T *)p_dataPacket, MOT_PACKET_DATA_MODE_DATA, sizeData + 4U);
}

uint16_t MotPacket_DataWrite_ParseChecksum(const MotPacket_DataMode_T * p_dataPacket)   { return p_dataPacket->DataMode.Checksum; }
uint16_t MotPacket_DataWrite_ParseSequence(const MotPacket_DataMode_T * p_dataPacket)   { return p_dataPacket->DataMode.Sequence; }
uint8_t MotPacket_DataWrite_ParseDataSize(const MotPacket_DataMode_T * p_dataPacket)    { return MotPacket_GetPayloadLength((MotPacket_T *)p_dataPacket) - 4U; }
const uint8_t * MotPacket_DataWrite_ParsePtrData(const MotPacket_DataMode_T * p_dataPacket)   { return &p_dataPacket->DataMode.ByteData[0U]; } /* Flash loader handle from here */


// /******************************************************************************/
// /*! Read Var */
// /******************************************************************************/
// uint16_t MotPacket_ReadVarReq_ParseVarId(const MotPacket_ReadVarReq_T * p_reqPacket) { return p_reqPacket->Header.Immediate16; }

// uint8_t MotPacket_ReadVarResp_Build(MotPacket_ReadVarResp_T * p_respPacket, uint32_t value)
// {
//     p_respPacket->ReadVarResp.Value32 = value; /* Upper 2-Bytes is written 0, if uint16 value. */
//     return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVarResp_Payload_T), MOT_PACKET_STATUS_OK);
// }

// /******************************************************************************/
// /*! Write Var */
// /******************************************************************************/
// uint16_t MotPacket_WriteVarReq_ParseVarId(const MotPacket_WriteVarReq_T * p_reqPacket) { return p_reqPacket->Header.Immediate16; }
// uint32_t MotPacket_WriteVarReq_ParseVarValue(const MotPacket_WriteVarReq_T * p_reqPacket) { p_reqPacket->WriteVarReq.Value32; }

// uint8_t MotPacket_WriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, uint16_t status)
// {
//     return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, 0U, status);
// }

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
//     return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_VERSION, 0, MOT_PACKET_STATUS_OK);
// }
// uint8_t MotPacket_VersionReq_GetRespLength(void) { return sizeof(MotPacket_VersionResp_T); }
// uint32_t MotPacket_VersionResp_Parse(const MotPacket_VersionResp_T * p_respPacket) { return (*(uint32_t *)&p_respPacket->VersionResp.Version[0U]); }


// /******************************************************************************/
// /*! Save Nvm Params */
// /******************************************************************************/
// uint8_t MotPacket_CallReq_Build(MotPacket_CallReq_T * p_reqPacket)
// {
//     return Packet_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_SAVE_PARAMS, 0U, MOT_PACKET_STATUS_OK);
// }
// uint8_t MotPacket_CallReq_GetRespLength(void) { return sizeof(MotPacket_CallResp_T); }
// /* returns NvMemory Write Status */
// MotPacket_HeaderStatus_T MotPacket_CallResp_Parse(const MotPacket_CallResp_T * p_respPacket) { return p_respPacket->Header.Status; }

// /******************************************************************************/
// /*! Read Var */
// /******************************************************************************/
// uint8_t MotPacket_ReadVarReq_Build(MotPacket_ReadVarReq_T * p_reqPacket, uint16_t motVarId)
// {
//     p_reqPacket->Header.Immediate16 = (uint16_t)motVarId;
//     return BuildHeaderOpt((MotPacket_T *)p_reqPacket, MOT_PACKET_READ_VAR, 0, MOT_PACKET_STATUS_OK);
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
//     return BuildHeaderOpt((MotPacket_T *)p_reqPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVarReq_Payload_T), MOT_PACKET_STATUS_OK);
// }

// // uint8_t MotPacket_WriteVarReq_GetRespLength(void) { return sizeof(MotPacket_WriteVarReq_T); }

// MotPacket_HeaderStatus_T MotPacket_WriteVarResp_Parse(const MotPacket_WriteVarResp_T * p_respPacket)
// {
//     return p_respPacket->Header.Status;    // *p_status = p_respPacket->WriteResp.Status;
// }

// /******************************************************************************/
// /*! Read Vars */
// /******************************************************************************/
// uint8_t MotPacket_VarReadReq_Build
// (
//     MotPacket_VarReadReq_T * p_respPacket,
//     void (* p_ReadVar)(void*),
//     uint16_t motVarId, uint32_t value
//     )
// {
//     p_respPacket->ReadResp.Value = value;
//     return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_VarReadReq_Payload_T), MOT_PACKET_STATUS_OK);
// }

// /******************************************************************************/
// /*! Write Vars */
// /******************************************************************************/
// uint8_t MotPacket_VarWrite8Req_Build(MotPacket_VarWriteReq_T * p_respPacket,  )
// {
//     // return Packet_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_VarWriteReq_Payload_T), status);
// }

