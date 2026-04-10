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

    @brief
*/
/******************************************************************************/
#include "MotPacket.h"
#include "../Version.h"

#include "Utility/Protocol/Packet/Packet.h"

#include <string.h>
#include <stddef.h>
#include <assert.h>

/******************************************************************************/
/*!
    Common
*/
/******************************************************************************/
static uint16_t Sum(const uint8_t * p_src, size_t size)
{
    uint16_t checksum = 0U;
    for(size_t iByte = 0U; iByte < size; iByte++) { checksum += p_src[iByte]; }
    return checksum;
}

uint16_t Packet_Checksum(const MotPacket_T * p_packet, size_t totalSize)
{
    static const uint8_t checksumStart = offsetof(MotPacket_Header_T, Checksum);
    static const uint8_t checksumSize = sizeof(checksum_t);
    static const uint8_t checksumEnd = checksumStart + checksumSize;

    uint16_t checkSum = 0U;
    checkSum += Sum(&(p_packet->Bytes[0U]), checksumStart);
    checkSum += Sum(&(p_packet->Bytes[checksumEnd]), totalSize - checksumEnd);
    return checkSum;
}

bool MotPacket_ProcChecksum(const MotPacket_T * p_packet, size_t totalSize)
{
    return (Packet_Checksum(p_packet, totalSize) == p_packet->Header.Checksum);
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId)
{
    assert((syncId == MOT_PACKET_PING) || (syncId == MOT_PACKET_SYNC_ACK) || (syncId == MOT_PACKET_SYNC_NACK) || (syncId == MOT_PACKET_SYNC_ABORT));
    p_txPacket->Start = MOT_PACKET_START_BYTE;
    p_txPacket->SyncId = syncId;
    p_txPacket->Control = 0U; /* reserved */
    p_txPacket->Checksum = p_txPacket->Start ^ p_txPacket->SyncId ^ p_txPacket->Control;
    return sizeof(MotPacket_Sync_T);
}

// static inline uint8_t MotPacket_BuildFixed(MotPacket_Fixed_T * p_packet, MotPacket_Id_T headerId, uint8_t payloadLength)
// {
//     p_packet->Header.Start = MOT_PACKET_START_BYTE;
//     p_packet->Header.Id = headerId;
//     p_packet->Header.Checksum = Packet_Checksum(p_packet);
//     return payloadLength + sizeof(MotPacket_Fixed_T);
// }

/*!
    @brief  Set header and build checksum. call last.
    @return size of full packet. Header + Payload
*/
uint8_t MotPacket_BuildHeader(MotPacket_T * p_packet, MotPacket_Id_T headerId, uint8_t payloadLength)
{
    p_packet->Header.Start = MOT_PACKET_START_BYTE;
    p_packet->Header.Id = headerId;
    p_packet->Header.Length = payloadLength + sizeof(MotPacket_Header_T);
    p_packet->Header.Sequence = 0U;
    // p_packet->Header.Flags = 0U;
    p_packet->Header.Checksum = Packet_Checksum(p_packet, payloadLength + sizeof(MotPacket_Header_T));
    return p_packet->Header.Length;
}




/******************************************************************************/
/*!
    Controller Side - Parse Req In, Build Resp Out
    Build Packet Functions - pass all parameters
    Parse Packet Functions - return single value, or double buffer
    @return size of Packet(TxLength)
*/
/******************************************************************************/
/******************************************************************************/
/*! Fixed Length */
/******************************************************************************/
/******************************************************************************/
/*! Ping */
/******************************************************************************/
uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket, MotPacket_Id_T syncId)
{
    return MotPacket_Sync_Build((MotPacket_Sync_T *)p_respPacket, syncId);
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
uint8_t MotPacket_VersionResp_Build(MotPacket_T * p_packet, uint32_t firmware)
{
    MotPacket_VersionResp_T * p_payload = (MotPacket_VersionResp_T *)p_packet->Payload;
    p_payload->Protocol = MOT_PACKET_VERSION_WORD32;
    p_payload->Library  = MOTOR_LIBRARY_VERSION;
    p_payload->Firmware = firmware;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_VERSION, sizeof(MotPacket_VersionResp_T));
}

/******************************************************************************/
/*! Stop */
/******************************************************************************/
uint8_t MotPacket_StopResp_Build(MotPacket_T * p_packet, uint16_t status)
{
    ((MotPacket_StopResp_T *)p_packet->Payload)->Status = status;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_STOP_ALL, sizeof(MotPacket_StopResp_T));
}

/******************************************************************************/
/*! Call */
/******************************************************************************/
uint8_t MotPacket_CallResp_Build(MotPacket_T * p_packet, uint32_t id, uint16_t status)
{
    MotPacket_CallResp_T * p_payload = (MotPacket_CallResp_T *)p_packet->Payload;
    p_payload->Id     = id;
    p_payload->Status = status;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_CALL, sizeof(MotPacket_CallResp_T));
}

/******************************************************************************/
/*! Read Var */
/******************************************************************************/
// uint8_t MotPacket_ReadVarResp_Build(MotPacket_ReadVarResp_T * p_respPacket, uint32_t value)
// {
//     p_respPacket->ReadVarResp.Value32 = value; /* Upper 2-Bytes is written 0, if uint16 value. */
//     return MotPacket_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_ReadVarResp_T), MOT_PACKET_STATUS_OK);
// }

/******************************************************************************/
/*! Write Var */
/******************************************************************************/
// uint8_t MotPacket_WriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, uint16_t status)
// {
//     return MotPacket_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, 0U, status);
// }

/******************************************************************************/
/*!
    Variable Length
*/
/******************************************************************************/
uint8_t MotPacket_VersionFlexResp_Build(MotPacket_T * p_packet, uint32_t * p_versions, uint8_t count)
{
    uint8_t size = count * sizeof(uint32_t);
    memcpy(p_packet->Payload, p_versions, size);
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_VERSION, size);
}

/******************************************************************************/
/*!
    Read/Write Vars
*/
/******************************************************************************/

/******************************************************************************/
/*! ReadVars */
/******************************************************************************/
uint8_t MotPacket_VarReadReq_ParseVarIdCount(const MotPacket_T * p_packet) { return MotPacket_ParsePayloadLength(p_packet) / sizeof(uint16_t); }

uint8_t MotPacket_VarReadResp_BuildHeader(MotPacket_T * p_packet, uint8_t varsCount) { return MotPacket_BuildHeader(p_packet, MOT_PACKET_VAR_READ, varsCount * sizeof(uint16_t)); }

// static uint16_t VarId_Checksum(const MotPacket_T * p_packet, uint8_t varCount) { return Checksum((uint8_t *)&p_packet->Payload[0U], varCount * 2U); }
// void MotPacket_VarReadResp_BuildVarValue(MotPacket_T * p_packet, uint8_t index, uint16_t value) { ((MotPacket_VarReadResp_T *)p_packet->Payload)->Value16[index] = value; }
// void MotPacket_VarReadResp_BuildMeta(MotPacket_VarReadResp_T * p_respPacket, uint8_t varsStatus)
// {
//     // p_respPacket->Header.Imm16 = idChecksum;
//     // p_respPacket->VarReadResp.Status16 = status16;
// }

/******************************************************************************/
/*! WriteVars */
/******************************************************************************/
uint8_t MotPacket_VarWriteReq_ParseVarCount(const MotPacket_T * p_packet) { return MotPacket_ParsePayloadLength(p_packet) / sizeof(uint16_t) / 2U; }

uint8_t MotPacket_VarWriteResp_BuildHeader(MotPacket_T * p_packet, uint8_t varsCount) { return MotPacket_BuildHeader(p_packet, MOT_PACKET_VAR_WRITE, varsCount * sizeof(uint8_t)); }

// void MotPacket_VarWriteResp_BuildVarStatus(MotPacket_T * p_packet, uint8_t index, uint16_t status) { ((MotPacket_VarWriteResp_T *)p_packet->Payload)->VarStatus[index] = status; }
// void MotPacket_VarWriteResp_BuildMeta(MotPacket_VarWriteResp_T * p_respPacket, uint16_t status16)
// {
//     p_respPacket->Header.Imm16  = idChecksum;
//     // p_respPacket->VarWriteResp.headerStatus = status16;
// }

/******************************************************************************/
/*! Mem */
/******************************************************************************/
uint8_t MotPacket_MemWriteResp_Build(MotPacket_T * p_packet, uint16_t status)
{
    ((MotPacket_MemWriteResp_T *)p_packet->Payload)->Status = status;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_MEM_WRITE, sizeof(MotPacket_MemWriteResp_T));
}

/* Data filled by caller. No double buffering */
uint8_t MotPacket_MemReadResp_BuildHeader(MotPacket_T * p_packet, uint8_t size, uint16_t status)
{
    // p_packet->Header.Flags = status;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_MEM_READ, size);
}

uint8_t MotPacket_MemReadResp_Build(MotPacket_T * p_packet, const uint8_t * p_data, uint8_t size, uint16_t status)
{
    memcpy(p_packet->Payload, p_data, size);
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_MEM_READ, size);
}

/******************************************************************************/
/*! Stateful Read/Write */
/******************************************************************************/
/******************************************************************************/
/*! DataModeReq Read/Write Initial Common */
/******************************************************************************/
uint8_t MotPacket_DataModeReadResp_Build(MotPacket_T * p_packet, uint16_t status)
{
    ((MotPacket_DataModeResp_T *)p_packet->Payload)->Status = status;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_DATA_MODE_READ, sizeof(MotPacket_DataModeResp_T));
}

uint8_t MotPacket_DataModeWriteResp_Build(MotPacket_T * p_packet, uint16_t status)
{
    ((MotPacket_DataModeResp_T *)p_packet->Payload)->Status = status;
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_DATA_MODE_WRITE, sizeof(MotPacket_DataModeResp_T));
}

/******************************************************************************/
/*! Data */
/******************************************************************************/
uint8_t MotPacket_ByteData_Build(MotPacket_T * p_packet, const uint8_t * p_data, uint8_t size)
{
    memcpy(p_packet->Payload, p_data, size);
    return MotPacket_BuildHeader(p_packet, MOT_PACKET_DATA_MODE_DATA, size);
}

uint8_t MotPacket_ByteData_ParseSize(const MotPacket_T * p_packet) { return MotPacket_ParsePayloadLength(p_packet); }



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
//     return MotPacket_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_VERSION, 0, MOT_PACKET_STATUS_OK);
// }
// uint8_t MotPacket_VersionReq_GetRespLength(void) { return sizeof(MotPacket_VersionResp_T); }
// uint32_t MotPacket_VersionResp_Parse(const MotPacket_VersionResp_T * p_respPacket) { return (*(uint32_t *)&p_respPacket->VersionResp.Version[0U]); }

// /******************************************************************************/
// /*! Save Nvm Config */
// /******************************************************************************/
// uint8_t MotPacket_CallReq_Build(MotPacket_CallReq_T * p_reqPacket)
// {
//     return MotPacket_BuildHeader((MotPacket_T *)p_reqPacket, MOT_PACKET_SAVE_CONFIG, 0U, MOT_PACKET_STATUS_OK);
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
//     return BuildHeaderOpt((MotPacket_T *)p_reqPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_WriteVarReq_T), MOT_PACKET_STATUS_OK);
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
//     return MotPacket_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_READ_VAR, sizeof(MotPacket_VarReadReq_T), MOT_PACKET_STATUS_OK);
// }

// /******************************************************************************/
// /*! Write Vars */
// /******************************************************************************/
// uint8_t MotPacket_VarWrite8Req_Build(MotPacket_VarWriteReq_T * p_respPacket,  )
// {
//     // return MotPacket_BuildHeader((MotPacket_T *)p_respPacket, MOT_PACKET_WRITE_VAR, sizeof(MotPacket_VarWriteReq_T), status);
// }

