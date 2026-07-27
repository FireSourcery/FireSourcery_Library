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

#include "Framework/Protocol/Packet/Packet.h"

#include <string.h>
#include <stddef.h>
#include <assert.h>

/******************************************************************************/
/*!
    Common
*/
/******************************************************************************/
uint16_t MotPacket_Checksum(const MotPacket_T * p_packet, size_t totalSize)
{
    return Packet_Checksum((const uint8_t *)p_packet, totalSize, offsetof(MotPacket_Header_T, Checksum), sizeof(p_packet->Header.Checksum));
}

uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId)
{
    assert((syncId == MOT_PACKET_PING) || (syncId == MOT_PACKET_SYNC_ACK) || (syncId == MOT_PACKET_SYNC_NACK) || (syncId == MOT_PACKET_SYNC_ABORT));
    p_txPacket->Start = MOT_PACKET_START_BYTE;
    p_txPacket->SyncId = syncId;
    p_txPacket->Flex = 0U; /* reserved */
    p_txPacket->Checksum = p_txPacket->Start ^ p_txPacket->SyncId ^ p_txPacket->Flex;
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
    p_packet->Header.Flags = 0U;
    // p_packet->Header.Checksum = Packet_Checksum(p_packet, payloadLength + sizeof(MotPacket_Header_T));
    p_packet->Header.Checksum = MotPacket_Checksum(p_packet, payloadLength + sizeof(MotPacket_Header_T));
    return p_packet->Header.Length;
}


/*
    todo depreciate for build header
*/
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


/******************************************************************************/
/*! WriteVars */
/******************************************************************************/
uint8_t MotPacket_VarWriteReq_ParseVarCount(const MotPacket_T * p_packet) { return MotPacket_ParsePayloadLength(p_packet) / sizeof(uint16_t) / 2U; }

uint8_t MotPacket_VarWriteResp_BuildHeader(MotPacket_T * p_packet, uint8_t varsCount) { return MotPacket_BuildHeader(p_packet, MOT_PACKET_VAR_WRITE, varsCount * sizeof(uint8_t)); }



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



