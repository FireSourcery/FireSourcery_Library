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

#include "Framework/Protocol/Packet.h"

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

uint8_t MotPacket_Sync_Build(MotPacket_Control_T * p_txPacket, MotPacket_Id_T syncId)
{
    assert((syncId == MOT_PACKET_PING) || (syncId == MOT_PACKET_SYNC_ACK) || (syncId == MOT_PACKET_SYNC_NACK) || (syncId == MOT_PACKET_SYNC_ABORT));
    p_txPacket->Start = MOT_PACKET_START_BYTE;
    p_txPacket->SyncId = syncId;
    p_txPacket->Flex = 0U; /* reserved */
    p_txPacket->Flags = p_txPacket->Start ^ p_txPacket->SyncId ^ p_txPacket->Flex;
    return sizeof(MotPacket_Control_T);
}

// static inline uint8_t MotPacket_BuildFixed(MotPacket_HeaderShort_T * p_packet, MotPacket_Id_T headerId, uint8_t payloadLength)
// {
//     p_packet->Start = MOT_PACKET_START_BYTE;
//     p_packet->Id = headerId;
//     p_packet->Checksum = Packet_Checksum(p_packet);
//     return payloadLength + sizeof(MotPacket_HeaderShort_T);
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
    p_packet->Header.Checksum = MotPacket_Checksum(p_packet, payloadLength + sizeof(MotPacket_Header_T));
    return p_packet->Header.Length;
}

/******************************************************************************/
/*!
    Packet Interface - the codec bound into MOT_PROTOCOL_PACKET_CLASS

    Two frame shapes, selected by Id alone:
        sync    4 bytes,  MotPacket_Control_T,  Start ^ SyncId ^ Flex, no payload
        data    8 bytes,  MotPacket_Header_T,   16-bit sum, variable payload

    Packet_Meta_T.Length is the PAYLOAD length in both. MotPacket_Header_T.Length is the TOTAL
    frame length on the wire, so the two differ by sizeof(MotPacket_Header_T), and this codec
    is the only place that conversion happens.
*/
/******************************************************************************/
static const Packet_FrameFormat_T MOT_FRAME_SYNC = { .HEADER_LENGTH = sizeof(MotPacket_Control_T),  .BODY_LENGTH = 0U, .TRAILER_LENGTH = 0U };
static const Packet_FrameFormat_T MOT_FRAME_DATA = { .HEADER_LENGTH = sizeof(MotPacket_Header_T),   .BODY_LENGTH = 0U, .TRAILER_LENGTH = 0U };
// static const Packet_FrameFormat_T MOT_FRAME_SHORT = { .HEADER_LENGTH = sizeof(MotPacket_Control_T),   .BODY_LENGTH = 0U, .TRAILER_LENGTH = 0U };

/*
    The set of ids carrying the 4-byte shape. Must match MotPacket_ParseLength exactly: that
    sizes the frame, and this decides how the same bytes are then read. A disagreement between
    the two is what the engine's frame-consistency check exists to catch.
*/
static inline bool IsSyncShape(packet_id_t id)
{
    switch ((MotPacket_Id_T)id)
    {
        case MOT_PACKET_SYNC_ACK:
        case MOT_PACKET_SYNC_NACK:
        case MOT_PACKET_SYNC_ABORT:
        case MOT_PACKET_PING:
        case MOT_PACKET_PING_BOOT:
        case MOT_PACKET_PING_ALT:   return true;
        default:                    return false;
    }
}

// packet_size_t _MotPacket_ParseLength(const uint8_t rxLeading[MOT_PACKET_LENGTH_MIN])

/*! Phase 1. Total frame length, or 0 while not yet determinable. */
// known after min
packet_size_t MotPacket_ParseLength(const MotPacket_T * p_rxPacket, packet_size_t rxCount)
{
    switch (p_rxPacket->Header.Id)
    {
        // Sync packets — complete immediately, no checksum verification needed
        case MOT_PACKET_SYNC_ACK:   return sizeof(MotPacket_Control_T);
        case MOT_PACKET_SYNC_NACK:  return sizeof(MotPacket_Control_T);
        case MOT_PACKET_SYNC_ABORT: return sizeof(MotPacket_Control_T);
        case MOT_PACKET_PING:       return sizeof(MotPacket_Control_T);
        case MOT_PACKET_PING_BOOT:  return sizeof(MotPacket_Control_T);
        case MOT_PACKET_PING_ALT:   return sizeof(MotPacket_Control_T);

            /*
                Fixed length, mapped from the id rather than trusted from the length field.

                The MotPacket_*Req_T types are PAYLOAD structs, not whole packets, so the
                header has to be added. Returning the payload size alone made STOP_ALL and
                VERSION resolve to 0 - which this function reserves for "not yet
                determinable", so the parser grew the header a byte at a time to LENGTH_MAX
                and then rejected the frame - and made CALL an 8-byte frame whose checksum
                was then computed over half of itself.
            */
        case MOT_PACKET_STOP_ALL:           return sizeof(MotPacket_Header_T) + sizeof(MotPacket_StopReq_T);
        case MOT_PACKET_VERSION:            return sizeof(MotPacket_Header_T) + sizeof(MotPacket_VersionReq_T);
            // case MOT_PACKET_REBOOT:      return sizeof(MotPacket_Header_T) + sizeof(MotPacket_CallReq_T);
        case MOT_PACKET_CALL:               return sizeof(MotPacket_Header_T) + sizeof(MotPacket_CallReq_T);
        case MOT_PACKET_FIXED_VAR_READ:     return sizeof(MotPacket_Header_T) + sizeof(MotPacket_VarReadFixedReq_T);
        case MOT_PACKET_FIXED_VAR_WRITE:    return sizeof(MotPacket_Header_T) + sizeof(MotPacket_VarWriteFixedReq_T);

        // Data packets — set length, await remaining bytes
        default:
            if (rxCount < offsetof(MotPacket_Header_T, Length))
            {
                return 0;
            }
            else
            {
                return _MotPacket_FrameLength(p_rxPacket);
            }
    }
}

/*! Phase 2. Integrity over the complete frame. Length alone separates the two shapes. */
bool MotProtocol_IsRxValid(const MotPacket_T * p_packet, packet_size_t length)
{
    if (length == sizeof(MotPacket_Control_T))
    {
        // const MotPacket_Control_T * p_sync = (const MotPacket_Control_T *)p_buffer;
        // return (p_sync->Checksum == (uint8_t)(p_sync->Start ^ p_sync->SyncId ^ p_sync->Flex));
        return true;
    }

    return (MotPacket_Checksum(p_packet, length) == p_packet->Header.Checksum);
}

/*!
    Phase 2. The only source of Meta.Id, and so of the frame's class.
    @return NULL when the header cannot describe a frame - the engine nacks and counts it.
*/
Packet_FrameFormat_T * MotProtocol_ParseRxHeader(Packet_Meta_T * p_meta, const MotPacket_T * p_packet)
{

    p_meta->Id = p_packet->Header.Id;   /* offset 1 in both shapes */

    if (IsSyncShape(p_meta->Id) == true)
    {
        p_meta->Length = 0U;
        return (Packet_FrameFormat_T *)&MOT_FRAME_SYNC;
    }

    /* A total shorter than its own header describes nothing. Reject before the subtraction. */
    if (p_packet->Header.Length < sizeof(MotPacket_Header_T)) { return NULL; }

    p_meta->Length   = (packet_size_t)(p_packet->Header.Length - sizeof(MotPacket_Header_T));
    p_meta->Sequence = p_packet->Header.Sequence;
    p_meta->Flags    = p_packet->Header.Flags;
    return (Packet_FrameFormat_T *)&MOT_FRAME_DATA;
}

/*! Symmetric with PARSE_RX_HEADER. Called after the handler has written its payload. */
Packet_FrameFormat_T * MotProtocol_BuildTxHeader(const Packet_Meta_T * p_meta, MotPacket_T * p_buffer)
{
    if (IsSyncShape(p_meta->Id) == true)
    {
        (void)MotPacket_Sync_Build((MotPacket_Control_T *)p_buffer, (MotPacket_Id_T)p_meta->Id);
        return (Packet_FrameFormat_T *)&MOT_FRAME_SYNC;
    }

    (void)MotPacket_BuildHeader((MotPacket_T *)p_buffer, (MotPacket_Id_T)p_meta->Id, p_meta->Length);
    return (Packet_FrameFormat_T *)&MOT_FRAME_DATA;
}

const Packet_Codec_T MOT_PROTOCOL_PACKET_CLASS =
{
    .LENGTH_MIN         = MOT_PACKET_LENGTH_MIN,
    .LENGTH_MAX         = MOT_PACKET_LENGTH_MAX,
    .START_ID           = MOT_PACKET_START_BYTE,
    .START_ID_LENGTH    = 1U,

    .PARSE_RX_LENGTH    = (Packet_ParseRxLength_T)MotPacket_ParseLength,
    .IS_RX_VALID        = (Packet_ValidateRx_T)MotProtocol_IsRxValid,
    .PARSE_RX_HEADER    = (Packet_ParseRxHeader_T)MotProtocol_ParseRxHeader,
    .BUILD_TX_HEADER    = (Packet_BuildTxHeader_T)MotProtocol_BuildTxHeader,

    .ACK_ID             = MOT_PACKET_SYNC_ACK,
    .NACK_ID            = MOT_PACKET_SYNC_NACK,
    .ABORT_ID           = MOT_PACKET_SYNC_ABORT,
};