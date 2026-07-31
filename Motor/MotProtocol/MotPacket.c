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


// known after min
size_t MotPacket_ParseLength(const MotPacket_T * p_rxPacket, packet_size_t rxCount)
{
    switch (p_rxPacket->Header.Id)
    {
        // Sync packets — complete immediately, no checksum verification needed
        case MOT_PACKET_SYNC_ACK:   return sizeof(MotPacket_Sync_T);
        case MOT_PACKET_SYNC_NACK:  return sizeof(MotPacket_Sync_T);
        case MOT_PACKET_SYNC_ABORT: return sizeof(MotPacket_Sync_T);
        case MOT_PACKET_PING:       return sizeof(MotPacket_Sync_T);
        case MOT_PACKET_PING_BOOT:  return sizeof(MotPacket_Sync_T);
        case MOT_PACKET_PING_ALT:   return sizeof(MotPacket_Sync_T);

            /* fixed length, directly map id */
        case MOT_PACKET_STOP_ALL:       return sizeof(MotPacket_StopReq_T);     break;
        case MOT_PACKET_VERSION:        return sizeof(MotPacket_VersionReq_T);  break;
            // case MOT_PACKET_REBOOT:        return sizeof(MotPacket_CallReq_T);     break;
        case MOT_PACKET_CALL:               return sizeof(MotPacket_CallReq_T);     break;
        case MOT_PACKET_FIXED_VAR_READ:     return sizeof(MotPacket_VarReadFixedReq_T); break;
        case MOT_PACKET_FIXED_VAR_WRITE:    return sizeof(MotPacket_VarWriteFixedReq_T); break;

        // Data packets — set length, await remaining bytes
        default:
            if (rxCount < offsetof(MotPacket_Header_T, Length))
            {
                return 0;
            }
            else
            {
                return MotPacket_ParseTotalLength(p_rxPacket);
            }
    }
}

// on complete
packet_id_t MotPacket_ParseId(const MotPacket_T * p_rxPacket, packet_size_t rxCount)
{
    return (p_rxPacket->Header.Id);
}