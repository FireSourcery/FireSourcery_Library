/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
    @file     MotPacket.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "MotProtocol.h"

#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*!
    Common functions, not requiring appInterface, mapping directly to Protocol Specs
*/
/******************************************************************************/
void MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId)
{
    MotPacket_HeaderId_T syncId;

    switch(txId)
    {
        case PROTOCOL_TX_SYNC_ACK_REQ_ID:            syncId = MOT_PACKET_SYNC_ACK;         break;
        case PROTOCOL_TX_SYNC_ACK_REQ_EXT:            syncId = MOT_PACKET_SYNC_ACK;         break;
        case PROTOCOL_TX_SYNC_NACK_REQ_ID:            syncId = MOT_PACKET_SYNC_NACK;         break;
        case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:    syncId = MOT_PACKET_SYNC_NACK;         break;
        case PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT:        syncId = MOT_PACKET_SYNC_NACK;         break;
        case PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT:        syncId = MOT_PACKET_SYNC_NACK;         break;
        case PROTOCOL_TX_SYNC_NACK_REQ_EXT:            syncId = MOT_PACKET_SYNC_NACK;         break;
        case PROTOCOL_TX_SYNC_ABORT:                syncId = MOT_PACKET_SYNC_ABORT;     break;
        default: *p_txSize = 0U; syncId = MOT_PACKET_ID_RESERVED_255; break;
    }

    *p_txSize = MotPacket_Sync_Build(p_txPacket, syncId);
}

Protocol_RxCode_T MotProtocol_ParseRxMeta(protocol_reqid_t * p_reqId, size_t * p_packetLength, const MotPacket_T * p_rxPacket, size_t rxCount)
{
    Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_WAIT_PACKET;

     /* Move PACKET_LENGTH_INDEX to protocol module handle, for cases where header length index defined */
    if(rxCount >= MOT_PACKET_LENGTH_BYTE_INDEX)
    {
        if(rxCount == MotPacket_ParseTotalLength(p_rxPacket)) /* Packet Complete */
        {
            *p_reqId = p_rxPacket->Header.HeaderId;
            rxCode = (MotPacket_CheckChecksum(p_rxPacket) == true) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_PACKET_ERROR;
        }
        else /* Packet Length Known */
        {
            *p_packetLength = MotPacket_ParseTotalLength(p_rxPacket);
        }
    }
    else if(rxCount >= MOT_PACKET_LENGTH_MIN) /* Packet is Sync type */
    {
        switch(p_rxPacket->Header.HeaderId)
        {
            case MOT_PACKET_STOP_ALL:        rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE; *p_reqId = MOT_PACKET_STOP_ALL;    break;
            case MOT_PACKET_PING:            rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE; *p_reqId = MOT_PACKET_PING;        break;
            case MOT_PACKET_SYNC_ACK:        rxCode = PROTOCOL_RX_CODE_ACK;                 break;
            case MOT_PACKET_SYNC_NACK:        rxCode = PROTOCOL_RX_CODE_NACK;             break;
            case MOT_PACKET_SYNC_ABORT:        rxCode = PROTOCOL_RX_CODE_ABORT;             break;
            default: break;
        }
    }
    else /* ParseMeta should not have been called */
    {
        rxCode = PROTOCOL_RX_CODE_PACKET_ERROR;
    }

    return rxCode;
}

/******************************************************************************/
/*!
    Ctrlr Side Functions
    Memory Access Functions Does not require
*/
/******************************************************************************/




