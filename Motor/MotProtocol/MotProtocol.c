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
    MotPacket_Id_T syncChar;

    switch(txId)
    {
        case PROTOCOL_TX_SYNC_ACK_REQ:          syncChar = MOT_PACKET_SYNC_ACK;   break;
        case PROTOCOL_TX_SYNC_ACK_REQ_EXT:      syncChar = MOT_PACKET_SYNC_ACK;   break;
        case PROTOCOL_TX_SYNC_NACK_REQ:         syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_PACKET_META: syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_PACKET_DATA: syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT: syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT:  syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_REQ_EXT:     syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_ACK_ABORT:        syncChar = MOT_PACKET_SYNC_ABORT; break;
        // case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:    syncChar = MOT_PACKET_SYNC_NACK;         break;
        // case PROTOCOL_TX_SYNC_ABORT:         syncChar = MOT_PACKET_SYNC_ABORT; break;
        default: *p_txSize = 0U; syncChar = MOT_PACKET_ID_RESERVED_255; break;
    }

    *p_txSize = MotPacket_Sync_Build(p_txPacket, syncChar);
}

Protocol_RxCode_T MotProtocol_ParseRxMeta(Protocol_HeaderMeta_T * p_rxMeta, const MotPacket_T * p_rxPacket, size_t rxCount)
{
    volatile Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_AWAIT_PACKET;
    volatile Protocol_RxCode_T rxCode2 = PROTOCOL_RX_CODE_AWAIT_PACKET;
    rxCode = rxCode2;

    if(rxCount >= MOT_PACKET_LENGTH_BYTE_INDEX) /* length index is valid */
    {
        if(rxCount == MotPacket_ParseTotalLength(p_rxPacket)) /* Packet Complete */
        {
            p_rxMeta->ReqId = p_rxPacket->Header.Id;
            rxCode = (MotPacket_CheckChecksum(p_rxPacket) == true) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_ERROR_DATA;
        }
        else /* Packet Length Known */
        {
            p_rxMeta->Length = MotPacket_ParseTotalLength(p_rxPacket);
        }
    }
    else if(rxCount >= MOT_PACKET_LENGTH_MIN) /* Check Packet is Sync type */
    {
        switch(p_rxPacket->Header.Id)
        {
            // case MOT_PACKET_STOP_ALL:   rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE; p_rxMeta->ReqId = MOT_PACKET_STOP_ALL;   break;
            case MOT_PACKET_PING:       rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE; p_rxMeta->ReqId = MOT_PACKET_PING;       break;
            case MOT_PACKET_SYNC_ACK:   rxCode = PROTOCOL_RX_CODE_ACK;      break;
            case MOT_PACKET_SYNC_NACK:  rxCode = PROTOCOL_RX_CODE_NACK;     break;
            case MOT_PACKET_SYNC_ABORT: rxCode = PROTOCOL_RX_CODE_ABORT;    break;
            default: break;
        }
    }
    else /* (rxCount < MOT_PACKET_LENGTH_MIN) ParseMeta should not have been called */
    {
        rxCode = PROTOCOL_RX_CODE_ERROR_META;
    }

    return rxCode;
}

/******************************************************************************/
/*!
    Ctrlr Side Functions
*/
/******************************************************************************/




