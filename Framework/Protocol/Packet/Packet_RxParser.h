#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Packet.h
    @author FireSourcery
    @brief  Packet Interface
*/
/******************************************************************************/
#include "Packet.h"
#include <stddef.h>

typedef enum Protocol_RxState
{
    PROTOCOL_RX_STATE_INACTIVE,
    PROTOCOL_RX_STATE_WAIT_BYTE_1, /* SYNC */
    PROTOCOL_RX_STATE_WAIT_LENGTH, /* HEADER */
    PROTOCOL_RX_STATE_WAIT_PACKET, /* DATA */
    // PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
    //     PROTOCOL_PARSE_STATE_SYNC,          /* Looking for sync/start marker */
    //     PROTOCOL_PARSE_STATE_HEADER,        /* Parsing header */
    //     PROTOCOL_PARSE_STATE_PAYLOAD,       /* Parsing payload */
    //     PROTOCOL_PARSE_STATE_CHECKSUM,      /* Parsing checksum/CRC */
    //     PROTOCOL_PARSE_STATE_COMPLETE,      /* Packet complete */
    //     PROTOCOL_PARSE_STATE_ERROR          /* Error state */
}
Protocol_RxState_T;


// typedef const struct Packet_Context
// {
//     const Packet_Format_T * P_FORMAT;
//     uint8_t * P_BUFFER;
//     Protocol_HeaderMeta_T * P_META;
// }
// Packet_Context_T;

// typedef const struct
// {
//     const Packet_Format_T * P_FORMAT;
//     uint8_t * P_BUFFER;
//     Protocol_HeaderMeta_T * P_META;
//     const Packet_BuildTxHeader_T  BUILD_TX_HEADER;  // symmetric with Phase 2
//     const Packet_BuildTxSync_T    BUILD_TX_SYNC;
// }
// Protocol_TxContext_T;

// typedef const struct
// {
//     const Packet_Format_T * P_FORMAT;
//     uint8_t * P_BUFFER;
//     Protocol_HeaderMeta_T * P_META;
//     Packet_RxParserState_T * P_RX_STATE;
//     const Packet_ParseRxFraming_T PARSE_RX_FRAMING;  // Phase 1: Id + Length
//     const Packet_ParseRxHeader_T  PARSE_RX_HEADER;   // Phase 2: checksum + fields extraction
// }
// Protocol_RxContext_T;

typedef struct Packet_RxParserState
{
    Protocol_RxState_T RxState;
    packet_size_t RxIndex;          /* Index into P_RX_PACKET_BUFFER, number of bytes received */
    uint32_t RxTimeStart;
    Protocol_HeaderMeta_T RxMeta;   /* Rx Parse Packet Meta */

    // // alternatively seperate parser state. copy header copies 1 extra field.
    // packet_size_t RxIndex;          /* Index into P_RX_PACKET_BUFFER, number of bytes received */
    // packet_id_t Id;                 /* Packet type identifier. Index into P_REQ_TABLE */
    // packet_size_t Length;           /* Total packet length */
}
Packet_RxParserState_T;


static inline Protocol_RxCode_T Packet_ProcRxState(const Packet_Format_T * p_specs, const uint8_t * p_rxBuffer, Packet_RxParserState_T * p_state)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;

    switch (p_state->RxState)
    {
        case PROTOCOL_RX_STATE_WAIT_BYTE_1:
            if (p_state->RxIndex > 0U)
            {
                if ((p_rxBuffer[0U] == p_specs->RX_START_ID) || (p_specs->RX_START_ID == 0x00U))
                {
                    p_state->RxMeta.Length = 0U;
                    // p_state->RxTimeStart = *p_socket->P_TIMER;
                    p_state->RxState = PROTOCOL_RX_STATE_WAIT_LENGTH;
                }
                else
                {
                    p_state->RxIndex = 0U; // reset and keep waiting
                }
            }
            break;

        case PROTOCOL_RX_STATE_WAIT_LENGTH:
            if (p_state->RxIndex >= p_specs->RX_LENGTH_MIN)
            {
                rxStatus = p_specs->PARSE_RX_FRAMING(p_rxBuffer, p_state->RxIndex, &p_state->RxMeta);
                switch (rxStatus)
                {
                    case PROTOCOL_RX_CODE_AWAIT_PACKET:     // Length now set, need payload
                        p_state->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
                        break;
                    case PROTOCOL_RX_CODE_ACK:              // Sync — complete, no Phase 2
                    case PROTOCOL_RX_CODE_NACK:
                    case PROTOCOL_RX_CODE_ABORT:
                    case PROTOCOL_RX_CODE_PACKET_COMPLETE:  // Header only packer / Ping — complete, no Phase 2
                        p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
                        break;
                    case PROTOCOL_RX_CODE_ERROR_META:
                        // TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_PACKET_META);
                        p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
                        break;
                    default:
                        p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
                        break;
                }
            }
            break;

        case PROTOCOL_RX_STATE_WAIT_PACKET:
            if (p_state->RxIndex >= p_state->RxMeta.Length)
            {
                rxStatus = p_specs->PARSE_RX_HEADER(p_rxBuffer, &p_state->RxMeta);   // PACKET_COMPLETE or ERROR_DATA
                switch (rxStatus)
                {
                    case PROTOCOL_RX_CODE_PACKET_COMPLETE: break;
                    case PROTOCOL_RX_CODE_ERROR_DATA: break; // TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_PACKET_DATA);
                    default: break;
                }
                p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
            }
            break;

        case PROTOCOL_RX_STATE_INACTIVE:
            break;
    }


    return rxStatus;
}





static inline Protocol_RxState_T _Packet_RxStateOf(const Packet_Format_T * p_specs, size_t rxCount)
{
    if (rxCount == 0U) { return PROTOCOL_RX_STATE_WAIT_BYTE_1; }
    else if (rxCount < p_specs->RX_LENGTH_MIN) { return PROTOCOL_RX_STATE_WAIT_LENGTH; }
    // else if (rxCount < p_state->RxMeta.Length) { return PROTOCOL_RX_STATE_WAIT_PACKET; }
    else if (rxCount < p_specs->RX_LENGTH_MAX) { return PROTOCOL_RX_STATE_WAIT_PACKET; }
    else { return PROTOCOL_RX_STATE_WAIT_BYTE_1; } /* Invalid length, reset */
}

static inline Protocol_RxCode_T _Packet_ProcRxParser(const Packet_Format_T * p_specs, const uint8_t * p_rxBuffer, packet_size_t * p_RxIndex, Protocol_HeaderMeta_T * p_rxMeta)
{
    Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_AWAIT_PACKET;
    switch (_Packet_RxStateOf(p_specs, *p_RxIndex))
    {
        case PROTOCOL_RX_STATE_WAIT_BYTE_1:
            if (*p_RxIndex > 0U)
            {
                if ((p_rxBuffer[0U] == p_specs->RX_START_ID) || (p_specs->RX_START_ID == 0x00U)) { p_rxMeta->Length = 0U; }
                else { *p_RxIndex = 0U; }                // reset and keep waiting
            }
            break;
        case PROTOCOL_RX_STATE_WAIT_LENGTH:   if (*p_RxIndex >= p_specs->RX_LENGTH_MIN) { rxStatus = p_specs->PARSE_RX_FRAMING(p_rxBuffer, *p_RxIndex, p_rxMeta); }  break;
        case PROTOCOL_RX_STATE_WAIT_PACKET:   if (*p_RxIndex >= p_rxMeta->Length) { rxStatus = p_specs->PARSE_RX_HEADER(p_rxBuffer, p_rxMeta); } break; // PACKET_COMPLETE or ERROR_DATA
        case PROTOCOL_RX_STATE_INACTIVE:    break;
        default: break;
    }
    if (rxStatus != PROTOCOL_RX_CODE_AWAIT_PACKET) { *p_RxIndex = 0U; } //   p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
}



// typedef enum Protocol_RxCode
// {
//     // Success codes
//     PROTOCOL_RX_CODE_AWAIT_PACKET = 0x00,  /* Continue receiving */
//     PROTOCOL_RX_CODE_PACKET_COMPLETE = 0x01,  /* Complete packet received */
//     PROTOCOL_RX_CODE_PACKET_FRAGMENT = 0x02,  /* Fragment received, more expected */

//     // Sync/Control codes
//     PROTOCOL_RX_CODE_ACK = 0x10,
//     PROTOCOL_RX_CODE_NACK = 0x11,
//     PROTOCOL_RX_CODE_ABORT = 0x12,
//     PROTOCOL_RX_CODE_RESET = 0x13,  /* Protocol reset requested */
//     PROTOCOL_RX_CODE_HEARTBEAT = 0x14,  /* Keep-alive packet */

//     // Error codes - Header/Meta
//     PROTOCOL_RX_CODE_ERROR_TIMEOUT = 0x20,
//     PROTOCOL_RX_CODE_ERROR_INVALID_ID = 0x21,  /* Unknown packet ID */
//     PROTOCOL_RX_CODE_ERROR_INVALID_LENGTH = 0x22, /* Invalid length field */
//     PROTOCOL_RX_CODE_ERROR_HEADER_CRC = 0x23,  /* Header checksum error */
//     PROTOCOL_RX_CODE_ERROR_START_MARKER = 0x24,  /* Missing start delimiter */
//     PROTOCOL_RX_CODE_ERROR_SEQUENCE = 0x25,  /* Sequence number error */

//     // Error codes - Data/Payload
//     PROTOCOL_RX_CODE_ERROR_DATA_CRC = 0x30,  /* Payload checksum error */
//     PROTOCOL_RX_CODE_ERROR_DATA_LENGTH = 0x31,  /* Payload length mismatch */
//     PROTOCOL_RX_CODE_ERROR_DATA_FORMAT = 0x32,  /* Invalid data format */
//     PROTOCOL_RX_CODE_ERROR_BUFFER_FULL = 0x33,  /* Receive buffer overflow */

//     // System errors
//     PROTOCOL_RX_CODE_ERROR_SYSTEM = 0xF0,  /* Generic system error */
//     PROTOCOL_RX_CODE_ERROR_NOT_READY = 0xF1,  /* System not ready */
//     PROTOCOL_RX_CODE_ERROR_BUSY = 0xF2,  /* System busy */
// } Protocol_RxCode_T;









// /*!
//     Reset framing state for a new packet (start byte already consumed).
// */
// static inline void Packet_RxBegin(Packet_RxState_T * p_rx)
// {
//     p_rx->RxIndex = 1U;
//     p_rx->RxMeta.Length = 0U;
//     p_rx->RxMeta.Id = 0U;
// }

// /*!
//     Validate start byte against format spec.
// */
// static inline bool Packet_IsStartByte(const Packet_Format_T * p_specs, uint8_t byte)
// {
//     return (byte == p_specs->RX_START_ID) || (p_specs->RX_START_ID == 0x00U);
// }



