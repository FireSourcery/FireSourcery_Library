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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

typedef uint8_t packet_id_t;   /* Index into P_REQ_TABLE. Child module define */
typedef uint8_t packet_size_t;    /* Packet Size */

/******************************************************************************/
/*!
    Rx Packet Meta/Header Parser
    User app provide child function to determine completion of rx packet.
    returns reqid code via pointer upon completion.
*/
/******************************************************************************/
/*
    User functions return status - communicate module handled behaviors
    overload to eliminate need of addtional parser functions
*/
typedef enum Protocol_RxCode
{
    PROTOCOL_RX_CODE_AWAIT_PACKET,       /* Wait CaptureRx */
    PROTOCOL_RX_CODE_PACKET_COMPLETE,   /* Success Complete Req/ReqExt Packet */

    // PROTOCOL_RX_CODE_ERROR,
    PROTOCOL_RX_CODE_ERROR_TIMEOUT,     /* Error Timeout */
    PROTOCOL_RX_CODE_ERROR_META,        /* Error Req Packet meta data */
    PROTOCOL_RX_CODE_ERROR_DATA,        /* Error Req Packet Checksum/CRC */

    /* Sync packets allocated RxCode */
    // PROTOCOL_RX_CODE_CONTROL_META, /* Sync type for further parsing */
    PROTOCOL_RX_CODE_ACK,
    PROTOCOL_RX_CODE_NACK,
    PROTOCOL_RX_CODE_ABORT,

    // PROTOCOL_RX_CODE_REQ_ID,
}
Protocol_RxCode_T;

/* Parse with PARSE_RX_META */
typedef struct Protocol_HeaderMeta
{
    packet_id_t ReqId;    /* packet_id_t values defined by child module. Index into P_REQ_TABLE */
    packet_size_t Length;     /* Rx Packet Total Length. */
    // uint32_t Sequence;
}
Protocol_HeaderMeta_T;

typedef Protocol_RxCode_T(*Packet_ParseRxMeta_T)(Protocol_HeaderMeta_T * p_rxMeta, const void * p_rxPacket, packet_size_t rxCount);

/*
    Tx
*/
typedef void (* const Packet_BuildTxSync_T)(uint8_t * p_txPacket, packet_size_t * p_txSize, packet_id_t txId);
// typedef void (* const Packet_BuildTxHeader_T)(uint8_t * p_txPacket, const uint8_t * p_txPayload, packet_id_t txId);


/******************************************************************************/
/*!
    Packet Class Variables / Format Specs
    Protocol_Transport
*/
/******************************************************************************/
typedef const struct PacketClass
{
    const uint8_t RX_LENGTH_MIN;                    /* Rx this many bytes before calling PARSE_RX */
    const uint8_t RX_LENGTH_MAX;
    const Packet_ParseRxMeta_T PARSE_RX_META;     /* Parse Header for RxReqId and RxRemaining, and check data */
    const Packet_BuildTxSync_T BUILD_TX_SYNC;     /* Build Sync Packets */
    // const Packet_BuildTxHeader_T BUILD_TX_HEADER;  /* todo */

    /* Optional */
    const uint32_t RX_START_ID;             /* 0x00 for not applicable */
    // const uint32_t RX_END_ID;

    // defaults over write in protocol Param
    // const uint32_t BAUD_RATE_DEFAULT;
    const uint32_t RX_TIMEOUT;              /* Reset cumulative per packet */
    // const uint32_t RX_TIMEOUT_BYTE;      /* Reset per byte */
    const uint32_t REQ_TIMEOUT;             /* checked for stateful Req only */
    const uint8_t NACK_COUNT;

    // const bool ENCODED;                  /* Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */
}
PacketClass_T;


//     // alternative to PARSE_RX_META
// typedef const struct Packet_HeaderClass
// {
//     const uint8_t ID_FIELD_START;                /* Packet ID */
//     const uint8_t ID_FIELD_SIZE;
//     const uint8_t LENGTH_FIELD_START;            /* Packet Length */
//     const uint8_t LENGTH_FIELD_SIZE;

//     // const packet_size_t RX_LENGTH_INDEX;
//     // const packet_size_t RX_REQ_ID_INDEX;
//     // const packet_size_t RX_HEADER_LENGTH;     /* fixed header length known to include contain data length value */
// }