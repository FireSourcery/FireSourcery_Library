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
#include <stdint.h>


#ifndef PACKET_ID_TYPE
#define PACKET_ID_TYPE      uint8_t
#endif

#ifndef PACKET_SIZE_TYPE
#define PACKET_SIZE_TYPE    uint8_t
#endif

typedef PACKET_ID_TYPE      packet_id_t;
typedef PACKET_SIZE_TYPE    packet_size_t;

#ifndef PACKET_PACKED
#define PACKET_PACKED __attribute__((packed))
#endif

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
/* FramingMeta ControlMeta */
/* Effectively common struct from unknown Packet struct */
typedef struct Protocol_HeaderMeta
{
    packet_id_t Id;                 /* Packet type identifier. Index into P_REQ_TABLE */
    packet_size_t Length;           /* Total packet length */

    // uint32_t Sequence;   /* Sequence number (optional) */
    // uint32_t Status;
    // uint8_t SourceId;               /* Source node ID (optional) */
    // uint8_t DestId;                 /* Destination node ID (optional) */
    // uint16_t HeaderCrc;             /* Header integrity check (optional) */
    // Extended metadata for complex protocols
    // void * p_ExtendedMeta;          /* Protocol-specific extensions */
}
Protocol_HeaderMeta_T;

typedef Protocol_RxCode_T(*Packet_ParseRxMeta_T)(Protocol_HeaderMeta_T * p_rxMeta, const void * p_rxPacket, packet_size_t rxCount);
// typedef Protocol_RxCode_T(*Packet_ParseRxMeta_T)(const void * p_rxPacket, strcut * p_rxState, Protocol_HeaderMeta_T * p_rxMeta);

// combine passing base and child state
// const context pass with function
// typedef struct Packet_Parser
// {
    // const void * const p_rxPacket;
    // const packet_size_t rxCount;
    // uint16_t Checksum;   /* Running checksum */

    //     Protocol_RxCode_T rxStatus; /* Status of the parsing operation */
    // const Packet_ParseRxMeta_T parseRxMeta;
//     Protocol_HeaderMeta_T HeaderMeta;
// }
// Packet_Parser_T;
// typedef void(*Packet_ParseRxMeta_T)(Packet_Parser_T * p_parser);
// typedef Protocol_RxCode_T(*Packet_ParseRxMeta_T)(const Packet_Parser_T * p_parser, Protocol_HeaderMeta_T * rxMeta);

// typedef Protocol_RxCode_T(*Packet_ParseRxMeta_T)(Packet_HeaderParser_T * p_parser);
// typedef const struct Packet_Parser
// {
//     const uint8_t * P_BUFFER;
//     const void * P_RX_STATE;
//     // void * P_RX_STATE;
//     Protocol_HeaderMeta_T * P_META;
//     Packet_ParseRxMeta_T PARSE_RX_META; // 1 virtual function
// }
// Packet_HeaderParser_T;


/*
    Tx
*/
typedef void (* const Packet_BuildTxSync_T)(void * p_txPacket, packet_size_t * p_txSize, packet_id_t txId);
// typedef packet_size_t(* const Packet_BuildTxSignal_T)(void * p_txPacket, packet_id_t txId);
typedef void (* const Packet_BuildTxMeta_T)(void * p_txPacket, const Protocol_HeaderMeta_T * p_meta);
// typedef void (* const Packet_BuildTxMeta_T)(Packet_T * p_txPacket);

/******************************************************************************/
/*!
    Packet Class Variables / Meta/Format Specs
    Protocol_Transport
*/
/******************************************************************************/
typedef const struct Packet_Class
{
    const uint8_t RX_LENGTH_MIN;                  /* Rx this many bytes before calling PARSE_RX */
    const uint8_t RX_LENGTH_MAX;
    const Packet_ParseRxMeta_T PARSE_RX_META;     /* Parse Header for RxReqId and RxRemaining, and check data */
    const Packet_BuildTxSync_T BUILD_TX_SYNC;     /* Build Sync Packets */
    // const Packet_BuildTxHeader_T BUILD_TX_HEADER;  /* todo */

    /* Optional */
    const uint32_t RX_START_ID;             /* 0x00 for not applicable */
    // const uint32_t RX_END_ID;

    // move to protocol param
    // defaults over write in protocol Param
    // const uint32_t BAUD_RATE_DEFAULT;
    const uint32_t RX_TIMEOUT;              /* Reset cumulative per packet */
    // const uint32_t RX_TIMEOUT_BYTE;      /* Reset per byte */
    const uint32_t REQ_TIMEOUT;             /* checked for stateful Req only */
    const uint8_t NACK_COUNT;

    // const bool ENCODED;                  /* Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */
}
Packet_Class_T;


// alternative to PARSE_RX_META
// typedef const struct Packet_HeaderClass
// {
//     const uint8_t ID_FIELD_START;                /* Packet ID */
//     const uint8_t ID_FIELD_SIZE;
//     const uint8_t LENGTH_FIELD_START;            /* Packet Length */
//     const uint8_t LENGTH_FIELD_SIZE;

//     // const packet_size_t RX_LENGTH_INDEX;
//     // const packet_size_t RX_REQ_ID_INDEX;
//     // const packet_size_t RX_HEADER_LENGTH;     /* fixed header length known to include contain data length value */
// } Packet_HeaderClass_T;

// optionally compile time define contigous context
// static inline   Packet_Action(const Packet_Class_T * , uint8_t *, ...)
// typedef const struct Packet_Context
// {
//     const Packet_Class_T * P_SPECS;
//     uint8_t * P_BUFFER;
//     //   * P_RX_STATE;
//     Protocol_HeaderMeta_T * P_META;
// }
// Packet_Context_T;

// static inline Protocol_RxCode_T Packet_ParseRxFraming(const Packet_Context_T * p_context)
// {
//     p_context->P_SPECS->PARSE_RX_META(p_context->P_META, p_context->P_BUFFER, p_context->LENGTH);
// }




// typedef enum Protocol_TxType
// {
//     PROTOCOL_TX_TYPE_DATA,              /* Regular data packet */
//     PROTOCOL_TX_TYPE_ACK,               /* Acknowledgment */
//     PROTOCOL_TX_TYPE_NACK,              /* Negative acknowledgment */
//     PROTOCOL_TX_TYPE_ABORT,             /* Abort transmission */
//     PROTOCOL_TX_TYPE_HEARTBEAT,         /* Keep-alive */
//     PROTOCOL_TX_TYPE_RESET,             /* Protocol reset */
// } Protocol_TxType_T;

// typedef struct Protocol_TxRequest
// {
//     Protocol_TxType_T Type;             /* Packet type */
//     packet_id_t PacketId;               /* Packet identifier */
//     packet_sequence_t Sequence;         /* Sequence number */
//     const void * p_Payload;             /* Payload data */
//     packet_size_t PayloadLength;        /* Payload length */
//     uint8_t Priority;                   /* Transmission priority */
//     bool AckRequired;                   /* Require acknowledgment */
//     uint32_t Timeout;                   /* Transmission timeout */
// } Protocol_TxRequest_T;

// // Enhanced builder function signatures
// typedef packet_size_t(*Packet_BuildTxData_T)(
//     void * p_txPacket,
//     packet_size_t maxTxSize,
//     const Protocol_TxRequest_T * p_request
//     );

// typedef packet_size_t(*Packet_BuildTxSync_T)(
//     void * p_txPacket,
//     packet_size_t maxTxSize,
//     Protocol_TxType_T syncType,
//     packet_id_t packetId
//     );

// typedef packet_size_t(*Packet_BuildTxFragment_T)(
//     void * p_txPacket,
//     packet_size_t maxTxSize,
//     const Protocol_TxRequest_T * p_request,
//     packet_size_t fragmentOffset,
//     packet_size_t fragmentSize
//     );


// typedef enum Protocol_ParseState
// {
//     PROTOCOL_PARSE_STATE_SYNC,          /* Looking for sync/start marker */
//     PROTOCOL_PARSE_STATE_HEADER,        /* Parsing header */
//     PROTOCOL_PARSE_STATE_PAYLOAD,       /* Parsing payload */
//     PROTOCOL_PARSE_STATE_CHECKSUM,      /* Parsing checksum/CRC */
//     PROTOCOL_PARSE_STATE_COMPLETE,      /* Packet complete */
//     PROTOCOL_PARSE_STATE_ERROR          /* Error state */
// } Protocol_ParseState_T;

// typedef struct Packet_ParserState + const context pass with function
// typedef struct Protocol_ParseContext
// {
//     Protocol_ParseState_T State;        /* Current parser state */
//     packet_size_t BytesReceived;        /* Total bytes received */
//     packet_size_t BytesExpected;        /* Expected total bytes */
//     packet_size_t HeaderBytesReceived;  /* Header bytes received */
//     Protocol_HeaderMeta_T HeaderMeta;   /* Parsed header information */
//     uint32_t Checksum;                  /* Running checksum */
//     uint32_t Timeout;                   /* Timeout counter */
//     void * p_ProtocolState;             /* Protocol-specific state */
// } Protocol_ParseContext_T;

// // Enhanced parser function signature
// typedef Protocol_RxCode_T(*Packet_ParseRx_T)(
//     Protocol_ParseContext_T * p_context,
//     const void * p_rxData,
//     packet_size_t rxCount,
//     Protocol_HeaderMeta_T * p_rxMeta
//     );




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