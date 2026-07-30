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
#include <stddef.h>


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

/* Ensure alignment for packet buffers */
#define PACKET_BUFFER_ALLOC(BufferLength) (alignas(4U) uint8_t[BufferLength]){0}
// #define PACKET_BUFFER_ALLOC(BufferLength) (uint8_t *)((uint32_t[BufferLength/sizeof(uint32_t)]){0})

/******************************************************************************/
/*!
    Rx Packet Meta/Header Parser
    User app provide child function to determine completion of rx packet.
    returns reqid code via pointer upon completion.
*/
/******************************************************************************/

/*
    User functions return status - communicate module handled behaviors
    Handle status that can be determined by the framing/header alone.
    overload to eliminate need of addtional parser functions
    include sync ids from framing/header
*/
typedef enum Protocol_RxCode
{
    PROTOCOL_RX_CODE_AWAIT_PACKET,       /* Wait CaptureRx */
    // PROTOCOL_RX_CODE_AWAIT_HEADER,
    // PROTOCOL_RX_CODE_AWAIT_PAYLOAD,
    PROTOCOL_RX_CODE_PACKET_COMPLETE,    /* Success Complete Payload Packet */

    PROTOCOL_RX_CODE_ERROR_TIMEOUT,     /* Error Timeout */
    PROTOCOL_RX_CODE_ERROR_META,        /* Error Req Packet meta data */
    PROTOCOL_RX_CODE_ERROR_DATA,        /* Error Req Packet Checksum/CRC */

    /* Sync packets allocated RxCode */
    // PROTOCOL_RX_CODE_CONTROL_META, /* Sync type pass to req, share sub id Protocol_TxSyncId_T */
    PROTOCOL_RX_CODE_ACK,
    PROTOCOL_RX_CODE_NACK,
    PROTOCOL_RX_CODE_ABORT,
}
Protocol_RxCode_T;
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


/* Framing */
/* Effectively VirtualHeader for unknown Packet struct */
typedef struct Protocol_HeaderMeta
{
    packet_id_t Id;                 /* Packet type identifier. Index into P_REQ_TABLE */
    packet_size_t Length;           /* Total packet length */
// uint32_t PayloadIndex;
// uint32_t responseId;
// uint32_t Sequence;   /* Sequence number (optional) */
}
Protocol_HeaderMeta_T;


/*!
    Phase 1 — Framing / Boundary Detection
    Called by CaptureRx when RxIndex >= RX_LENGTH_MIN and Length is unknown.
    Extracts Id and Length from partial header.
    Sync packets return completion code directly (ACK/NACK/ABORT).
    Data packets set p_meta->Length and return AWAIT_PACKET.
    Phase 1 errors:  ERROR_META(invalid ID, nonsensical length)
    Phase 1 control : ACK, NACK, ABORT(sync packets — no Phase 2)
*/
typedef Protocol_RxCode_T(*Packet_ParseRxFraming_T)(const void * p_buffer, packet_size_t rxCount, Protocol_HeaderMeta_T * p_meta);
// force lenght first. return 0 for unknown, after min, per byte call.
// typedef size_t(*Packet_ParseRxLength_T)(const void * p_buffer, packet_size_t rxCount);

/*!
    Phase 2 — Validation / Completion
    Called by CaptureRx when RxIndex == Length (full packet in buffer).
    Validates checksum/CRC. Extracts remaining header fields.
    No rxCount parameter — Length is already known from Phase 1.
    Phase 2 errors : ERROR_DATA(checksum failure)
    Phase 2 success : PACKET_COMPLETE
*/
typedef Protocol_RxCode_T(*Packet_ParseRxComplete_T)(const void * p_buffer, Protocol_HeaderMeta_T * p_meta);
// typedef bool (*Packet_ValidateRx_T)(const void * p_buffer);
// id or directly return handler
// typedef Protocol_RxCode_T(*Packet_ParseRx_T)(const void * p_buffer, intptr_t req);

/*!
    Tx — Build Header
    Called after handler fills payload.
    Writes all header fields (start, id, length, checksum).
*/
typedef void (*Packet_BuildTxHeader_T)(void * p_buffer, const Protocol_HeaderMeta_T * p_meta);
// split essential fields implementation
// typedef void (*Packet_BuildTxHeader_T)(void * p_buffer, packet_id_t id, packet_size_t length);
// typedef void (*Packet_BuildTxHeaderExt_T)(void * p_buffer, const Protocol_HeaderMeta_T * p_meta);

typedef packet_size_t(*Packet_BuildTxSync_T)(void * p_txPacket, packet_id_t txId);


/******************************************************************************/
/*!
    Packet Class Variables / Meta/Format Specs
*/
/******************************************************************************/
typedef const struct Packet_Format
{
    const uint8_t RX_LENGTH_MIN;                  /* Rx this many bytes before calling PARSE_RX */
    const uint8_t RX_LENGTH_MAX;

    /* Enframe/Deframe */
    /* Rx */
    const Packet_ParseRxFraming_T PARSE_RX_FRAMING;  // Phase 1: Id + Length
    const Packet_ParseRxComplete_T PARSE_RX_HEADER;   // Phase 2: checksum + fields extraction

    /* Tx */
    const Packet_BuildTxHeader_T BUILD_TX_HEADER;  // symmetric with Phase 2
    const Packet_BuildTxSync_T BUILD_TX_SYNC;

    /* Optional */
    const uint32_t RX_START_ID;             /* 0x00 for Rx Parser handle */
    const uint32_t RX_TIMEOUT;              /* Reset cumulative per packet */

    // move protocol param
    const uint32_t REQ_TIMEOUT;             /* checked for stateful Req only */
    const uint8_t NACK_COUNT;

    // const uint32_t BAUD_RATE_DEFAULT;
    // const bool ENCODED;                  /* Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */
}
Packet_Format_T;

static inline Protocol_RxCode_T Packet_ParseRxHeader(const Packet_Format_T * p_specs, const uint8_t * p_header, Protocol_HeaderMeta_T * p_meta) { return p_specs->PARSE_RX_HEADER(p_header, p_meta); }
static inline void Packet_BuildTxHeader(const Packet_Format_T * p_specs, uint8_t * p_header, const Protocol_HeaderMeta_T * p_meta) { p_specs->BUILD_TX_HEADER(p_header, p_meta); }

/*
    Default selection
*/
static uint16_t _Packet_Checksum(const uint8_t * p_src, size_t size)
{
    uint16_t checksum = 0U;
    for (size_t index = 0U; index < size; index++) { checksum += p_src[index]; }
    return checksum;
}

static uint16_t Packet_Checksum(const uint8_t * p_packet, size_t totalSize, size_t checksumStart, size_t checksumSize)
{
    const size_t checksumEnd = checksumStart + checksumSize;
    uint16_t checksum = 0U;
    checksum += _Packet_Checksum(&p_packet[0U], checksumStart);
    checksum += _Packet_Checksum(&p_packet[checksumEnd], totalSize - checksumEnd);
    return checksum;
}

/******************************************************************************/
/*!
    Tx Sync
    User supply function to build Tx Ack, Nack, etc
    alternatively combine rx sync with map
*/
/******************************************************************************/
typedef enum Protocol_TxSyncId
{
    PROTOCOL_TX_SYNC_ACK_REQ,
    PROTOCOL_TX_SYNC_NACK_REQ,
    PROTOCOL_TX_SYNC_ACK_ABORT,
    PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT,
    PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT,
    PROTOCOL_TX_SYNC_NACK_PACKET_META,
    PROTOCOL_TX_SYNC_NACK_PACKET_DATA,
    PROTOCOL_TX_SYNC_ACK_REQ_EXT,
    PROTOCOL_TX_SYNC_NACK_REQ_EXT,
    // PROTOCOL_TX_RESP, alternatively update and map
}
Protocol_TxSyncId_T;
// typedef enum Protocol_TxType
// {
//     PROTOCOL_TX_TYPE_DATA,              /* Regular data packet */
//     PROTOCOL_TX_TYPE_ACK,               /* Acknowledgment */
//     PROTOCOL_TX_TYPE_NACK,              /* Negative acknowledgment */
//     PROTOCOL_TX_TYPE_ABORT,             /* Abort transmission */
//     PROTOCOL_TX_TYPE_HEARTBEAT,         /* Keep-alive */
//     PROTOCOL_TX_TYPE_RESET,             /* Protocol reset */
// } Protocol_TxType_T;

/*
    structured def for socket def
*/
typedef const struct Packet_Context
{
    uint8_t * P_BUFFER;
    Protocol_HeaderMeta_T * P_META;
}
Packet_Context_T;

// typedef const struct
// {
//     const Packet_Context_T * P_Rx;
//     const Packet_Context_T * P_Tx;
// }
// Protocol_ReqContext_T;


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
//     const Packet_ParseRxComplete_T  PARSE_RX_HEADER;   // Phase 2: checksum + fields extraction
// }
// Protocol_RxContext_T;





/*
    by  descriptor
*/
// typedef const struct Packet_HeaderFormat
// {
//     Field_T SYNC;
//     Field_T ID;
//     Field_T LENGTH;
//     Field_T CHECKSUM;
//     Packet_ValidateChecksum_T   VALIDATE;       // Rx: verify integrity
//     Packet_ComputeChecksum_T    COMPUTE;        // Tx: compute integrity
//     // const packet_size_t LENGTH_MIN;
//     // const packet_size_t HEADER_LENGTH;     /* fixed header length known to include contain data length value */
// } Packet_HeaderFormat_T;

// Rx — generic incremental parser driven by descriptor
// Protocol_RxCode_T Packet_ParseRx(const Packet_Format_T * p_fmt, Protocol_HeaderMeta_T * p_meta, const uint8_t * p_buffer, packet_size_t rxCount)
// {
//     if (rxCount < p_fmt->HEADER_SIZE)
//         return PROTOCOL_RX_CODE_AWAIT_PACKET;

//     // Extract length from described position
//     p_meta->Length = Packet_ReadField(p_buffer, p_fmt->LENGTH_OFFSET, p_fmt->LENGTH_SIZE);
//     if (!p_fmt->LENGTH_INCLUDES_HEADER)
//         p_meta->Length += p_fmt->HEADER_SIZE;

//     if (rxCount < p_meta->Length)
//         return PROTOCOL_RX_CODE_AWAIT_PACKET;

//     // Extract ID from described position
//     p_meta->Id = Packet_ReadField(p_buffer, p_fmt->ID_OFFSET, p_fmt->ID_SIZE);

//     // Validate integrity using described algorithm
//     if (!Packet_ValidateChecksum(p_fmt, p_buffer, p_meta->Length))
//         return PROTOCOL_RX_CODE_ERROR_DATA;

//     return PROTOCOL_RX_CODE_PACKET_COMPLETE;
// }

// // Tx — generic header builder driven by descriptor
// packet_size_t Packet_BuildTx(const Packet_Format_T * p_fmt, uint8_t * p_buffer, packet_id_t id, packet_size_t payloadLength)
// {
//     packet_size_t totalLength = p_fmt->HEADER_SIZE + payloadLength;

//     // Start byte
//     if (p_fmt->START_BYTE != 0)
//         p_buffer[0] = p_fmt->START_BYTE;

//     // ID at described position
//     Packet_WriteField(p_buffer, p_fmt->ID_OFFSET, p_fmt->ID_SIZE, id);

//     // Length at described position
//     packet_size_t lengthValue = p_fmt->LENGTH_INCLUDES_HEADER ? totalLength : payloadLength;
//     Packet_WriteField(p_buffer, p_fmt->LENGTH_OFFSET, p_fmt->LENGTH_SIZE, lengthValue);

//     // Checksum at described position
//     Packet_ComputeChecksum(p_fmt, p_buffer, totalLength);

//     return totalLength;
// }
// Generic Rx — handles start detection, length extraction, ID extraction
// Protocol_RxCode_T Packet_ParseRx(const Packet_Class_T * p_specs, Protocol_HeaderMeta_T * p_meta,  const uint8_t * p_buffer, packet_size_t rxCount)
// {
//     // Common: extract length and ID from known offsets
//     if (rxCount >= p_specs->HEADER_SIZE && p_meta->Length == 0)
//     {
//         p_meta->Length = p_buffer[p_specs->LENGTH_OFFSET];
//         p_meta->Id = p_buffer[p_specs->ID_OFFSET];
//     }

//     if (rxCount < p_meta->Length)
//         return PROTOCOL_RX_CODE_AWAIT_PACKET;

//     // Protocol-specific: checksum validation
//     if (!p_specs->VALIDATE(p_buffer, p_meta->Length))
//         return PROTOCOL_RX_CODE_ERROR_DATA;

//     return PROTOCOL_RX_CODE_PACKET_COMPLETE;
// }

// // Generic Tx — handles common header fields
// packet_size_t Packet_BuildTx(const Packet_Class_T * p_specs, uint8_t * p_buffer, const Protocol_HeaderMeta_T * p_meta)
// {
//     // Common: start byte, ID, length
//     p_buffer[0] = p_specs->RX_START_ID;
//     p_buffer[p_specs->ID_OFFSET] = p_meta->Id;
//     p_buffer[p_specs->LENGTH_OFFSET] = p_meta->Length;

//     // Protocol-specific: checksum computation
//     p_specs->COMPUTE(p_buffer, p_meta->Length);

//     return p_meta->Length;
// }

