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

/*
    Stack frame for a payload-less control packet (ack / nack / abort).
    Built on the stack so a staged response survives for retransmission.
    Must be >= the largest LENGTH_MIN across every format bound to a socket.
*/
#ifndef PACKET_CONTROL_LENGTH_MAX
#define PACKET_CONTROL_LENGTH_MAX 16U
#endif

/* Ensure alignment for packet buffers */
#define PACKET_BUFFER_ALLOC(BufferLength) (alignas(4U) uint8_t[BufferLength]){0}


/******************************************************************************/
/*!
    Rx Packet Meta/Header Parser
    User app provide child function to determine completion of rx packet.
    returns reqid code via pointer upon completion.
*/
/******************************************************************************/
typedef enum Packet_ClassId
{
    PACKET_CLASS_DATA,              /* Regular data packet */
    PACKET_CLASS_ACK,               /* Acknowledgment */
    PACKET_CLASS_NACK,              /* Negative acknowledgment */
    PACKET_CLASS_ABORT,             /* Abort transmission */
    // PACKET_CLASS_ERROR,
    // PACKET_CLASS_HEARTBEAT,         /* Keep-alive */
    // PACKET_CLASS_RESET,             /* Protocol reset */
    // PACKET_CLASS_CONFIG,            /* Protocol configuration */
}
Packet_ClassId_T;

/*
    Effectively VirtualHeader for unknown Packet struct
    denotation and semsntics
    Extracted Field Values
*/
typedef struct Packet_Meta
{
    packet_id_t Id;         /* Packet type identifier. Index into P_REQ_TABLE */
    packet_size_t Length;   /* Payload length. Handler decoupled from header shape. Framing layer / build header determines total packet length */
    uint32_t Sequence;      /* Sequence number (optional) */
    uint16_t  Source;       /* normalized, not a wire field */
    uint16_t  Dest;
    uint16_t  Flags;        /* REPLY_EXPECTED | BROADCAST */
}
Packet_Meta_T;

/* Allocation context */
typedef struct Packet_Context
{
    Packet_Meta_T Meta;
    // packet_size_t TotalLength;
    uint8_t Packet[]; /* Physical Header and Payload contiguous. Parser passes payload pointer */
}
Packet_Context_T;

#define PACKET_CONTEXT_ALLOC(BufferLength) (Packet_Context_T *)PACKET_BUFFER_ALLOC(BufferLength + sizeof(Packet_Meta_T))


/*!
    Frame/header variants
    Frame Encoding/Decoding
    layout is a view with the buffer's lifetime

    Returned by PARSE_RX_HEADER and BUILD_TX_HEADER, and held by the engine across the handler
    call, so it must point at storage that outlives the call - a static const per shape, never
    a compound literal in the callback's own frame.

    CONTRACT, currently unenforced: the engine offsets BOTH payload pointers by the HEADER_LENGTH
    that PARSE_RX_HEADER reported, because the handler must write its payload before
    BUILD_TX_HEADER can run and so its header length is not yet knowable. A format whose response
    header differs in length from its request header will therefore place the Tx payload at the
    wrong offset. Either keep the two equal, or add a query that yields the Tx frame format from
    Meta.Id alone and offset by that.
*/
typedef const struct Packet_FrameFormat
{
    packet_size_t HEADER_LENGTH;     /* bytes consumed from frame start by the header */
    packet_size_t BODY_LENGTH;       /* 0 for variable-length body */
    packet_size_t TRAILER_LENGTH;
}
Packet_FrameFormat_T;

/*!
    Rx framing primitives - narrow queries, one answer each.

    Each returns a single value with no out-parameter and no status enum, so none of them can
    express protocol meaning. Whether an Id is an ack is decided by Packet_ClassOf, above the
    parser, from the ids declared on this format.

    PARSE_RX_LENGTH   Phase 1. Total frame length, or 0 while not yet determinable.
                Called once at LENGTH_MIN, then per additional byte for formats whose length
                is not at a fixed offset. Free to inspect the Id to pick a frame shape.
    IS_RX_VALID    Phase 2. Checksum / CRC over the complete frame.
    PARSE_RX_HEADER Phase 2. The only source of Meta.Id, and so of the frame's class.
*/
typedef packet_size_t (*Packet_ParseRxLength_T)(const void * p_buffer, packet_size_t rxCount);
typedef bool          (*Packet_ValidateRx_T)   (const void * p_buffer, packet_size_t length);
/* return optional length field / checksum descriptor */
// typedef Packet_FrameFormat_T * (*Packet_ParseRxFrame_T)(const void * p_buffer, packet_size_t rxCount);

/*!
    Phase 2 — Validation / Completion
    Called by CaptureRx when RxIndex == Length (full packet in buffer).
    Validates checksum/CRC. Extracts remaining header fields.
    No rxCount parameter — Length is already known from Phase 1.
    Phase 2 errors : ERROR_DATA(checksum failure)
    Phase 2 success : PACKET_COMPLETE
*/
/*!
    Tx — Build Header
    Called after handler fills payload.
    Writes all header fields (start, id, length, checksum).
*/
typedef Packet_FrameFormat_T * (*Packet_ParseRxHeader_T)(Packet_Meta_T * p_meta, const void * p_buffer);
typedef Packet_FrameFormat_T * (*Packet_BuildTxHeader_T)(const Packet_Meta_T * p_meta, void * p_buffer);

/******************************************************************************/
/*!
    Packet Codec
    Frame Operations
    swap for delimited | length-prefixed | bus-based

    Per Framing set.
    Different sync, delimiter, escaping or CRC → separate ops.

    Author's contract - every field below is a compile-time constant of a const table, so these
    are invariants to assert where the format is defined, not conditions for the engine to
    re-test on every selection. Nothing in the engine re-checks them.

        START_ID_LENGTH <= LENGTH_MIN <= LENGTH_MAX <= the socket's PACKET_BUFFER_LENGTH
            The parser sets its read target from START_ID_LENGTH and LENGTH_MIN without a bound
            of its own; only a target derived from PARSE_RX_LENGTH is clamped to LENGTH_MAX.

        START_ID_LENGTH is 0 or 1
            Only p_buffer[0] is tested, so a multi-byte sync is not implemented. 0 means no
            delimiter and pairs with START_ID == 0 - a 0 length against a real START_ID leaves
            the reject branch asking for no bytes, and the capture loop cannot advance.

        a zero-payload frame fits PACKET_CONTROL_LENGTH_MAX
            Protocol_TxControl builds acks and nacks on a stack array of that size.

    Assert these on the constants that initialise the format, not on the struct: in C a const
    object is not a constant expression, so static_assert(fmt.LENGTH_MAX <= N) will not compile.

        #define MOT_PACKET_LENGTH_MAX   40U
        static_assert(MOT_PACKET_LENGTH_MAX <= PACKET_BUFFER_LENGTH, "frame exceeds buffer");
        static const Packet_Format_T MOT_FORMAT = { .LENGTH_MAX = MOT_PACKET_LENGTH_MAX, ... };
*/
/******************************************************************************/
typedef const struct Packet_Format
{
    uint8_t LENGTH_MIN;             /* Rx this many bytes before calling PARSE_RX */
    uint8_t LENGTH_MAX;             /* the buffer length */
    uint32_t START_ID;              /* 0x00 for Rx Parser handle */
    uint32_t START_ID_LENGTH;
    // Packet_FrameFormat_T FrameFormat; /* Default FrameFromat */
    /* Enframe/Deframe */
    /* Rx */
    Packet_ParseRxLength_T PARSE_RX_LENGTH;    // Phase 1: frame length, 0 while unknown
    Packet_ValidateRx_T    IS_RX_VALID;         // Phase 2: integrity
    /* On a completed frame */
    Packet_ParseRxHeader_T PARSE_RX_HEADER;   // Phase 2: fields extraction
    /* Tx */
    Packet_BuildTxHeader_T BUILD_TX_HEADER;  // symmetric with Phase 2

    uint8_t CONTROL_CHAR_LENGTH;
    packet_id_t ACK_ID;
    packet_id_t NACK_ID;
    packet_id_t ABORT_ID;
}
Packet_Format_T;



/*
    Extract Fields
*/
/* Packet_RxCode_T rxCode == COMPLETE */
static inline Packet_FrameFormat_T * Packet_ParseRxHeader(Packet_Format_T * p_codec, Packet_Meta_T * p_meta, const uint8_t * p_header)
{
    return p_codec->PARSE_RX_HEADER(p_meta, p_header);
    // return p_framing->HEADER_LENGTH + p_framing->TRAILER_LENGTH + ((p_framing->BODY_LENGTH == 0U) ? p_meta->Length : p_framing->BODY_LENGTH);
}

static inline packet_size_t Packet_BuildTxHeader(Packet_Format_T * p_codec, const Packet_Meta_T * p_meta, uint8_t * p_header)
{
    Packet_FrameFormat_T * p_framing = p_codec->BUILD_TX_HEADER(p_meta, p_header);
    return p_framing->HEADER_LENGTH + p_framing->TRAILER_LENGTH + ((p_framing->BODY_LENGTH == 0U) ? p_meta->Length : p_framing->BODY_LENGTH);

    /* Variable length payload use Meta.Length. Fixed use BODY_LENGTH. */
    // return p_framing->HEADER_LENGTH + p_framing->TRAILER_LENGTH + p_framing->BODY_LENGTH + p_tx->Meta.Length;
}

static inline packet_id_t Packet_ControlIdOf(Packet_Format_T * p_format, Packet_ClassId_T classId)
{
    switch (classId)
    {
        case PACKET_CLASS_ACK:      return p_format->ACK_ID;            /* Acknowledgment */
        case PACKET_CLASS_NACK:     return p_format->NACK_ID;           /* Negative acknowledgment */
        case PACKET_CLASS_ABORT:    return p_format->ABORT_ID;          /* Abort transmission */
        // case PACKET_CLASS_DATA:         return p_format->ACK_ID;        /* Regular data packet */
        // case PACKET_CLASS_ERROR:        return p_format->NACK_ID;
        // case PACKET_CLASS_HEARTBEAT:    return p_format->ACK_ID;        /* Keep-alive */
        // case PACKET_CLASS_RESET:        return p_format->ACK_ID;        /* Protocol reset */
        // case PACKET_CLASS_CONFIG:       return p_format->ACK_ID;        /* Protocol configuration */
        default:                        return p_format->NACK_ID;
    }
}

static inline Packet_ClassId_T Packet_ClassOf(const Packet_Format_T * p_format, packet_id_t id)
{
    if (id == p_format->ACK_ID)   { return PACKET_CLASS_ACK; }
    if (id == p_format->NACK_ID)  { return PACKET_CLASS_NACK; }
    if (id == p_format->ABORT_ID) { return PACKET_CLASS_ABORT; }
    return PACKET_CLASS_DATA;
}


/*
    Default selection
*/
static inline uint16_t _Packet_Checksum(const uint8_t * p_src, size_t size)
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




/*
    by  descriptor
*/
// typedef const struct Packet_HeaderFormat
// {
//     Field_T SYNC;
//     Field_T ID;
//     Field_T LENGTH;
//     Field_T CHECKSUM;
    // const packet_size_t LENGTH_MIN;
    // const packet_size_t HEADER_LENGTH;     /* fixed header length known to include contain data length value */
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

// typedef void (*Packet_BuildTxHeader_T)(void * p_buffer, const Protocol_HeaderMeta_T * p_meta);

// // Tx — generic header builder driven by descriptor
// packet_size_t Packet_BuildTx(const Packet_Format_T * p_fmt, Protocol_HeaderMeta_T * p_meta, )
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
