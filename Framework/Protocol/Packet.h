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
/*
    Effectively VirtualHeader for unknown Packet struct
    denotation and semsntics
    Extracted Field Values
*/
typedef struct Packet_Meta
{
    packet_id_t Id;         /* Packet type identifier. Index into P_REQ_TABLE */
    /* Data length passed to handler. for variable-length payloads / responses */
    packet_size_t Length;   /* Payload length. Handler decoupled from header shape. Framing layer / build header determines total packet length */
    /* Ext engine side processed elements */
    uint32_t Sequence;      /* Sequence number (optional) */
    uint16_t  Source;       /* normalized, not a wire field */
    uint16_t  Dest;
    uint16_t  Flags;
}
Packet_Meta_T;

/* Allocation context */
typedef struct __attribute__((aligned(sizeof(uintptr_t)))) Packet_Context
{
    Packet_Meta_T Meta;
    uint8_t Packet[]; /* Physical Header and Payload contiguous. Parser passes payload pointer */
}
Packet_Context_T;

#define PACKET_CONTEXT_ALLOC_T(BufferLength) union { Packet_Context_T Context; uint8_t Bytes[sizeof(Packet_Meta_T) + (BufferLength)]; }

/*
    The literal is declared as a union containing Packet_Context_T, so the storage's effective
    type includes the struct and reading it back as one is defined. Casting a uint8_t[] literal
    to Packet_Context_T * is not: alignas fixes the address, not the effective type, and GCC
    turns on strict aliasing at -O2. The union member also carries the struct's own alignment.
*/
#define PACKET_CONTEXT_ALLOC(BufferLength)                                      \
    (&(union { Packet_Context_T Context; uint8_t Bytes[sizeof(Packet_Meta_T) + (BufferLength)]; }){0}.Context)


/*!
    Frame/header variants
    Frame Encoding/Decoding
    layout is a view with the buffer's lifetime

    Returned by PARSE_RX_HEADER and BUILD_TX_HEADER, and held by the engine across the handler
    call, so it must point at storage that outlives the call - a static const per shape, never
    a compound literal in the callback's own frame.

    The engine offsets the Rx payload by the HEADER_LENGTH of the frame that just arrived, and
    the Tx payload by the HEADER_LENGTH of the frame that OPENED the exchange - held in
    Protocol_State_T.p_ReqFraming. The two are not the same on a continuation, where the
    arriving frame is an ack and need not share the data frame's header length.

    CONTRACT, still unenforced: a format whose RESPONSE header differs in length from its
    REQUEST header places the Tx payload at the request's offset, because the handler must
    write its payload before BUILD_TX_HEADER can run and so the response header length is not
    yet knowable. Keep the two equal, or add a query yielding the Tx frame format from
    Meta.Id alone and offset by that.
*/
typedef const struct Packet_FrameFormat
{
    packet_size_t HEADER_LENGTH;     /* bytes consumed from frame start by the header */
    packet_size_t BODY_LENGTH;        /* FIXED_LENGTH, 0 for variable-length body */
    packet_size_t TRAILER_LENGTH;
    // uint8_t MIN_LENGTH; /* Payload length  */
    // uint8_t MAX_LENGTH;
}
Packet_FrameFormat_T;

/*
    Packet content type, for handling at the protocol layer
    Protocol_Sync decoupled from [Packet_Codec_T]
*/
typedef enum Packet_ClassId
{
    PACKET_CLASS_DATA,              /* Regular data packet */
    PACKET_CLASS_ACK,               /* Acknowledgment */
    PACKET_CLASS_NACK,              /* Negative acknowledgment */
    PACKET_CLASS_ABORT,             /* Abort transmission */
    // PACKET_CLASS_STATUS,
    // PACKET_CLASS_HEARTBEAT,         /* Keep-alive */
    // PACKET_CLASS_RESET,             /* Protocol reset */
    // PACKET_CLASS_CONFIG,            /* Protocol configuration */
}
Packet_ClassId_T;

/* callback maped to id entry */
/*
    embed in Protocol_Req_T or couple with forward declaration
*/
typedef const struct
{
    packet_id_t ID;
    Packet_FrameFormat_T * FRAME_FORMAT;    /* effectivelt payload type and offset */
    // Packet_ClassId_T CLASS_ID;              /* no comparision on class id look up this way */
}
Packet_Id_T;

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

    keep separate from ParseRxFrame: handle 3 conditions need more bytes, complete with a row, complete with no row
*/
typedef packet_size_t (*Packet_ParseRxLength_T)(const void * p_buffer, packet_size_t rxCount);
typedef bool          (*Packet_ValidateRx_T)   (const void * p_buffer, packet_size_t length);



/*!
    Phase 2 — Validation / Completion
    Called by CaptureRx when RxIndex == Length (full packet in buffer).
    Extracts remaining header fields.
    No rxCount parameter — Length is already known from Phase 1.

    @return [Packet_Id_T *] MUST be from the Request Table
            Rx returned Packet_Id_T determines response payload offset.
*/
/*!
    Tx — Build Header
    Called after handler fills payload.
    Writes all header fields (start, id, length, checksum).
*/
/* optionally engine provide checksum */
typedef Packet_Id_T * (*Packet_ParseRxFrame_T)(Packet_Meta_T * p_meta, const void * p_header);
typedef void (*Packet_BuildTxFrame_T)(const Packet_Meta_T * p_meta, void * p_header);


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
*/
/******************************************************************************/
/*
    Every frame's total length is computable from a fixed prefix < LENGTH_MIN => resolve in 2 virtualized calls
    Frame Length Field Index < LENGTH_MIN
*/
typedef const struct Packet_Codec
{
    uint8_t LENGTH_MIN;             /* Rx this many bytes before calling PARSE_RX */
    uint8_t LENGTH_MAX;             /* the buffer length */
    uint32_t START_ID;              /* 0x00 for Rx Parser handle */
    uint32_t START_ID_LENGTH;
    // Packet_FrameFormat_T FrameFormat; /* Default FrameFromat */

    /* Enframe/Deframe */
    /* Rx */
    Packet_ParseRxLength_T PARSE_RX_LENGTH;    // Phase 1: frame length, 0 while unknown
    Packet_ValidateRx_T    IS_RX_VALID;        // Phase 2: integrity
    /* On a completed frame */
    Packet_ParseRxFrame_T PARSE_RX_HEADER;    // Phase 2: fields extraction
    /* Tx */
    Packet_BuildTxFrame_T BUILD_TX_HEADER;    // symmetric with Phase 2

    Packet_FrameFormat_T CONTROL_FRAME_FORMAT;
    packet_id_t ACK_ID;
    packet_id_t NACK_ID;
    packet_id_t ABORT_ID;

    // keep this here so Protocol_Req can remain a table only
    // .RX_TIMEOUT_DEFAULT
    // BAUD_RATE_DEFAULT
}
Packet_Codec_T;

#define PACKET_CODEC_ASSERT(Codec, PacketBufferLength) \
    static_assert((Codec.LENGTH_MAX) <= (PacketBufferLength) , "frame exceeds buffer"); \
    static_assert((Codec.START_ID_LENGTH) <= (Codec.LENGTH_MIN) , "frame exceeds buffer");

/*
    Extract Fields
*/
/* Packet_RxCode_T rxCode == COMPLETE */
static inline Packet_Id_T * Packet_ParseRxHeader(Packet_Codec_T * p_codec, Packet_Meta_T * p_meta, const uint8_t * p_header) { return p_codec->PARSE_RX_HEADER(p_meta, p_header); }
static inline void Packet_BuildTxHeader(Packet_Codec_T * p_codec, const Packet_Meta_T * p_meta, uint8_t * p_header) { p_codec->BUILD_TX_HEADER(p_meta, p_header); }

/* Variable length payload use Meta.Length. Fixed use BODY_LENGTH. */
static inline packet_size_t Packet_FrameLengthOf(Packet_FrameFormat_T * p_format, const Packet_Meta_T * p_meta)
{
    return p_format->HEADER_LENGTH + p_format->TRAILER_LENGTH + ((p_format->BODY_LENGTH == 0U) ? p_meta->Length : p_format->BODY_LENGTH);
}

/*!
    The format's account of the frame against the parser's own.

    PARSE_RX_LENGTH and PARSE_RX_HEADER read the wire independently, so they can disagree -
    and Meta.Length is what sizes the payload the handler is handed. Checking it against the
    length the parser actually collected is the one bound only the engine can apply.
*/
static inline bool Packet_IsFrameConsistent(Packet_FrameFormat_T * p_framing, const Packet_Meta_T * p_meta, packet_size_t frameLength)
{
    return (Packet_FrameLengthOf(p_framing, p_meta) == (size_t)frameLength);
}

static inline packet_id_t Packet_ControlIdOf(Packet_Codec_T * p_format, Packet_ClassId_T classId)
{
    switch (classId)
    {
        case PACKET_CLASS_ACK:      return p_format->ACK_ID;            /* Acknowledgment */
        case PACKET_CLASS_NACK:     return p_format->NACK_ID;           /* Negative acknowledgment */
        case PACKET_CLASS_ABORT:    return p_format->ABORT_ID;          /* Abort transmission */
        case PACKET_CLASS_DATA:     return 0U;
        default:                    return 0U;
    }
}

static inline Packet_ClassId_T Packet_ClassOf(const Packet_Codec_T * p_format, packet_id_t id)
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

static inline uint16_t Packet_Checksum(const uint8_t * p_packet, size_t totalSize, size_t checksumStart, size_t checksumSize)
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


