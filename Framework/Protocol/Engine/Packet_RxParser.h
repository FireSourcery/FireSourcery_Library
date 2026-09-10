#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Packet_RxParser.h
    @author FireSourcery
    @brief  Frame boundary detection. Where does the frame end, and is it intact.
*/
/******************************************************************************/
#include "Packet.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Pure. No Xcvr, no timer, no transmit, no protocol vocabulary.
    The parser holds a frame's progress and nothing else, so it can be exercised with a
    buffer and a struct.

    Control is inverted: the parser says how many bytes it needs, the caller supplies them.
    That keeps the byte source out of this header and makes the target unambiguous per state.

        START       1 byte                          start delimiter
        HEADER      LENGTH_MIN, then +1 at a time   until PARSE_RX_LENGTH yields
        PAYLOAD     Length                          the whole frame, for IS_RX_VALID

    Two axes, deliberately not merged:

        outcome     Packet_RxCode_T     did the frame resolve, and how      - here
        class       Packet_ClassId_T    ack / nack / abort / data           - Packet_ClassOf

    An ACK frame is also a COMPLETE frame. Those answer different questions, so they do not
    share an enum. This parser cannot say "ACK", and does not need the control ids in order
    to avoid saying it: it reports an intact frame with an Id, and the caller classifies.
    Nor can it say "TIMEOUT" - a stalled line is not something a parse concludes, and the
    deadline belongs to whoever owns the clock.

    The set of ids using a short frame shape is not the set of control ids: PING shares the
    sync layout but is a request. PARSE_RX_LENGTH answers the first, Packet_ClassOf the second.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Rx Code - shape and integrity. Everything a parse can conclude, and nothing else.
    Frame response
*/
/******************************************************************************/
typedef enum Packet_RxCode
{
    PACKET_RX_AWAIT,            /* Frame incomplete. Feed more. */
    PACKET_RX_COMPLETE,         /* Intact frame in the buffer. Id and Length are set. */
    PACKET_RX_ERROR_FRAME,      /* No usable length. Unframeable. */
    PACKET_RX_ERROR_DATA,       /* Failed validation. */
}
Packet_RxCode_T;

/******************************************************************************/
/*!
    Parser State
*/
/******************************************************************************/
typedef enum Packet_RxState
{
    PACKET_RX_STATE_START,      /* Scanning for the delimiter */
    PACKET_RX_STATE_HEADER,     /* Growing the header until the length is known */
    PACKET_RX_STATE_PAYLOAD,    /* Collecting the rest of the frame */
}
Packet_RxState_T;

typedef struct Packet_RxParser
{
    Packet_RxState_T StateId;
    packet_size_t Index;        /* Bytes held in the buffer */
    packet_size_t Target;       /* NextIndex. Bytes wanted before the next Advance. Never exceeds LENGTH_MAX. */

    /* Result. Valid from PACKET_RX_COMPLETE until the next frame overwrites it. */
    packet_size_t Length;       /* Total frame length. 0 while unknown. */
    packet_id_t Id;
}
Packet_RxParser_T;


/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
// static inline packet_size_t Packet_NextRxIndex(const Packet_RxParser_T * p_parser, const Packet_Format_T * p_format)
// {
//     /*
//         Set xcvrRxLimit for PARSE_RX_FRAMING. Prevent reading bytes from the following packet.
//         RxLength known => Check Rx remaining
//         RxLength unknown => Check 1 byte or a known const value, until RxLength is known
//     */
//     if (p_parser->Length > 0U) { return p_parser->Length; } /* PacketLength is known. */
//     /* PacketLength is unknown */ /*nextRxIndex = max(p_format->RX_LENGTH_MIN, p_parser->Index + 1U)  */
//     else { return (p_parser->Index < p_format->LENGTH_MIN) ? p_format->LENGTH_MIN : p_parser->Index + 1U; }
// }

// static inline packet_size_t Packet_NextRxSize(const Packet_RxParser_T * p_parser, const Packet_Format_T * p_format)
// {
//     return Packet_NextRxIndex(p_parser, p_format) - p_parser->Index;
// }

/*!
    @brief  Interpret a met target. Call only when Packet_RxParser_Remaining is 0.
    @return AWAIT to keep collecting. Anything else resolves the frame and rewinds the parser.
*/
static inline Packet_RxCode_T Packet_ProcRxParser(Packet_RxParser_T * p_parser, const Packet_Format_T * p_format, const uint8_t * p_buffer)
{
    Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

    switch (p_parser->StateId)
    {
        case PACKET_RX_STATE_START:
            /* Anything but the delimiter is dropped, a byte at a time. START_ID of 0 accepts every byte. */
            if ((p_buffer[0U] == p_format->START_ID) || (p_format->START_ID == 0x00U))
            {
                p_parser->StateId = PACKET_RX_STATE_HEADER;
                p_parser->Target = p_format->LENGTH_MIN;
                p_parser->Length = 0U;
            }
            else
            {
                p_parser->Index = 0U;
            }
            break;

        case PACKET_RX_STATE_HEADER:
            p_parser->Length = p_format->PARSE_RX_LENGTH(p_buffer, p_parser->Index);

            // if (p_parser->Length > 0U) { p_parser->Target = p_parser->Length; } /* PacketLength is known. */
            // /* PacketLength is unknown */ /*nextRxIndex = max(p_format->RX_LENGTH_MIN, p_parser->Index + 1U)  */
            // else { p_parser->Target = (p_parser->Index < p_format->LENGTH_MIN) ? p_format->LENGTH_MIN : p_parser->Index + 1U; }

            // if (p_parser->Target > p_format->LENGTH_MAX || p_parser->Target < p_parser->Index) { rxCode = PACKET_RX_ERROR_FRAME; }
            // else { p_parser->StateId = PACKET_RX_STATE_PAYLOAD; }

            /* Declined to answer. Grow the header by a byte, until it can no longer become a frame. */
            if (p_parser->Length == 0U)
            {
                if (p_parser->Index >= p_format->LENGTH_MAX) { rxCode = PACKET_RX_ERROR_FRAME; }
                else { p_parser->Target = p_parser->Index + 1U; }
            }
            /* A length that cannot describe this frame makes everything after it meaningless. */
            else
            {
                if ((p_parser->Length < p_parser->Index) || (p_parser->Length > p_format->LENGTH_MAX))
                {
                    rxCode = PACKET_RX_ERROR_FRAME;
                }
                else
                {
                    p_parser->StateId = PACKET_RX_STATE_PAYLOAD;
                    p_parser->Target = p_parser->Length;
                }
            }
            break;

        case PACKET_RX_STATE_PAYLOAD:
            /* The whole frame is present, so this always resolves. */
            rxCode = (p_format->IS_RX_VALID(p_buffer, p_parser->Length) == true) ? PACKET_RX_COMPLETE : PACKET_RX_ERROR_DATA;
            if (rxCode == PACKET_RX_COMPLETE) { p_parser->Id = p_format->ID_OF(p_buffer); }
            break;

        default:
            break;
    }

    /* Rewind. Index must clear with the state, or the next frame builds from a stale offset. */
    if (rxCode != PACKET_RX_AWAIT)
    {
        p_parser->StateId = PACKET_RX_STATE_START;
        p_parser->Index = 0U;
        p_parser->Target = 1U;
    }

    return rxCode;
}


/*!
    Bytes still wanted before Advance may run. 0 means Advance is due - which happens with no
    further input when a frame ends exactly on a block boundary.

    Target is set by Advance rather than derived from Index, because "how many bytes do I
    want" and "how many do I hold" are independent: on first reaching LENGTH_MIN the parser
    wants exactly LENGTH_MIN, and only wants one more after PARSE_RX_LENGTH has declined to answer.
*/
static inline packet_size_t Packet_RxRemaining(const Packet_RxParser_T * p_parser)
{
    return (p_parser->Index < p_parser->Target) ? (packet_size_t)(p_parser->Target - p_parser->Index) : 0U;
}

/*!
    Rewind for the next frame. Retains Id and Length, which the caller still needs after a
    frame resolves.
*/
static inline void Packet_RxParser_Rewind(Packet_RxParser_T * p_parser)
{
    p_parser->StateId = PACKET_RX_STATE_START;
    p_parser->Index = 0U;
    p_parser->Target = 1U;
}

/*! Rewind and discard the last result. */
static inline void Packet_RxParser_Reset(Packet_RxParser_T * p_parser)
{
    p_parser->StateId = PACKET_RX_STATE_START;
    p_parser->Index = 0U;
    p_parser->Target = 1U;
    p_parser->Length = 0U;
    p_parser->Id = 0U;
}

/*! true once a delimiter has been accepted and the frame is still incomplete. */
static inline bool Packet_RxParser_IsInFrame(const Packet_RxParser_T * p_parser) { return (p_parser->StateId != PACKET_RX_STATE_START); }


/******************************************************************************/
/*!

*/
/******************************************************************************/



/*!
    @brief  Feed a block of bytes. Convenience over Remaining / Advance for a caller that
            already holds them.

    @param  p_buffer    destination, at least LENGTH_MAX bytes
    @param  p_consumed  bytes taken from p_src. Less than srcCount when a frame resolved part
                        way, leaving the remainder for the next call.
    @return the first frame to resolve, or AWAIT when p_src is exhausted.
*/
// static inline Packet_RxCode_T Packet_RxParser_Feed
// (
//     Packet_RxParser_T * p_parser, Packet_Format_T * p_format,
//     const uint8_t * p_src, packet_size_t srcCount,
//     uint8_t * p_buffer, packet_size_t * p_consumed
// )
// {
//     Packet_RxCode_T rxCode = PACKET_RX_AWAIT;
//     packet_size_t consumed = 0U;

//     while (rxCode == PACKET_RX_AWAIT)
//     {
//         packet_size_t remaining = Packet_RxParser_Remaining(p_parser);

//         if (remaining > 0U)
//         {
//             packet_size_t available = (srcCount - consumed);
//             packet_size_t take = (available < remaining) ? available : remaining;

//             for (packet_size_t index = 0U; index < take; index++) { p_buffer[p_parser->Index + index] = p_src[consumed + index]; }
//             p_parser->Index += take;
//             consumed += take;

//             if (take < remaining) { break; }     /* Source exhausted mid-target */
//         }

//         /* Reached with remaining == 0 when a frame ends exactly on a block boundary. */
//         rxCode = Packet_RxParser_Advance(p_format, p_buffer, p_parser);
//     }

//     *p_consumed = consumed;
//     return rxCode;
// }



/*  */
/* directly mapped to count */
// static inline Packet_RxState_T _Packet_RxStateOf(Packet_Format_T * p_specs, size_t rxCount)
// {
//     if (rxCount == 0U) { return PACKET_RX_STATE_START; }
//     else if (rxCount < p_specs->LENGTH_MIN) { return PACKET_RX_STATE_HEADER; }
//     // else if (rxCount < p_state-> Length) { return PROTOCOL_RX_STATE_WAIT_PACKET; }
//     else if (rxCount < p_specs->LENGTH_MAX) { return PACKET_RX_STATE_PAYLOAD; }
//     else { return PACKET_RX_STATE_START; } /* Invalid length, reset */
// }

// // static inline Packet_RxState_T Packet_RxStateOf(const Packet_RxParser_T * p_parser)
// // {
// //     if (p_parser->Index == 0U) { return PACKET_RX_STATE_START; }    /* nothing accepted yet */
// //     if (p_parser->Length == 0U) { return PACKET_RX_STATE_HEADER; }   /* delimiter in, length unresolved */
// //     return PACKET_RX_STATE_PAYLOAD;
// // }

// static inline Packet_RxCode_T _Packet_ProcRxParser(Packet_RxParser_T * p_parser, Packet_Format_T * p_specs, const uint8_t * p_rxBuffer)
// {
//     Packet_RxCode_T rxStatus = PACKET_RX_AWAIT;
//     switch (_Packet_RxStateOf(p_specs, p_parser->Index))
//     {
//         case PACKET_RX_STATE_START:
//             if (p_parser->Index > 0U)
//             {
//                 if ((p_rxBuffer[0U] == p_specs->START_ID) || (p_specs->START_ID == 0x00U)) { p_parser->Length = 0U; }
//                 else { p_parser->Index = 0U; }                // reset and keep waiting
//             }
//             break;
//         case PACKET_RX_STATE_HEADER:
//             if (p_parser->Index >= p_specs->LENGTH_MIN)
//             {
//                 p_parser->Length = p_specs->PARSE_RX_LENGTH(p_rxBuffer, p_parser->Index);
//                 if (p_parser->Length > p_specs->LENGTH_MAX || (p_parser->Length < p_parser->Index && p_parser->Length != 0U)) { rxStatus = PACKET_RX_ERROR_FRAME; }
//             }
//             break;
//         case PACKET_RX_STATE_PAYLOAD:
//             if (p_parser->Index >= p_parser->Length)
//             {
//                 rxStatus = (p_specs->IS_RX_VALID(p_rxBuffer, p_parser->Length) == true) ? PACKET_RX_COMPLETE : PACKET_RX_ERROR_DATA;
//                 // if (rxStatus == PACKET_RX_COMPLETE) { p_parser->Id = p_specs->ID_OF(p_rxBuffer); }
//                 // if (rxStatus == PACKET_RX_COMPLETE) { p_specs->PARSE_RX_HEADER(p_rxMeta, p_rxBuffer, p_parser->Index); }
//             }
//             break; // PACKET_COMPLETE or ERROR_DATA
//             // case PACKET_RX_STATE_INACTIVE:    break;
//         default: break;
//     }
//     if (rxStatus != PACKET_RX_AWAIT) { p_parser->Index = 0U; } //   p_state->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
//     return rxStatus;
// }




/* Drains the Xcvr Rx buffer into the Protocol Rx buffer */
/*
    Receive into P_RX_PACKET_BUFFER and run PARSE_RX_FRAMING for RxMeta.Length and ReqCode / Rx completion
    Packet is complete => Req, ReqExt or Sync, or Error
    Read into a contiguous buffer, which can be cast to Packet format
*/
// static inline Packet_RxCode_T CaptureRx(Packet_RxParser_T * p_parser, Packet_Format_T * p_format, const uint8_t * p_rx )
// {
//     Packet_RxCode_T rxStatus;
//     uint8_t nextRxIndex;

//     do /* Loop to empty Xcvr Rx buffer. Check for completetion, per Rx 1 byte, during unknown length, or up to known length */
//     {
//         /*
//             Set xcvrRxLimit for PARSE_RX_FRAMING. Prevent reading bytes from the following packet.
//             RxMeta.Length known => Check Rx remaining
//             RxMeta.Length unknown => Check 1 byte or a known const value, until RxMeta.Length is known
//         */
//         if (p_parser->Length > 0U) { nextRxIndex = p_parser->Length; } /* PacketLength is known. */

//         /* PacketLength is unknown */ /*nextRxIndex = max(p_format->RX_LENGTH_MIN, p_parser->Index + 1U)  */
//         else { nextRxIndex = (p_parser->Index < p_format->LENGTH_MIN) ? p_format->LENGTH_MIN : p_parser->Index + 1U; }

//         if (nextRxIndex > p_format->LENGTH_MAX || nextRxIndex < p_parser->Index) { return PACKET_RX_ERROR_FRAME; }
//         /* (RxIndex == nextRxIndex) => (xcvrRxLimit == 0), when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */

//         /* Copy from Xcvr buffer to Protocol buffer, up to xcvrRxLimit */
//         p_parser->Index += Xcvr_RxMax(p_parser->p_Xcvr, &p_rx[p_parser->Index], nextRxIndex - p_parser->Index);

//         if (p_parser->Index < nextRxIndex) { return PACKET_RX_AWAIT; } /* Xcvr Rx Buffer empty, wait for Xcvr */

//         /* returns PACKET_RX_AWAIT on successful set of meta data */ /* more bytes in Xcvr Buffer, continue while loop */
//         rxStatus = p_format->PARSE_RX_FRAMING(p_rx, p_parser->Index, &p_parser->RxMeta);
//     }
//     while (rxStatus == PACKET_RX_AWAIT);

//     return rxStatus;
// }