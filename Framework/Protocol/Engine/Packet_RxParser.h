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

/*
    Frame parser
*/
typedef struct Packet_RxParser
{
    Packet_RxState_T StateId;
    packet_size_t Index;        /* Bytes held in the buffer */
    packet_size_t NextIndex;    /* NextIndex. Bytes wanted before the next Packet_ProcRxParser. Never exceeds LENGTH_MAX. */
                                /* alternatively overload Length */
    // Packet_FrameFormat_T * p_FrameFormat; /* determine optional length field */
    /* Result. Valid from PACKET_RX_COMPLETE until the next frame overwrites it. */
    packet_size_t Length;       /* Total frame length. 0 while unknown. */
}
Packet_RxParser_T;


/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
// static inline packet_size_t _Packet_NextRxIndex(const Packet_RxParser_T * p_parser, const Packet_Format_T * p_format)
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

/*!
    Rewind for the next frame. Retains Id and Length, which the caller still needs after a
    frame resolves.
*/
static inline void Packet_ResetRx(Packet_RxParser_T * p_parser)
{
    p_parser->StateId = PACKET_RX_STATE_START;
    p_parser->Index = 0U;
    p_parser->NextIndex = 1U;
    p_parser->Length = 0U;
}


// static inline Packet_RxState_T Packet_RxStateOf(const Packet_RxParser_T * p_parser)
// {
//     if (p_parser->Index == 0U) { return PACKET_RX_STATE_START; }    /* nothing accepted yet */
//     if (p_parser->Length == 0U) { return PACKET_RX_STATE_HEADER; }   /* delimiter in, length unresolved */
//     return PACKET_RX_STATE_PAYLOAD;
//     // if (p_parser->Index >= p_parser->Length) { return PACKET_RX_STATE_PAYLOAD; }
//     // return PACKET_RX_STATE_START;
// }
/* directly mapped to count */
// static inline Packet_RxState_T _Packet_RxStateOf(Packet_Format_T * p_specs, size_t rxCount)
// {
//     if (rxCount == 0U) { return PACKET_RX_STATE_START; }
//     else if (rxCount < p_specs->LENGTH_MIN) { return PACKET_RX_STATE_HEADER; }
//     // else if (rxCount < p_state-> Length) { return PROTOCOL_RX_STATE_WAIT_PACKET; }
//     else if (rxCount < p_specs->LENGTH_MAX) { return PACKET_RX_STATE_PAYLOAD; }
//     else { return PACKET_RX_STATE_START; } /* Invalid length, reset */
// }

/*!
    Feed p_buffer, Packet_RxParser_T.Index holds valid length
*/
static inline Packet_RxCode_T Packet_ProcRxParser(Packet_RxParser_T * p_parser, const Packet_Format_T * p_format, const uint8_t * p_buffer)
{
    Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

    switch (p_parser->StateId)
    {
        case PACKET_RX_STATE_START:
            /* Anything but the delimiter is dropped, a byte at a time. START_ID of 0 accepts every byte. */
            if (p_parser->Index >= p_format->START_ID_LENGTH)
            {
                if ((p_buffer[0U] == p_format->START_ID) || (p_format->START_ID == 0x00U))
                {
                    p_parser->StateId = PACKET_RX_STATE_HEADER;
                    p_parser->Length = 0U;
                }
                else
                {
                    // p_parser->Index = 0U;
                    // p_parser->NextIndex = 1U;
                    p_parser->Index--; /* discard the last byte and retry. for LENGTH_MIN handling */
                }
            }
            p_parser->NextIndex = p_format->LENGTH_MIN;
            break;

        case PACKET_RX_STATE_HEADER: /* Wait for Length */
            p_parser->Length = p_format->PARSE_RX_LENGTH(p_buffer, p_parser->Index);

            if (p_parser->Length > 0U) { p_parser->NextIndex = p_parser->Length; } /* PacketLength is known. */
            else { p_parser->NextIndex = p_parser->Index + 1U; }  /* PacketLength is unknown. p_format->LENGTH_MIN already reached */

            /* Check Length through Target */
            /* (RxIndex == nextRxIndex) => (RxSize == 0), when rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET erroneously i.e. received full packet without completion status */
            if (p_parser->NextIndex > p_format->LENGTH_MAX || p_parser->NextIndex < p_parser->Index) { rxCode = PACKET_RX_ERROR_FRAME; }
            else if (p_parser->NextIndex == p_parser->Length) { p_parser->StateId = PACKET_RX_STATE_PAYLOAD; } /* p_parser->Target != 0U  */

            // /* Declined to answer. Grow the header by a byte, until it can no longer become a frame. */
            // if (p_parser->Length == 0U)
            // {
            //     if (p_parser->Index >= p_format->LENGTH_MAX) { rxCode = PACKET_RX_ERROR_FRAME; }
            //     else { p_parser->Target = p_parser->Index + 1U; }
            // }
            // else
            // {
                // /* A length that cannot describe this frame makes everything after it meaningless. */
            //     if ((p_parser->Length < p_parser->Index) || (p_parser->Length > p_format->LENGTH_MAX))
            //     {
            //         rxCode = PACKET_RX_ERROR_FRAME;
            //     }
            //     else
            //     {
            //         p_parser->StateId = PACKET_RX_STATE_PAYLOAD;
            //         p_parser->Target = p_parser->Length;
            //     }
            // }
            break;

        case PACKET_RX_STATE_PAYLOAD:
            // assert(p_parser->Index == p_parser->NextIndex); /* Ensure the whole payload has been received */
            // assert(p_parser->Index == p_parser->Length); /* Ensure the whole payload has been received */
            /* The whole frame is present, so this always resolves. */
            /* Frame is complete. caller parse remaining meta with PARSE_RX_HEADER */
            rxCode = (p_format->IS_RX_VALID(p_buffer, p_parser->Length) == true) ? PACKET_RX_COMPLETE : PACKET_RX_ERROR_DATA;
            break;

        default:
            break;
    }

    /* Rewind. Index must clear with the state, or the next frame builds from a stale offset. */
    if (rxCode != PACKET_RX_AWAIT) { Packet_ResetRx(p_parser); }

    return rxCode;
}


/*!

*/
static inline packet_size_t Packet_RxRemaining(const Packet_RxParser_T * p_parser)
{
    return (p_parser->Index < p_parser->NextIndex) ? (p_parser->NextIndex - p_parser->Index) : 0U;
}

/*! true once a delimiter has been accepted and the frame is still incomplete. */
static inline bool Packet_IsRxWaiting(const Packet_RxParser_T * p_parser) { return (p_parser->StateId != PACKET_RX_STATE_START); }


/******************************************************************************/
/*!

*/
/******************************************************************************/







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