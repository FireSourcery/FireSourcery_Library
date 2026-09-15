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
#include <assert.h>

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
    PACKET_RX_ERROR_TIMEOUT,    /* Frame stalled part way. Reached by the time event. */
}
Packet_RxCode_T;

/******************************************************************************/
/*!
    Parser State
*/
/******************************************************************************/
typedef enum Packet_RxState
{
    /* Feed convention */
    PACKET_RX_STATE_START,      /* Scanning for the delimiter */
    PACKET_RX_STATE_LENGTH,     /* Growing the header until the length is known */
    PACKET_RX_STATE_END,        /* Collecting the rest of the frame */
}
Packet_RxState_T;

/*
    Frame parser
*/
typedef struct Packet_RxParser
{
    Packet_RxState_T StateId;
    packet_size_t Index;        /* Bytes held in the buffer */
    packet_size_t Remaining;

    /*
        Total frame length. 0 while unknown This is the parser's own account of the frame, independent
        of anything the format says about it, so it is what a caller checks Meta.Length against.
        Cleared on entry to HEADER, when the next frame makes it meaningless.
    */
    packet_size_t ResolvedLength;
    // Packet_FrameFormat_T * p_FrameFormat;
    uint32_t TimeStart;
}
Packet_RxParser_T;


/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*!
    Rewind the progress for the next frame, keeping ResolvedLength.

    The resolved length is the parser's own account of the frame, and the only number the
    caller can check the format's Meta.Length against - PARSE_RX_LENGTH and PARSE_RX_HEADER
    read the wire independently and can disagree. Clearing it here would leave the caller
    with nothing but the format's word for how long its own frame is.

    ResolvedLength is cleared on entry to HEADER, where it stops describing anything.
*/
static inline void Packet_ResetRxState(Packet_RxParser_T * p_parser)
{
    p_parser->StateId = PACKET_RX_STATE_START;
    p_parser->Index = 0U;
    // p_parser->NextIndex = 1U;
    p_parser->Remaining = 1U;
}

/*! Full reset. Drops the resolved length too - for abandonment, not for a frame that landed. */
static inline void Packet_ResetRx(Packet_RxParser_T * p_parser)
{
    Packet_ResetRxState(p_parser);
    // p_parser->ResolvedLength = 0U;
}

/*!
    The one exit convention, shared by both events.

    Any code but AWAIT has resolved the frame, so the progress is rewound for the next one -
    whichever event produced it. Having it named once is the point: a byte event and a time
    event that each rewound for themselves would be two conventions that merely agree.
*/
static inline Packet_RxCode_T _Packet_ResolveRx(Packet_RxParser_T * p_parser, Packet_RxCode_T rxCode)
{
    if (rxCode != PACKET_RX_AWAIT) { Packet_ResetRxState(p_parser); }
    return rxCode;
}

/*!
    Single byte delimiter, per the format's author contract: START_ID_LENGTH is 0 or 1, so
    only p_buffer[0] is tested. A wider START_ID would compare a uint8_t against a value that
    cannot fit it and never match - the contract is what keeps that unreachable.
*/
static inline bool _Packet_IsStartId(const Packet_Codec_T * p_format, const uint8_t * p_buffer)
{
    return ((p_format->START_ID == 0x00U) || (p_buffer[0U] == p_format->START_ID));
}

/*!
    Feed p_buffer, Packet_RxParser_T.Index holds valid length
    called when p_parser->Index == p_parser->NextIndex
    Drains the Xcvr Rx buffer into a contiguous buffer, which can be cast to Packet format
    Receive into P_RX_PACKET_BUFFER and run PARSE_RX_FRAMING for RxMeta.Length and ReqCode / Rx completion
    Packet is complete => Req, ReqExt or Sync, or Error
*/
static inline Packet_RxCode_T Packet_ProcRxParser(Packet_RxParser_T * p_parser, const Packet_Codec_T * p_format, const uint8_t * p_buffer)
{
    // assert(p_parser->Index == p_parser->NextIndex); /* using Xcvr_RxN length or nothing */

    Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

    switch (p_parser->StateId)
    {
        case PACKET_RX_STATE_START:
            /*
                Scanning a byte at a time is what keeps the reject branch from losing bytes.
                Bulk-reading LENGTH_MIN and searching inside it would need the delimiter shifted to the front
                on a clean line the delimiter is already byte 0, so this costs one extra RxN per frame.
            */
            if (_Packet_IsStartId(p_format, p_buffer) == true)
            {
                p_parser->Remaining = p_format->LENGTH_MIN - p_parser->Index;
                p_parser->StateId = PACKET_RX_STATE_LENGTH;
                // p_parser->NextIndex = p_format->LENGTH_MIN;
            }
            else
            {
                p_parser->Remaining = p_format->START_ID_LENGTH; /* 1 */
                p_parser->Index = 0U;                                       /* discard, rescan from the front */
                // p_parser->NextIndex = p_format->START_ID_LENGTH;
            }
            break;

        case PACKET_RX_STATE_LENGTH: /* Wait for Length */
            /* 0 = not yet determinable. Grow the header a byte at a time until the format answers. */
            packet_size_t frameLength = p_format->PARSE_RX_LENGTH(p_buffer, p_parser->Index);
            /*
                Set for NextRx before the next PARSE_RX_FRAMING. Prevent reading bytes from the following packet.
                RxLength known => Get Rx remaining
                RxLength unknown => Get 1 byte or a known const value, until RxLength is known
            */
            packet_size_t nextIndex = (frameLength > 0U) ? frameLength : (p_parser->Index + 1U);
            // p_parser->Remaining = (frameLength > 0U) ? (frameLength - p_parser->Index) : 1U;
            /* One bound covers both: a length that overruns the buffer, and one that undercuts what is already held. */
            if ((nextIndex > p_format->LENGTH_MAX) || (nextIndex < p_parser->Index)) { rxCode = PACKET_RX_ERROR_FRAME; }
            else
            {
                p_parser->Remaining = nextIndex - p_parser->Index;
                p_parser->StateId = PACKET_RX_STATE_END;
            }

            p_parser->ResolvedLength = frameLength; /* storage for consistency check only */
            break;

        case PACKET_RX_STATE_END:
            // assert(p_parser->Index == p_parser->ResolvedLength); /* Ensure the whole payload has been received */
            /* The whole frame is present, so this always resolves. */
            /* Frame is complete. caller parse remaining meta with PARSE_RX_HEADER */
            rxCode = (p_format->IS_RX_VALID(p_buffer, p_parser->Index) == true) ? PACKET_RX_COMPLETE : PACKET_RX_ERROR_DATA;
            break;

        default:
            break;
    }
    /*
        Continue CaptureRx during ReqExt processing
        Buffers may be overwritten after Req returns. No repeat process on same Rx Data State
        Req ensure packet data is processed, or copied
        Rx can queue out of sequence. Invalid Rx sequence until timeout buffer flush
        check for Abort without user signal, persistent wait process
    */
    /*
        Rewind. [Index] must clear with the state, or the next frame builds from a stale offset.
        ResolvedLength survives so the caller can check the format's Meta.Length against it.
    */
    return _Packet_ResolveRx(p_parser, rxCode);
}

/*! true once a delimiter has been accepted and the frame is still incomplete. */
static inline bool Packet_IsRxActive(const Packet_RxParser_T * p_parser) { return (p_parser->StateId != PACKET_RX_STATE_START); }


/******************************************************************************/
/*!
    Frame deadline - the machine's second event

    One state, two events, one exit. Packet_ProcRxParser is the byte event, this is the time
    event, and both resolve through _Packet_ResolveRx. That is why PACKET_RX_ERROR_TIMEOUT is
    a member of Packet_RxCode_T rather than a bool the caller has to fold in on the side.
*/
/******************************************************************************/
static inline Packet_RxCode_T Packet_ProcRxTimeout(Packet_RxParser_T * p_parser, const Packet_Codec_T * p_format, uint32_t timerNow)
{
    if (p_parser->StateId == PACKET_RX_STATE_START) { p_parser->TimeStart = timerNow; return PACKET_RX_AWAIT; }
    if (p_format->RX_TIMEOUT == 0U) { return PACKET_RX_AWAIT; }

    return ((timerNow - p_parser->TimeStart) > p_format->RX_TIMEOUT) ? _Packet_ResolveRx(p_parser, PACKET_RX_ERROR_TIMEOUT) : PACKET_RX_AWAIT;
}

/*!
    Query after Proc
*/
// return (p_parser->ResolvedLength > 0U) ? (p_parser->ResolvedLength - p_parser->Index) : 1U;
static inline packet_size_t Packet_RxRemaining(const Packet_RxParser_T * p_parser) { return p_parser->Remaining; }

/*! The parser's own account of the resolved frame. Valid until the next frame reaches HEADER. */
static inline packet_size_t Packet_RxFrameLength(const Packet_RxParser_T * p_parser) { return p_parser->ResolvedLength; }
// static inline packet_size_t Packet_RxEnd(const Packet_RxParser_T * p_parser) { return p_parser->ResolvedLength; }


