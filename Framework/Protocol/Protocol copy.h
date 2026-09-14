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
    @file   Protocol.h
    @author FireSourcery
    @brief  Composition. The one layer that owns the Xcvr and the clock.
*/
/******************************************************************************/
#include "Packet.h"
#include "Packet_RxParser.h"
#include "Protocol_Sync.h"
#include "Protocol_Request.h"
#include "Peripheral/Xcvr/Xcvr.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/******************************************************************************/
/*
    Layer   Module              In              Out                     I/O
    -----   ------------------  --------------  ----------------------  ---
    2       Packet_RxParser     bytes           Packet_RxCode_T         no
    -       Packet_ClassOf      packet_id_t     Packet_ClassId_T        no
    4       Protocol_Sync       Packet_ClassId  Protocol_SyncEvent_T    no
    5       Protocol_Request    the floor       Protocol_ReqCode_T      no
    -       here                                bytes                   YES

    Every layer below is a pure function of its state and its input. This file is where the
    Xcvr, the buffers and the clock live, and it is the only place a byte moves or a deadline
    is read. That boundary is the point of the whole arrangement: the protocol logic can be
    driven from a test vector, and only this file needs hardware.

    Two deadlines, deliberately distinct, each held by the state machine it measures, and
    deliberately NOT the same mechanism:

        RxParser.RxTimeElapsed  a frame stalled part way. Resyncs the parser. Counts the
                                deltas it is given, because it has one consumer and firing
                                late only means holding a partial frame longer.
        Sync.AckTimeStart       an exchange stalled. Retransmits or abandons via ARQ. Stays
                                on absolute time: Socket's watchdog reads the same base at a
                                longer threshold, and one base serves N thresholds for free
                                where an accumulator serves exactly one.

    This file owns P_TIMER, reads it once per pass, and hands down a reading or a duration.
    Nothing below reads a clock, so every layer stays drivable from a table of inputs - and
    neither deadline can be re-armed out of step with the transition it belongs to, because
    the arm lives with the transition.
*/
/******************************************************************************/
/*
    Everything constant for the duration of a pass, except the two things the socket selects
    at runtime. The Xcvr and the format travel as arguments precisely because they change:
    holding them here would mean rebuilding the whole struct to swap a port.
*/
typedef const struct Protocol_Base
{
    Packet_Context_T * P_RX_PACKET;         /* Meta + contiguous frame buffer */
    Packet_Context_T * P_TX_PACKET;
    uint8_t PACKET_BUFFER_LENGTH;           /* Must be >= FORMAT->LENGTH_MAX */

    /* The request service. Id -> handler, plus the storage handlers run against. */
    // Packet_ReqIdResolver_T REQ_ID_RESOLVER; /* Function to resolve request IDs */
    const Protocol_Req_T * P_REQ_TABLE;
    uint8_t REQ_TABLE_LENGTH;
    void * P_APP_CONTEXT;                   /* Passed to every handler */
    void * P_REQ_CONTEXT;                   /* Handler sub-state. Sized for the largest handler */

    const volatile uint32_t * P_TIMER;
    const uint32_t REQ_TIMEOUT;             /* Exchange deadline. The frame deadline is the codec's RX_TIMEOUT. */
}
Protocol_Base_T;


#ifndef PROTOCOL_CHECK_RX_FRAME_CONSISTENCY
#define PROTOCOL_CHECK_RX_FRAME_CONSISTENCY (0)
#endif

#ifndef PROTOCOL_CHECK_TX_FRAME_BOUNDS
#define PROTOCOL_CHECK_TX_FRAME_BOUNDS (0)
#endif

/*
    P_RX_PACKET and P_TX_PACKET must be distinct buffers - the handler's two payload pointers
    are declared restrict, so one buffer serving both makes every handler call undefined.

        assert(p_link->P_RX_PACKET != p_link->P_TX_PACKET);
*/
typedef struct Protocol_State
{
    Packet_RxParser_T RxParser;
    Protocol_SyncState_T Sync; /* Sync already holds everything that outlives the [Req] binding */
    Protocol_ReqState_T Req;
    /*
        Previous pass's clock reading, and the only timestamp left in this struct. The Rx
        deadline counts durations, so the delta has to be derived once somewhere; doing it
        here keeps every layer below free of a clock. Written once per Protocol_Proc.
    */
    uint32_t LastTime;
    // Xcvr_T * p_xcvr;
    // Packet_Codec_T * p_format;

    /*
        Observation only. Nothing reads these back.
        A link is diagnosed from these, not from a logic analyser, so they are part of the
        engine rather than a debug build.
    */
    struct
    {
        uint16_t Frames;        /* Intact frames delivered */
        uint16_t FrameErrors;   /* Unframeable - no usable length */
        uint16_t DataErrors;    /* Checksum failures */
        uint16_t RxTimeouts;    /* Frames abandoned part way */
        uint16_t ReqTimeouts;   /* Exchanges abandoned - deadline expired or retry budget spent */
        uint16_t Retransmits;
        uint16_t NoHandler;     /* Intact frames whose id has no table entry */
    }
    Stat;
}
Protocol_State_T;


static inline void _Protocol_TickStat(uint16_t * p_counter)
{
#ifndef NDEBUG
    (*p_counter)++;
#else
    (void)p_counter;
#endif
}


static inline void Protocol_ResetReqSync(Protocol_State_T * p_state)
{
    Protocol_ResetReq(&p_state->Req);
    Protocol_ResetSync(&p_state->Sync);
    // p_state->p_ReqFraming = NULL;
    // p_state->TxLength = 0U;
}

/*! Full reset. Drops any frame in progress along with the exchange. Counters survive. */
static inline void Protocol_Reset(Protocol_Base_T * p_base, Protocol_State_T * p_state)
{
    Packet_ResetRx(&p_state->RxParser);
    Protocol_ResetReqSync(p_state);
    Packet_ResetRxTime(&p_state->RxParser);
    p_state->LastTime = *p_base->P_TIMER;
    Protocol_MarkSyncTime(&p_state->Sync, *p_base->P_TIMER);
}


/******************************************************************************/
/*!
    Tx - every byte out of this engine passes through here
*/
/******************************************************************************/
static inline bool Protocol_TxResponse(const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec, const Packet_FrameFormat_T * p_format, Packet_Context_T * p_tx)
{
    Packet_BuildTxFrame(p_codec, &p_tx->Meta, p_tx->Packet);
    return Xcvr_TxN(p_xcvr, p_tx->Packet, Packet_FrameLengthOf(p_format, &p_tx->Meta));
}

/*!
    Transmit a control frame. A control frame is an ordinary frame with a control id and no payload

    Built into a local frame so the staged response survives for retransmission.

    The bound on frame[] is admission-time, in Socket_SetFormat: a format whose zero-payload
    frame exceeds PACKET_CONTROL_LENGTH_MAX is refused before it can ever be selected.
*/
static inline void Protocol_TxControl(const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec, Packet_ClassId_T txClass)
{
    uint8_t frame[PACKET_CONTROL_LENGTH_MAX];
    Packet_Meta_T meta = { .Id = Packet_ControlIdOf(p_codec, txClass), .Length = 0U };
    Packet_BuildTxFrame(p_codec, &meta, frame);
    Xcvr_TxN(p_xcvr, frame, p_codec->CONTROL_FRAME_FORMAT.HEADER_LENGTH);
}

/******************************************************************************/
/*!
    Rx - drive the sans-IO parser from the Xcvr
*/
/******************************************************************************/
/*
    Drain the Xcvr into the frame buffer. The parser only ever asks for what the current
    frame still needs, so a burst is consumed without reading into the following frame.

    Zero remaining skips the read and advances anyway, which is the phase boundary that lands
    exactly on the frame end. Termination: every AWAIT path out of the parser either leaves
    NextIndex > Index, so the next turn needs bytes and the Xcvr eventually runs dry, or
    changes state, which cannot repeat - START to HEADER to PAYLOAD resolves.
*/
static inline Packet_RxCode_T Protocol_CaptureRx(const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec, Packet_RxParser_T * p_parser, uint8_t * p_rxFrame)
{
    Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

    while (rxCode == PACKET_RX_AWAIT)
    {
        packet_size_t remaining = Packet_RxRemaining(p_parser);

        /* Nothing moves until the whole target is available. */
        if (remaining > 0U)
        {
            if (Xcvr_RxN(p_xcvr, &p_rxFrame[p_parser->Index], remaining) == false) { break; }
            p_parser->Index += remaining;
        }

        rxCode = Packet_ProcRxParser(p_parser, p_codec, p_rxFrame);
    }

    return rxCode;
}


/******************************************************************************/
/*!
    Proc Protocol_Base_T buffer context
*/
/******************************************************************************/
/*!
    Bytes into one intact frame, and the frame into Meta.

    No id is resolved here. A control frame carries no handler and needs none, so looking one
    up on every complete frame is what would nack an arriving ACK - the id belongs to the
    request block, which is the only block that needs a handler.
*/
static inline Packet_RxCode_T Protocol_ProcRxFrame(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec, uint32_t deltaTime)
{
    Packet_RxCode_T rxCode = Protocol_CaptureRx(p_xcvr, p_codec, &p_state->RxParser, p_link->P_RX_PACKET->Packet);

    /* Aged after draining, so expiry means the line really did go quiet. */
    if (rxCode == PACKET_RX_AWAIT) { rxCode = Packet_ProcRxTimeout(&p_state->RxParser, p_codec, deltaTime); }

    switch (rxCode)
    {
        /* Stores Meta with either Request or Ack format */
        case PACKET_RX_COMPLETE:
            Packet_ParseRxFrame(p_codec, &p_link->P_RX_PACKET->Meta, p_link->P_RX_PACKET->Packet);
            p_state->Stat.Frames++;
            break;

        /* Unusable frame. No id was learned, so no handler policy can apply. */
        case PACKET_RX_ERROR_FRAME:     Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK); p_state->Stat.FrameErrors++;    break;
        case PACKET_RX_ERROR_DATA:      Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK); p_state->Stat.DataErrors++;     break;
        case PACKET_RX_ERROR_TIMEOUT:   Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK); p_state->Stat.RxTimeouts++;     break;
        /* Frame still open, deadline intact. */
        case PACKET_RX_AWAIT:           break;
        default:            break;
    }

    return rxCode;
}



/*
    Deliver a frame to the handler.

    Binding precedes the ack because the ack policy is the handler's; invocation follows it
    because "received" and "processed" are different claims.
*/
/*! @return false when the id has no handler - the caller must not go on to dispatch. */
static inline bool Protocol_StartRequest(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec)
{
    if (Protocol_CaptureReqOfTable(&p_state->Req, p_link->P_REQ_TABLE, p_link->REQ_TABLE_LENGTH, p_link->P_RX_PACKET->Meta.Id) == false)
    {
        Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);     /* no handler for this id */
        p_state->Stat.NoHandler++;
        return false;
    }

    /*
        Is FRAME_FORMAT the shape the codec framed these bytes as? The offsets the handler is
        about to be given come from that row, and the bytes came from the codec.

        Not a re-check of the length: the capture was already held to LENGTH_MAX through
        Packet_RxRemaining, and IS_RX_VALID covered exactly the bytes collected.
    */
#if PROTOCOL_CHECK_RX_FRAME_CONSISTENCY
    if (Packet_IsFrameConsistent(Protocol_ReqId(&p_state->Req)->FRAME_FORMAT, &p_link->P_RX_PACKET->Meta, Packet_RxFrameLength(&p_state->RxParser)) == false)
    {
        Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);
        p_state->Stat.FrameErrors++;
        Protocol_ResetReqSync(p_state);
        return false;
    }
#endif

    Protocol_AckPolicy_T policy = Protocol_ReqAckPolicy(&p_state->Req);

    /* Receiver-side ack: a reflex on arrival, not a state. */
    if (policy.SEND_ACK_REQ) { Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK); }
    return true;
}

/*!
    Both payloads start past the header of the bound row's shape. The binding is fixed for the
    life of the exchange, so neither a control frame nor a foreign id can move them.
*/
static void Protocol_ProcRequest(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec, uint32_t timerNow)
{
    Packet_Xfer_T xfer = { .p_RxMeta = &p_link->P_RX_PACKET->Meta, .p_TxMeta = &p_link->P_TX_PACKET->Meta, .p_Substate = p_link->P_REQ_CONTEXT, };

    Protocol_AckPolicy_T policy = Protocol_ReqAckPolicy(&p_state->Req); /* PROTOCOL_REQ_DONE unbinds Req */
    /* The shape the row ANSWERS with - the same lookup Protocol_ProcReqState offsets tx by. */
    Packet_FrameFormat_T * p_respFormat = Protocol_ReqRespId(&p_state->Req)->FRAME_FORMAT;

    /* PROTOCOL_REQ_RESPOND or PROTOCOL_REQ_DONE unbinds the requesy Id */
    Protocol_ReqCode_T reqCode = Protocol_ProcReqState(&p_state->Req, p_link->P_APP_CONTEXT, &xfer, p_link->P_RX_PACKET->Packet, p_link->P_TX_PACKET->Packet);

    /*
        The handler's Length, before the frame builder indexes by it. A constant at most call
        sites - see the conditions on PROTOCOL_CHECK_TX_FRAME_BOUNDS - but a bulk transfer
        computes it from a cursor and a configured chunk size, and the bound that matters is
        the whole frame against the buffer: a payload one byte under LENGTH_MAX still overruns
        once the header and trailer are added.

        A response that does not fit is the handler's bug, so it is dropped and the exchange
        abandoned rather than nacked - a nack would invite a retransmit of the same frame.
    */
#if PROTOCOL_CHECK_TX_FRAME_BOUNDS
    if ((reqCode == PROTOCOL_REQ_RESPOND) || (reqCode == PROTOCOL_REQ_DONE))
    {
        if (Packet_IsFrameWithin(p_respFormat, &p_link->P_TX_PACKET->Meta, p_link->PACKET_BUFFER_LENGTH) == false)
        {
            Protocol_ResetReqSync(p_state);
            return;
        }
    }
#endif

    switch (reqCode)
    {
        /* Act on the handler's verdict. The only place a request's output reaches the wire. */
        case PROTOCOL_REQ_RESPOND:
        case PROTOCOL_REQ_DONE:
            if (Protocol_TxResponse(p_xcvr, p_codec, p_respFormat, p_link->P_TX_PACKET))
            {
                if (policy.EXPECT_ACK_RESP) { Protocol_ExpectAck(&p_state->Sync, policy, p_respFormat, timerNow); }
                else { Protocol_ResetSync(&p_state->Sync); }
            }
            break;

        /* Handler judged the frame itself, in place of the arrival reflex. */
        case PROTOCOL_REQ_ACCEPT:   Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK);   break;
        case PROTOCOL_REQ_REJECT:   Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);  break;
        case PROTOCOL_REQ_ABORT:    Protocol_ResetReqSync(p_state); break;
        case PROTOCOL_REQ_AWAIT:    break;      /* yields, nothing goes out */
        default: break;
    }
}

/*! true while an exchange occupies the socket, in either layer. */
static inline bool Protocol_IsReqSyncActive(const Protocol_State_T * p_state) { return Protocol_IsReqActive(&p_state->Req) || Protocol_IsAckWaiting(&p_state->Sync); }

/*
    (PROTOCOL_REQ_STATE_ACTIVE, PROTOCOL_SYNC_AWAIT_ACK) => Await next ACK then returns to PROTOCOL_SYNC_AWAIT_ACK
    (PROTOCOL_REQ_STATE_IDLE, PROTOCOL_SYNC_AWAIT_ACK) => Await final ACK, Req == NULL

    OPEN	IDLE	nothing in progress
    OPEN	ACTIVE	data-paced exchange, handler awaiting the next frame
    AWAIT_ACK	IDLE	stateless response outstanding
    AWAIT_ACK	ACTIVE	staged response outstanding, sequence continues
*/
static inline void Protocol_Proc(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec)
{
    /*
        One reading for the whole pass, and the delta the Rx deadline ages itself by. Two
        volatile reads could straddle a tick and time a frame against a clock that moved
        between arming it and testing it.

        The delta is derived here and nowhere else - the parser counts durations, so this is
        the only place that has to know what a clock is.
    */
    uint32_t timerNow = *p_link->P_TIMER;
    uint32_t deltaTime = timerNow - p_state->LastTime;
    p_state->LastTime = timerNow;

    /* 1. Rx - bytes into one intact frame, or the frame deadline on a line gone quiet. */
    Packet_RxCode_T rxCode = Protocol_ProcRxFrame(p_link, p_state, p_xcvr, p_codec, deltaTime);

    /*
        2. Sync - one event, whichever source raised it. The Rx outcome picks the source: a
        resolved frame has a class to fold in, an open one lets the exchange deadline age.

        A frame that resolved BADLY was already answered in step 1, and must not also be timed
        out in the same pass - that would put two control frames on the line for one event.
        Nor does a good frame age the deadline: it just re-armed it.
    */
    Protocol_SyncEvent_T syncEvent;

    switch (rxCode)
    {
        case PACKET_RX_COMPLETE:
            syncEvent = Protocol_ProcSyncPacket(&p_state->Sync, Packet_ClassOf(p_codec, p_link->P_RX_PACKET->Meta.Id), timerNow);
            break;

        /* The gate spans both layers, so it is computed here and handed down. */
        case PACKET_RX_AWAIT:
            if (Protocol_IsReqSyncActive(p_state) == false) { return PROTOCOL_SYNC_EVENT_NONE; }   /* no slide: the watchdog reads this base */
            syncEvent = Protocol_ProcSyncTimeout(&p_state->Sync, p_link->REQ_TIMEOUT, timerNow);
            break;

        default:
            syncEvent = PROTOCOL_SYNC_EVENT_NONE;
            break;
    }

    /*
        3. Act. One effect per event, once each. Sync has already moved its own state and
        armed its own deadline, so nothing below re-stamps a clock, and nothing re-tests a
        condition the event itself asserts.
    */
    switch (syncEvent)
    {
        /* A bound request is necessarily ACTIVE, so both arms converge on the same call. */
        case PROTOCOL_SYNC_EVENT_REQUEST:
            if (Protocol_StartRequest(p_link, p_state, p_xcvr, p_codec) == false) { Protocol_ProcRequest(p_link, p_state, p_xcvr, p_codec, timerNow); }
            break;

        /* Our frame landed. Resume a sequence; a stateless exchange has already closed. */
        case PROTOCOL_SYNC_EVENT_RESUME:
            if (Protocol_IsReqActive(&p_state->Req) == true) { Protocol_ProcRequest(p_link, p_state, p_xcvr, p_codec, timerNow); }
            break;

        /*
            THE retransmit. A nack and an expired deadline both arrive here, and the event is
            not raised at all unless a shape is latched to re-send - see _Protocol_CanRetry.
        */
        case PROTOCOL_SYNC_EVENT_RETRANSMIT:
            Protocol_TxResponse(p_xcvr, p_codec, Protocol_SyncRespFormat(&p_state->Sync), p_link->P_TX_PACKET);
            p_state->Stat.Retransmits++;
            break;

        /* Acknowledging a received abort, rather than raising a second one. */
        case PROTOCOL_SYNC_EVENT_ABORT:
            if (Protocol_ReqAckPolicy(&p_state->Req).SEND_ACK_ABORT == true) { Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK); }
            Protocol_ResetReq(&p_state->Req);   /* Sync reset itself in _Protocol_ResolveSync */
            break;

        case PROTOCOL_SYNC_EVENT_REJECT:
            Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);
            break;

        /*
            Budget spent, or the deadline expired with nothing left to retry. Silent, and
            deliberately: the trigger may have been a nack, and answering a nack with a nack
            is the storm path this layer exists to avoid. A peer that has said nothing for a
            whole REQ_TIMEOUT would not hear it either. ABORT is the frame that means
            "abandoned", if the far side ever needs telling.
        */
        case PROTOCOL_SYNC_EVENT_FAILED:
            Protocol_ResetReq(&p_state->Req);
            p_state->Stat.ReqTimeouts++;
            break;

        case PROTOCOL_SYNC_EVENT_NONE:
        default:
            break;
    }
}


// static inline void Protocol_ProcRequestTimeout(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Codec_T * p_format)
// {
//     uint32_t timerNow = *p_link->P_TIMER;
//     if (Protocol_IsReqSyncActive(p_state) && Protocol_IsSyncElapsed(&p_state->Sync, p_link->REQ_TIMEOUT, timerNow))
//     {
//         switch (Protocol_ResolveAckTimeout(&p_state->Sync))
//         {
//             case PROTOCOL_SYNC_EVENT_RETRANSMIT:
//                 /*
//                     Gating on IsReqActive would be safe but wrong: a stateless handler that
//                     returned DONE is already unbound while its response is still outstanding,
//                     so the retransmit would be skipped silently and the budget spent on
//                     nothing until the exchange failed. The latched shape is what is actually
//                     outstanding, and it is set for exactly as long as AWAIT_ACK holds.
//                 */
//                 if (Protocol_SyncRespFormat(&p_state->Sync) != NULL)
//                 {
//                     Protocol_TxResponse(p_xcvr, p_format, Protocol_SyncRespFormat(&p_state->Sync), p_link->P_TX_PACKET);
//                     Protocol_MarkSyncTime(&p_state->Sync, timerNow);
//                     p_state->Stat.Retransmits++;
//                 }
//                 break;
//             case PROTOCOL_SYNC_EVENT_FAILED:
//             default:
//                 Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
//                 Protocol_ResetReqSync(p_state);
//                 p_state->Stat.ReqTimeouts++;
//                 break;
//         }
//     }
// }


// static inline void Protocol_Proc1(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Codec_T * p_codec)
// {
//     /* 1. Rx - bytes into one intact frame, or the frame deadline on a line gone quiet. */
//     uint32_t timerNow = *p_link->P_TIMER;
//     Packet_RxCode_T rxCode = Protocol_ProcRxFrame(p_link, p_state, p_xcvr, p_codec, deltaTime);

//     /* 2. Sync - what that frame means to the handshake. Only a resolved frame has a class. */
//     Protocol_SyncEvent_T syncEvent = (rxCode == PACKET_RX_COMPLETE)
//         ? Protocol_ProcSyncState(&p_state->Sync, Packet_ClassOf(p_codec, p_link->P_RX_PACKET->Meta.Id))
//         : PROTOCOL_SYNC_EVENT_NONE;

//     /* 3. Act. Acks and aborts resolve here; only a data frame reaches a handler. */

//     switch (syncEvent)
//     {
//         case PROTOCOL_SYNC_EVENT_REQUEST:
//             Protocol_MarkSyncTime(&p_state->Sync, timerNow);
//             if (Protocol_StartRequest(p_link, p_state, p_xcvr, p_codec) == true) { Protocol_ProcRequest(p_link, p_state, p_xcvr, p_codec); }
//             break;

//             /* Our frame landed. Resume a sequence, or close a stateless exchange. */
//         case PROTOCOL_SYNC_EVENT_RESUME:
//             Protocol_MarkSyncTime(&p_state->Sync, timerNow);
//             if (Protocol_IsReqActive(&p_state->Req) == true) { Protocol_ProcRequest(p_link, p_state, p_xcvr, p_codec); }
//             break;

//         case PROTOCOL_SYNC_EVENT_RETRANSMIT:
//             if (Protocol_SyncRespFormat(&p_state->Sync) != NULL)
//             {
//                 Protocol_TxResponse(p_xcvr, p_codec, Protocol_SyncRespFormat(&p_state->Sync), p_link->P_TX_PACKET);
//                 Protocol_MarkSyncTime(&p_state->Sync, timerNow);
//                 p_state->Stat.Retransmits++;
//             }
//             break;

//             /*
//                 Answering an abort with an ack - the field acknowledges a received abort rather than raising a second one.
//             */
//         case PROTOCOL_SYNC_EVENT_ABORT:
//             if (Protocol_ReqAckPolicy(&p_state->Req).SEND_ACK_ABORT == true) { Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK); }
//             Protocol_ResetReqSync(p_state);
//             break;

//         case PROTOCOL_SYNC_EVENT_REJECT:
//             Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);
//             break;

//         case PROTOCOL_SYNC_EVENT_FAILED:
//             Protocol_ResetReqSync(p_state);
//             break;

//         case PROTOCOL_SYNC_EVENT_NONE:
//             break;

//         default:
//             break;
//     }

//     /* Exchange deadline, spanning Sync and Request both */
//     Protocol_ProcRequestTimeout(p_link, p_state, p_xcvr, p_codec);
// }