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
               Packet_RxParser     bytes           Packet_RxCode_T         no
               Packet_ClassOf      packet_id_t     Packet_ClassId_T        no
               Protocol_Sync       Packet_ClassId  Protocol_SyncEvent_T    no
               Protocol_Request    the floor       Protocol_ReqCode_T      no
               here                                bytes                   YES

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
#ifndef PROTOCOL_CHECK_RX_FRAME_CONSISTENCY
#define PROTOCOL_CHECK_RX_FRAME_CONSISTENCY (0)
#endif

#ifndef PROTOCOL_CHECK_TX_FRAME_BOUNDS
#define PROTOCOL_CHECK_TX_FRAME_BOUNDS (0)
#endif


/******************************************************************************/
/*!
    Helpers
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Tx - every byte out of this engine passes through here
*/
/******************************************************************************/
static inline bool Protocol_TxResponse(Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, Packet_FrameFormat_T * p_format, Packet_Context_T * p_tx)
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
static inline void Protocol_TxControl(Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, Packet_ClassId_T txClass)
{
    uint8_t frame[PACKET_CONTROL_LENGTH_MAX];
    Packet_Meta_T meta = { .Id = Packet_ControlIdOf(p_codec, txClass), .Length = 0U };
    Packet_BuildTxFrame(p_codec, &meta, frame);
    Xcvr_TxN(p_xcvr, frame, p_codec->CONTROL_FRAME_LENGTH);
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
static inline Packet_RxCode_T Protocol_CaptureRx(Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, Packet_RxParser_T * p_parser, uint8_t * p_rxFrame)
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


static inline void _Protocol_TickStat(uint16_t * p_counter)
{
#ifndef NDEBUG
    (*p_counter)++;
#else
    (void)p_counter;
#endif
}

/******************************************************************************/
/*!

*/
/******************************************************************************/

/*
    Multiple context segments
    request service
    alternatively keep sync config here
*/
typedef const struct
{
    /* The request service. Id -> handler, plus the storage handlers run against. */
    const Protocol_Req_T * P_REQ_TABLE;
    uint8_t REQ_TABLE_LENGTH;
    void * P_APP_CONTEXT;                      /* Passed to every handler */
    // Packet_ReqIdResolver_T REQ_ID_RESOLVER; /* Function to resolve request IDs */
    const uint32_t REQ_TIMEOUT;              /* Exchange deadline. The frame deadline is the codec's RX_TIMEOUT. */
}
Protocol_ReqTable_T;

/*
    Compile-time wiring, resident in flash - the handler's Packet_Xfer_T is not rebuilt
    per call.

    Effectively:
    struct
    {
        union { const Packet_Meta_T * const p_RxMeta;  Packet_Context_T * const p_RxBuffer; };
        union { Packet_Meta_T * const p_TxMeta;        Packet_Context_T * const p_TxBuffer; };
        void * const p_Substate;
    }
*/
typedef const union
{
    Packet_Xfer_T Xfer;
    struct
    {
        Packet_Context_T * p_RxBuffer;
        Packet_Context_T * p_TxBuffer;
        void * p_Substate;
    };
}
Protocol_ReqContext_T;

static_assert(offsetof(Protocol_ReqContext_T, Xfer.p_RxMeta) == offsetof(Protocol_ReqContext_T, p_RxBuffer));
static_assert(offsetof(Protocol_ReqContext_T, Xfer.p_TxMeta) == offsetof(Protocol_ReqContext_T, p_TxBuffer));
static_assert(offsetof(Protocol_ReqContext_T, Xfer.p_Substate) == offsetof(Protocol_ReqContext_T, p_Substate));


// typedef const struct
// {
    //     struct
    //     {
    //         Packet_Context_T * P_RX_BUFFER;
    //         Packet_Context_T * P_TX_BUFFER;
    //         void * p_SUB_STATE;   /* Handler sub-state. Sized for the largest handler */
    //     };
    //     Packet_Xfer_T PACKET_XFER;
    // P_TIMER
    // P_PACKET_LENGTH
// }
// Protocol_Binding_T;

/*
    REQ_CONTEXT.p_RxBuffer and .p_TxBuffer must be distinct - the handler's two payload
    pointers are declared restrict, so one buffer serving both makes every handler call
    undefined.

        assert(p_bindings->p_RxBuffer != p_bindings->p_TxBuffer);
*/
typedef struct Protocol_State
{
    Packet_RxParser_T RxParser;
    Protocol_SyncState_T Sync; /* Sync already holds everything that outlives the [Req] binding */
    Protocol_ReqState_T Req;
    uint32_t LastCompleteTime;
    // Xcvr_T * p_xcvr;
    // Packet_Codec_T * p_codec;

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

static inline void Protocol_ResetReqSync(Protocol_State_T * p_state)
{
    Protocol_ResetReq(&p_state->Req);
    Protocol_ResetSync(&p_state->Sync);
    // p_state->p_ReqFraming = NULL;
    // p_state->TxLength = 0U;
}

/*! Full reset. Drops any frame in progress along with the exchange. Counters survive. */
static inline void Protocol_Reset(Protocol_State_T * p_state)
{
    Packet_ResetRx(&p_state->RxParser);
    Protocol_ResetReqSync(p_state);
}

/*!
    Stamp every deadline base. Not a state reset - Protocol_Reset does that - but the two go
    together wherever the link starts measuring from now: Init, and Enable.
*/
static inline void Protocol_Init(Protocol_State_T * p_state, uint32_t timerNow)
{
    p_state->RxParser.TimeStart = timerNow;
    p_state->Sync.AckTimeStart = timerNow;
    p_state->LastCompleteTime = timerNow;
}

/*!
    Time since the last intact frame was delivered - what link liveness is measured on.
*/
static inline uint32_t Protocol_RxLostTime(const Protocol_State_T * p_state, uint32_t timerNow)
{
    return (timerNow - p_state->LastCompleteTime);
}

/******************************************************************************/
/*!
    Proc Protocol_Base_T buffer context
    optionally move to socket
*/
/******************************************************************************/
/*!
    Bytes into one intact frame, and the frame into Meta.

    No id is resolved here. A control frame carries no handler and needs none, so looking one
    up on every complete frame is what would nack an arriving ACK - the id belongs to the
    request block, which is the only block that needs a handler.
*/
static inline Packet_RxCode_T Protocol_ProcRxFrame(Protocol_State_T * p_state, Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, Packet_Context_T * p_rx, uint32_t timerNow)
{
    /*
        Drain first, age second. Packet_ProcRxTimeout resets the parser when it fires, so
        testing the deadline ahead of the drain discards a frame whose last bytes arrived on
        this very pass - and the parser, already rewound, cannot then reach COMPLETE from
        them. Expiry after draining means the line really did go quiet.
    */
    Packet_RxCode_T rxCode = Protocol_CaptureRx(p_xcvr, p_codec, &p_state->RxParser, p_rx->Packet);
    if (rxCode == PACKET_RX_AWAIT) { rxCode = Packet_ProcRxTimeout(&p_state->RxParser, p_codec, timerNow); }

    // expire stale frame
    // Packet_RxCode_T timeoutStatus = Packet_ProcRxTimeout(&p_state->RxParser, p_codec, timerNow);
    // Packet_RxCode_T parserStatus = Protocol_CaptureRx(p_xcvr, p_codec, &p_state->RxParser, p_rx->Packet);
    // Packet_RxCode_T rxCode = (parserStatus != PACKET_RX_AWAIT) ? parserStatus : timeoutStatus;
    switch (rxCode)
    {
        /* Stores Meta with either Request or Ack format */
        case PACKET_RX_COMPLETE:
            p_state->LastCompleteTime = timerNow;
            Packet_ParseRxFrame(p_codec, &p_rx->Meta, p_rx->Packet);
            p_state->Stat.Frames++;
            break;

            /* Unusable frame. No id was learned, so no handler policy can apply. */
        case PACKET_RX_ERROR_FRAME:     Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK); p_state->Stat.FrameErrors++;    break;
        case PACKET_RX_ERROR_DATA:      Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK); p_state->Stat.DataErrors++;     break;
        case PACKET_RX_ERROR_TIMEOUT:   Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK); p_state->Stat.RxTimeouts++;     break;
            /* Frame still open, deadline intact. */
            /* Checked after draining, so expiry means the line really did go quiet. */
        case PACKET_RX_AWAIT: break;
        default: break;
    }

    return rxCode;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*! true while an exchange occupies the socket, in either layer. */
static inline bool Protocol_IsReqSyncActive(const Protocol_State_T * p_state) { return Protocol_IsReqActive(&p_state->Req) || Protocol_IsAckWaiting(&p_state->Sync); }

/*
    Deliver a frame to the handler.

    Binding precedes the ack because the ack policy is the handler's; invocation follows it
    because "received" and "processed" are different claims.
*/
/*! @return false when the id has no handler - the caller must not go on to dispatch. */
static inline bool Protocol_StartRequest(Protocol_State_T * p_state, Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, Protocol_Req_T * p_table, size_t tableLength, const Packet_Context_T * p_rx)
{
    if (Protocol_CaptureReqOfTable(&p_state->Req, p_table, tableLength, p_rx->Meta.Id) == false)
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
    if (Packet_IsFrameConsistent(Protocol_ReqId(&p_state->Req)->FRAME_FORMAT, &p_rx->Meta, Packet_RxFrameLength(&p_state->RxParser)) == false)
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
/*!
    One dispatch, whichever event reached here. An ack that resumes an exchange arrives with its own Meta
    - the control id, and a Length of 0 - and that is a sufficient signal:
*/
static inline Protocol_ReqCode_T Protocol_ProcRequest(Protocol_State_T * p_state, Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, void * p_appContext, const Protocol_ReqContext_T * p_bindings, uint32_t timerNow)
{
    Protocol_AckPolicy_T policy = Protocol_ReqAckPolicy(&p_state->Req); /* PROTOCOL_REQ_DONE unbinds Req */
    /* The shape the row ANSWERS with - the same lookup Protocol_ProcReqState offsets tx by. */
    Packet_FrameFormat_T * p_respFormat = Protocol_ReqRespId(&p_state->Req)->FRAME_FORMAT;

    /* PROTOCOL_REQ_RESPOND or PROTOCOL_REQ_DONE unbinds the request Id */
    Protocol_ReqCode_T reqCode = Protocol_ProcReqState(&p_state->Req, p_appContext, &p_bindings->Xfer, p_bindings->p_RxBuffer->Packet, p_bindings->p_TxBuffer->Packet);

    /*
        The handler's Length, before the frame builder indexes by it. A constant at most call sites
    */
#if PROTOCOL_CHECK_TX_FRAME_BOUNDS
    if ((reqCode == PROTOCOL_REQ_RESPOND) || (reqCode == PROTOCOL_REQ_DONE))
    {
        if (Packet_IsFrameWithin(p_respFormat, &p_bindings->p_TxBuffer->Meta, p_codec->LENGTH_MAX) == false)
        {
            Protocol_ResetReqSync(p_state);
            return PROTOCOL_REQ_ABORT;      /* dropped, not nacked - a nack would invite the same frame back */
        }
    }
#endif

    switch (reqCode)
    {
        /* Act on the handler's verdict. The only place a request's output reaches the wire. */
        case PROTOCOL_REQ_RESPOND:
        case PROTOCOL_REQ_DONE:
            if (Protocol_TxResponse(p_xcvr, p_codec, p_respFormat, p_bindings->p_TxBuffer))
            {
                if (policy.EXPECT_ACK_RESP) { Protocol_ExpectAck(&p_state->Sync, policy, p_respFormat, timerNow); }
                else { Protocol_ResetSync(&p_state->Sync); }
            }
            else
            {
                /* Sync State is still OPEN */
            }
            break;

        /* Handler judged the frame itself, in place of the arrival reflex. */
        case PROTOCOL_REQ_ACCEPT:   Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK);   break;
        case PROTOCOL_REQ_REJECT:   Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);  break;
        case PROTOCOL_REQ_ABORT:    Protocol_ResetReqSync(p_state); break;
        case PROTOCOL_REQ_AWAIT:
            if (policy.SEND_ACK_REQ) { Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK); }
            break;      /* yields, nothing goes out */
        default: break;
    }

    return reqCode;
}

/*!
    @brief  One pass of the handshake, whichever source had something to say.
*/
// static inline Protocol_SyncEvent_T Protocol_ProcSync(Protocol_State_T * p_state, Packet_Codec_T * p_codec, Packet_RxCode_T rxCode, uint32_t timerNow)
// {
//     if (rxCode == PACKET_RX_COMPLETE) { return Protocol_ProcSyncPacket(&p_state->Sync, Packet_ClassOf(p_codec, p_link->P_RX_PACKET->Meta.Id), timerNow); }
//     if (Protocol_IsReqSyncActive(p_state) == true) { return Protocol_ProcSyncTimeout(&p_state->Sync, p_link->REQ_TIMEOUT, timerNow); }
//     return PROTOCOL_SYNC_EVENT_NONE;
// }

/*
    (PROTOCOL_REQ_STATE_ACTIVE, PROTOCOL_SYNC_AWAIT_ACK) => Await next ACK then returns to PROTOCOL_SYNC_AWAIT_ACK
    (PROTOCOL_REQ_STATE_IDLE, PROTOCOL_SYNC_AWAIT_ACK) => Await final ACK, Req == NULL

    OPEN	    IDLE	Socket idle
    OPEN	    ACTIVE	handler awaiting the next frame. Data-paced receive stream.
    AWAIT_ACK	ACTIVE	Await next ACK then sequence continues. Ack-paced response stream.
    AWAIT_ACK	IDLE	Await final ACK. All stateless with ack
*/
static inline void Protocol_Proc(Protocol_State_T * p_state, Xcvr_T * p_xcvr, Packet_Codec_T * p_codec, Protocol_ReqTable_T * p_reqTable, const Protocol_ReqContext_T * p_bindings, uint32_t timerNow)
{

    /* 1. Rx - bytes into one intact frame, or the frame deadline on a line gone quiet. */
    Packet_RxCode_T rxCode = Protocol_ProcRxFrame(p_state, p_xcvr, p_codec, p_bindings->p_RxBuffer, timerNow);

    Protocol_SyncEvent_T syncEvent;
    /* 2. Sync - one event, from a frame or from the deadline. */
    if (rxCode == PACKET_RX_COMPLETE) { syncEvent = Protocol_ProcSyncPacket(&p_state->Sync, Packet_ClassOf(p_codec, p_bindings->p_RxBuffer->Meta.Id), timerNow); }
    else if (Protocol_IsReqSyncActive(p_state) == true) { syncEvent = Protocol_ProcSyncTimeout(&p_state->Sync, p_reqTable->REQ_TIMEOUT, timerNow); }
    else { syncEvent = PROTOCOL_SYNC_EVENT_NONE; }

    /*
        3. Act. One switch, one effect per event, every event handled. Sync has already moved
        its own state and armed its own deadline, so nothing below re-stamps a clock, and
        nothing re-tests a condition the event itself already asserts.
    */
    switch (syncEvent)
    {
        case PROTOCOL_SYNC_EVENT_REQUEST:
            if (Protocol_StartRequest(p_state, p_xcvr, p_codec, p_reqTable->P_REQ_TABLE, p_reqTable->REQ_TABLE_LENGTH, p_bindings->p_RxBuffer) == false) { break; }
            [[fallthrough]];

        /* An ack passes as payload length 0 */
        case PROTOCOL_SYNC_EVENT_RESUME:
            if (Protocol_IsReqActive(&p_state->Req) == true) { Protocol_ProcRequest(p_state, p_xcvr, p_codec, p_reqTable->P_APP_CONTEXT, p_bindings, timerNow); }
            break;

        /* THE common entry for retransmit. A nack and an expired deadline both arrive here. */
        case PROTOCOL_SYNC_EVENT_RETRANSMIT:
            Protocol_TxResponse(p_xcvr, p_codec, Protocol_SyncRespFormat(&p_state->Sync), p_bindings->p_TxBuffer);
            p_state->Stat.Retransmits++;
            break;

        /* Acknowledging a received abort, rather than raising a second one. */
        case PROTOCOL_SYNC_EVENT_ABORT:
            if (Protocol_ReqAckPolicy(&p_state->Req).SEND_ACK_ABORT == true) { Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_ACK); }
            Protocol_ResetReq(&p_state->Req);
            break;

        case PROTOCOL_SYNC_EVENT_REJECT:
            Protocol_TxControl(p_xcvr, p_codec, PACKET_CLASS_NACK);
            break;

        case PROTOCOL_SYNC_EVENT_FAILED:
            Protocol_ResetReq(&p_state->Req);
            p_state->Stat.ReqTimeouts++;
            break;

        case PROTOCOL_SYNC_EVENT_NONE:
        default:
            break;
    }

}
