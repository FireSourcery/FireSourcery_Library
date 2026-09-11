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

    Two deadlines, deliberately distinct:
        RxTimeStart     a frame stalled part way. Resyncs the parser.
        ReqTimeStart    an exchange stalled. Retransmits or abandons via ARQ.
*/
/******************************************************************************/
/*
    Everything constant for the duration of a pass, except the two things the socket selects
    at runtime. The Xcvr and the format travel as arguments precisely because they change:
    holding them here would mean rebuilding the whole struct to swap a port.
*/
typedef const struct Protocol_Link
{
    Packet_Context_T * P_RX_PACKET;         /* Meta + contiguous frame buffer */
    Packet_Context_T * P_TX_PACKET;
    uint8_t PACKET_BUFFER_LENGTH;           /* Must be >= FORMAT->LENGTH_MAX */

    /* The request service. Id -> handler, plus the storage handlers run against. */
    const Protocol_Req_T * P_REQ_TABLE;
    uint8_t REQ_TABLE_LENGTH;
    void * P_APP_CONTEXT;                   /* Passed to every handler */
    void * P_REQ_CONTEXT;                   /* Handler sub-state. Sized for the largest handler */

    const volatile uint32_t * P_TIMER;
    const uint32_t RX_TIMEOUT;              /* Frame deadline */
    const uint32_t REQ_TIMEOUT;             /* Exchange deadline */
}
Protocol_Base_T;

typedef struct Protocol_State
{
    Packet_RxParser_T RxParser;
    Protocol_SyncState_T Sync;
    Protocol_ReqState_T Req;

    // packet_size_t TxLength;     /* Staged response, retained for retransmission */
    uint32_t RxTimeStart;       /* Frame deadline base */
    uint32_t ReqTimeStart;      /* Exchange deadline base */

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
        uint16_t ReqTimeouts;   /* Exchanges abandoned */
        uint16_t Retransmits;
        uint16_t NoHandler;     /* Intact frames whose id has no table entry */
    }
    Stat;
}
Protocol_State_T;




static inline bool _Protocol_IsElapsed(uint32_t timerNow, uint32_t timeStart, uint32_t timeout) { return ((timerNow - timeStart) > timeout); }

/*! true while an exchange occupies the socket, in either layer. */
static inline bool Protocol_IsReqSyncActive(const Protocol_State_T * p_state) { return Protocol_IsReqActive(&p_state->Req) || Protocol_IsSyncWaiting(&p_state->Sync); }

static inline void Protocol_ResetReqSync(Protocol_State_T * p_state)
{
    Protocol_ResetReq(&p_state->Req);
    Protocol_ResetSync(&p_state->Sync);
    // p_state->TxLength = 0U;
}

/*! Full reset. Drops any frame in progress along with the exchange. Counters survive. */
static inline void Protocol_Reset(Protocol_State_T * p_state, uint32_t timerNow)
{
    Packet_ResetRx(&p_state->RxParser);
    Protocol_ResetReqSync(p_state);
    p_state->RxTimeStart = timerNow;
    p_state->ReqTimeStart = timerNow;
}


/******************************************************************************/
/*!
    Tx - every byte out of this engine passes through here
*/
/******************************************************************************/
static bool Protocol_TxResponse(const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_Context_T * p_tx)
{
    packet_size_t frameLength = Packet_BuildTxHeader(p_format, &p_tx->Meta, p_tx->Packet);
    return Xcvr_TxN(p_xcvr, p_tx->Packet, frameLength);
}

/*!
    Transmit a control frame. A control frame is an ordinary frame with a control id and no
    payload - there is no separate builder, which is what the declared ACK_ID / NACK_ID /
    ABORT_ID buy over a BUILD_TX_SYNC callback.

    Built into a local frame so the staged response survives for retransmission.
*/
static void Protocol_TxControl(const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_ClassId_T txClass)
{
    uint8_t frame[PACKET_CONTROL_LENGTH_MAX];
    Packet_Meta_T meta = { .Id = Packet_ControlIdOf(p_format, txClass), .Length = 0U };
    packet_size_t frameLength = Packet_BuildTxHeader(p_format, &meta, frame);

    // assert(frameLength <= PACKET_CONTROL_LENGTH_MAX);

    Xcvr_TxN(p_xcvr, frame, frameLength);
}

/******************************************************************************/
/*!
    Rx - drive the sans-IO parser from the Xcvr
*/
/******************************************************************************/
/*
    Drain the Xcvr into the frame buffer. The parser only ever asks for what the current
    frame still needs, so a burst is consumed without reading into the following frame.

    Terminates: only START repeats on AWAIT, and each repeat consumes a byte.
*/
/*
    ProcRxParser
*/
static inline Packet_RxCode_T Protocol_CaptureRx(const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_RxParser_T * p_parser, uint8_t * p_rxBuffer)
{
    Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

    while (rxCode == PACKET_RX_AWAIT)
    {
        rxCode = Packet_ProcRxParser(p_parser, p_format, p_rxBuffer);

        packet_size_t remaining = Packet_RxRemaining(p_parser);

        /* Nothing moves until the whole target is available. */
        if ((remaining > 0U) && (Xcvr_RxN(p_xcvr, &p_rxBuffer[p_parser->Index], remaining) == false)) { break; }
        p_parser->Index += remaining;
    }

    return rxCode;
}

/*
    The frame deadline. The parser holds no clock, so a frame that stalls part way is
    abandoned here. Checked after draining, so expiry means the line really did go quiet.

    No edge to detect: the base slides forward on every pass the parser is idle, so it stops
    sliding the moment a frame opens and the deadline measures from there. Costs a store per
    idle pass and removes the "was in frame" snapshot the caller used to have to carry.
*/
static inline bool Protocol_ProcRxDeadline(Protocol_State_T * p_state, uint32_t rxTimeout, uint32_t timerNow)
{
    // if (Packet_IsRxWaiting(&p_state->RxParser) == false) { p_state->RxTimeStart = timerNow; return false; }
    // if (_Protocol_IsElapsed(timerNow, p_state->RxTimeStart, rxTimeout) == false) { return false; }

    // Packet_ResetRx(&p_state->RxParser);
    // return true;
    if (Packet_IsRxWaiting(&p_state->RxParser) && _Protocol_IsElapsed(timerNow, p_state->RxTimeStart, rxTimeout))
    {
        Packet_ResetRx(&p_state->RxParser);
        return true;
    }
    return false;
}

/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*
    Deliver a frame to the handler.

    Binding precedes the ack because the ack policy is the handler's; invocation follows it
    because "received" and "processed" are different claims.
*/
static inline void Protocol_StartRequest(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format)
{
    if (Protocol_CaptureReq(&p_state->Req, p_link->P_REQ_TABLE, p_link->REQ_TABLE_LENGTH, p_link->P_RX_PACKET->Meta.Id) == false)
    {
        Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);     /* no handler for this id */
        p_state->Stat.NoHandler++;
        return;
    }

    Protocol_AckPolicy_T policy = Protocol_ReqAckPolicy(&p_state->Req);

    /* Receiver-side ack: a reflex on arrival, not a state. */
    if (policy.ACK_REQ) { Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_ACK); }
}

static inline void Protocol_ProcRequest(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_FrameFormat_T * p_framing)
{
    if (p_framing == NULL) { return; }

    Protocol_AckPolicy_T policy = Protocol_ReqAckPolicy(&p_state->Req);

    Packet_Xfer_T xfer =
    {
        .p_RxMeta   = &p_link->P_RX_PACKET->Meta,
        .p_TxMeta   = &p_link->P_TX_PACKET->Meta,
        .p_Substate = p_link->P_REQ_CONTEXT,
        .p_Step     = &p_state->Req.Step,
    };

    switch (Protocol_ProcReq(&p_state->Req, p_link->P_APP_CONTEXT, &xfer, &p_link->P_RX_PACKET->Packet[p_framing->HEADER_LENGTH], &p_link->P_TX_PACKET->Packet[p_framing->HEADER_LENGTH]))
    {
        /* Act on the handler's verdict. The only place a request's output reaches the wire. */
        case PROTOCOL_REQ_RESPOND:
        case PROTOCOL_REQ_DONE:
            if (Protocol_TxResponse(p_xcvr, p_format, p_link->P_TX_PACKET))
            {
                if (policy.EXPECT_ACK_RESP) { Protocol_ExpectAck(&p_state->Sync); }
                else { Protocol_ResetSync(&p_state->Sync); }
            }
            break;

        /* Handler judged the frame itself, in place of the arrival reflex. */
        case PROTOCOL_REQ_ACCEPT:   Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_ACK);   break;
        case PROTOCOL_REQ_REJECT:   Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);  break;
        case PROTOCOL_REQ_ABORT:    Protocol_ResetReqSync(p_state); break;

        case PROTOCOL_REQ_AWAIT:    break;      /* yields, nothing goes out */
        default: break;
    }
}

static inline void Protocol_ProcRequestTimeout(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format)
{
    uint32_t timerNow = *p_link->P_TIMER;
    if (Protocol_IsReqSyncActive(p_state) && _Protocol_IsElapsed(timerNow, p_state->ReqTimeStart, p_link->REQ_TIMEOUT))
    {
        switch (Protocol_ResolveSyncRxTimeout(&p_state->Sync, Protocol_ReqAckPolicy(&p_state->Req)))
        {
            case PROTOCOL_SYNC_EVENT_RETRANSMIT:
                Protocol_TxResponse(p_xcvr, p_format, p_link->P_TX_PACKET);
                p_state->ReqTimeStart = timerNow;
                p_state->Stat.Retransmits++;
                break;

            default:
                Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
                Protocol_ResetReqSync(p_state);
                p_state->Stat.ReqTimeouts++;
                break;
        }
    }
}

/*
    One resolved frame, through the handshake and into the handler.
*/
static inline Protocol_SyncEvent_T Protocol_ProcFrame(const Protocol_Base_T * p_link, Protocol_State_T * p_state, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format)
{
    Protocol_SyncEvent_T syncEvent = Protocol_ProcSyncRx(&p_state->Sync, Protocol_ReqAckPolicy(&p_state->Req), Packet_ClassOf(p_format, p_link->P_RX_PACKET->Meta.Id));
    uint32_t timerNow = *p_link->P_TIMER;

    /* cross product of sync state and  */
    switch (syncEvent)
    {
        case PROTOCOL_SYNC_EVENT_REQUEST:
            p_state->ReqTimeStart = timerNow;
            // Protocol_ProcRequest(p_link, p_state, p_xcvr, p_format);
            break;

        /* Our frame landed. Resume a sequence, or close a stateless exchange. */
        case PROTOCOL_SYNC_EVENT_RESUME:
            p_state->ReqTimeStart = timerNow;
            // if (Protocol_IsReqActive(&p_state->Req) == true)
            // {
            //     Protocol_ProcRequest(p_link, p_state, p_xcvr, p_format);
            // }
            break;

        case PROTOCOL_SYNC_EVENT_RETRANSMIT:
            Protocol_TxResponse(p_xcvr, p_format, p_link->P_TX_PACKET);
            p_state->ReqTimeStart = timerNow;
            p_state->Stat.Retransmits++;
            break;

        case PROTOCOL_SYNC_EVENT_ABORT:
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_ABORT);
            Protocol_ResetReqSync(p_state);
            break;

        case PROTOCOL_SYNC_EVENT_REJECT:
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            break;

        case PROTOCOL_SYNC_EVENT_FAILED:
            Protocol_ResetReqSync(p_state);
            break;

        case PROTOCOL_SYNC_EVENT_NONE:
            break;

        default:
            break;
    }

    return syncEvent;
}

/*!
    @brief  One non-blocking pass. Single threaded.

    @param  p_xcvr   the selected port
    @param  p_format the selected framing. Both travel as arguments rather than in p_link
                     because the socket may swap either between passes.
*/
static inline void Protocol_Proc(const Protocol_Base_T * p_link, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Protocol_State_T * p_state)
{
    uint32_t timerNow = *p_link->P_TIMER;
    Protocol_SyncEvent_T syncEvent = PROTOCOL_SYNC_EVENT_NONE;
    Packet_FrameFormat_T * p_framing = NULL;

    /* 1. Transport + framing */
    switch (Protocol_CaptureRx(p_xcvr, p_format, &p_state->RxParser, p_link->P_RX_PACKET->Packet))
    {
        /* 2. Classify, handshake, dispatch */
        case PACKET_RX_COMPLETE:
            p_framing = Packet_ParseRxHeader(p_format, &p_link->P_RX_PACKET->Meta, p_link->P_RX_PACKET->Packet);
            syncEvent = Protocol_ProcFrame(p_link, p_state, p_xcvr, p_format);
            p_state->Stat.Frames++;
            break;

            /* 3. Unusable frame. No id was learned, so no handler policy can apply. */
        case PACKET_RX_ERROR_FRAME:
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            p_state->Stat.FrameErrors++;
            break;

        case PACKET_RX_ERROR_DATA:
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            p_state->Stat.DataErrors++;
            break;

        case PACKET_RX_AWAIT:
            if (Protocol_ProcRxDeadline(p_state, p_link->RX_TIMEOUT, timerNow) == true)
            {
                p_state->Stat.RxTimeouts++;
                Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            }
            break;

        default:
            break;
    }

    /* Protocol_IsReqActive checked on Proc */
    if (syncEvent == PROTOCOL_SYNC_EVENT_REQUEST) { Protocol_StartRequest(p_link, p_state, p_xcvr, p_format); }
    if (syncEvent == PROTOCOL_SYNC_EVENT_REQUEST || syncEvent == PROTOCOL_SYNC_EVENT_RESUME) { Protocol_ProcRequest(p_link, p_state, p_xcvr, p_format, p_framing); }

    /* 4. Exchange deadline, spanning Sync and Request both */
    Protocol_ProcRequestTimeout(p_link, p_state, p_xcvr, p_format);
}
