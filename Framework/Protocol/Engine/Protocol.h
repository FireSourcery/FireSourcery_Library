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
Protocol_Link_T;

typedef struct Protocol_State
{
    Packet_RxParser_T RxParser;
    Protocol_SyncState_T Sync;
    Protocol_ReqState_T Req;

    packet_size_t TxLength;     /* Staged response, retained for retransmission */
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




static inline bool Protocol_IsElapsed(uint32_t timerNow, uint32_t timeStart, uint32_t timeout) { return ((timerNow - timeStart) > timeout); }

/*! true while an exchange occupies the socket, in either layer. */
static inline bool Protocol_IsInFlight(const Protocol_State_T * p_state)
{
    return Protocol_Req_IsActive(&p_state->Req) || Protocol_Sync_IsAwaitingAck(&p_state->Sync);
}

static inline void Protocol_Unwind(Protocol_State_T * p_state)
{
    Protocol_Req_Reset(&p_state->Req);
    Protocol_Sync_Reset(&p_state->Sync);
    p_state->TxLength = 0U;
}

/*! Full reset. Drops any frame in progress along with the exchange. Counters survive. */
static inline void Protocol_Reset(Protocol_State_T * p_state, uint32_t timerNow)
{
    Packet_RxParser_Reset(&p_state->RxParser);
    Protocol_Unwind(p_state);
    p_state->RxTimeStart = timerNow;
    p_state->ReqTimeStart = timerNow;
}

/*
    The frame deadline. The parser holds no clock, so a frame that stalls part way is
    abandoned here. Checked after draining, so expiry means the line really did go quiet.

    No edge to detect: the base slides forward on every pass the parser is idle, so it stops
    sliding the moment a frame opens and the deadline measures from there. Costs a store per
    idle pass and removes the "was in frame" snapshot the caller used to have to carry.
*/
static inline bool Protocol_ProcRxDeadline(Protocol_State_T * p_state, uint32_t timerNow, uint32_t rxTimeout)
{
    if (Packet_RxParser_IsInFrame(&p_state->RxParser) == false) { p_state->RxTimeStart = timerNow; return false; }
    if (Protocol_IsElapsed(timerNow, p_state->RxTimeStart, rxTimeout) == false) { return false; }

    Packet_RxParser_Reset(&p_state->RxParser);
    return true;
}

/******************************************************************************/
/*!
    Tx - every byte out of this engine passes through here
*/
/******************************************************************************/
/*! Transmit the staged response. @return false when the handler produced none. */
static inline bool Protocol_TxResponse(const Xcvr_T * p_xcvr, const uint8_t * p_tx, packet_size_t txLength)
{
    return (txLength > 0U) ? Xcvr_TxN(p_xcvr, p_tx, txLength) : false;
}

/*!
    Transmit a control frame. A control frame is an ordinary frame with a control id and no
    payload - there is no separate builder, which is what the declared ACK_ID / NACK_ID /
    ABORT_ID buy over a BUILD_TX_SYNC callback.

    Built into a local frame so the staged response survives for retransmission.
*/
static inline void Protocol_TxControl(const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_ClassId_T txClass)
{
    uint8_t frame[PACKET_CONTROL_LENGTH_MAX];
    Packet_Meta_T meta = { .Id = Packet_ControlIdOf(p_format, txClass), .Length = 0U };

    p_format->BUILD_TX_HEADER(&meta, frame, 0U);
    Xcvr_TxN(p_xcvr, frame, p_format->LENGTH_MIN);
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
static inline Packet_RxCode_T Protocol_CaptureRx(const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Packet_RxParser_T * p_parser, uint8_t * p_rxBuffer)
{
    Packet_RxCode_T rxCode = PACKET_RX_AWAIT;

    while (rxCode == PACKET_RX_AWAIT)
    {
        packet_size_t remaining = Packet_RxRemaining(p_parser);

        /* Nothing moves until the whole target is available. */
        if ((remaining > 0U) && (Xcvr_RxN(p_xcvr, &p_rxBuffer[p_parser->Index], remaining) == false)) { break; }
        p_parser->Index += remaining;

        /* Reached with remaining == 0 when a frame ends exactly on a phase boundary. */
        rxCode = Packet_ProcRxParser(p_parser, p_format, p_rxBuffer);
    }

    return rxCode;
}



/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*
    Act on the handler's verdict. The only place a request's output reaches the wire.
*/
/*!
    @return true when a response reached the wire, so the caller can arm the ack wait.

            Arming is the caller's because only the caller still knows whether this was the
            opening exchange or a continuation, and DONE has already returned the request
            state to IDLE by the time this runs.
*/
static inline bool Protocol_ProcReqCode
(
    const Protocol_Link_T * p_link, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format,
    Protocol_State_T * p_state, Protocol_ReqCode_T reqCode
)
{
    bool isTransmitted = false;

    switch (reqCode)
    {
        case PROTOCOL_REQ_RESPOND:
        case PROTOCOL_REQ_DONE:
            /* Build the header the handler described. */
            p_state->TxLength = p_link->P_TX_PACKET->Meta.Length;
            p_format->BUILD_TX_HEADER(&p_link->P_TX_PACKET->Meta, p_link->P_TX_PACKET->Packet, p_state->TxLength);
            isTransmitted = Protocol_TxResponse(p_xcvr, p_link->P_TX_PACKET->Packet, p_state->TxLength);
            break;

        /* Handler judged the frame itself, in place of the arrival reflex. */
        case PROTOCOL_REQ_ACCEPT:   Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_ACK);   break;
        case PROTOCOL_REQ_REJECT:   Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);  break;

        case PROTOCOL_REQ_AWAIT:    break;      /* yields, nothing goes out */
        case PROTOCOL_REQ_ABORT:    Protocol_Unwind(p_state); break;
        default: break;
    }

    return isTransmitted;
}

/*
    Deliver a frame to the handler.

    Binding precedes the ack because the ack policy is the handler's; invocation follows it
    because "received" and "processed" are different claims.
*/
static inline void Protocol_ProcRequest(const Protocol_Link_T * p_link, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Protocol_State_T * p_state)
{
    /*
        Read once, before Select or Proc can move it: IDLE here means this frame opens the
        exchange. It stays a local - the OPEN / STEP choice is made twice below and never
        leaves this function, because after Proc the request state no longer answers it.
    */
    bool isStep = Protocol_Req_IsActive(&p_state->Req);
    Protocol_AckPolicy_T policy;
    Protocol_ReqCode_T reqCode;

    if (isStep == false)
    {
        if (Protocol_Req_Select(&p_state->Req, p_link->P_REQ_TABLE, p_link->REQ_TABLE_LENGTH, p_state->RxParser.Id) == false)
        {
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);     /* no handler for this id */
            p_state->Stat.NoHandler++;
            return;
        }
        /* Nothing staged yet for this exchange. A retransmit before the first response must send nothing. */
        p_state->TxLength = 0U;
        p_link->P_TX_PACKET->Meta.Length = 0U;
    }

    /* Captured while the request is still bound - the handler may close it. */
    policy = Protocol_Req_AckPolicy(&p_state->Req);

    /* Receiver-side ack: a reflex on arrival, not a state. */
    if (((isStep == true) ? policy.TX_ACK_STEP : policy.TX_ACK_OPEN) != 0U)
    {
        Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_ACK);
    }

    p_format->PARSE_RX_HEADER(&p_link->P_RX_PACKET->Meta, p_link->P_RX_PACKET->Packet, p_state->RxParser.Length);
    p_state->ReqTimeStart = *p_link->P_TIMER;

    Packet_Xfer_T xfer =
    {
        .p_RxMeta   = &p_link->P_RX_PACKET->Meta,
        .p_TxMeta   = &p_link->P_TX_PACKET->Meta,
        .p_Substate = p_link->P_REQ_CONTEXT,
        .p_Step     = &p_state->Req.Step,
    };

    reqCode = Protocol_Req_Proc(&p_state->Req, p_link->P_APP_CONTEXT, &xfer,
                                p_link->P_RX_PACKET->Packet, p_link->P_TX_PACKET->Packet);

    /*
        Arm the handshake here, where the bound policy and the opening / continuation choice
        both still exist. Reset rather than leave it: a response that expects no ack ends the
        outstanding frame, and that is what clears the retry budget.
    */
    if (Protocol_ProcReqCode(p_link, p_xcvr, p_format, p_state, reqCode) == true)
    {
        if (((isStep == true) ? policy.RX_ACK_STEP : policy.RX_ACK_OPEN) != 0U) { Protocol_ExpectAck(&p_state->Sync); }
        else                                                                    { Protocol_Sync_Reset(&p_state->Sync); }
    }
}

/*
    One resolved frame, through the handshake and into the handler.
*/
static inline void Protocol_ProcFrame(const Protocol_Link_T * p_link, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Protocol_State_T * p_state)
{
    Packet_ClassId_T rxClass = Packet_ClassOf(p_format, p_state->RxParser.Id);
    Protocol_AckPolicy_T policy = Protocol_Req_AckPolicy(&p_state->Req);

    uint32_t timerNow = *p_link->P_TIMER;

    switch (Protocol_Sync_OnRx(&p_state->Sync, policy, rxClass))
    {
        case PROTOCOL_SYNC_EVENT_REQUEST:
            Protocol_ProcRequest(p_link, p_xcvr, p_format, p_state);
            break;

        /* Our frame landed. Resume a sequence, or close a stateless exchange. */
        case PROTOCOL_SYNC_EVENT_RESUME:
            p_state->ReqTimeStart = timerNow;
            if (Protocol_Req_IsActive(&p_state->Req) == true)
            {
                Protocol_ProcRequest(p_link, p_xcvr, p_format, p_state);
            }
            break;

        case PROTOCOL_SYNC_EVENT_RETRANSMIT:
            Protocol_TxResponse(p_xcvr, p_link->P_TX_PACKET->Packet, p_state->TxLength);
            p_state->ReqTimeStart = timerNow;
            p_state->Stat.Retransmits++;
            break;

        case PROTOCOL_SYNC_EVENT_ABORT:
            if (policy.TX_ACK_ABORT != 0U) { Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_ABORT); }
            Protocol_Unwind(p_state);
            break;

        case PROTOCOL_SYNC_EVENT_REJECT:
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            break;

        case PROTOCOL_SYNC_EVENT_FAILED:
            Protocol_Unwind(p_state);
            break;

        default:
            break;
    }
}

/*!
    @brief  One non-blocking pass. Single threaded.

    @param  p_xcvr   the selected port
    @param  p_format the selected framing. Both travel as arguments rather than in p_link
                     because the socket may swap either between passes.
*/
static inline void Protocol_Proc(const Protocol_Link_T * p_link, const Xcvr_T * p_xcvr, const Packet_Format_T * p_format, Protocol_State_T * p_state)
{
    uint32_t timerNow = *p_link->P_TIMER;

    /* 1. Transport + framing */
    switch (Protocol_CaptureRx(p_xcvr, p_format, &p_state->RxParser, p_link->P_RX_PACKET->Packet))
    {
        /* 2. Classify, handshake, dispatch */
        case PACKET_RX_COMPLETE:
            p_state->Stat.Frames++;
            Protocol_ProcFrame(p_link, p_xcvr, p_format, p_state);
            break;

            /* 3. Unusable frame. No id was learned, so no handler policy can apply. */
        case PACKET_RX_ERROR_FRAME:
            p_state->Stat.FrameErrors++;
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            break;

        case PACKET_RX_ERROR_DATA:
            p_state->Stat.DataErrors++;
            Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            break;

        case PACKET_RX_AWAIT:
            if (Protocol_ProcRxDeadline(p_state, timerNow, p_link->RX_TIMEOUT) == true)
            {
                p_state->Stat.RxTimeouts++;
                Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
            }
            break;

        default:
            break;
    }

    /* 4. Exchange deadline, spanning Sync and Request both */
    if (Protocol_IsInFlight(p_state) && Protocol_IsElapsed(timerNow, p_state->ReqTimeStart, p_link->REQ_TIMEOUT))
    {
        switch (Protocol_Sync_OnTimeout(&p_state->Sync, Protocol_Req_AckPolicy(&p_state->Req)))
        {
            case PROTOCOL_SYNC_EVENT_RETRANSMIT:
                Protocol_TxResponse(p_xcvr, p_link->P_TX_PACKET->Packet, p_state->TxLength);
                p_state->ReqTimeStart = timerNow;
                p_state->Stat.Retransmits++;
                break;

            default:
                p_state->Stat.ReqTimeouts++;
                Protocol_TxControl(p_xcvr, p_format, PACKET_CLASS_NACK);
                Protocol_Unwind(p_state);
                break;
        }
    }
}
