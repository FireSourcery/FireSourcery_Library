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
    @file   Protocol_Sync.h
    @author FireSourcery
    @brief  Stop-and-wait ARQ, sender side. One frame outstanding at a time.
*/
/******************************************************************************/
#include "Packet.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    This is stop-and-wait ARQ with a window of 1, named so the shape can be checked against
    the published protocol rather than against taste.

        OPEN        nothing outstanding. An arriving data frame is deliverable.
        AWAIT_ACK   one frame transmitted, unacknowledged.

    Sender side only. Acking a frame we RECEIVED is not a state machine - it is a reflex on
    arrival, and its policy lives with the bound handler, so it belongs to Protocol_Request.
    Splitting the two is what lets this layer stay two states.

    Sans-I/O, like the parser: this layer never touches the Xcvr. A retransmit is reported as
    an event for the caller to perform, not performed here. The whole layer is a pure function
    of (state, class), which is what makes it checkable in isolation.

    Two event sources, one event stream. A frame arrives, or the deadline expires; both fold
    through the same evaluate-then-resolve pair and yield a Protocol_SyncEvent_T, so the
    composition acts on one enum in one switch and RETRANSMIT exists at exactly one call site.

    The clock is an input, like a byte to the parser - this layer is handed a reading, never
    reads one. What it does own is the arming, because the base is only meaningful against
    its own transitions. See the note above Protocol_ProcSyncTimeout for why absolute.

    NOT YET IMPLEMENTED - the alternating bit. See the note at the foot of this file.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Ack Policy - configured per handler.

    One pair of bits, applied to every frame of the exchange alike. The opening request and
    its continuation steps are no longer configured separately: Protocol_CaptureReq returns
    early once ACTIVE, so the composition has no isStep to select on. PROTOCOL_ACK_EVERY_STEP
    is therefore an alias of PROTOCOL_ACK_ON_REQ, kept only so call sites read as intended.
    Restoring the distinction means restoring the OPEN / STEP bit pairs below and the isStep
    local in Protocol_ProcRequest.
*/
/******************************************************************************/
typedef struct Protocol_AckPolicy
{
    uint8_t SEND_ACK_REQ     : 1U;   /* Ack the request */
    uint8_t EXPECT_ACK_RESP  : 1U;   /* Await an ack after response */
    // uint8_t TX_ACK_OPEN     : 1U;   /* On Entry. Ack the request packet on arrival */
    // uint8_t RX_ACK_OPEN     : 1U;   /* On Entry. Await an ack after the opening response */
    // uint8_t TX_ACK_STEP     : 1U;   /* Ack each continuation packet on arrival */
    // uint8_t RX_ACK_STEP     : 1U;   /* Await an ack after each continuation response */
    uint8_t SEND_ACK_ABORT   : 1U;   /* Acknowledge a received abort */
    uint8_t RETRANSMIT_MAX   : 3U;   /* Retries before the exchange is abandoned. 0 = no retry */
}
Protocol_AckPolicy_T;

#define PROTOCOL_ACK_NONE       { 0U }
#define PROTOCOL_ACK_ON_REQ     { .SEND_ACK_REQ = 1U, .EXPECT_ACK_RESP = 1U, .RETRANSMIT_MAX = 3U }
#define PROTOCOL_ACK_EVERY_STEP { .SEND_ACK_REQ = 1U, .EXPECT_ACK_RESP = 1U, .RETRANSMIT_MAX = 3U }

/*!
    product of rx char, sync state, request state,
    What the handshake yields to the layer above.
*/
typedef enum Protocol_SyncEvent
{
    PROTOCOL_SYNC_EVENT_NONE,           /* Nothing to act on this pass */
    PROTOCOL_SYNC_EVENT_REQUEST,        /* Data frame accepted - deliver it to the handler */
    PROTOCOL_SYNC_EVENT_RESUME,         /* Our frame was acked - the sequence may continue */
    PROTOCOL_SYNC_EVENT_RETRANSMIT,     /* Caller re-sends the staged response */
    PROTOCOL_SYNC_EVENT_ABORT,          /* Remote terminated the exchange */
    PROTOCOL_SYNC_EVENT_REJECT,         /* Out of sequence. Caller nacks. */
    PROTOCOL_SYNC_EVENT_FAILED,         /* Retransmit limit reached, or deadline expired */
}
Protocol_SyncEvent_T;

/******************************************************************************/
/*!
    Sync State
*/
/******************************************************************************/
typedef enum Protocol_SyncStateId
{
    // PROTOCOL_SYNC_DISABLED, /* Act is module disable */
    PROTOCOL_SYNC_OPEN,
    PROTOCOL_SYNC_AWAIT_ACK,
}
Protocol_SyncStateId_T;

typedef struct Protocol_SyncState
{
    Protocol_SyncStateId_T StateId;
    uint8_t RetryCount;    /* Retries spent on the outstanding frame */
    /*
        Latched from the bound [ReqId] policy when the frame goes out
        A handler returning DONE unbinds the active [ReqId]
        retransmission after DONE rely on this latched value.
    */
    uint8_t RetryMax;
    Packet_FrameFormat_T * p_RetryFormat;
    uint32_t AckTimeStart; /* Exchange deadline base. the exchange outlives the request binding.*/

    // Protocol_AckPolicy_T AckPolicy; //alternatively latched policy handle internally

    // uint32_t Timeout;
}
Protocol_SyncState_T;

/*!
    Pure. Is a retransmission offerable at all.

    Budget AND a latched shape, because the two can part company: a stateless handler that
    returned DONE is unbound while its response is still outstanding, and the shape is what
    survives that. Asking here is what lets the caller act on RETRANSMIT without re-testing
    anything - the event is not raised unless it can be carried out.
*/
static inline bool _Protocol_CanRetry(const Protocol_SyncState_T * p_state)
{
    return ((p_state->RetryCount < p_state->RetryMax) && (p_state->p_RetryFormat != NULL));
}

/*! Pure. What an arriving frame means, given what is outstanding. No writes. */
static inline Protocol_SyncEvent_T _Protocol_EvaluatePacket(const Protocol_SyncState_T * p_state, Packet_ClassId_T rxClass)
{
    if (rxClass == PACKET_CLASS_ABORT) { return PROTOCOL_SYNC_EVENT_ABORT; }

    switch (p_state->StateId)
    {
        case PROTOCOL_SYNC_OPEN:
            /* An ack or nack refers to nothing and is dropped - answering it is the storm path. */
            return (rxClass == PACKET_CLASS_DATA) ? PROTOCOL_SYNC_EVENT_REQUEST : PROTOCOL_SYNC_EVENT_NONE;

        case PROTOCOL_SYNC_AWAIT_ACK:
            switch (rxClass)
            {
                case PACKET_CLASS_ACK:  return PROTOCOL_SYNC_EVENT_RESUME;
                case PACKET_CLASS_NACK: return _Protocol_CanRetry(p_state) ? PROTOCOL_SYNC_EVENT_RETRANSMIT : PROTOCOL_SYNC_EVENT_FAILED;
                default:                return PROTOCOL_SYNC_EVENT_REJECT;   /* DATA before the ack: out of sequence */
            }

        default: return PROTOCOL_SYNC_EVENT_NONE;
    }
}

/*! Pure. What the deadline means. Same choice as a nack, different trigger. */
static inline Protocol_SyncEvent_T _Protocol_EvaluateTimeout(const Protocol_SyncState_T * p_state)
{
    if (p_state->StateId != PROTOCOL_SYNC_AWAIT_ACK) { return PROTOCOL_SYNC_EVENT_FAILED; }
    return _Protocol_CanRetry(p_state) ? PROTOCOL_SYNC_EVENT_RETRANSMIT : PROTOCOL_SYNC_EVENT_FAILED;
}


/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*!
    Takes the policy and the shape because this is the moment both must be captured - see
    RetryMax and p_RetryFormat. Everything a retransmission needs is latched here, so the
    retransmit paths never consult the request binding.
*/
static inline void Protocol_ExpectAck(Protocol_SyncState_T * p_state, Protocol_AckPolicy_T policy, Packet_FrameFormat_T * p_retryFormat, uint32_t timerNow)
{
    p_state->StateId       = PROTOCOL_SYNC_AWAIT_ACK;
    p_state->RetryCount    = 0U;
    p_state->RetryMax      = policy.RETRANSMIT_MAX;
    p_state->p_RetryFormat = p_retryFormat;
    p_state->AckTimeStart  = timerNow;   /* OPEN -> AWAIT_ACK is where the ack deadline begins */
}

/*
    After a response is transmitted the caller picks the destination directly:

        Protocol_ExpectAck      one frame outstanding, retry budget intact
        Protocol_ResetSync      nothing outstanding, budget cleared

    Which one is a reading of the bound handler's RX_ACK bit, and only the caller knows
    whether the OPEN or the STEP bit applies.
*/
static inline void Protocol_ResetSync(Protocol_SyncState_T * p_state)
{
    p_state->StateId = PROTOCOL_SYNC_OPEN;
    p_state->RetryCount = 0U;
    p_state->RetryMax = 0U;
    p_state->p_RetryFormat = NULL;
}

/******************************************************************************/
/*!
    Exchange deadline - absolute, unlike the parser's

    The parser ages a duration: firing late costs only a held buffer
*/
/******************************************************************************/
/*!
    The one place sync state moves - the counterpart of _Packet_ResolveRx.

    Re-arming is not uniform, and the watchdog is why: an advance says the peer is alive and
    feeds it; a failure must leave the base where it was so silence stays visible.
*/
static inline Protocol_SyncEvent_T _Protocol_ResolveSync(Protocol_SyncState_T * p_state, Protocol_SyncEvent_T event, uint32_t timerNow)
{
    switch (event)
    {
        case PROTOCOL_SYNC_EVENT_REQUEST:    p_state->AckTimeStart = timerNow;                              break;
        /* Received Nack */
        case PROTOCOL_SYNC_EVENT_RETRANSMIT: p_state->RetryCount++; p_state->AckTimeStart = timerNow;       break;
        /* Received Ack */
        case PROTOCOL_SYNC_EVENT_RESUME:     Protocol_ResetSync(p_state); p_state->AckTimeStart = timerNow; break;
        case PROTOCOL_SYNC_EVENT_ABORT:      Protocol_ResetSync(p_state); p_state->AckTimeStart = timerNow; break;
        /* exceeded [RetryCount] */
        case PROTOCOL_SYNC_EVENT_FAILED:     Protocol_ResetSync(p_state);                                   break;
        case PROTOCOL_SYNC_EVENT_REJECT:
        case PROTOCOL_SYNC_EVENT_NONE:
        default:                                                                                            break;
    }
    return event;
}

/*! @brief  Fold one classified frame into the handshake. */
static inline Protocol_SyncEvent_T Protocol_ProcSyncPacket(Protocol_SyncState_T * p_state, Packet_ClassId_T rxClass, uint32_t timerNow)
{
    return _Protocol_ResolveSync(p_state, _Protocol_EvaluatePacket(p_state, rxClass), timerNow);
}

/*! @brief  The time event. Self-gating on everything except whether an exchange exists. */
static inline Protocol_SyncEvent_T Protocol_ProcSyncTimeout(Protocol_SyncState_T * p_state, uint32_t reqTimeout, uint32_t timerNow)
{
    if (reqTimeout == 0U) { return PROTOCOL_SYNC_EVENT_NONE; }   /* deadline disabled */
    if (timerNow - p_state->AckTimeStart <= reqTimeout) { return PROTOCOL_SYNC_EVENT_NONE; }

    return _Protocol_ResolveSync(p_state, _Protocol_EvaluateTimeout(p_state), timerNow);
}

/*

*/
static inline bool Protocol_IsAckWaiting(const Protocol_SyncState_T * p_state) { return (p_state->StateId == PROTOCOL_SYNC_AWAIT_ACK); }

/*! The outstanding frame's shape. NULL when nothing is outstanding. */
static inline Packet_FrameFormat_T * Protocol_SyncRespFormat(const Protocol_SyncState_T * p_state) { return p_state->p_RetryFormat; }

/******************************************************************************/
/*
    Stop-and-wait ARQ is only correct with a [sequence] field.
    Without one, a lost ACK is indistinguishable from a lost DATA frame at the remote, so it
    retransmits, and we execute the request a second time.

        we receive REQ(n), execute, ACK  ->  ACK lost
        remote times out, retransmits REQ(n)
        we execute AGAIN

    The wire field already exists - Packet_Meta_T.Sequence -
    The receiver-side rule is small and belongs next to the ack reflex
    in Protocol_Request:

        if (rxMeta.Sequence == LastAccepted) { re-ack, do NOT re-run the handler; }
        else { run the handler; LastAccepted = rxMeta.Sequence; ack; }

    Flash case remote side handles restart.
*/
/******************************************************************************/

