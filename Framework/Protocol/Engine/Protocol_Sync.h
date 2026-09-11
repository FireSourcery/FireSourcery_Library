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

    No timer. A request's deadline spans this layer and Request both, so the composition owns
    it and delivers expiry as PROTOCOL_SYNC_EVENT_TIMEOUT.

    NOT YET IMPLEMENTED - the alternating bit. See the note at the foot of this file.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Ack Policy - configured per handler.

    A request's opening exchange and its continuation steps are configured separately, so a
    handler can ack the request but not each chunk, or the reverse.
*/
/******************************************************************************/
typedef struct Protocol_AckPolicy
{
    uint8_t ACK_REQ         : 1U;   /* Ack the request */
    uint8_t EXPECT_ACK_RESP : 1U;   /* Await an ack after response */
    // uint8_t TX_ACK_OPEN     : 1U;   /* On Entry. Ack the request packet on arrival */
    // uint8_t RX_ACK_OPEN     : 1U;   /* On Entry. Await an ack after the opening response */
    // uint8_t TX_ACK_STEP     : 1U;   /* Ack each continuation packet on arrival */
    // uint8_t RX_ACK_STEP     : 1U;   /* Await an ack after each continuation response */
    uint8_t TX_ACK_ABORT    : 1U;   /* Acknowledge a received abort */
    uint8_t RETRANSMIT_MAX  : 3U;   /* Retries before the exchange is abandoned. 0 = no retry */
}
Protocol_AckPolicy_T;

#define PROTOCOL_ACK_NONE       { 0U }
#define PROTOCOL_ACK_ON_REQ     { .TX_ACK_OPEN = 1U, .RX_ACK_OPEN = 1U, .RETRANSMIT_MAX = 3U }
#define PROTOCOL_ACK_EVERY_STEP { .TX_ACK_OPEN = 1U, .RX_ACK_OPEN = 1U, .TX_ACK_STEP = 1U, .RX_ACK_STEP = 1U, .RETRANSMIT_MAX = 3U }

/*
    The OPEN / STEP selection is made where isStep is known, in the composition. Neither bit
    pair is read here.
*/

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
    PROTOCOL_SYNC_OPEN,
    PROTOCOL_SYNC_AWAIT_ACK,
}
Protocol_SyncStateId_T;

typedef struct Protocol_SyncState
{
    Protocol_SyncStateId_T StateId;
    uint8_t RetransmitCount;    /* Retries spent on the outstanding frame */
}
Protocol_SyncState_T;


/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
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
    p_state->RetransmitCount = 0U;
}

static inline void Protocol_ExpectAck(Protocol_SyncState_T * p_state)
{
    p_state->StateId = PROTOCOL_SYNC_AWAIT_ACK;
}

/*
    Retransmit while the budget lasts, otherwise abandon. Shared by the nack and deadline
    paths, which differ only in what triggered them.
*/
static inline Protocol_SyncEvent_T Protocol_ResolveNackCount(Protocol_SyncState_T * p_state, Protocol_AckPolicy_T policy)
{
    if (p_state->RetransmitCount >= policy.RETRANSMIT_MAX)
    {
        Protocol_ResetSync(p_state);
        return PROTOCOL_SYNC_EVENT_FAILED;
    }

    p_state->RetransmitCount++;
    return PROTOCOL_SYNC_EVENT_RETRANSMIT;
}

static inline Protocol_SyncEvent_T Protocol_ResolveAck(Protocol_SyncState_T * p_state, Protocol_AckPolicy_T policy)
{
    Protocol_ResetSync(p_state);
    return PROTOCOL_SYNC_EVENT_RESUME;
}

/*!
    @brief  Fold one classified frame into the handshake.
    @param  rxClass  from Packet_ClassOf. This layer never sees an Id or a format.
*/
static inline Protocol_SyncEvent_T Protocol_ProcSyncRx(Protocol_SyncState_T * p_state, Protocol_AckPolicy_T policy, Packet_ClassId_T rxClass)
{
    /* An abort ends the exchange wherever it was. The caller acks it if policy says so. */
    if (rxClass == PACKET_CLASS_ABORT) { Protocol_ResetSync(p_state); return PROTOCOL_SYNC_EVENT_ABORT; }

    switch (p_state->StateId)
    {
        /* Nothing outstanding, so an ack or nack refers to nothing. */
        case PROTOCOL_SYNC_OPEN:  return (rxClass == PACKET_CLASS_DATA) ? PROTOCOL_SYNC_EVENT_REQUEST : PROTOCOL_SYNC_EVENT_REJECT;

        case PROTOCOL_SYNC_AWAIT_ACK:
            switch (rxClass)
            {
                case PACKET_CLASS_ACK: return Protocol_ResolveAck(p_state, policy);
                case PACKET_CLASS_NACK: return Protocol_ResolveNackCount(p_state, policy);
                    /* A data frame before the ack is out of sequence - the remote is ahead of us. */
                default: return PROTOCOL_SYNC_EVENT_REJECT;
            }

        default: return PROTOCOL_SYNC_EVENT_NONE;
    }
}

/*!
    @brief  The request deadline expired. Same choice as a nack: retry or abandon.
*/
static inline Protocol_SyncEvent_T Protocol_ResolveSyncRxTimeout(Protocol_SyncState_T * p_state, Protocol_AckPolicy_T policy)
{
    return (p_state->StateId == PROTOCOL_SYNC_AWAIT_ACK) ? Protocol_ResolveNackCount(p_state, policy) : PROTOCOL_SYNC_EVENT_FAILED;
}


static inline bool Protocol_IsSyncWaiting(const Protocol_SyncState_T * p_state) { return (p_state->StateId == PROTOCOL_SYNC_AWAIT_ACK); }

/******************************************************************************/
/*
    GAP - the alternating bit.

    Stop-and-wait ARQ is only correct with a sequence bit on both the data frame and its ack.
    Without one, a lost ACK is indistinguishable from a lost DATA frame at the remote, so it
    retransmits, and we execute the request a second time.

        we receive REQ(n), execute, ACK  ->  ACK lost
        remote times out, retransmits REQ(n)
        we execute AGAIN

    Harmless for VarRead. Not harmless for SaveNvm, a data-mode chunk write, or anything that
    accumulates.

    The wire field already exists - MotPacket_Header_T.Sequence, and Packet_Meta_T.Sequence -
    both currently unused. The receiver-side rule is small and belongs next to the ack reflex
    in Protocol_Request:

        if (rxMeta.Sequence == LastAccepted) { re-ack, do NOT re-run the handler; }
        else { run the handler; LastAccepted = rxMeta.Sequence; ack; }

    One byte of state per socket, plus echoing the sequence in the ack so the remote can pair
    them. Worth doing before this is relied on for flash writes.
*/
/******************************************************************************/
