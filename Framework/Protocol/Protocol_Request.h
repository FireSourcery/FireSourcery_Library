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
    @file   Protocol_Request.
    @brief  Request dispatch. Table lookup, handler invocation, resumable sub-state.
*/
/******************************************************************************/
#include "Packet.h"
#include "Protocol_Sync.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/cdefs.h>

/******************************************************************************/
/*
    Command-table dispatch over a resumable handler.

        IDLE      no request bound
        ACTIVE    a handler is mid-sequence

    dispatch state and handshake state are orthogonal regions (Harel),

    Sans-I/O, like the parser and the handshake. This layer never touches the Xcvr; it
    returns a Protocol_ReqCode_T saying what should go out, and the composition sends it.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Handler interface
*/
/******************************************************************************/
/*! What a handler tells the engine to do next. */
typedef enum Protocol_ReqCode
{
    PROTOCOL_REQ_DONE,      /* Final. Transmit any staged response, then close. */
    PROTOCOL_REQ_AWAIT,     /* No response. Wait for the next packet. */
    PROTOCOL_REQ_RESPOND,   /* Response staged. Transmit it, the sequence continues. */
    PROTOCOL_REQ_ACCEPT,    /* Rx validated. Ack it, no response. */
    PROTOCOL_REQ_REJECT,    /* Rx rejected. Nack it, no response. */
    PROTOCOL_REQ_ABORT,     /* Terminate without a response. */
}
Protocol_ReqCode_T;

/*  Optionally
    Step is the handler's resume point - a program counter it advances itself. That makes a
    multi-step handler a coroutine in the Protothreads sense (Dunkels): one function, switch
    on Step, return between yields, sub-state carried in P_SUB_STATE rather than the C stack.
    The engine does not interpret Step; only the handler assigns meaning to it.

*/
/* sub type inherit. */
typedef struct Protocol_Substate
{
    uint32_t Step;
    uint8_t _Resv[];
}
Protocol_Substate_T;

/*
    Packet Substate Context.
    Everything the handler may read or write, and nothing about the transport.
    Protocol_ReqParams_T
*/
typedef struct Packet_Xfer
{
    const Packet_Meta_T * const p_RxMeta;     /* Virtual header of the frame just delivered */
    Packet_Meta_T * const p_TxMeta;           /* Handler sets Id and Length; the format builds the header */
    void * const p_Substate;                  /* Handler's own storage, sized by the child protocol */
    // Protocol_Substate_T * p_Substate;   /* Handler's own storage, sized by the child protocol */
}
Packet_Xfer_T;

/*
    Table can cast payload with exact type for handling ergonomics
*/
typedef Protocol_ReqCode_T(*Protocol_ProcReqResp_T)(void * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload);

/* Process handles wire header */
// typedef Protocol_ReqCode_T(*Protocol_ProcReqRespFrame_T) (void * p_context, const void * p_rxFrame, void * p_txFrame);

/* continuity through substate */
// typedef Protocol_ReqCode_T(*Protocol_ProcStatefulReq_T) (void * p_context, void * p_substate, const Packet_Meta_T * p_rxMeta, const void * p_rxPayload);
// typedef Protocol_ReqCode_T(*Protocol_ProcStatefulResp_T)(void * p_context, void * p_substate, Packet_Meta_T * p_txMeta, void * p_txPayload);

/*!
    A request table entry.

    One PROC. A stateless exchange returns DONE on its first call; a sequence returns
    RESPOND or AWAIT and is called again when the next frame or ack arrives. The engine does
    not need to know which it is in advance, so there is no second function pointer.
*/
typedef const struct Protocol_Req
{
    Packet_Id_T ID; /* Must be embedded for __container_of */
#ifdef PROTOCOL_REQUEST_TX_OFFSET
    Packet_Id_T RESP_ID; /* optionally, or assume same as ID if not specified */
#endif
    Protocol_ProcReqResp_T PROC;
    Protocol_AckPolicy_T ACK;
}
Protocol_Req_T;


#define PROTOCOL_REQ(Id, Proc, AckPolicy) { .ID = (packet_id_t)(Id), .PROC = (Protocol_ProcReqResp_T)(Proc), .ACK = AckPolicy }

#ifndef __containerof
#define __containerof(x, s, m) ((s *)(const void *)((const char *)(x) - offsetof(s, m)))
#endif


/******************************************************************************/
/*!
    Dispatch State
*/
/******************************************************************************/
typedef enum Protocol_ReqStateId
{
    PROTOCOL_REQ_STATE_IDLE,    /* WAIT_RX_ID */
    PROTOCOL_REQ_STATE_ACTIVE,  /* WAIT_RX_NEXT */
    /* Optionally add PROCESS_REQ_EXT for multi response */
}
Protocol_ReqStateId_T;

typedef struct Protocol_ReqState
{
    Protocol_ReqStateId_T StateId;
    const Protocol_Req_T * p_ReqActive;     /* Bound by Select, retained while ACTIVE */
    uint32_t ReqTimeStart;      /* Exchange deadline base */
}
Protocol_ReqState_T;

/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
static inline bool Protocol_CaptureReq(Protocol_ReqState_T * p_state, Packet_Id_T * p_packetId)
{
    if (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE) { return true; } /* stay on the same request even if id changes */
    p_state->p_ReqActive = __containerof(p_packetId, Protocol_Req_T, ID);
    return true;
}

static inline const Packet_Id_T * Protocol_ReqRespId(const Protocol_ReqState_T * p_state)
{
#ifndef PROTOCOL_REQUEST_TX_OFFSET
    return (p_state->p_ReqActive != NULL) ? &p_state->p_ReqActive->ID : NULL;
    //   p_state->p_ReqActive->RESP_ID
#endif
}


static inline void Protocol_ResetReq(Protocol_ReqState_T * p_state)
{
    p_state->StateId = PROTOCOL_REQ_STATE_IDLE;
    p_state->p_ReqActive = NULL;
    // p_state->Step = 0U;
}

/*!
    @brief  Invoke the bound handler.

            Identical for the opening call and every continuation - the handler distinguishes
            them by Step, which the engine only clears at Select. Returns to IDLE on DONE and
            ABORT.

    @param  p_xfer  handler context. p_Step must point at this state's Step.

    Handler contract
    p_rxPayload / p_txPayload arrive already offset past the header - a handler never
    sees a delimiter, a length or a checksum, and never builds a header. It sets Id and
    Length on p_TxMeta and BUILD_TX_HEADER does the rest.

    Length on p_TxMeta is the PAYLOAD length, not the frame length.

    A stateless handler returns DONE on its first call. Anything that does not return
    DONE or ABORT leaves the request bound and the socket busy.
*/
static inline Protocol_ReqCode_T _Protocol_ProcReq(Protocol_ReqState_T * p_state, void * p_app, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
{
    Protocol_ReqCode_T reqCode;

    switch (p_state->StateId)
    {
        case PROTOCOL_REQ_STATE_ACTIVE:
            reqCode = p_state->p_ReqActive->PROC(p_app, p_xfer, p_rxPayload, p_txPayload);
            if ((reqCode == PROTOCOL_REQ_DONE) || (reqCode == PROTOCOL_REQ_ABORT)) { Protocol_ResetReq(p_state); }
            break;

        /* Bound but not ACTIVE cannot arise - ResetReq clears both together. */
        case PROTOCOL_REQ_STATE_IDLE:
        default:
            reqCode = PROTOCOL_REQ_DONE;
            break;
    }

    return reqCode;
}

static inline Protocol_ReqCode_T Protocol_ProcReqState(Protocol_ReqState_T * p_state, void * p_app, Packet_Xfer_T * p_xfer, const void * p_rxFrame, void * p_txFrame)
{
    if (p_state->p_ReqActive == NULL) { return PROTOCOL_REQ_ABORT; }

    packet_size_t rxOffset = p_state->p_ReqActive->ID.FRAME_FORMAT->HEADER_LENGTH;
#ifndef PROTOCOL_REQUEST_TX_OFFSET
    packet_size_t txOffset = rxOffset;
    // packet_size_t txOffset = p_state->p_ReqActive->RESP_ID.FRAME_FORMAT == NULL ? rxOffset : p_state->p_ReqActive->RESP_ID.FRAME_FORMAT->HEADER_LENGTH;
#endif
    return _Protocol_ProcReq(p_state, p_app, p_xfer, &p_rxFrame[rxOffset], &p_txFrame[txOffset]);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline bool Protocol_IsReqActive(const Protocol_ReqState_T * p_state) { return (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE); }
// static inline bool Protocol_IsReqActive(const Protocol_ReqState_T * p_state) { return (p_state->p_ReqActive != NULL); }

/*! Zeroed when nothing is bound, so an unbound socket acks nothing. */
static inline Protocol_AckPolicy_T Protocol_ReqAckPolicy(const Protocol_ReqState_T * p_state)
{
    return (p_state->p_ReqActive != NULL) ? p_state->p_ReqActive->ACK : (Protocol_AckPolicy_T)PROTOCOL_ACK_NONE;
}



/******************************************************************************/
/*!
    Alternative table search
*/
/******************************************************************************/
/*! @return pointer to Req, NULL when the id has no handler */
static inline const Protocol_Req_T * _Protocol_SearchReqTable(const Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    const Protocol_Req_T * p_req = NULL;
    for (uint8_t iReq = 0U; iReq < tableLength; iReq++) { if (p_reqTable[iReq].ID.ID == id) { p_req = &p_reqTable[iReq]; break; } }
    return p_req;
}

/*!
    @brief  Bind the handler for an incoming id.

            Separate from Proc because the ack policy that decides whether this very frame
            gets acked is a property of the handler - so binding must precede the ack, and
            invocation must follow it.

    @return false when the id has no handler.
*/
static inline bool Protocol_CaptureReqOfTable(Protocol_ReqState_T * p_state, const Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    if (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE) { return true; } /* stay on the same request even if id changes */
    p_state->p_ReqActive = _Protocol_SearchReqTable(p_reqTable, tableLength, id);
    // p_state->Step = 0U;
    if (p_state->p_ReqActive != NULL) { p_state->StateId = PROTOCOL_REQ_STATE_ACTIVE; }
    return (p_state->p_ReqActive != NULL);
}

// full map the descriptor at the main.c layer minus buffers
// typedef const struct Protocol_ReqService
// {
//     const Protocol_Req_T * P_TABLE;             /* id -> handler */
//     uint8_t TABLE_LENGTH;

//     void * P_APP;           /* Passed to every handler */
//     void * P_SUB_STATE;     /* Handler sub-state buffer */

    /* The request service. Id -> handler */
    // const Protocol_Req_T * P_REQ_TABLE;
    // uint8_t REQ_TABLE_LENGTH;
    // void * P_APP_CONTEXT;                   /* Passed to every handler */
    // void * P_REQ_CONTEXT;                   /* Handler sub-state. Sized for the largest handler */

    // const volatile uint32_t * P_TIMER;
    // const uint32_t RX_TIMEOUT;              /* Frame deadline */
    // const uint32_t REQ_TIMEOUT;             /* Exchange deadline */
// }
// Protocol_ReqService_T;


/******************************************************************************/
/*
    Flow traces - the four shapes the request table actually uses.
*/
/******************************************************************************/
/*
    Stateless (VarRead) - PROTOCOL_ACK_NONE

    Parse:      COMPLETE
    Class:      DATA
    Sync:       OPEN + DATA -> REQUEST
    Request:    Select, no ack, PROC -> DONE, TxMeta.Length set
    Compose:    DONE -> Tx response. OnTx: RX_ACK_OPEN = 0 -> stays OPEN
    One pass. No state persists.
*/

/*
    Stateless with ack (CallExt) - PROTOCOL_ACK_ON_REQ

    Pass 1:
    Sync:       OPEN + DATA -> REQUEST
    Request:    Select, TX_ACK_OPEN -> Tx ACK, PROC -> DONE
    Compose:    Tx response. OnTx: RX_ACK_OPEN -> AWAIT_ACK

    Pass N:
    Class:      ACK
    Sync:       AWAIT_ACK + ACK -> RESUME, back to OPEN
    Request:    IDLE, nothing to resume -> exchange closed
*/

/*
    Stateful stream (DataModeRead) - PROTOCOL_ACK_EVERY_STEP, ack-paced

    Pass 1:
    Request:    Select, Tx ACK, PROC(Step 0) -> RESPOND, chunk staged
    Compose:    Tx response -> AWAIT_ACK

    Pass N:
    Sync:       AWAIT_ACK + ACK -> RESUME
    Request:    ACTIVE, PROC(Step n) -> RESPOND, next chunk
    Compose:    Tx response -> AWAIT_ACK

    Final:
    Request:    PROC -> DONE -> IDLE
*/

/*
    Stateful sink (DataModeWrite) - data-paced, handler validates

    Pass 1:
    Request:    Select, PROC(Step 0) -> AWAIT. Nothing transmitted, stays ACTIVE.

    Pass n:
    Sync:       OPEN + DATA -> REQUEST
    Request:    ACTIVE, TX_ACK_STEP -> Tx ACK, PROC -> ACCEPT or REJECT
    Compose:    ACCEPT -> Tx ACK.  REJECT -> Tx NACK.

    With TX_ACK_STEP = 0 the handler's ACCEPT is the only ack, so the remote learns the
    chunk was not merely received but validated.
*/

/******************************************************************************/
/*!
*/
/******************************************************************************/