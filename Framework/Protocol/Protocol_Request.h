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
#include <assert.h>
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
/*!
    p_RxMeta describes the frame that caused THIS invocation, which is not always a data frame.
    However Protocol_ProcReqResp_T is only called when p_RxMeta is filled with a data frame.
*/
typedef struct Packet_Xfer
{
    const Packet_Meta_T * const p_RxMeta;     /* Header of the frame that caused this call - may be a control frame */
    Packet_Meta_T * const p_TxMeta;           /* Handler sets Id and Length; the format builds the header */
    void * const p_Substate;                  /* Handler's own storage, sized by the child protocol */
    // Protocol_Substate_T * p_Substate;   /* Handler's own storage, sized by the child protocol */
}
Packet_Xfer_T;


/*
    This function signature exposes the void * payloads so they can be cast through the function pointer for ergonomics.
    Whereas the following would not
    struct
    {
        const Packet_Meta_T * const p_RxMeta;
        const void * restrict p_rxPayload;
    }
*/
typedef Protocol_ReqCode_T(*Protocol_ProcReqResp_T)(void * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload);

/* Process handles wire header */
// typedef Protocol_ReqCode_T(*Protocol_ProcReqRespFrame_T) (void * p_context, const void * p_rxFrame, void * p_txFrame);

/* continuity through substate */
// typedef Protocol_ReqCode_T(*Protocol_ProcStatefulReq_T) (void * p_context, void * p_substate, const Packet_Meta_T * p_rxMeta, const void * p_rxPayload);
// typedef Protocol_ReqCode_T(*Protocol_ProcStatefulResp_T)(void * p_context, void * p_substate, Packet_Meta_T * p_txMeta, void * p_txPayload);

typedef Packet_Id_T Packet_ReqId_T;

// typedef const struct Protocol_ReqId
// {
//     Packet_Id_T ID; /* Must be embedded for __container_of */
// #ifdef PROTOCOL_RESPONSE_FORMAT_SEPARATE
//     Packet_Id_T RESP_ID; /* optionally, or assume same as ID if not specified */
// #endif
// }
// Packet_ReqId_T;

/*
    Req look up by embedded Id
*/
// typedef Packet_ReqId_T * (*Packet_ReqIdResolver_T)(packet_id_t id);

/*!
    A request table entry.

    One PROC. A stateless exchange returns DONE on its first call; a sequence returns
    RESPOND or AWAIT and is called again when the next frame or ack arrives. The engine does
    not need to know which it is in advance, so there is no second function pointer.
*/
typedef const struct Protocol_Req
{
    Packet_ReqId_T ID; /* Must be embedded for __container_of */
    Protocol_ProcReqResp_T PROC;
    Protocol_AckPolicy_T ACK;
}
Protocol_Req_T;


/*
    Frame shape is per id, so it belongs on the row. RESP_ID defaults to the request's id and
    shape; a response that answers with a different shape names its own.
*/
#define PROTOCOL_REQ(Id, Format, Proc, AckPolicy)                       \
{                                                                       \
    .ID      = { .ID = (packet_id_t)(Id), .FRAME_FORMAT = (Format) },   \
    .PROC    = (Protocol_ProcReqResp_T)(Proc),                          \
    .ACK     = AckPolicy,                                               \
}


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
    Protocol_Req_T * p_ReqActive;     /* Bound by Select, retained while ACTIVE */
    Packet_Id_T * p_ReqId; /* The most recently arrived id, which may differ from the active handler processing */
    /*
        No deadline base here. The exchange outlives the binding - a handler returning DONE
        unbinds while its response is still outstanding - so the base is latched in
        Protocol_SyncState_T beside RetryMax and p_RetryFormat, which outlive it for the same
        reason. Protocol_ResetReq would otherwise clear the clock on a frame still in flight.
    */
}
Protocol_ReqState_T;

/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*! @return pointer to Req, NULL when the id has no handler */
static inline const Protocol_Req_T * _Protocol_SearchReqTable(const Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    for (size_t iReq = 0U; iReq < tableLength; iReq++) { if (p_reqTable[iReq].ID.ID == id) { return &p_reqTable[iReq]; } }
    return NULL;
}

/*!
    @brief  Bind the handler for an incoming id.

            Separate from Proc because the ack policy that decides whether this very frame
            gets acked is a property of the handler - so binding must precede the ack, and
            invocation must follow it.

    @return false when the id has no handler.
*/
/*
    Only a data frame reaches here - PROTOCOL_SYNC_EVENT_REQUEST is raised for
    PACKET_CLASS_DATA alone, so an ack never rebinds.
*/
static inline bool Protocol_CaptureReqOfTable(Protocol_ReqState_T * p_state, const Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    /* Stay on the same outer request even if the rx id changes. */
    if (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE)
    {
        if (id == p_state->p_ReqActive->ID.ID) { p_state->p_ReqId = &p_state->p_ReqActive->ID; return true; }
        const Protocol_Req_T * p_req = _Protocol_SearchReqTable(p_reqTable, tableLength, id);
        p_state->p_ReqId = (p_req != NULL) ? &p_req->ID : NULL;
        return(p_state->p_ReqId != NULL);
    }

    p_state->p_ReqActive = _Protocol_SearchReqTable(p_reqTable, tableLength, id);
    p_state->p_ReqId = (p_state->p_ReqActive != NULL) ? &p_state->p_ReqActive->ID : NULL;
    if (p_state->p_ReqActive != NULL) { p_state->StateId = PROTOCOL_REQ_STATE_ACTIVE; }
    return (p_state->p_ReqActive != NULL);
}

/*!
    The most recent id
*/
static inline Packet_Id_T * Protocol_ReqActiveId(const Protocol_ReqState_T * p_state)
{
    if (p_state->p_ReqActive == NULL) { return NULL; }
    return (p_state->p_ReqId != NULL) ? p_state->p_ReqId : &p_state->p_ReqActive->ID;
}

/*! The shape the bound request answers with. NULL when nothing is bound. */
static inline Packet_Id_T * Protocol_ReqRespId(const Protocol_ReqState_T * p_state)
{
    if (p_state->p_ReqActive == NULL) { return NULL; }
    return &p_state->p_ReqActive->ID; // the opening id
    // return Protocol_ReqActiveId(p_state); //using the latest
// #ifdef PROTOCOL_RESPONSE_FORMAT_SEPARATE
//     return (p_state->p_ReqActive->RESP_ID.FRAME_FORMAT != NULL) ? &p_state->p_ReqActive->RESP_ID : &p_state->p_ReqActive->ID;
// #else
    // return &p_state->p_ReqActive->ID;
// #endif
}

static inline void Protocol_ResetReq(Protocol_ReqState_T * p_state)
{
    p_state->StateId = PROTOCOL_REQ_STATE_IDLE;
    p_state->p_ReqActive = NULL;
    p_state->p_ReqId = NULL;   /* outlives its exchange otherwise, and Protocol_ReqActiveId would report it */
}

/*!
    @brief  Invoke the bound handler.

            Identical for the opening call and every continuation -
            the handler distinguishes them by Step, which the engine only clears at Select.
            Returns to IDLE on DONE and ABORT.

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


/*!
    Offset both payloads past their headers and invoke the handler.

    Rx offset is the last data frame's, held in p_ReqId - a control frame never rebinds, so
    on an ack-driven continuation it still names the packet the handler is working through.
    Tx offset is the mirror and comes from RESP_ID, the only thing that knows the answer's shape.
*/
static inline Protocol_ReqCode_T Protocol_ProcReqState(Protocol_ReqState_T * p_state, void * p_app, Packet_Xfer_T * p_xfer, const uint8_t * p_rxFrame, uint8_t * p_txFrame)
{
    if (p_state->p_ReqActive == NULL) { return PROTOCOL_REQ_ABORT; }

    assert(Protocol_ReqActiveId(p_state)->FRAME_FORMAT != NULL);
    assert(Protocol_ReqRespId(p_state)->FRAME_FORMAT != NULL);

    packet_size_t rxOffset = Protocol_ReqActiveId(p_state)->FRAME_FORMAT->HEADER_LENGTH; /* the last data frame */
    packet_size_t txOffset = Protocol_ReqRespId(p_state)->FRAME_FORMAT->HEADER_LENGTH;   /* the shape the row answers with */

    return _Protocol_ProcReq(p_state, p_app, p_xfer, &p_rxFrame[rxOffset], &p_txFrame[txOffset]);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline bool Protocol_IsReqActive(const Protocol_ReqState_T * p_state)
{
    return (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE) && (p_state->p_ReqActive != NULL);
}

/*! Zeroed when nothing is bound, so an unbound socket acks nothing. */
static inline Protocol_AckPolicy_T Protocol_ReqAckPolicy(const Protocol_ReqState_T * p_state)
{
    return (p_state->p_ReqActive != NULL) ? p_state->p_ReqActive->ACK : (Protocol_AckPolicy_T)PROTOCOL_ACK_NONE;
}


/*
    Alternatively 1 look up. Packet_ParseRxFrame_T would be coupled to Req Table

    [Packet_Id_T *] MUST be from the Request Table
            Rx returned Packet_Id_T determines response payload offset.
*/
// static inline bool Protocol_CaptureReq(Protocol_ReqState_T * p_state, const Packet_Id_T * p_packetId)
// {
//     assert(p_packetId != NULL);
//     if (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE) { return true; } /* stay on the same request even if id changes */
//     p_state->p_ReqActive = __containerof(p_packetId, Protocol_Req_T, ID);
//     if (p_state->p_ReqActive != NULL) { p_state->StateId = PROTOCOL_REQ_STATE_ACTIVE; }
//     return (p_state->p_ReqActive != NULL);
// }



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