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
    PROTOCOL_REQ_AWAIT,     /* No response. Wait for the next packet. Same as Tx Length 0, if Xcvr_TxN length 0 returns true. */
    PROTOCOL_REQ_RESPOND,   /* Response staged. Transmit it, the sequence continues. */
    PROTOCOL_REQ_ACCEPT,    /* Rx validated. Ack it, no response. */
    PROTOCOL_REQ_REJECT,    /* Rx rejected. Nack it, no response. */
    PROTOCOL_REQ_ABORT,     /* Terminate without a response. */
}
Protocol_ReqCode_T;


/*
    Optionally
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
    Everything the handler may read or write, and nothing about the transport.
    Protocol_ReqParams_T
*/
/*!
    p_RxMeta describes the frame that caused THIS invocation, which is not always a data frame.
    However Protocol_ProcReqResp_T is only called when p_RxMeta is filled with a data frame.
    a union def here: union { const Packet_Meta_T * const p_RxMeta; const Packet_Context_T * const p_RxContext; };
    cannot be reused for socket initializers since the pointers use different const.
*/
typedef struct __attribute__((aligned(sizeof(uintptr_t)))) Packet_Xfer
{
    const Packet_Meta_T * const p_RxMeta;     /* Header of the frame that caused this call - may be a control frame */
    Packet_Meta_T * const p_TxMeta;           /* Handler sets Id and Length; the format builds the header */
    void * const p_Substate;                  /* Handler's own storage, sized by the child protocol */
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
    Xfer already in shape of packet buffer alloc
*/
typedef Protocol_ReqCode_T(*Protocol_ProcReqResp_T)(void * p_context, const Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload);

/* Process handles wire header */
// typedef Protocol_ReqCode_T(*Protocol_ProcReqRespFrame_T) (void * p_context, const void * p_rxFrame, void * p_txFrame);
/*  */
// typedef Protocol_ReqCode_T(*Protocol_ProcReqResp_T)(void * p_context, void * p_substate, const Packet_Context_T * restrict p_rx, Packet_Context_T * restrict p_tx);
/* continuity through substate */
// typedef Protocol_ReqCode_T(*Protocol_DecodeReq_T) (void * p_context, void * p_substate, const Packet_Meta_T * p_rxMeta, const void * p_rxPayload);
// typedef Protocol_ReqCode_T(*Protocol_EncodeResp_T)(void * p_context, void * p_substate, Packet_Meta_T * p_txMeta, void * p_txPayload);

/*

*/
typedef Packet_Id_T Packet_ReqId_T;

// typedef const struct Protocol_ReqId
// {
//     Packet_Id_T ID; /* Must be embedded for __container_of */
// #ifdef PROTOCOL_RESPONSE_FORMAT_ASYMMETRIC
//     Packet_Id_T RESP_ID; /* optionally, or assume same as ID if not specified */
// #endif
// }
// Packet_ReqId_T;


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

/* Passes sizeof(REQS) / sizeof(REQS[0]) from site where the request table is defined */
// typedef const struct { const Protocol_Req_T * P_REQS; uint8_t LENGTH; } Protocol_ReqTable_T;

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
static inline bool Protocol_CaptureReqOfTable(Protocol_ReqState_T * p_state, Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    /*
        Stay on the same outer request. A continuation frame does not re-look-up: the bound row owns the exchange until the handler ends it.
    */
    if (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE) { return true; }

    p_state->p_ReqActive = _Protocol_SearchReqTable(p_reqTable, tableLength, id);
    if (p_state->p_ReqActive != NULL) { p_state->StateId = PROTOCOL_REQ_STATE_ACTIVE; }
    return (p_state->p_ReqActive != NULL);
}

static inline bool Protocol_IsReqActive(const Protocol_ReqState_T * p_state)
{
    return (p_state->p_ReqActive != NULL); /* && (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE)   */
}

/*!
    The bound row's id, carrying the frame shape for both directions.

    One shape per exchange: the request and its answer share the row's FRAME_FORMAT, so the
    rx and tx payload offsets are the same lookup. A row that must answer with a different
    shape names it by restoring RESP_ID here - see the commented alternative in Protocol_Req_T.

    @return NULL when nothing is bound.
*/
static inline Packet_Id_T * Protocol_ReqId(const Protocol_ReqState_T * p_state)
{
    return (p_state->p_ReqActive != NULL) ? &p_state->p_ReqActive->ID : NULL;
}

static inline Packet_Id_T * Protocol_ReqRespId(const Protocol_ReqState_T * p_state)
{
#ifdef PROTOCOL_RESPONSE_ID_ASYMMETRIC
    return (p_state->p_ReqActive->RESP_ID.FRAME_FORMAT != NULL) ? &p_state->p_ReqActive->RESP_ID : &p_state->p_ReqActive->ID;
#else
    return Protocol_ReqId(p_state);
#endif
}

static inline void Protocol_ResetReq(Protocol_ReqState_T * p_state)
{
    p_state->StateId = PROTOCOL_REQ_STATE_IDLE;
    p_state->p_ReqActive = NULL;
}

/*!
    @brief  Invoke the bound handler.

            Identical for the opening call and every continuation -
            the handler distinguishes them by Step, which the engine only clears at Select.
            Returns to IDLE on DONE and ABORT.

    @param  p_xfer  handler context. Payloads arrive already offset by the caller.

    Handler contract
    p_rxPayload / p_txPayload arrive already offset past the header - a handler never
    sees a delimiter, a length or a checksum, and never builds a header. It sets Id and
    Length on p_TxMeta and BUILD_TX_HEADER does the rest.

    Length on p_TxMeta is the PAYLOAD length, not the frame length.

    A stateless handler returns DONE on its first call. Anything that does not return
    DONE or ABORT leaves the request bound and the socket busy.
*/
static inline Protocol_ReqCode_T _Protocol_ProcReq(Protocol_ReqState_T * p_state, void * p_app, const Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
{
    Protocol_ReqCode_T reqCode;

    assert(Protocol_IsReqActive(p_state));

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

/******************************************************************************/
/*!

*/
/******************************************************************************/
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
/*
    Req look up by embedded Id
*/
// typedef Packet_ReqId_T * (*Packet_ReqIdResolver_T)(packet_id_t id);
// #ifndef __containerof
// #define __containerof(x, s, m) ((s *)(const void *)((const char *)(x) - offsetof(s, m)))
// #endif

// static inline bool Protocol_CaptureReq(Protocol_ReqState_T * p_state, const Packet_Id_T * p_packetId)
// {
//     assert(p_packetId != NULL);
//     if (p_state->StateId == PROTOCOL_REQ_STATE_ACTIVE) { return true; } /* stay on the same request even if id changes */
//     p_state->p_ReqActive = __containerof(p_packetId, Protocol_Req_T, ID);
//     if (p_state->p_ReqActive != NULL) { p_state->StateId = PROTOCOL_REQ_STATE_ACTIVE; }
//     return (p_state->p_ReqActive != NULL);
// }
