#pragma once

/******************************************************************************/
/*!
    @file   Protocol_DataMode.h
    @author FireSourcery
    @brief  Stateful bulk transfer - Read and Write - on the REQ / RESP handler pair.
*/
/******************************************************************************/
#include "../Protocol_Request.h"
#include "../Packet.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>


/******************************************************************************/
/*!
    Wire payloads
*/
/******************************************************************************/
typedef struct PACKET_PACKED Protocol_DataMode_Req { uint32_t Address; uint32_t Size; uint32_t Config; } Protocol_DataMode_Req_T;
typedef struct PACKET_PACKED Protocol_DataMode_Resp { uint16_t Status; }                                 Protocol_DataMode_Resp_T;

#define PROTOCOL_DATA_MODE_STATUS_OK (0U)   /* Application status. Non-zero ends the transfer. */


/******************************************************************************/
/*!
    Application binding
*/
/******************************************************************************/
/*!
    Memory operations. Each returns an application status, 0 for success.
    OPEN prepares a transfer - bounds check, unlock, erase - and is called for both directions.
*/
typedef const struct Protocol_DataMode_Ops
{
    uint16_t(*OPEN)  (void * p_app, uintptr_t address, size_t size, uint32_t config);
    uint16_t(*READ)  (void * p_app, uintptr_t address, size_t size, void * p_dest);
    uint16_t(*WRITE) (void * p_app, uintptr_t address, const void * p_src, size_t size);
}
Protocol_DataMode_Ops_T;

typedef const struct Protocol_DataModeInterface
{
    Protocol_DataMode_Ops_T * P_OPS;
    void * P_MODULE;
    packet_id_t DATA_ID;        /* Id carrying a raw chunk in either direction */
    packet_id_t RESP_ID;
    packet_size_t CHUNK_MAX;    /* Bounded by the format's payload capacity */
}
Protocol_DataModeInterface_T;


/******************************************************************************/
/*!
    Sub-state

    Held in the socket's P_SUB_STATE buffer. The handler's own storage - the engine
    no longer carries a Step for it.
*/
/******************************************************************************/
/*!
    What a pass is doing. Shared by both transfers so the two flows read side by side.

    Derived per pass from the inbound packet and the cursor - never an input. The copy kept
    in Protocol_DataModeState_T is for inspection only; deriving it is what keeps a stored
    step counter from drifting out of sync with the transfer it describes.
*/
typedef enum Protocol_DataModeStateId
{
    PROTOCOL_DATA_MODE_STATE_IDLE,      /* Nothing to absorb, nothing owed */
    PROTOCOL_DATA_MODE_STATE_OPEN,      /* Opening request - set up the transfer, reply with status */
    PROTOCOL_DATA_MODE_STATE_DATA,      /* Move one chunk, the transfer continues */
    PROTOCOL_DATA_MODE_STATE_CLOSE,     /* Cursor spent, or the last chunk - reply and close */
    PROTOCOL_DATA_MODE_STATE_ERROR,     /* Malformed for this transfer - nack */
}
Protocol_DataModeStateId_T;

typedef struct Protocol_DataMode_State
{
    uintptr_t Address;      /* Transfer base */
    size_t Size;            /* Total bytes */
    size_t Index;           /* Bytes transferred so far */

    packet_id_t ReqId;      /* Echoed on the status replies. Taken from the opening request. */
    // uint16_t Status;        /* Carries REQ's verdict across to RESP */
    // bool IsOpened;          /* The opening status reply has been emitted */
    Protocol_DataModeStateId_T StateId;
}
Protocol_DataModeState_T;

static inline void Protocol_DataMode_Begin(Protocol_DataModeState_T * p_xfer, const Protocol_DataMode_Req_T * p_req, packet_id_t reqId)
{
    p_xfer->Address = p_req->Address;
    p_xfer->Size = p_req->Size;
    p_xfer->Index = 0U;
    p_xfer->ReqId = reqId;
}

/*! Bytes remaining, clamped to one chunk. CHUNK_MAX moved to the interface, so this takes it. */
// static inline packet_size_t Protocol_DataMode_ChunkOf(Protocol_DataModeInterface_T * p_app, const Protocol_DataModeState_T * p_state)
// {
//     return (packet_size_t)math_min(p_state->Size - p_state->Index, p_app->CHUNK_MAX);
// }


static inline void Protocol_DataModeReq_Setup(Protocol_DataModeState_T * p_substate, const Packet_Meta_T * p_rxMeta, const Protocol_DataMode_Req_T * p_req)
{
    p_substate->Address = p_req->Address;
    p_substate->Size = p_req->Size;
    p_substate->Index = 0U;
    p_substate->ReqId = p_rxMeta->Id;
}

/*! Stage a status reply. Length is the payload length - the header codec adds its own. */
// static inline void Protocol_DataMode_BuildStatus(const Protocol_DataModeState_T * p_substate, Packet_Meta_T * p_txMeta, Protocol_DataMode_Resp_T * p_txPayload)
// {
//     p_txPayload->Status = p_substate->Status;
//     p_txMeta->Id = p_substate->ReqId;
//     p_txMeta->Length = sizeof(Protocol_DataMode_Resp_T);
// }


/******************************************************************************/
/*
    How a pass is classified

    The handler is re-entered for every frame that arrives while the exchange is open - the
    acks that pace a Read included - so "was anything delivered" is not a usable signal.
    Two things are:

        *p_Step     0 on the pass that bound the request, non-zero afterwards. The engine
                    clears it at bind and never reads it again.
        RxMeta.Id   DATA_ID marks a chunk. Anything else mid-transfer is a pacing frame.

    Together they separate the opening request from a continuation without a flag in
    sub-state, and without the handler knowing which Ids mean ack - Sync has already
    rejected anything genuinely out of sequence before a handler sees it.

    Sub-state is the cursor alone:  { Address; Size; Index; ReqId; }
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Stage derivation

    One if/else per transfer, stating in a single place how a pass is classified. Pure over
    (packet, cursor), so each is testable on its own and neither handler body has to
    re-derive anything.
*/
/******************************************************************************/
/*!
    Read - the opening request, then one chunk per pacing ack until the cursor runs out.
*/
static inline Protocol_DataModeStateId_T Protocol_DataModeRead_StateOf(const Protocol_DataModeState_T * p_state, const Packet_Xfer_T * p_xfer)
{
    if (*p_xfer->p_Step == 0U)
    {
        return (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T)) ? PROTOCOL_DATA_MODE_STATE_ERROR : PROTOCOL_DATA_MODE_STATE_OPEN;
    }

    return (p_state->Index < p_state->Size) ? PROTOCOL_DATA_MODE_STATE_DATA : PROTOCOL_DATA_MODE_STATE_CLOSE;
}

/*!
    Write - the opening request, then a chunk per inbound DATA frame.

    The ack answering the opening reply re-enters the handler with a non-DATA Id; there is
    nothing to absorb on that pass, so it classifies as IDLE rather than as a chunk.
*/
static inline Protocol_DataModeStateId_T Protocol_DataModeWrite_StateOf(Protocol_DataModeInterface_T * p_app, const Protocol_DataModeState_T * p_state, const Packet_Xfer_T * p_xfer)
{
    if (*p_xfer->p_Step == 0U)
    {
        return (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T)) ? PROTOCOL_DATA_MODE_STATE_ERROR : PROTOCOL_DATA_MODE_STATE_OPEN;
    }

    if (p_xfer->p_RxMeta->Id != p_app->DATA_ID) { return PROTOCOL_DATA_MODE_STATE_IDLE; }

    /* A chunk past the declared size is the host misbehaving, not a memory fault. */
    if (p_state->Index + p_xfer->p_RxMeta->Length > p_state->Size) { return PROTOCOL_DATA_MODE_STATE_ERROR; }

    return (p_state->Index + p_xfer->p_RxMeta->Length < p_state->Size) ? PROTOCOL_DATA_MODE_STATE_DATA : PROTOCOL_DATA_MODE_STATE_CLOSE;
}

/******************************************************************************/
/*!
    Stage bodies
*/
/******************************************************************************/
/*! Set up a transfer and answer with the result, in the pass that received the request. */
static inline Protocol_ReqCode_T Protocol_DataMode_ProcOpen(Protocol_DataModeInterface_T * p_app, Protocol_DataModeState_T * p_state, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataModeReq_Setup(p_state, p_xfer->p_RxMeta, p_req);

    *p_xfer->p_Step = 1U;   /* Every later pass is a continuation */

    p_resp->Status = p_app->P_OPS->OPEN(p_app->P_MODULE, p_state->Address, p_state->Size, p_req->Config);

    p_xfer->p_TxMeta->Id = p_state->ReqId;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    return (p_resp->Status == PROTOCOL_DATA_MODE_STATUS_OK) ? PROTOCOL_REQ_RESPOND : PROTOCOL_REQ_DONE;
}


/*! Stage a status reply from an explicit value, rather than from sub-state. */
static inline Protocol_ReqCode_T Protocol_DataMode_Reply(const Protocol_DataModeState_T * p_state, Packet_Xfer_T * p_xfer, void * p_txPayload, uint16_t status, Protocol_ReqCode_T onOk)
{
    ((Protocol_DataMode_Resp_T *)p_txPayload)->Status = status;
    p_xfer->p_TxMeta->Id = p_state->ReqId;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    return (status == PROTOCOL_DATA_MODE_STATUS_OK) ? onOk : PROTOCOL_REQ_DONE;
}

/*! Move one inbound chunk into memory and advance. Shared by the DATA and CLOSE stages. */
// static inline uint16_t Protocol_DataMode_ProcWriteChunk(Protocol_DataModeInterface_T * p_app, Packet_Xfer_T * p_xfer,  const uint8_t * p_data)
static inline uint16_t Protocol_DataMode_ProcWriteChunk(Protocol_DataModeInterface_T * p_app, Protocol_DataModeState_T * p_state, const Packet_Meta_T * p_meta, const uint8_t * p_data)
{
    uint16_t status = p_app->P_OPS->WRITE(p_app->P_MODULE, p_state->Address + p_state->Index, p_data, p_meta->Length);

    if (status == PROTOCOL_DATA_MODE_STATUS_OK) { p_state->Index += p_meta->Length; }

    return status;
}

/*! Stage one outbound chunk and advance. */
static inline uint16_t Protocol_DataMode_ProcReadChunk(Protocol_DataModeInterface_T * p_app, Protocol_DataModeState_T * p_state, Packet_Xfer_T * p_xfer, uint8_t * p_data)
{
    packet_size_t chunk = math_min(p_state->Size - p_state->Index, p_app->CHUNK_MAX);
    uint16_t status = p_app->P_OPS->READ(p_app->P_MODULE, p_state->Address + p_state->Index, chunk, p_data);

    if (status == PROTOCOL_DATA_MODE_STATUS_OK)
    {
        p_state->Index += chunk;
        p_xfer->p_TxMeta->Id = p_app->DATA_ID;
        p_xfer->p_TxMeta->Length = chunk;
    }

    return status;
}

/******************************************************************************/
/*!
    Read - device streams memory to the host.

    Paced by the host's acks: every ack frees the floor and pulls the next chunk.
*/
/******************************************************************************/
static inline Protocol_ReqCode_T Protocol_DataMode_Read(void * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
{
    Protocol_DataModeInterface_T * p_app = p_context;
    Protocol_DataModeState_T * p_state = p_xfer->p_Substate;
    uint16_t status;

    p_state->StateId = Protocol_DataModeRead_StateOf(p_state, p_xfer);

    switch (p_state->StateId)
    {
        case PROTOCOL_DATA_MODE_STATE_OPEN:
            return Protocol_DataMode_ProcOpen(p_app, p_state, p_xfer, p_rxPayload, p_txPayload);

        /* A read fault reports and ends here; otherwise the chunk is already staged. */
        case PROTOCOL_DATA_MODE_STATE_DATA:
            status = Protocol_DataMode_ProcReadChunk(p_app, p_state, p_xfer, p_txPayload);
            return (status == PROTOCOL_DATA_MODE_STATUS_OK) ? PROTOCOL_REQ_RESPOND : Protocol_DataMode_Reply(p_state, p_xfer, p_txPayload, status, PROTOCOL_REQ_DONE);

        case PROTOCOL_DATA_MODE_STATE_CLOSE:
            return Protocol_DataMode_Reply(p_state, p_xfer, p_txPayload, PROTOCOL_DATA_MODE_STATUS_OK, PROTOCOL_REQ_DONE);

        case PROTOCOL_DATA_MODE_STATE_ERROR:
            return PROTOCOL_REQ_REJECT;

        case PROTOCOL_DATA_MODE_STATE_IDLE:
        default:
            return PROTOCOL_REQ_AWAIT;
    }
}

/******************************************************************************/
/*!
    Write - host streams memory to the device.

    Paced by the host's data packets. Only the first and last earn a reply; the rest are
    answered by the Sync layer's ack.
*/
/******************************************************************************/
static inline Protocol_ReqCode_T Protocol_DataMode_Write(void * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
{
    Protocol_DataModeInterface_T * p_app = p_context;
    Protocol_DataModeState_T * p_state = p_xfer->p_Substate;
    uint16_t status;

    p_state->StateId = Protocol_DataModeWrite_StateOf(p_app, p_state, p_xfer);

    switch (p_state->StateId)
    {
        case PROTOCOL_DATA_MODE_STATE_OPEN:
            return Protocol_DataMode_ProcOpen(p_app, p_state, p_xfer, p_rxPayload, p_txPayload);

        /* ACCEPT is the silent path: the ack is the whole reply. A fault reports and ends. */
        case PROTOCOL_DATA_MODE_STATE_DATA:
            status = Protocol_DataMode_ProcWriteChunk(p_app, p_state, p_xfer->p_RxMeta, p_rxPayload);
            return (status == PROTOCOL_DATA_MODE_STATUS_OK) ? PROTOCOL_REQ_ACCEPT : Protocol_DataMode_Reply(p_state, p_xfer, p_txPayload, status, PROTOCOL_REQ_DONE);

        /* Last chunk: absorb it, then answer with the transfer's outcome. */
        case PROTOCOL_DATA_MODE_STATE_CLOSE:
            status = Protocol_DataMode_ProcWriteChunk(p_app, p_state, p_xfer->p_RxMeta, p_rxPayload);
            return Protocol_DataMode_Reply(p_state, p_xfer, p_txPayload, status, PROTOCOL_REQ_DONE);

        case PROTOCOL_DATA_MODE_STATE_ERROR:
            return PROTOCOL_REQ_REJECT;

        case PROTOCOL_DATA_MODE_STATE_IDLE:
        default:
            return PROTOCOL_REQ_AWAIT;
    }
}

/******************************************************************************/
/*
    Pass traces

    Read - PROTOCOL_ACK_ON_REQ, ack-paced

        bind, Step 0, DATA frame    OPEN    setup, OPEN(), status reply     -> RESPOND
        ack                         DATA    one chunk staged                -> RESPOND
        ...
        ack, cursor spent           CLOSE   closing status                  -> DONE

    Write - data-paced

        bind, Step 0, DATA frame    OPEN    setup, OPEN(), status reply     -> RESPOND
        ack of that reply           IDLE    nothing to absorb               -> AWAIT
        DATA frame                  DATA    chunk written, silent           -> ACCEPT
        ...
        DATA frame, last            CLOSE   chunk written, status reply     -> DONE

    A media fault in either direction stages the status and returns DONE, so the remote
    always learns why a transfer stopped rather than waiting out REQ_TIMEOUT.
*/
/******************************************************************************/
