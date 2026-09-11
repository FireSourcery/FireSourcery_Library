#pragma once

/******************************************************************************/
/*!
    @file   Protocol_FlashLoader.h
    @author FireSourcery
    @brief  Stateful bulk transfer - Read and Write - on the REQ / RESP handler pair.
*/
/******************************************************************************/
#include "Protocol_DataMode.h"
#include "Peripheral/NvMemory/Flash/Flash.h"




/******************************************************************************/
/*!
    Stage bodies
*/
/******************************************************************************/
/*! Set up a transfer and answer with the result, in the pass that received the request. */
static inline Protocol_ReqCode_T Protocol_DataMode_ProcOpen(Flash_T * p_app, Protocol_DataMode_State_T * p_state, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataModeReq_Setup(p_state, p_xfer->p_RxMeta, p_req);

    p_resp->Status = p_app->P_OPS->OPEN(p_app->P_MODULE, p_state->Address, p_state->Size, p_req->Config);

    p_xfer->p_TxMeta->Id = p_state->ReqId;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    return (p_resp->Status == PROTOCOL_DATA_MODE_STATUS_OK) ? PROTOCOL_REQ_RESPOND : PROTOCOL_REQ_DONE;
}


// static inline Protocol_ReqCode_T Protocol_DataMode_Reply(void * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
// {
//     p_resp->Status = PROTOCOL_DATA_MODE_STATUS_OK;
//     return PROTOCOL_REQ_RESPOND;
// }

/*! Stage a status reply from an explicit value, rather than from sub-state. */
static inline Protocol_ReqCode_T Protocol_DataMode_Reply(const Protocol_DataMode_State_T * p_state, Packet_Xfer_T * p_xfer, void * p_txPayload, uint16_t status, Protocol_ReqCode_T onOk)
{
    ((Protocol_DataMode_Resp_T *)p_txPayload)->Status = status;
    p_xfer->p_TxMeta->Id = p_state->ReqId;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    return (status == PROTOCOL_DATA_MODE_STATUS_OK) ? onOk : PROTOCOL_REQ_DONE;
}

/*! Move one inbound chunk into memory and advance. Shared by the DATA and CLOSE stages. */
// static inline uint16_t Protocol_DataMode_ProcWriteChunk(Flash_T * p_app, Packet_Xfer_T * p_xfer,  const uint8_t * p_data)
static inline uint16_t Protocol_DataMode_ProcWriteChunk(Flash_T * p_app, Protocol_DataMode_State_T * p_state, const Packet_Meta_T * p_meta, const uint8_t * p_data)
{
    uint16_t status = p_app->P_OPS->WRITE(p_app->P_MODULE, p_state->Address + p_state->Index, p_data, p_meta->Length);

    if (status == PROTOCOL_DATA_MODE_STATUS_OK) { p_state->Index += p_meta->Length; }

    return status;
}

/*! Stage one outbound chunk and advance. */
static inline uint16_t Protocol_DataMode_ProcReadChunk(Flash_T * p_app, Protocol_DataMode_State_T * p_state, Packet_Xfer_T * p_xfer, uint8_t * p_data)
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
static inline Protocol_ReqCode_T Protocol_DataMode_Read(Flash_T * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
{
    Flash_T * p_app = p_context;
    Protocol_DataMode_State_T * p_state = p_xfer->p_Substate;
    uint16_t status;

    p_state->StateId = Protocol_DataModeRead_StateOf(p_state, p_xfer->p_RxMeta);

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
static inline Protocol_ReqCode_T Protocol_DataMode_Write(Flash_T * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
{
    Flash_T * p_app = p_context;
    Protocol_DataMode_State_T * p_state = p_xfer->p_Substate;
    uint16_t status;

    p_state->StateId = Protocol_DataModeWrite_StateOf(p_app, p_state, p_xfer->p_RxMeta);

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

