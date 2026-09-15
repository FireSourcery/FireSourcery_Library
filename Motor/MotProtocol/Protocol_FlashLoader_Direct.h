#pragma once

/******************************************************************************/
/*!
    @file   Protocol_FlashLoader_Direct.h
    @author FireSourcery
    @brief  The same flash loader, written without the generic DataMode engine.

    Two handlers against Flash_T directly - no ops table, no interface indirection. The
    cursor, the chunking and the status replies are written out here instead of being reached
    through Protocol_DataMode_Ops_T.

    Wire compatible with the generic version: same request and response payloads, same ids,
    same pass traces. Register one or the other, never both.

    See the foot of Protocol_FlashLoader.h for when each is the better choice.
*/
/******************************************************************************/
#include "Protocol_DataMode.h"
#include "../Protocol_Request.h"
#include "../Packet.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Math/math_general.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*!
    Wire payloads - identical to the generic engine's, so a host speaks to either.
*/
/******************************************************************************/
/*
    Protocol_DataMode_Req_T / _Resp_T come from Protocol_DataMode.h, which this file includes
    for the shared vocabulary - payloads, cursor, stage enum. What is NOT shared is the ops
    table: every media call below is a direct Flash_* call. Redefining the payloads here would
    collide with identical definitions rather than decouple anything.
*/

#define PROTOCOL_FLASH_LOADER_STATUS_OK         (0U)        /* == NV_MEMORY_STATUS_SUCCESS */
#define PROTOCOL_FLASH_LOADER_STATUS_MALFORMED  (0xE001U)   /* Opening request shorter than its payload type */
#define PROTOCOL_FLASH_LOADER_STATUS_OVERRUN    (0xE002U)   /* A chunk past the declared size */

/*
    Single default instance
*/
#ifndef PROTOCOL_FLASH_LOADER_READ_ID
#define PROTOCOL_FLASH_LOADER_READ_ID    (0xDAU)
#endif

#ifndef PROTOCOL_FLASH_LOADER_WRITE_ID
#define PROTOCOL_FLASH_LOADER_WRITE_ID   (0xDBU)
#endif

#ifndef PROTOCOL_FLASH_LOADER_DATA_ID
#define PROTOCOL_FLASH_LOADER_DATA_ID    (0xDDU)
#endif

#ifndef PROTOCOL_FLASH_LOADER_DATA_LENGTH
#define PROTOCOL_FLASH_LOADER_DATA_LENGTH (32U)
#endif

/******************************************************************************/
/*!
    Context and sub-state

    The context is what the ops table carried in the generic version, minus the ops: the
    media instance and the three ids that classify a pass.
*/
/******************************************************************************/
// typedef const struct Protocol_FlashLoader
// {
//     Flash_T * P_FLASH;
//     packet_id_t READ_ID;        /* Opens a read, and labels its status replies */
//     packet_id_t WRITE_ID;       /* Opens a write, and labels its status replies */
//     packet_id_t DATA_ID;        /* Carries a raw chunk in either direction */
//     packet_size_t CHUNK_MAX;    /* Bounded by the format's payload capacity */
// }
// Protocol_FlashLoader_T;

// #define PROTOCOL_FLASH_LOADER_DIRECT(p_Flash, ReadId, WriteId, DataId, ChunkMax)
// (Protocol_FlashLoader_T)
// {
//     .P_FLASH    = (p_Flash),
//     .READ_ID    = (packet_id_t)(ReadId),
//     .WRITE_ID   = (packet_id_t)(WriteId),
//     .DATA_ID    = (packet_id_t)(DataId),
//     .CHUNK_MAX  = (packet_size_t)(ChunkMax),
// }

/*!
    The cursor, held in the socket's P_SUB_STATE buffer.

    No step field, for the same reason the generic engine has none: P_SUB_STATE is never
    cleared between exchanges, so stored progress would start each transfer holding whatever
    the previous one left behind. The pass is derived from the arriving id and the cursor.
*/
// typedef struct Protocol_FlashLoader_State
// {
//     uintptr_t Address;
//     size_t Size;
//     size_t Index;
//     packet_id_t ReqId;      /* Echoed on the status replies */
//     uint16_t Status;
// }
// Protocol_DataMode_State_T;

/******************************************************************************/
/*!
    Shared steps
*/
/******************************************************************************/


static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Open(Flash_T * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    p_state->Address = p_req->Address;
    p_state->Size = p_req->Size;
    p_state->Count = 0U;

    p_resp->Status = Flash_SetContinueWrite(p_app, p_state->Address, p_state->Size);  // check bounds on read
    p_xfer->p_TxMeta->Id = p_xfer->p_RxMeta->Id;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    if (p_resp->Status != NV_MEMORY_STATUS_SUCCESS) { return PROTOCOL_REQ_DONE; }
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_DATA;
    return PROTOCOL_REQ_RESPOND;
}


/*! Nothing to absorb and nothing owed - the exchange stays open. */
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Idle(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_resp)
// {
//     (void)p_app; (void)p_xfer; (void)p_rxPayload; (void)p_resp;

//     return PROTOCOL_REQ_AWAIT;
// }


/******************************************************************************/
/*!
    Read - device streams flash to the host, paced by the host's acks.

    Flash is memory mapped for reads, so a chunk is a copy straight into the tx payload.
*/
/******************************************************************************/


/******************************************************************************/
/*! Stateful Read Data - ack-paced. RESPOND a chunk, wait for the ack, RESPOND the next. */
/******************************************************************************/
static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_ReadOpen(Flash_T * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);
    (void)p_app;    /* a read needs no preparation - flash is memory mapped */

    p_state->Address = (uintptr_t)p_req->Address;
    p_state->Size = (size_t)p_req->Size;
    p_state->Count = 0U;
    // p_state->ReqId = p_xfer->p_RxMeta->Id;
    p_resp->Status = NV_MEMORY_STATUS_SUCCESS;
    p_xfer->p_TxMeta->Id = PROTOCOL_FLASH_LOADER_READ_ID;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    p_state->StateId = PROTOCOL_DATA_MODE_STATE_DATA;
    return PROTOCOL_REQ_RESPOND;
}


/*!
    Read, continuing - stage the next chunk.

    The tx payload is a raw chunk here, not a status, which is why it is typed uint8_t * and
    the id is set by _ReadChunk rather than by _Reply. A fault swaps the frame for a status
    and closes, so the remote learns why the stream stopped.
*/
static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_ReadData(void * p_flash, Packet_Xfer_T * p_xfer, const void * p_rx, uint8_t * p_chunk)
{
    (void)p_flash; (void)p_rx;
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    if (p_state->Count < p_state->Size)
    {
        packet_size_t readSize = math_min(p_state->Size - p_state->Count, PROTOCOL_FLASH_LOADER_DATA_LENGTH);
        memcpy(p_chunk, (const uint8_t *)(p_state->Address + p_state->Count), readSize);
        p_state->Count += readSize;

        p_xfer->p_TxMeta->Id = PROTOCOL_FLASH_LOADER_DATA_ID;
        p_xfer->p_TxMeta->Length = readSize;
        return PROTOCOL_REQ_RESPOND;
    }

    ((Protocol_DataMode_Resp_T *)p_chunk)->Status = NV_MEMORY_STATUS_SUCCESS;
    p_xfer->p_TxMeta->Id = PROTOCOL_FLASH_LOADER_READ_ID;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
    return PROTOCOL_REQ_DONE;
}


/*!
    Read - device streams memory to the host.

    Paced by the host's acks: every ack frees the floor and pulls the next chunk. Register
    with PROTOCOL_ACK_ON_REQ so each chunk is acked and the next is pulled by that ack.
*/
static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Read(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    if (p_xfer->p_RxMeta->Id == PROTOCOL_FLASH_LOADER_READ_ID) { p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN; }

    switch (p_state->StateId)
    {
        case PROTOCOL_DATA_MODE_STATE_OPEN:     return Protocol_FlashLoaderDirect_ReadOpen(p_flash, p_xfer, p_rxPayload, p_resp);
        case PROTOCOL_DATA_MODE_STATE_DATA:     return Protocol_FlashLoaderDirect_ReadData(p_flash, p_xfer, p_rxPayload, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_CLOSE:    return Protocol_DataMode_ReadClose(p_flash, p_xfer, p_rxPayload, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_ERROR:    return Protocol_DataMode_Malformed(p_flash, p_xfer, p_rxPayload, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_IDLE:
        // default:                                return Protocol_DataMode_Idle     (p_flash, p_xfer, p_rxPayload, p_resp);
        default:
            p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
            return PROTOCOL_REQ_ABORT;
    }
}

/******************************************************************************/
/*!
    Write - host streams to flash, paced by its own data frames.

    Only the opening request and the last chunk earn a reply; the rest are answered by
    ACCEPT, which is the engine's ack and nothing more.
*/
/******************************************************************************/

/******************************************************************************/
/*! Stateful Write Data - data-paced. ACCEPT or REJECT each arriving chunk. */
/******************************************************************************/
static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_WriteOpen(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    p_state->Address = p_req->Address;
    p_state->Size = p_req->Size;
    p_state->Count = 0U;

    p_resp->Status = Flash_SetContinueWrite(p_flash, p_state->Address, p_state->Size);
    p_xfer->p_TxMeta->Id = PROTOCOL_FLASH_LOADER_WRITE_ID;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    if (p_resp->Status != NV_MEMORY_STATUS_SUCCESS) { return PROTOCOL_REQ_DONE; }
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_DATA;
    return PROTOCOL_REQ_RESPOND;

}

/*!
    Write, continuing - absorb one chunk, silently.

    ACCEPT is the silent path: the engine's ack is the whole reply, so nothing is staged.
    A media fault reports and ends.
*/
static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_WriteData(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const uint8_t * p_data, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);
    packet_size_t writeSize = p_xfer->p_RxMeta->Length;

     /* todo reume does no run handler */
    // if (IsRxAck(p_xfer) == true) { return PROTOCOL_REQ_AWAIT; }

    /* The remote is ahead of the size it declared. Refuse rather than overrun. */
    if ((size_t)writeSize > (p_state->Size - p_state->Count)) { return PROTOCOL_REQ_REJECT; }

    p_resp->Status = Flash_ContinueWrite_Blocking(p_flash, (const uint8_t *)p_data, writeSize);

    if (p_resp->Status != NV_MEMORY_STATUS_SUCCESS)
    {
        p_xfer->p_TxMeta->Id = PROTOCOL_FLASH_LOADER_WRITE_ID;
        p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
        return PROTOCOL_REQ_DONE;   /* ABORT would discard the status just staged */
    }

    p_state->Count += writeSize;
    if (p_state->Count < p_state->Size) { return PROTOCOL_REQ_ACCEPT; } /* TxLength == 0 */
    // p_state->StateId = PROTOCOL_DATA_MODE_STATE_CLOSE; return PROTOCOL_REQ_ACCEPT;

    /* If we reach here, the write is complete and successful. */
    p_resp->Status = NV_MEMORY_STATUS_SUCCESS;
    p_xfer->p_TxMeta->Id = PROTOCOL_FLASH_LOADER_WRITE_ID;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
    return PROTOCOL_REQ_DONE;
}

/*!
    Write - host streams memory to the device.

    Paced by the host's data packets. Only the opening request and the last chunk earn a
    reply; the rest are answered by ACCEPT, which is the engine's ack and nothing more.
*/
static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Write(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    if (p_xfer->p_RxMeta->Id == PROTOCOL_FLASH_LOADER_WRITE_ID) { p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN; }
    // else if(p_xfer->p_RxMeta->Id == PROTOCOL_FLASH_LOADER_DATA_ID) { p_state->StateId = PROTOCOL_DATA_MODE_STATE_DATA; }
    // else { p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN; return PROTOCOL_REQ_ABORT; }

    switch (p_state->StateId)
    {
        case PROTOCOL_DATA_MODE_STATE_OPEN:     return Protocol_FlashLoaderDirect_WriteOpen(p_flash, p_xfer, p_rxPayload, p_resp);
        case PROTOCOL_DATA_MODE_STATE_DATA:     return Protocol_FlashLoaderDirect_WriteData(p_flash, p_xfer, p_rxPayload, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_CLOSE:    return Protocol_DataMode_WriteClose(p_flash, p_xfer, p_rxPayload, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_ERROR:    return Protocol_DataMode_Overrun   (p_flash, p_xfer, p_rxPayload, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_IDLE:
        // default:                                return Protocol_DataMode_Idle      (p_flash, p_xfer, p_rxPayload, p_resp);
        default:
            p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
            return PROTOCOL_REQ_ABORT;
    }
}

/******************************************************************************/
/*
    Registration

        static Protocol_FlashLoader_T FLASH_LOADER =
            PROTOCOL_FLASH_LOADER_DIRECT(&Flash, MOT_PACKET_DATA_MODE_READ, MOT_PACKET_DATA_MODE_WRITE,
                                         MOT_PACKET_DATA_MODE_DATA, MOT_PACKET_PAYLOAD_LENGTH_MAX);

        static const Protocol_Req_T REQ_TABLE[] =
        {
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,  &MOT_FRAME_DATA, Protocol_FlashLoaderDirect_Read,  PROTOCOL_ACK_ON_REQ),
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE, &MOT_FRAME_DATA, Protocol_FlashLoaderDirect_Write, PROTOCOL_ACK_ON_REQ),
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_ERASE, &MOT_FRAME_DATA, Protocol_FlashLoaderDirect_Erase_Blocking, PROTOCOL_ACK_NONE),
        };

    P_SUB_STATE must be at least sizeof(Protocol_DataMode_State_T), and P_APP_CONTEXT is
    the interface above.
*/
/******************************************************************************/

/*! Stage a status reply. A non-OK status always closes the exchange. */
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Reply( Packet_Meta_T * p_txMeta, Protocol_DataMode_Resp_T * p_resp, packet_id_t respId, uint16_t status, Protocol_ReqCode_T onOk)
// {
//     ((Protocol_DataMode_Resp_T *)p_resp)->Status = status;
//     p_txMeta->Id     = respId;
//     p_txMeta->Length = sizeof(Protocol_DataMode_Resp_T);
//     return (status == PROTOCOL_FLASH_LOADER_STATUS_OK) ? onOk : PROTOCOL_REQ_DONE;
// }

// /*!
//     Rewrite the cursor whole and arm the flash controller.

//     Both directions arm the continue-write cursor. A read never advances it, so arming on a
//     read costs nothing and keeps the opening path single.
// */
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Open(Flash_T * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     p_state->Address = (uintptr_t)p_req->Address;
//     p_state->Size    = (size_t)p_req->Size;
//     p_state->Count   = 0U;
//     // p_state->ReqId   = p_xfer->p_RxMeta->Id;
//     p_state->Status  = (uint16_t)Flash_SetContinueWrite(p_app, p_state->Address, p_state->Size); // check bounds on read

//     /* Nothing to stream: the opening status is also the closing one. */
//     return Protocol_FlashLoaderDirect_Reply(p_xfer->p_TxMeta, p_resp, p_xfer->p_RxMeta->Id, p_state->Status, (p_state->Size == 0U) ? PROTOCOL_REQ_DONE : PROTOCOL_REQ_RESPOND);
// }

// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Read(void * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_resp)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     /* The opening request. Anything else is a pacing ack - the floor is free, send the next chunk. */
//     if (p_xfer->p_RxMeta->Id == PROTOCOL_FLASH_LOADER_READ_ID)
//     {
//         return Protocol_FlashLoaderDirect_Open((Flash_T *)p_context, p_xfer, (const Protocol_DataMode_Req_T *)p_rxPayload, (Protocol_DataMode_Resp_T *)p_resp);
//     }

//     if (p_state->Count >= p_state->Size)
//     {
//         return Protocol_FlashLoaderDirect_Reply(p_xfer->p_TxMeta, p_resp, p_xfer->p_RxMeta->Id, PROTOCOL_FLASH_LOADER_STATUS_OK, PROTOCOL_REQ_DONE);
//     }

//     packet_size_t chunk = (packet_size_t)math_min(p_state->Size - p_state->Count, PROTOCOL_FLASH_LOADER_DATA_LENGTH);

//     memcpy(p_resp, (const void *)(p_state->Address + p_state->Count), chunk);
//     p_state->Count += chunk;

//     p_xfer->p_TxMeta->Id     = PROTOCOL_FLASH_LOADER_DATA_ID;
//     p_xfer->p_TxMeta->Length = chunk;
//     return PROTOCOL_REQ_RESPOND;
// }

// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Write(Flash_T * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_resp)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     if (p_xfer->p_RxMeta->Id == PROTOCOL_FLASH_LOADER_WRITE_ID)
//     {
//         return Protocol_FlashLoaderDirect_Open(p_context, p_xfer, (const Protocol_DataMode_Req_T *)p_rxPayload, (Protocol_DataMode_Resp_T *)p_resp);
//     }

//     /* The ack of the opening reply. Nothing to absorb. */
//     if (p_xfer->p_RxMeta->Id != PROTOCOL_FLASH_LOADER_DATA_ID) { return PROTOCOL_REQ_AWAIT; }

//     /* The remote is ahead of the size it declared. Report rather than overrun. */
//     if ((size_t)p_xfer->p_RxMeta->Length > (p_state->Size - p_state->Count))
//     {
//         return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_resp, PROTOCOL_FLASH_LOADER_STATUS_OVERRUN, PROTOCOL_REQ_DONE);
//     }

//     /* Writes continue from the cursor armed by Open, so the address is the controller's to track. */
//     p_state->Status = (uint16_t)Flash_ContinueWrite_Blocking(((Flash_T *)p_context), p_rxPayload, p_xfer->p_RxMeta->Length);

//     if (p_state->Status != PROTOCOL_FLASH_LOADER_STATUS_OK)
//     {
//         return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_resp, p_state->Status, PROTOCOL_REQ_DONE);
//     }

//     p_state->Count += p_xfer->p_RxMeta->Length;

//     /* Silent while the cursor lasts; the last chunk carries the transfer's outcome back. */
//     return (p_state->Count < p_state->Size)
//          ? PROTOCOL_REQ_ACCEPT
//          : Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_resp, PROTOCOL_FLASH_LOADER_STATUS_OK, PROTOCOL_REQ_DONE);
// }
