#pragma once

/******************************************************************************/
/*!
    @file   Protocol_FlashLoader.h
    @author FireSourcery
    @brief  Flash bound to the generic DataMode transfer.

    A binding, not an implementation. The transfer logic - staging, cursor, chunking, status
    replies - lives once in Protocol_DataMode.h; this file only says what OPEN, READ and
    WRITE mean for a Flash_T. Register Protocol_DataMode_Read / Protocol_DataMode_Write in
    the request table and pass a PROTOCOL_FLASH_LOADER interface as the handler context.

    Nothing here is a handler, so there is no second copy of the staging to keep in step.

    Protocol_FlashLoader_Direct.h is the same loader written without the generic engine, for
    comparison - see the note at the foot of this file.
*/
/******************************************************************************/
#include "Protocol_DataMode.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

#include <string.h>

/******************************************************************************/
/*!
    Ops - Flash_T against Protocol_DataMode_Ops_T

    Flash_Status_T maps onto the generic status directly: NV_MEMORY_STATUS_SUCCESS is 0,
    which is PROTOCOL_DATA_MODE_STATUS_OK, and every other value is a fault the reply
    carries back verbatim.
*/
/******************************************************************************/
/*!
    Arm a write, or validate a read range.

    Config selects the operation the transfer is about to perform - a read needs no
    preparation, a write arms the controller's continue-write cursor. Arming on both is
    harmless: a read never calls WRITE, so the armed cursor is simply never advanced.
*/
static inline uint16_t Protocol_FlashLoader_Open(void * p_module, uintptr_t address, size_t size, uint32_t config)
{
    (void)config;
    return (uint16_t)Flash_SetContinueWrite((Flash_T *)p_module, address, size);
}

/*! Flash is memory mapped for reads, so a chunk is a copy. */
static inline uint16_t Protocol_FlashLoader_Read(void * p_module, uintptr_t address, size_t size, void * p_dest)
{
    (void)p_module;
    memcpy(p_dest, (const void *)address, size);
    return PROTOCOL_DATA_MODE_STATUS_OK;
}

/*!
    Writes continue from the cursor armed by Open.

    The address is therefore redundant here and is ignored rather than used - the controller
    owns the position, and passing it again would invite the two to disagree about where the
    next chunk belongs.
*/
static inline uint16_t Protocol_FlashLoader_Write(void * p_module, uintptr_t address, const void * p_src, size_t size)
{
    (void)address;
    return (uint16_t)Flash_ContinueWrite_Blocking((Flash_T *)p_module, p_src, size);
}

static const Protocol_DataMode_Ops_T PROTOCOL_FLASH_LOADER_OPS =
{
    .OPEN   = Protocol_FlashLoader_Open,
    .READ   = Protocol_FlashLoader_Read,
    .WRITE  = Protocol_FlashLoader_Write,
};

/******************************************************************************/
/*!
    Interface

    Cheap enough to build where it is needed - a pointer and four ids - which keeps the Flash
    instance a runtime value rather than forcing a file-scope constant into the integration
    layer.

    @param  p_Flash     the Flash_T this transfer moves bytes through
    @param  ReadId      packet id that opens a read
    @param  WriteId     packet id that opens a write
    @param  DataId      packet id carrying a raw chunk in either direction
    @param  ChunkMax    payload capacity of the format this socket speaks
*/
/******************************************************************************/
#define PROTOCOL_FLASH_LOADER(p_Flash, ReadId, WriteId, DataId, ChunkMax)    \
(Protocol_DataModeInterface_T)                                              \
{                                                                           \
    .P_OPS      = &PROTOCOL_FLASH_LOADER_OPS,                               \
    .P_MODULE   = (p_Flash),                                                \
    .READ_ID    = (packet_id_t)(ReadId),                                    \
    .WRITE_ID   = (packet_id_t)(WriteId),                                   \
    .DATA_ID    = (packet_id_t)(DataId),                                    \
    .CHUNK_MAX  = (packet_size_t)(ChunkMax),                                \
}

/******************************************************************************/
/*!
    Erase - a stateless request, not a transfer.

    Blocking, and can run to seconds on a large range, so it belongs on a socket whose
    REQ_TIMEOUT accommodates it or behind a handler that yields. Kept here rather than in the
    generic engine because erase has no cursor and no chunks - it is one call and one reply.
*/
/******************************************************************************/
static inline Protocol_ReqCode_T Protocol_FlashLoader_Erase_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, packet_id_t respId, const void * p_rxPayload, void * p_txPayload)
{
    const Protocol_DataMode_Req_T * p_req = p_rxPayload;

    if (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T))
    {
        ((Protocol_DataMode_Resp_T *)p_txPayload)->Status = PROTOCOL_DATA_MODE_STATUS_MALFORMED;
    }
    else
    {
        ((Protocol_DataMode_Resp_T *)p_txPayload)->Status = (uint16_t)Flash_Erase_Blocking(p_flash, (uintptr_t)p_req->Address, (size_t)p_req->Size);
    }

    p_xfer->p_TxMeta->Id     = respId;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
    return PROTOCOL_REQ_DONE;
}

/******************************************************************************/
/*
    Registration

        static Protocol_DataModeInterface_T FLASH_LOADER =
            PROTOCOL_FLASH_LOADER(&Flash, MOT_PACKET_DATA_MODE_READ, MOT_PACKET_DATA_MODE_WRITE,
                                  MOT_PACKET_DATA_MODE_DATA, MOT_PACKET_PAYLOAD_LENGTH_MAX);

        static const Protocol_Req_T REQ_TABLE[] =
        {
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,  &MOT_FRAME_DATA, Protocol_DataMode_Read,  PROTOCOL_ACK_ON_REQ),
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE, &MOT_FRAME_DATA, Protocol_DataMode_Write, PROTOCOL_ACK_ON_REQ),
        };

    MOT_PACKET_DATA_MODE_DATA needs no row of its own. A chunk arrives while the write is
    already bound, so Protocol_CaptureReqOfTable keeps the WRITE row standing and the chunk
    reaches the same handler - which is what lets one row absorb a whole stream.

    P_REQ_CONTEXT must be at least sizeof(Protocol_DataModeState_T), and P_APP_CONTEXT is the
    interface above.
*/
/******************************************************************************/

/******************************************************************************/
/*
    Why there are two versions

    Protocol_FlashLoader_Direct.h implements the same two transfers without the generic
    engine - the cursor, the staging and the pacing written out against Flash_T.

    The comparison is the point. The direct version is roughly the same length for ONE media
    type and reads more directly, because nothing is reached through an ops table. It stops
    paying for itself at the second media type: RAM, EEPROM, a once-programmable region and a
    remote peer each need the same cursor, the same chunking and the same three status
    replies, and in the direct form each one is another copy of the pacing to keep in step
    with the others.

    The generic version's cost is the indirection and one more type to read. Its return is
    that Protocol_DataMode_Read is written, argued about and tested once, and a new media
    type is three functions with no protocol in them.

    Take the direct one when the loader is the only bulk transfer the product will ever have.
*/
/******************************************************************************/
