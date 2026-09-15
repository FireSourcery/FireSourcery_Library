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
#define PROTOCOL_FLASH_LOADER(p_Flash, ReadId, WriteId, DataId, ChunkMax) (Protocol_DataMode_Interface_T) \
{                                                                           \
    .P_OPS      = &PROTOCOL_FLASH_LOADER_OPS,                               \
    .P_MODULE   = (p_Flash),                                                \
    .READ_ID    = (packet_id_t)(ReadId),                                    \
    .WRITE_ID   = (packet_id_t)(WriteId),                                   \
    .DATA_ID    = (packet_id_t)(DataId),                                    \
    .CHUNK_MAX  = (packet_size_t)(ChunkMax),                                \
}

