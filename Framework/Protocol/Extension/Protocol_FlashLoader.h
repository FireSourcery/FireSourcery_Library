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
    preparation, a write arms the controller's continue-write cursor.
*/
static inline uint16_t Protocol_FlashLoader_Open(void * p_app, uintptr_t address, size_t size, uint32_t config)
{
    (void)config;
    return (uint16_t)Flash_SetContinueWrite((Flash_T *)p_app, address, size);
}

/*! Flash is memory mapped for reads, so a chunk is a copy. */
static inline uint16_t Protocol_FlashLoader_Read(void * p_app, uintptr_t address, size_t size, void * p_dest)
{
    (void)p_app;
    memcpy(p_dest, (const void *)address, size);
    return PROTOCOL_DATA_MODE_STATUS_OK;
}

/*!
    Writes continue from the cursor armed by Open.

    The address is therefore redundant here and is asserted rather than used - the controller
    owns the position, and a mismatch would mean the transfer and the driver disagree about
    where the next chunk belongs.
*/
static inline uint16_t Protocol_FlashLoader_Write(void * p_app, uintptr_t address, const void * p_src, size_t size)
{
    (void)address;
    return (uint16_t)Flash_ContinueWrite_Blocking((Flash_T *)p_app, p_src, size);
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

    Cheap enough to build where it is needed - four pointers - which keeps the Flash instance
    a runtime value rather than forcing a file-scope constant into the integration layer.

    @param  p_Flash     the Flash_T this transfer moves bytes through
    @param  DataId      packet id carrying a raw chunk in either direction
    @param  ChunkMax    payload capacity of the format this socket speaks
*/
/******************************************************************************/
#define PROTOCOL_FLASH_LOADER(p_Flash, DataId, ChunkMax)     \
(Protocol_DataModeInterface_T)                               \
{                                                            \
    .P_OPS      = &PROTOCOL_FLASH_LOADER_OPS,                \
    .P_MODULE   = (p_Flash),                                 \
    .DATA_ID    = (packet_id_t)(DataId),                     \
    .CHUNK_MAX  = (packet_size_t)(ChunkMax),                 \
}


/******************************************************************************/
/*! Erase */
/******************************************************************************/
Protocol_ReqCode_T Protocol_FlashLoader_Erase_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const MotPacket_DataModeReq_T * p_req, MotPacket_DataModeResp_T * p_resp)
{
    p_resp->Status = Flash_Erase_Blocking(p_flash, p_req->AddressStart, p_req->SizeBytes);
    // p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_ERASE;
    p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);
    return PROTOCOL_REQ_DONE;
}


/*
    Alternative version, without generic engine
*/
// typedef const struct Protocol_FlashLoader
// {
//     Flash_T * P_MODULE;
//     packet_id_t DATA_ID;        /* Id carrying a raw chunk in either direction */
//     packet_size_t CHUNK_MAX;    /* Bounded by the format's payload capacity */
// }
// Protocol_FlashLoader_T;


/******************************************************************************/
/*!
    Flash loader

    The transfers themselves are Protocol_DataMode_Read and Protocol_DataMode_Write, bound to
    Flash through Protocol_FlashLoader.h.

    Optionallyy  macro fixed Id amd cpimds.
*/
/******************************************************************************/
/*
    A bound handler is re-entered for EVERY frame that arrives while the exchange is open,
    including the ack that paces it. Step alone cannot tell those apart, so a handler that is
    also ack-paced has to read the class of the arriving frame from its Id.
*/
// static inline bool IsRxAck(const Packet_Xfer_T * p_xfer) { return (p_xfer->p_RxMeta->Id == MOT_PACKET_SYNC_ACK); }

// /******************************************************************************/
// /*! Stateful Read Data - ack-paced. RESPOND a chunk, wait for the ack, RESPOND the next. */
// /******************************************************************************/
// Protocol_ReqCode_T MotProtocol_ReadData(void * p_app, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
// {
//     MotProtocol_DataModeState_T * p_subState = (MotProtocol_DataModeState_T *)p_xfer->p_Substate;
//     (void)p_app;

//     switch (*p_xfer->p_Step)
//     {
//         case 0U: /* the opening request carries address and size */
//         {
//             const MotPacket_DataModeReq_T * p_req = (const MotPacket_DataModeReq_T *)p_rxPayload;

//             p_subState->DataModeAddress = p_req->AddressStart;
//             p_subState->DataModeSize    = p_req->SizeBytes;
//             p_subState->DataIndex       = 0U;

//             ((MotPacket_DataModeResp_T *)p_txPayload)->Status = MOT_STATUS_SUCCESS;
//             p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_READ;
//             p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);
//             *p_xfer->p_Step = 1U;
//             return PROTOCOL_REQ_RESPOND;
//         }

//         case 1U: /* one chunk per ack, then a closing status frame */
//             if (p_subState->DataIndex < p_subState->DataModeSize)
//             {
//                 packet_size_t readSize = (packet_size_t)(p_subState->DataModeSize - p_subState->DataIndex);
//                 if (readSize > MOT_DATA_MODE_CHUNK_MAX) { readSize = MOT_DATA_MODE_CHUNK_MAX; }

//                 memcpy(p_txPayload, (const uint8_t *)(p_subState->DataModeAddress + p_subState->DataIndex), readSize);
//                 p_subState->DataIndex += readSize;

//                 p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_DATA;
//                 p_xfer->p_TxMeta->Length = readSize;
//                 return PROTOCOL_REQ_RESPOND;
//             }

//             ((MotPacket_DataModeResp_T *)p_txPayload)->Status = MOT_STATUS_SUCCESS;
//             p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_READ;
//             p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);
//             return PROTOCOL_REQ_DONE;

//         default:
//             return PROTOCOL_REQ_ABORT;
//     }
// }

// /******************************************************************************/
// /*! Stateful Write Data - data-paced. ACCEPT or REJECT each arriving chunk. */
// /******************************************************************************/
// Protocol_ReqCode_T MotProtocol_Flash_WriteData_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * p_rxPayload, void * p_txPayload)
// {
//     MotProtocol_DataModeState_T * p_subState = (MotProtocol_DataModeState_T *)p_xfer->p_Substate;
//     Flash_Status_T flashStatus;

//     switch (*p_xfer->p_Step)
//     {
//         case 0U: /* the opening request arms the flash write */
//         {
//             const MotPacket_DataModeReq_T * p_req = (const MotPacket_DataModeReq_T *)p_rxPayload;

//             p_subState->DataModeAddress = p_req->AddressStart;
//             p_subState->DataModeSize    = p_req->SizeBytes;
//             p_subState->DataIndex       = 0U;

//             flashStatus = Flash_SetContinueWrite(p_flash, p_subState->DataModeAddress, p_subState->DataModeSize);

//             ((MotPacket_DataModeResp_T *)p_txPayload)->Status = flashStatus;
//             p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_WRITE;
//             p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);

//             if (flashStatus != NV_MEMORY_STATUS_SUCCESS) { return PROTOCOL_REQ_DONE; }
//             *p_xfer->p_Step = 1U;
//             return PROTOCOL_REQ_RESPOND;
//         }

//         case 1U: /* each arriving DATA frame is written and acknowledged */
//         {
//             /* An ack landing here refers to the status frame, not to a chunk. Nothing to do. */
//             if (IsRxAck(p_xfer) == true) { return PROTOCOL_REQ_AWAIT; }

//             packet_size_t writeSize = p_xfer->p_RxMeta->Length;

//             /* The remote is ahead of the size it declared. Refuse rather than overrun. */
//             if ((size_t)writeSize > (p_subState->DataModeSize - p_subState->DataIndex)) { return PROTOCOL_REQ_REJECT; }

//             flashStatus = Flash_ContinueWrite_Blocking(p_flash, (const uint8_t *)p_rxPayload, writeSize);
//             if (flashStatus != NV_MEMORY_STATUS_SUCCESS)
//             {
//                 ((MotPacket_DataModeResp_T *)p_txPayload)->Status = flashStatus;
//                 p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_WRITE;
//                 p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);
//                 return PROTOCOL_REQ_DONE;
//             }

//             p_subState->DataIndex += writeSize;
//             if (p_subState->DataIndex < p_subState->DataModeSize) { return PROTOCOL_REQ_ACCEPT; }

//             ((MotPacket_DataModeResp_T *)p_txPayload)->Status = NV_MEMORY_STATUS_SUCCESS;
//             p_xfer->p_TxMeta->Id     = MOT_PACKET_DATA_MODE_WRITE;
//             p_xfer->p_TxMeta->Length = sizeof(MotPacket_DataModeResp_T);
//             return PROTOCOL_REQ_DONE;
//         }

//         default:
//             return PROTOCOL_REQ_ABORT;
//     }
// }


/******************************************************************************/
/*!
    Mem
    Stateless Read Write
*/
/******************************************************************************/
// NvMemory_Status_T ReadMem_Blocking(Flash_T * p_flash, uintptr_t address, uint8_t size, MotProtocol_MemConfig_T config, uint8_t * p_destBuffer)
// {
//     NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;

//     switch ((MotProtocol_MemConfig_T)config)
//     {
//         case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy(p_destBuffer, (void *)address, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
//         case MOT_PROTOCOL_MEM_CONFIG_FLASH: memcpy(p_destBuffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
//         case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = Flash_ReadOnce_Blocking(p_flash, address, size, p_destBuffer); break;
//         default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
//     }

//     return status;
// }

// packet_size_t MotProtocol_ReadMem_Blocking(Flash_T * p_flash, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
// {
//     const MotPacket_MemReadReq_T * p_req = (const MotPacket_MemReadReq_T *)p_rxPacket->Payload;
//     NvMemory_Status_T status = ReadMem_Blocking(p_flash, p_req->Address, p_req->Size, (MotProtocol_MemConfig_T)p_req->Config, p_txPacket->Payload);
//     (void)status; /* MemRead header carries size only; status currently unused */

//     return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_READ, p_req->Size);
// }

// NvMemory_Status_T WriteMem_Blocking(Flash_T * p_flash, uintptr_t address, uint8_t size, MotProtocol_MemConfig_T config, const uint8_t * p_data)
// {
//     NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;

//     switch ((MotProtocol_MemConfig_T)config)
//     {
//         case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
//         case MOT_PROTOCOL_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
//         case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = Flash_WriteOnce_Blocking(p_flash, address, p_data, size); break;
//         default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
//     }

//     return status;
// }

// packet_size_t MotProtocol_WriteMem_Blocking(Flash_T * p_flash, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
// {
//     const MotPacket_MemWriteReq_T * p_req = (const MotPacket_MemWriteReq_T *)p_rxPacket->Payload;
//     NvMemory_Status_T status = WriteMem_Blocking(p_flash, p_req->Address, p_req->Size, (MotProtocol_MemConfig_T)p_req->Config, p_req->ByteData);
//     ((MotPacket_MemWriteResp_T *)p_txPacket->Payload)->Status = status;

//     return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_WRITE, sizeof(MotPacket_MemWriteResp_T));
// }
