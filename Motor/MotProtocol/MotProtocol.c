/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotPacket.c
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "MotProtocol.h"
#include "MotPacket.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

#include <stddef.h>
#include <string.h>



/******************************************************************************/
/*!
    Packet Interface
    mapping directly to Protocol Specs
*/
/******************************************************************************/

// depreciate
Protocol_RxCode_T MotProtocol_ParseRxMeta(const MotPacket_T * p_rxPacket, packet_size_t rxCount, Protocol_HeaderMeta_T * p_rxMeta)
{
    Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_AWAIT_PACKET;

    /* Called after rxCount > MIN, rxCount != 0 */
    if(rxCount == p_rxMeta->Length) /* Packet Complete */
    {
        rxCode = (MotPacket_Checksum(p_rxPacket, p_rxMeta->Length) == p_rxPacket->Header.Checksum) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_ERROR_DATA;
    }
    else if(rxCount > (MOT_PACKET_LENGTH_MIN - 1U)) /* Length Field is valid */
    {
        p_rxMeta->Length = MotPacket_ParseTotalLength(p_rxPacket);

// else if(rxCount > MOT_PACKET_ID_INDEX)
// {

        p_rxMeta->Id = p_rxPacket->Header.Id;
        switch(p_rxPacket->Header.Id)
        {
            /* complete */
            case MOT_PACKET_SYNC_ACK:   rxCode = PROTOCOL_RX_CODE_ACK;      break;
            case MOT_PACKET_SYNC_NACK:  rxCode = PROTOCOL_RX_CODE_NACK;     break;
            case MOT_PACKET_SYNC_ABORT: rxCode = PROTOCOL_RX_CODE_ABORT;    break;
            case MOT_PACKET_PING:       rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE;  break;
            case MOT_PACKET_PING_BOOT:  rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE;  break;
            case MOT_PACKET_PING_ALT:   rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE;  break;

            /* await */
            default: break;
        }
    }
    else /* (rxCount < MOT_PACKET_LENGTH_MIN) ParseMeta should not have been called */
    {
        rxCode = PROTOCOL_RX_CODE_ERROR_META;
    }

    return rxCode;
}

/* todo */
Protocol_RxCode_T MotProtocol_ParseRxControl(const MotPacket_T * p_rxPacket, packet_size_t rxCount, Protocol_HeaderMeta_T * p_rxMeta)
{
    assert(rxCount > (MOT_PACKET_LENGTH_MIN - 1U)); /* Length Field is valid */ /* (rxCount < MOT_PACKET_LENGTH_MIN) ParseMeta should not have been called */

    p_rxMeta->Id = p_rxPacket->Header.Id;

    switch (p_rxPacket->Header.Id)
    {
        // Sync packets — complete immediately, no checksum verification needed
        case MOT_PACKET_SYNC_ACK:   return PROTOCOL_RX_CODE_ACK;
        case MOT_PACKET_SYNC_NACK:  return PROTOCOL_RX_CODE_NACK;
        case MOT_PACKET_SYNC_ABORT: return PROTOCOL_RX_CODE_ABORT;
        case MOT_PACKET_PING:       return PROTOCOL_RX_CODE_PACKET_COMPLETE;
        case MOT_PACKET_PING_BOOT:  return PROTOCOL_RX_CODE_PACKET_COMPLETE;
        case MOT_PACKET_PING_ALT:   return PROTOCOL_RX_CODE_PACKET_COMPLETE;

            /* fixed length, directly map id */
            // case MOT_PACKET_STOP_ALL:      p_rxMeta->Length = sizeof(MotPacket_StopReq_T);     break;
            // case MOT_PACKET_VERSION:       p_rxMeta->Length = sizeof(MotPacket_VersionReq_T);  break;
            // case MOT_PACKET_REBOOT:        p_rxMeta->Length = sizeof(MotPacket_CallReq_T);     break;
            // case MOT_PACKET_CALL:          p_rxMeta->Length = sizeof(MotPacket_CallReq_T);     break;
            // case MOT_PACKET_FIXED_VAR_READ:  p_rxMeta->Length = sizeof( ); break;
            // case MOT_PACKET_FIXED_VAR_WRITE:  p_rxMeta->Length = sizeof( ); break;
            // default: rxCode = PROTOCOL_RX_CODE_ERROR_META; break;

        // Data packets — set length, await remaining bytes
        default:
            p_rxMeta->Length = MotPacket_ParseTotalLength(p_rxPacket);
            return PROTOCOL_RX_CODE_AWAIT_PACKET;
    }
}

Protocol_RxCode_T MotProtocol_ParseRxComplete(const MotPacket_T * p_rxPacket, Protocol_HeaderMeta_T * p_rxMeta)
{
    assert(p_rxMeta->Length > MotPacket_ParseTotalLength(p_rxPacket));
    // return MotPacket_ProcChecksum(p_rxPacket, p_rxMeta->Length) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_ERROR_DATA;
    return (MotPacket_Checksum(p_rxPacket, p_rxMeta->Length) == p_rxPacket->Header.Checksum) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_ERROR_DATA;
    // p_rxMeta.Sequence = p_rxPacket->Header.Sequence; e.g.
}

/* passes totalLength */
void MotProtocol_BuildTxHeader(MotPacket_T * p_packet, const Protocol_HeaderMeta_T * p_meta)
{
    p_packet->Header.Start = MOT_PACKET_START_BYTE;
    p_packet->Header.Id = p_meta->Id;
    p_packet->Header.Length = p_meta->Length;
    p_packet->Header.Sequence = 0U;
    p_packet->Header.Flags = 0U;
    // p_packet->Header.Checksum = Packet_Checksum((const uint8_t *)p_packet, p_meta->Length, offsetof(MotPacket_Header_T, Checksum), sizeof(checksum_t));
    p_packet->Header.Checksum = MotPacket_Checksum(p_packet, p_meta->Length);
}

packet_size_t MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, Protocol_TxSyncId_T txId)
{
    MotPacket_Id_T syncChar;

    switch (txId)
    {
        case PROTOCOL_TX_SYNC_ACK_REQ:          syncChar = MOT_PACKET_SYNC_ACK;   break;
        case PROTOCOL_TX_SYNC_ACK_REQ_EXT:      syncChar = MOT_PACKET_SYNC_ACK;   break;
        case PROTOCOL_TX_SYNC_NACK_REQ:         syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_PACKET_META: syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_PACKET_DATA: syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT: syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT:  syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_NACK_REQ_EXT:     syncChar = MOT_PACKET_SYNC_NACK;  break;
        case PROTOCOL_TX_SYNC_ACK_ABORT:        syncChar = MOT_PACKET_SYNC_ABORT; break;
        default: syncChar = 0U; break;
    }

    return MotPacket_Sync_Build(p_txPacket, syncChar);
}

const Packet_Format_T MOT_PROTOCOL_PACKET_CLASS =
{
    .RX_LENGTH_MIN  = MOT_PACKET_LENGTH_MIN,
    .RX_LENGTH_MAX  = MOT_PACKET_LENGTH_MAX,
    .PARSE_RX_FRAMING   = (Packet_ParseRxFraming_T)MotProtocol_ParseRxMeta,
    // .PARSE_RX_FRAMING = (Packet_ParseRxFraming_T)MotProtocol_ParseRxControl,
    .PARSE_RX_HEADER    = (Packet_ParseRxComplete_T)MotProtocol_ParseRxComplete,  /* resv unused */
    .BUILD_TX_HEADER    = (Packet_BuildTxHeader_T)MotProtocol_BuildTxHeader, /* resv unused */
    .BUILD_TX_SYNC      = (Packet_BuildTxSync_T)MotProtocol_BuildTxSync,
    .RX_START_ID    = MOT_PACKET_START_BYTE,
    .RX_TIMEOUT     = MOT_PROTOCOL_TIMEOUT_RX,
    //
    .REQ_TIMEOUT    = MOT_PROTOCOL_TIMEOUT_REQ,
};


/******************************************************************************/
/*!
    Flash loader
*/
/******************************************************************************/
/******************************************************************************/
/*! Stateful Read Data */
/******************************************************************************/
Protocol_ReqCode_T MotProtocol_DataModeReadInit(void * p_app, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_DataModeReq_T * p_req = (const MotPacket_DataModeReq_T *)((const MotPacket_T *)p_reqContext->p_RxPacket)->Payload;
    Protocol_ReqCode_T reqCode;

    p_subState->DataModeAddress = p_req->AddressStart;
    p_subState->DataModeSize    = p_req->SizeBytes;
    p_subState->DataIndex = 0U;
    MotPacket_T * p_txPacket = p_reqContext->p_TxPacket;
    ((MotPacket_DataModeResp_T *)p_txPacket->Payload)->Status = MOT_STATUS_SUCCESS;
    *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_READ, sizeof(MotPacket_DataModeResp_T));
    *p_reqContext->p_SubStateIndex = 1U;
    reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE; // after receiving ack, control is transferred back to MotProtocol_DataModeReadData

    return reqCode;
}

Protocol_ReqCode_T MotProtocol_DataModeReadData(void * p_app, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    MotPacket_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    uint16_t readSize;

    /* Passing control for Tx, RxPacket is not valid during this time */
    if(p_subState->DataIndex < p_subState->DataModeSize)
    {
        readSize = (p_subState->DataModeSize - p_subState->DataIndex);
        if(readSize > 32U) readSize = 32U;
        memcpy(p_txPacket->Payload, (const uint8_t *)(p_subState->DataModeAddress + p_subState->DataIndex), readSize);
        *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_DATA, readSize);
        p_subState->DataIndex += readSize;
        reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;
    }
    else
    {
        ((MotPacket_DataModeResp_T *)p_txPacket->Payload)->Status = MOT_STATUS_SUCCESS;
        *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_READ, sizeof(MotPacket_DataModeResp_T));
        *p_reqContext->p_SubStateIndex = 2U;
        reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;
    }

    return reqCode;
}

Protocol_ReqCode_T MotProtocol_ReadData(void * p_app, Protocol_ReqContext_T * p_reqContext)
{
    Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;

    switch(*p_reqContext->p_SubStateIndex)
    {
        case 0U: /* Tx Ack handled by common Sync */ //MOT_PROTOCOL_DATA_MODE_INACTIVE
            reqCode = MotProtocol_DataModeReadInit(p_app, p_reqContext);
            break;
        case 1U: /* Tx Data */ /* todo on Nack */
            reqCode = MotProtocol_DataModeReadData(p_app, p_reqContext);
            break;
        case 2U:
            *p_reqContext->p_SubStateIndex = 0U;
            reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
            break;
        default:
            *p_reqContext->p_SubStateIndex = 0U;
            reqCode = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;
            break;
    }

    return reqCode;
}


/******************************************************************************/
/*! Memory common, non application state based */
/******************************************************************************/
/******************************************************************************/
/*! Stateful Write Data */
/******************************************************************************/
/*
    Response status type NvMemory_Status_T
*/
// Protocol_ReqResult_T MotProtocol_Flash_DataModeWriteInit_Blocking(Flash_T * p_flash, MotProtocol_DataModeState_T * p_subState, const MotPacket_T * p_rxPacket, MotPacket_T * p_txPacket )
Protocol_ReqCode_T MotProtocol_Flash_DataModeWriteInit_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_DataModeReq_T * p_req = (const MotPacket_DataModeReq_T *)((const MotPacket_T *)p_reqContext->p_RxPacket)->Payload;
    Protocol_ReqCode_T reqCode;
    Flash_Status_T flashStatus = NV_MEMORY_STATUS_SUCCESS;

    p_subState->DataModeAddress = p_req->AddressStart;
    p_subState->DataModeSize    = p_req->SizeBytes;
    p_subState->DataIndex = 0U;

    if(p_req->Config == MOT_PROTOCOL_DATA_MODE_CONFIG_ERASE)
    {
        flashStatus = Flash_Erase_Blocking(p_flash, p_subState->DataModeAddress, p_subState->DataModeSize);
    }

    if(flashStatus == NV_MEMORY_STATUS_SUCCESS)
    {
        flashStatus = Flash_SetContinueWrite(p_flash, p_subState->DataModeAddress, p_subState->DataModeSize);
    }

    MotPacket_T * p_txPacket = p_reqContext->p_TxPacket;
    ((MotPacket_DataModeResp_T *)p_txPacket->Payload)->Status = flashStatus;
    *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_WRITE, sizeof(MotPacket_DataModeResp_T));
    *p_reqContext->p_SubStateIndex = 1U;
    reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;
    // reqCode = PROTOCOL_REQ_CODE_TX_AWAIT_RX_CONTINUE; /* Wait for DataPacket, control is transferred to MotProtocol_Flash_DataModeWriteData */

    return reqCode;
}

Protocol_ReqCode_T MotProtocol_Flash_DataModeWriteData_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_T * p_rxPacket = p_reqContext->p_RxPacket;
    MotPacket_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    Flash_Status_T flashStatus;
    const uint8_t * p_sourceData; /* DataPacket Payload */
    uint8_t writeSize; /* DataPacket Size */

    p_sourceData = p_rxPacket->Payload;
    writeSize = MotPacket_ParsePayloadLength(p_rxPacket);
    flashStatus = Flash_ContinueWrite_Blocking(p_flash, p_sourceData, writeSize);

    if(flashStatus == NV_MEMORY_STATUS_SUCCESS)
    {
        p_subState->DataIndex += writeSize;
        if(p_subState->DataIndex < p_subState->DataModeSize) /* p_flash->OpAddress < p_subState->DataModeAddress + p_subState->DataModeSize */
        {
            *p_reqContext->p_TxSize = 0U; /* Tx Ack already handled on reception */
            reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE;
        }
        else
        {
            ((MotPacket_DataModeResp_T *)p_txPacket->Payload)->Status = NV_MEMORY_STATUS_SUCCESS;
            *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_WRITE, sizeof(MotPacket_DataModeResp_T));
            *p_reqContext->p_SubStateIndex = 3U;
            reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;
        }
    }
    else /* Error */
    {
        ((MotPacket_DataModeResp_T *)p_txPacket->Payload)->Status = flashStatus;
        *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_WRITE, sizeof(MotPacket_DataModeResp_T));
        *p_reqContext->p_SubStateIndex = 0U;
        reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
    }

    return reqCode;
}

/*
    Protocol only maintains 1 App Context Pointer.
    Caller pass flash controller context within App Context
*/
Protocol_ReqCode_T MotProtocol_Flash_WriteData_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    Protocol_ReqCode_T reqCode;

    switch(*p_reqContext->p_SubStateIndex)
    {
        case 0U: /* Tx Ack handled by Common Req Sync */
            reqCode = MotProtocol_Flash_DataModeWriteInit_Blocking(p_flash, p_reqContext);
            break;
        case 1U: /* No Tx. begin by waiting */
            *p_reqContext->p_TxSize = 0U;
            *p_reqContext->p_SubStateIndex = 2U;
            reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE;
            break;
        //todo reduce a state
        case 2U: /* Write Data - rxPacket is DataPacket */
            /* assert(id == MOT_PACKET_DATA_MODE_DATA) */
            reqCode = MotProtocol_Flash_DataModeWriteData_Blocking(p_flash, p_reqContext);
            // if (reqCode == PROTOCOL_REQ_CODE_TX_CONTINUE) *p_reqContext->p_SubStateIndex = 3 ;
            //     (reqCode == PROTOCOL_REQ_CODE_PROCESS_COMPLETE) *p_reqContext->p_SubStateIndex = 0 ;
            break;
        case 3U:
            *p_reqContext->p_SubStateIndex = 0U;
            reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
            break;
        default:
            *p_reqContext->p_SubStateIndex = 0U;
            reqCode = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;
            break;
    }

    return reqCode;
}

/******************************************************************************/
/*! Erase */
/******************************************************************************/
Protocol_ReqCode_T MotProtocol_Flash_Erase_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    const MotPacket_DataModeReq_T * p_req = (const MotPacket_DataModeReq_T *)((const MotPacket_T *)p_reqContext->p_RxPacket)->Payload;
    Flash_Status_T flashStatus;

    // MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    // p_subState->DataModeAddress = p_req->AddressStart;
    // p_subState->DataModeSize = p_req->SizeBytes;
    // p_subState->DataIndex = 0U;

    flashStatus = Flash_Erase_Blocking(p_flash, p_req->AddressStart, p_req->SizeBytes);

    MotPacket_T * p_txPacket = p_reqContext->p_TxPacket;
    ((MotPacket_DataModeResp_T *)p_txPacket->Payload)->Status = flashStatus;
    *p_reqContext->p_TxSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_DATA_MODE_ERASE, sizeof(MotPacket_DataModeResp_T));
    return PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
}


/******************************************************************************/
/*!
    Mem
*/
/******************************************************************************/
NvMemory_Status_T ReadMem_Blocking(Flash_T * p_flash, uintptr_t address, uint8_t size, MotProtocol_MemConfig_T config, uint8_t * p_destBuffer)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;

    switch ((MotProtocol_MemConfig_T)config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy(p_destBuffer, (void *)address, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_FLASH: memcpy(p_destBuffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = Flash_ReadOnce_Blocking(p_flash, address, size, p_destBuffer); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return status;
}

packet_size_t MotProtocol_ReadMem_Blocking(Flash_T * p_flash, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemReadReq_T * p_req = (const MotPacket_MemReadReq_T *)p_rxPacket->Payload;
    NvMemory_Status_T status = ReadMem_Blocking(p_flash, p_req->Address, p_req->Size, (MotProtocol_MemConfig_T)p_req->Config, p_txPacket->Payload);
    (void)status; /* MemRead header carries size only; status currently unused */

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_READ, p_req->Size);
}

NvMemory_Status_T WriteMem_Blocking(Flash_T * p_flash, uintptr_t address, uint8_t size, MotProtocol_MemConfig_T config, const uint8_t * p_data)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_ERROR_OTHER;

    switch ((MotProtocol_MemConfig_T)config)
    {
        case MOT_PROTOCOL_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_PROTOCOL_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
        case MOT_PROTOCOL_MEM_CONFIG_ONCE: status = Flash_WriteOnce_Blocking(p_flash, address, p_data, size); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return status;
}

packet_size_t MotProtocol_WriteMem_Blocking(Flash_T * p_flash, MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
{
    const MotPacket_MemWriteReq_T * p_req = (const MotPacket_MemWriteReq_T *)p_rxPacket->Payload;
    NvMemory_Status_T status = WriteMem_Blocking(p_flash, p_req->Address, p_req->Size, (MotProtocol_MemConfig_T)p_req->Config, p_req->ByteData);
    ((MotPacket_MemWriteResp_T *)p_txPacket->Payload)->Status = status;

    return MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MEM_WRITE, sizeof(MotPacket_MemWriteResp_T));
}


