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
    @version V0
*/
/******************************************************************************/
#include "MotProtocol.h"
#include "MotPacket.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*!
    Common functions, p_appContext independent
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Packet Interface
    mapping directly to Protocol Specs
*/
/******************************************************************************/
void MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, protocol_size_t * p_txSize, Protocol_TxSyncId_T txId)
{
    MotPacket_Id_T syncChar;

    switch(txId)
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
        // case PROTOCOL_TX_SYNC_ABORT:         syncChar = MOT_PACKET_SYNC_ABORT; break;
        default: *p_txSize = 0U; syncChar = 0U; break;
    }

    *p_txSize = MotPacket_Sync_Build(p_txPacket, syncChar);
}


Protocol_RxCode_T MotProtocol_ParseRxMeta(Protocol_HeaderMeta_T * p_rxMeta, const MotPacket_T * p_rxPacket, protocol_size_t rxCount)
{
     Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_AWAIT_PACKET;

    /* Called after rxCount > MIN, rxCount != 0 */
    if(rxCount == p_rxMeta->Length) /* Packet Complete */
    {
        rxCode = (MotPacket_ProcChecksum(p_rxPacket, rxCount) == true) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_ERROR_DATA;
    }
    else if(rxCount > MOT_PACKET_LENGTH_INDEX) /* Length Field is valid */
    {
        p_rxMeta->Length = MotPacket_GetTotalLength(p_rxPacket);
    }
    else if(rxCount > MOT_PACKET_ID_INDEX)
    {
        p_rxMeta->ReqId = p_rxPacket->Header.Id;
        switch(p_rxPacket->Header.Id)
        {
            /* complete */
            case MOT_PACKET_SYNC_ACK:   rxCode = PROTOCOL_RX_CODE_ACK;      break;
            case MOT_PACKET_SYNC_NACK:  rxCode = PROTOCOL_RX_CODE_NACK;     break;
            case MOT_PACKET_SYNC_ABORT: rxCode = PROTOCOL_RX_CODE_ABORT;    break;
            case MOT_PACKET_PING:       rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE;  break;
            case MOT_PACKET_PING_ALT:   rxCode = PROTOCOL_RX_CODE_PACKET_COMPLETE;  break;

            /* await */
            // case MOT_PACKET_STOP_ALL:      p_rxMeta->Length = sizeof(MotPacket_StopReq_T);     break;
            // case MOT_PACKET_VERSION:       p_rxMeta->Length = sizeof(MotPacket_VersionReq_T);  break;
            // case MOT_PACKET_REBOOT:        p_rxMeta->Length = sizeof(MotPacket_CallReq_T);     break;
            // case MOT_PACKET_CALL:          p_rxMeta->Length = sizeof(MotPacket_CallReq_T);     break;
            // case MOT_PACKET_CALL_ADDRESS:  p_rxMeta->Length = sizeof(MotPacket_CallReq_T);     break;
            // case MOT_PACKET_FIXED_VAR_READ:  p_rxMeta->Length = sizeof( ); break;
            // case MOT_PACKET_FIXED_VAR_WRITE:  p_rxMeta->Length = sizeof( ); break;
            // default: rxCode = PROTOCOL_RX_CODE_ERROR_META; break;
            default: break;
        }
    }
    else /* (rxCount < MOT_PACKET_LENGTH_MIN) ParseMeta should not have been called */
    {
        rxCode = PROTOCOL_RX_CODE_ERROR_META;
    }

    return rxCode;
}

/******************************************************************************/
/*!
    Ctrlr Side Functions
*/
/******************************************************************************/
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
    const MotPacket_DataModeReq_T * p_rxPacket = p_reqContext->p_RxPacket;
    MotPacket_DataModeResp_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;

    p_subState->DataModeAddress = MotPacket_DataModeReq_ParseAddress(p_rxPacket);
    p_subState->DataModeSize = MotPacket_DataModeReq_ParseSize(p_rxPacket);
    p_subState->DataIndex = 0U;
    *p_reqContext->p_TxSize = MotPacket_DataModeReadResp_Build(p_txPacket, MOT_STATUS_OK);
    *p_reqContext->p_SubStateIndex = 1U;
    reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE; // after receiving ack, control is transferred back to MotProtocol_DataModeReadData

    return reqCode;
}


Protocol_ReqCode_T MotProtocol_DataModeReadData(void * p_app, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    MotPacket_DataMode_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    uint16_t readSize;

    /* Passing control for Tx, RxPacket is not valid during this time */
    if(p_subState->DataIndex < p_subState->DataModeSize)
    {
        readSize = (p_subState->DataModeSize - p_subState->DataIndex);
        if(readSize > 32U) readSize = 32U;
        *p_reqContext->p_TxSize = MotPacket_ByteData_Build(p_txPacket, (const uint8_t *)(p_subState->DataModeAddress + p_subState->DataIndex), readSize);
        p_subState->DataIndex += readSize;
        reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;
    }
    else
    {
        *p_reqContext->p_TxSize = MotPacket_DataModeReadResp_Build((MotPacket_DataModeResp_T *)p_txPacket, MOT_STATUS_OK);
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
/*! Stateful Write Data */
/******************************************************************************/
Protocol_ReqCode_T MotProtocol_Flash_DataModeWriteInit_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_DataModeReq_T * p_rxPacket = p_reqContext->p_RxPacket;
    MotPacket_DataModeResp_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    Flash_Status_T flashStatus = NV_MEMORY_STATUS_SUCCESS;

    p_subState->DataModeAddress = MotPacket_DataModeReq_ParseAddress(p_rxPacket);
    p_subState->DataModeSize = MotPacket_DataModeReq_ParseSize(p_rxPacket);
    p_subState->DataIndex = 0U;

    //todo split
    if(p_rxPacket->DataModeReq.Config == MOT_PROTOCOL_DATA_MODE_CONFIG_ERASE)
    {
        flashStatus = Flash_Erase_Blocking(p_flash, p_subState->DataModeAddress, p_subState->DataModeSize);
    }
    // alternatively share with read, check boundaries
    // checks alignment
    if(flashStatus == NV_MEMORY_STATUS_SUCCESS)
    {
        flashStatus = Flash_SetContinueWrite(p_flash, p_subState->DataModeAddress, p_subState->DataModeSize);
    }

    *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build(p_txPacket, flashStatus);
    *p_reqContext->p_SubStateIndex = 1U;
    reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;

    return reqCode;
}

Protocol_ReqCode_T MotProtocol_Flash_DataModeWriteData_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_DataMode_T * p_rxPacket = p_reqContext->p_RxPacket;
    MotPacket_DataMode_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    Flash_Status_T flashStatus;
    const uint8_t * p_sourceData; /* DataPacket Payload */
    uint8_t writeSize; /* DataPacket Size */

    p_sourceData = MotPacket_ByteData_ParsePtrData(p_rxPacket);
    writeSize = MotPacket_ByteData_ParseSize(p_rxPacket);
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
            *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, NV_MEMORY_STATUS_SUCCESS);
            *p_reqContext->p_SubStateIndex = 3U;
            reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;
        }
    }
    else /* Error */
    {
        *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, flashStatus);
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
    Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;

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
        case 2U: /* Write Data - rxPacket is DataPacket */
            reqCode = MotProtocol_Flash_DataModeWriteData_Blocking(p_flash, p_reqContext);
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
// Protocol_ReqCode_T MotProtocol_Flash_Erase_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
// {
//     MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
//     const MotPacket_DataModeReq_T * p_rxPacket = p_reqContext->p_RxPacket;
//     MotPacket_DataModeResp_T * p_txPacket = p_reqContext->p_TxPacket;
//     Protocol_ReqCode_T reqCode;
//     Flash_Status_T flashStatus;

//     p_subState->DataModeAddress = MotPacket_DataModeReq_ParseAddress(p_rxPacket);
//     p_subState->DataModeSize = MotPacket_DataModeReq_ParseSize(p_rxPacket);
//     p_subState->DataIndex = 0U;

//     if(p_rxPacket->DataModeReq.Config == MOT_PROTOCOL_DATA_MODE_CONFIG_ERASE)
//     {
//         flashStatus = Flash_Erase_Blocking(p_flash, p_subState->DataModeAddress, p_subState->DataModeSize);
//     }

//     *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build(p_txPacket, flashStatus);
//     *p_reqContext->p_SubStateIndex = 1U;
//     reqCode = PROTOCOL_REQ_CODE_TX_CONTINUE;

//     return reqCode;
// }

/******************************************************************************/
/*! Mem */
/******************************************************************************/
// caller handle address mapping
protocol_size_t MotProtocol_ReadMem_Blocking(Flash_T * p_flash, MotPacket_MemReadResp_T * p_txPacket, const MotPacket_MemReadReq_T * p_rxPacket)
{
    uint32_t address = p_rxPacket->MemReadReq.Address;
    uint8_t size = p_rxPacket->MemReadReq.Size;
    uint16_t config = p_rxPacket->MemReadReq.Config;

    uint8_t * p_buffer = &(p_txPacket->MemReadResp.ByteData[0U]);
    uint8_t * p_data;
    NvMemory_Status_T status;

    switch((MotProtocol_MemConfig_T)config)
    {
        case MOT_MEM_CONFIG_RAM: memcpy(p_buffer, (void *)address, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_MEM_CONFIG_FLASH: memcpy(p_buffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_MEM_CONFIG_ONCE: status = Flash_ReadOnce_Blocking(p_flash, address, size, p_buffer); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return MotPacket_MemReadResp_BuildHeader(p_txPacket, size, status);
}

protocol_size_t MotProtocol_WriteMem_Blocking(Flash_T * p_flash, MotPacket_MemWriteResp_T * p_txPacket, const MotPacket_MemWriteReq_T * p_rxPacket)
{
    uint32_t address = p_rxPacket->MemWriteReq.Address;
    const uint8_t * p_data = &(p_rxPacket->MemWriteReq.ByteData[0U]);
    uint8_t size = p_rxPacket->MemWriteReq.Size;
    uint16_t config = p_rxPacket->MemWriteReq.Config; // alternatively move this to header
    NvMemory_Status_T status;

    switch((MotProtocol_MemConfig_T)config)
    {
        // blocking operation should block protocol buffer
        case MOT_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
        case MOT_MEM_CONFIG_ONCE: status = Flash_WriteOnce_Blocking(p_flash, address, p_data, size); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return MotPacket_MemWriteResp_Build(p_txPacket, status);
}




/******************************************************************************/
/*! Reboot */
/******************************************************************************/
protocol_size_t MotProtocol_Reboot(void * p_flash, void * p_txPacket, const void * p_rxPacket)
{

}



