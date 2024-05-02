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
    Packet Interface
    Common functions, not requiring appInterface, mapping directly to Protocol Specs
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
        default: *p_txSize = 0U; syncChar = MOT_PACKET_ID_RESERVED_255; break;
    }

    *p_txSize = MotPacket_Sync_Build(p_txPacket, syncChar);
}


Protocol_RxCode_T MotProtocol_ParseRxMeta(Protocol_HeaderMeta_T * p_rxMeta, const MotPacket_T * p_rxPacket, protocol_size_t rxCount)
{
    Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_AWAIT_PACKET;

    /* Called after rxCount > MIN, rxCount != 0 */
    if(rxCount == p_rxMeta->Length) /* Packet Complete */
    {
        rxCode = (MotPacket_ProcChecksum(p_rxPacket) == true) ? PROTOCOL_RX_CODE_PACKET_COMPLETE : PROTOCOL_RX_CODE_ERROR_DATA;
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
    Address functions p_appContext independent
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

    /* Tx Ack Sync on reception */
    // if(*p_reqContext->p_SubStateIndex == 0U)
    // {
    p_subState->DataModeAddress = MotPacket_DataModeReq_ParseAddress(p_rxPacket);
    p_subState->DataModeSize = MotPacket_DataModeReq_ParseSize(p_rxPacket);
    *p_reqContext->p_TxSize = MotPacket_DataModeReadResp_Build(p_txPacket, MOT_STATUS_OK);
    *p_reqContext->p_SubStateIndex = 1U;
    // p_subState->DataModeStateId = MOT_PROTOCOL_DATA_MODE_READ_ACTIVE;
    // p_subState->SequenceIndex = 0U;
    reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
    // await Ack
    // }
    // else
    // {
    //     reqCode = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;
    // }

    return reqCode;
}


Protocol_ReqCode_T MotProtocol_DataModeReadData(void * p_app, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_DataMode_T * p_rxPacket = p_reqContext->p_RxPacket;
    MotPacket_DataMode_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    uint16_t readSize;

    if(*p_reqContext->p_SubStateIndex == 1U) /* set by init */
    {
        if(p_subState->DataModeSize > 0U) //todo change to index
        {
            readSize = (p_subState->DataModeSize > 32U) ? 32U : p_subState->DataModeSize;
            *p_reqContext->p_TxSize = MotPacket_ByteData_Build(p_txPacket, (const uint8_t *)p_subState->DataModeAddress, readSize);
            p_subState->DataModeSize -= readSize;
            p_subState->DataModeAddress += readSize;
            reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
        }
        else /* (p_subState->DataModeSize == 0U) */
        {
            // p_subState->DataModeStateId = MOT_PROTOCOL_DATA_MODE_INACTIVE;
            *p_reqContext->p_TxSize = MotPacket_DataModeReadResp_Build((MotPacket_DataModeResp_T *)p_txPacket, MOT_STATUS_OK);
            *p_reqContext->p_SubStateIndex = 2U; // or handler clear
            reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
        }
    }
    else
    {
        *p_reqContext->p_SubStateIndex = 0U;
        reqCode = PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED;
    }

    return reqCode;
}
// MOT_PACKET_DATA_MODE_READ = 0xDAU,      /* Stateful NvMemory Read using Address */
// MOT_PACKET_DATA_MODE_WRITE = 0xDBU,     /* Stateful NvMemory Write using Address */
// MOT_PACKET_DATA_MODE_DATA = 0xDDU,      /* Data Mode Data */
// MOT_PACKET_DATA_MODE_ABORT
Protocol_ReqCode_T MotProtocol_ReadData(void * p_app, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE; // = PROTOCOL_REQ_CODE_AWAIT_PROCESS;

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
/* Share for read and write */
Protocol_ReqCode_T MotProtocol_Flash_DataModeWriteDataInit_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    const MotPacket_DataModeReq_T * p_rxPacket = p_reqContext->p_RxPacket;
    MotPacket_DataModeResp_T * p_txPacket = p_reqContext->p_TxPacket;
    Protocol_ReqCode_T reqCode;
    Flash_Status_T flashStatus;

    p_subState->DataModeAddress = MotPacket_DataModeReq_ParseAddress(p_rxPacket);
    p_subState->DataModeSize = MotPacket_DataModeReq_ParseSize(p_rxPacket);

    // flashStatus = Flash_StartContinueWrite(p_flash, (const uint8_t *)p_subState->DataModeAddress, p_subState->DataModeSize);
    flashStatus = NV_MEMORY_STATUS_SUCCESS;

    *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build(p_txPacket, flashStatus);
    *p_reqContext->p_SubStateIndex = 1U;
    reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;

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

    if(p_subState->DataModeSize >= writeSize)
    {
        // flashStatus = Flash_ContinueWrite_Blocking(p_flash, p_sourceData, writeSize);
        flashStatus = NV_MEMORY_STATUS_SUCCESS;
        if(flashStatus == NV_MEMORY_STATUS_SUCCESS)
        {
            p_subState->DataModeSize -= writeSize;
            *p_reqContext->p_TxSize = 0U;
            /* Tx Ack On Reception */// further nacks must refer to previous pack or seperate state for tx ack after process
            reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE;
        }
        else /* Error */
        {
            flashStatus = NV_MEMORY_STATUS_ERROR_BUSY;
            *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, flashStatus);
            *p_reqContext->p_SubStateIndex = 0U;
            reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
        }
    }
    else /* (p_subState->DataModeSize == 0U) */
    {
        *p_reqContext->p_TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, MOT_STATUS_OK);
        *p_reqContext->p_SubStateIndex = 3U;
        reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
    }

    return reqCode;
}

/*
    Protocol only maintains 1 App Context Pointer.
    Caller pass flash controller context within App Context
*/
Protocol_ReqCode_T MotProtocol_Flash_WriteData_Blocking(Flash_T * p_flash, Protocol_ReqContext_T * p_reqContext)
{
    MotProtocol_DataModeState_T * p_subState = p_reqContext->p_SubState;
    Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE; // = PROTOCOL_REQ_CODE_AWAIT_PROCESS;

    switch(*p_reqContext->p_SubStateIndex)
    {
        case 0U: /* Tx Ack handled by Common Req Sync */
            reqCode = MotProtocol_Flash_DataModeWriteDataInit_Blocking(p_flash, p_reqContext);
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
/*! Once */
/******************************************************************************/
protocol_size_t MotProtocol_Flash_WriteOnce_Blocking(Flash_T * p_flash, MotPacket_OnceWriteResp_T * p_txPacket, const MotPacket_OnceWriteReq_T * p_rxPacket)
{
    // blocking operation should block protocol buffer
    p_txPacket->Header.Status = Flash_WriteOnce_Blocking(p_flash, (uint8_t *)p_rxPacket->OnceWriteReq.Address, &(p_rxPacket->OnceWriteReq.ByteData[0U]), p_rxPacket->OnceWriteReq.Size);
    return MotPacket_BuildHeader((MotPacket_T *)p_txPacket, MOT_PACKET_WRITE_ONCE, sizeof(MotPacket_OnceWriteResp_Payload_T));
}

protocol_size_t MotProtocol_Flash_ReadOnce_Blocking(Flash_T * p_flash, MotPacket_OnceReadResp_T * p_txPacket, const MotPacket_OnceReadReq_T * p_rxPacket)
{
    p_txPacket->Header.Status = Flash_ReadOnce_Blocking(p_flash, &(p_txPacket->OnceReadResp.ByteData[0U]), (uint8_t *)p_rxPacket->OnceReadReq.Address, p_rxPacket->OnceReadReq.Size);
    return MotPacket_BuildHeader((MotPacket_T *)p_txPacket, MOT_PACKET_READ_ONCE, p_rxPacket->OnceReadReq.Size);
}







