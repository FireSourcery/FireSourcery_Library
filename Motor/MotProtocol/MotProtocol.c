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
    @file   MotProtocol.c
    @author FireSourcery
    @brief  MotPacket bound to the Framework/Protocol engine.
*/
/******************************************************************************/
#include "MotProtocol.h"
#include "MotPacket.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

#include <stddef.h>
#include <string.h>



/******************************************************************************/
/*!
    Shared steps
*/
/******************************************************************************/
Protocol_ReqCode_T MotProtocol_FlashLoader_Open(Flash_T * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    /*
        Address, Size and Config are read straight out of the payload, so the frame has to be
        long enough to hold them. A short opening request is well formed on the wire - the
        checksum passes - and would otherwise read past what arrived.
    */
    if (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T))
    {
        p_resp->Status = NV_MEMORY_STATUS_ERROR_OTHER;
        p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
        return PROTOCOL_REQ_DONE;
    }

    p_state->Address = p_req->Address;
    p_state->Size = p_req->Size;
    p_state->Count = 0U;

    p_resp->Status = Flash_SetContinueWrite(p_app, p_state->Address, p_state->Size);  // check bounds on read
    // p_xfer->p_TxMeta->Id = p_xfer->p_RxMeta->Id;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

    if (p_resp->Status != NV_MEMORY_STATUS_SUCCESS) { return PROTOCOL_REQ_DONE; }
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_DATA;
    return PROTOCOL_REQ_RESPOND;
}

/******************************************************************************/
/*!
    Read - device streams flash to the host, paced by the host's acks.

    Flash is memory mapped for reads, so a chunk is a copy straight into the tx payload.
*/
/******************************************************************************/
/******************************************************************************/
/*! MotProtocol Stateful Read Data - ack-paced. RESPOND a chunk, wait for the ack, RESPOND the next. */
/******************************************************************************/
/*!
    Read, continuing - stage the next chunk.

    The tx payload is a raw chunk here, not a status, which is why it is typed uint8_t * and
    the id is set by _ReadChunk rather than by _Reply. A fault swaps the frame for a status
    and closes, so the remote learns why the stream stopped.
*/
Protocol_ReqCode_T MotProtocol_FlashLoader_ReadData(const void * p_flash, Packet_Xfer_T * p_xfer, const void * p_rx, uint8_t * p_chunk)
{
    (void)p_flash; (void)p_rx;
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    if (p_state->Count < p_state->Size)
    {
        packet_size_t readSize = math_min(p_state->Size - p_state->Count, MOT_PACKET_PAYLOAD_LENGTH_MAX);
        memcpy(p_chunk, (const uint8_t *)(p_state->Address + p_state->Count), readSize);
        p_state->Count += readSize;

        p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_DATA;
        p_xfer->p_TxMeta->Length = readSize;
        return PROTOCOL_REQ_RESPOND;
    }

    ((Protocol_DataMode_Resp_T *)p_chunk)->Status = NV_MEMORY_STATUS_SUCCESS;
    p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_READ;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
    return PROTOCOL_REQ_DONE;
}


/*!
    Read - device streams memory to the host.

    Paced by the host's acks: every ack frees the floor and pulls the next chunk. Register
    with PROTOCOL_ACK_ON_REQ so each chunk is acked and the next is pulled by that ack.
*/
Protocol_ReqCode_T MotProtocol_FlashLoader_Read(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * restrict p_rx, void * restrict p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    if (p_xfer->p_RxMeta->Id == MOT_PACKET_DATA_MODE_READ) { p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN; }

    switch (p_state->StateId)
    {
        case PROTOCOL_DATA_MODE_STATE_OPEN:
            p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_READ;
            return MotProtocol_FlashLoader_Open(p_flash, p_xfer, p_rx, p_resp);
        case PROTOCOL_DATA_MODE_STATE_DATA:
            return MotProtocol_FlashLoader_ReadData(p_flash, p_xfer, p_rx, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_CLOSE:    return Protocol_DataMode_ReadClose(p_flash, p_xfer, p_rx, p_resp);
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
/*!
    Write, continuing - absorb one chunk, silently.

    ACCEPT is the silent path: the engine's ack is the whole reply, so nothing is staged.
    A media fault reports and ends.
*/
Protocol_ReqCode_T MotProtocol_FlashLoader_WriteData(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const uint8_t * p_data, Protocol_DataMode_Resp_T * p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);
    packet_size_t writeSize = p_xfer->p_RxMeta->Length;

   /* The remote is ahead of the size it declared. Refuse rather than overrun. */
    if ((size_t)writeSize > (p_state->Size - p_state->Count)) { return PROTOCOL_REQ_REJECT; }

    p_resp->Status = Flash_ContinueWrite_Blocking(p_flash, (const uint8_t *)p_data, writeSize);

    if (p_resp->Status != NV_MEMORY_STATUS_SUCCESS)
    {
        p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_WRITE;
        p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
        return PROTOCOL_REQ_DONE;   /* ABORT would discard the status just staged */
    }

    p_state->Count += writeSize;
    if (p_state->Count < p_state->Size) { return PROTOCOL_REQ_ACCEPT; } /* oe PROTOCOL_REQ_RESPOND, TxLength = 0 */
    // p_state->StateId = PROTOCOL_DATA_MODE_STATE_CLOSE; return PROTOCOL_REQ_ACCEPT;

    /* If we reach here, the write is complete and successful. */
    p_resp->Status = NV_MEMORY_STATUS_SUCCESS;
    p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_WRITE;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
    p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
    return PROTOCOL_REQ_DONE;
}

/*!
    Write - host streams memory to the device.

    Paced by the host's data packets. Only the opening request and the last chunk earn a
    reply; the rest are answered by ACCEPT, which is the engine's ack and nothing more.
*/
Protocol_ReqCode_T MotProtocol_FlashLoader_Write(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * restrict p_rx, void * restrict p_resp)
{
    Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

    if (p_xfer->p_RxMeta->Id == MOT_PACKET_DATA_MODE_WRITE) { p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN; }
    // else if(p_xfer->p_RxMeta->Id == MOT_PACKET_DATA_MODE_DATA) { p_state->StateId = PROTOCOL_DATA_MODE_STATE_DATA; }
    // else { p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN; return PROTOCOL_REQ_ABORT; }

    switch (p_state->StateId)
    {
        case PROTOCOL_DATA_MODE_STATE_OPEN:
            p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_WRITE;
            return MotProtocol_FlashLoader_Open(p_flash, p_xfer, p_rx, p_resp);
        case PROTOCOL_DATA_MODE_STATE_DATA:
            return MotProtocol_FlashLoader_WriteData(p_flash, p_xfer, p_rx, p_resp);
        // case PROTOCOL_DATA_MODE_STATE_CLOSE:    return Protocol_DataMode_WriteClose(p_flash, p_xfer, p_rx, p_resp);
        default:
            p_state->StateId = PROTOCOL_DATA_MODE_STATE_OPEN;
            return PROTOCOL_REQ_ABORT;
    }
}

/******************************************************************************/
/*! Erase - blocking erase of a flash memory region */
/******************************************************************************/
Protocol_ReqCode_T MotProtocol_FlashLoader_Erase_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
{
    /* Same exposure as Open: Address and Size come out of a payload that may be short. */
    p_resp->Status = (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T))
        ? NV_MEMORY_STATUS_ERROR_OTHER
        : (uint16_t)Flash_Erase_Blocking(p_flash, (uintptr_t)p_req->Address, (size_t)p_req->Size);
    p_xfer->p_TxMeta->Id = MOT_PACKET_DATA_MODE_ERASE;
    p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
    return PROTOCOL_REQ_DONE;
}



/******************************************************************************/
/*
    Registration

        static const Protocol_Req_T REQ_TABLE[] =
        {
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,  &MOT_FRAME_DATA, MotProtocol_FlashLoader_Read,  PROTOCOL_ACK_ON_REQ),
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE, &MOT_FRAME_DATA, MotProtocol_FlashLoader_Write, PROTOCOL_ACK_NONE),
            PROTOCOL_REQ(MOT_PACKET_DATA_MODE_ERASE, &MOT_FRAME_DATA, MotProtocol_FlashLoader_Erase_Blocking, PROTOCOL_ACK_NONE),
        };

    Read is ack-paced: each chunk is acked, and that ack pulls the next. Write is data-paced -
    the handler's own ACCEPT is the ack - so it is ACK_NONE, as in the live table.

    P_SUB_STATE must be at least sizeof(Protocol_DataMode_State_T), and P_APP_CONTEXT is the interface above.
*/
/******************************************************************************/

