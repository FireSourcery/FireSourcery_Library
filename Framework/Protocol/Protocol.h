#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Protocol.h
    @author FireSourcery
    @brief  Protocol abstract class equivalent
*/
/******************************************************************************/
//#include "Datagram.h"

#include "Packet/Packet.h"
#include "Packet/Packet_RxParser.h"
#include "Peripheral/Xcvr/Xcvr.h"
#include "Type/mux.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/******************************************************************************/
/*!
    Common Layer handling
        - Frame/Header Parsing
        - Build Tx Header
        - Sync Ack/Nack
        - Hook for substate handling

    OSI Layer 3,4
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Request / Request Table

    ProcReqResp - interface for Rx and Tx packets
    User provide functions to convert between Packet format and appInterface format

    appInterface as
        appInterface => buffer once, unrestricted scope
        packetInterface => parse p_rxPacket to packetInterface, then call proc on buffered data => buffers twice
*/
/******************************************************************************/
/******************************************************************************/
/*
    Stateless Req, fit for simple read write.
    Shared ReqResp function allows function to maintain temporary local state
*/
/******************************************************************************/
/*!
    Configurable for p_payload or p_header
    @return txSize
*/
typedef packet_size_t(*Protocol_ProcReqResp_T)(void * p_context, uint8_t * p_tx, const uint8_t * p_rx);
// typedef packet_size_t(*Protocol_ProcReqResp_T)(void * p_context, const uint8_t * p_rx, uint8_t * p_tx);
// typedef int(*Protocol_ProcReqResp_T)(void * p_context, const uint8_t * p_rx, packet_size_t rxSize, uint8_t * p_tx, packet_size_t * p_txSize);

/******************************************************************************/
/*
    Extended/Stateful Request - Support wait, loop, dynamic ack/nack and additional processes
*/
/******************************************************************************/

/* Common Req from child protocol to supported general protocol control, predefined behaviors */
typedef enum Protocol_ReqCode
{
    // PROTOCOL_REQ_CODE_AWAIT_RX_REQ_INITIAL,
    PROTOCOL_REQ_CODE_TX_CONTINUE,                  /* continue using default sync settings, wait for next packet *///option split with tx send
    // PROTOCOL_REQ_CODE_PROCESS_AWAIT_RX,          /* Expecting Rx new packet */
    PROTOCOL_REQ_CODE_PROCESS_COMPLETE,             /* Exit nonblocking wait processing state upon reception */
    // PROTOCOL_REQ_CODE_PROCESS_COMPLETE_WITH_ERROR,
    PROTOCOL_REQ_CODE_ABORT, /* Terminate */

    /* ack after process */
    PROTOCOL_REQ_CODE_PROCESS_ACK,
    PROTOCOL_REQ_CODE_PROCESS_NACK,

    PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE,        /* Expecting Rx new packet */
    PROTOCOL_REQ_CODE_AWAIT_RX_SYNC,            /* Expecting static ack nack */

    /* User Function Manual select next step */
    PROTOCOL_REQ_CODE_TX_RESPONSE,
    PROTOCOL_REQ_CODE_TX_ACK,
    PROTOCOL_REQ_CODE_TX_NACK,

    /* Error */
    PROTOCOL_REQ_CODE_ERROR_ID,             /* ID not found */
    PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED,  /* Out of sequence packet */
    PROTOCOL_REQ_CODE_ERROR_TIMEOUT,

    // PROTOCOL_REQ_CODE_CONTINUE,        /* continue using default sync settings, wait for next packet */
    // PROTOCOL_REQ_CODE_COMPLETE,
    // PROTOCOL_REQ_CODE_AWAIT_RX,        /* Expecting Rx new packet */
}
Protocol_ReqCode_T;

/*
    Stateful Req function, pass args collectively,
*/
typedef const struct Protocol_ReqContext
{
    // Rx side (filled by framing layer before calling handler)
    const void * p_RxPacket;
    // const void * p_RxPayload;
    const Protocol_HeaderMeta_T * p_RxMeta;

    // Tx side (filled by handler)
    void * p_TxPacket;
    // void * p_TxPayload;
    Protocol_HeaderMeta_T * p_TxMeta;  // handler sets Id + Length
    packet_size_t * p_TxSize;

    // State (for multi-step sequences)
    void * p_SubState;
    uint32_t * p_SubStateIndex;
// Optionally as unified handler context
// optionally compile time define contigous context
}
Protocol_ReqContext_T;
// PROC_EXT — async, iterative, returns control code
//   Input:  Rx packet (may be new or same as last call)
//   Output: Tx packet + size via context
//   Return: control code (continue, await rx, complete, ack, nack)
//   State:  persistent via p_SubState across call
typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(void * p_appContext, Protocol_ReqContext_T * p_interface);
// typedef Protocol_ReqCode_T(*Protocol_ProcReqExt1_T)(void * p_appContext, void * p_SubState, const void const * p_rx, const void * p_tx);

typedef void (*Protocol_ResetReqState_T)(void * p_subState);


/*
    Sync Options - Configure per Req, uses common timeout
    Stateless static string sync, ack, nack - Ack Nack Packet_BuildTxSync_T
    Dynamic ack nack string implementation use Protocol_ReqExtFunction_T
*/
typedef const struct Protocol_ReqSync
{
    uint32_t TX_ACK         : 1U;   /* Tx Ack after Rx initial Request*/
    uint32_t RX_ACK         : 1U;   /* Wait for Rx Ack after Tx Response */

    uint32_t NACK_REPEAT    : 3U;   /* Common setting. Number of repeat TxPacket on Rx Nack, Tx Nack on RxPacket error */
    uint32_t TX_ACK_EXT     : 1U;   /* Use for All Stateful Ext Request  */
    uint32_t RX_ACK_EXT     : 1U;
    uint32_t TX_ACK_ABORT   : 1U;
}
Protocol_ReqSync_T;

// typedef enum Protocol_SyncMode
// {
//     PROTOCOL_SYNC_MODE_NONE,
//     PROTOCOL_SYNC_MODE_ACK_ONLY,
//     PROTOCOL_SYNC_MODE_ACK_NACK,
//     PROTOCOL_SYNC_MODE_EXTENDED
// } Protocol_SyncMode_T;

#define PROTOCOL_SYNC_DISABLE \
    { .TX_ACK = 0U, .RX_ACK = 0U, .TX_ACK_EXT = 0U, .RX_ACK_EXT = 0U, .NACK_REPEAT = 0U, }

#define PROTOCOL_SYNC(UseTxAck, UseRxAck, NackRepeat) \
    { .TX_ACK = UseTxAck, .RX_ACK = UseRxAck, .TX_ACK_EXT = 0U, .RX_ACK_EXT = 0U, .NACK_REPEAT = NackRepeat, }

#define PROTOCOL_SYNC_EXT(UseTxAck, UseRxAck, UseTxAckExt, UseRxAckExt, NackRepeat) \
    { .TX_ACK = UseTxAck, .RX_ACK = UseRxAck, .TX_ACK_EXT = UseTxAckExt, .RX_ACK_EXT = UseRxAckExt, .NACK_REPEAT = NackRepeat, }

/*
    Protocol_Req_T
*/
typedef const struct Protocol_Req
{
    const packet_id_t               ID;
    const Protocol_ProcReqResp_T    PROC;
    const Protocol_ProcReqExt_T     PROC_EXT;
    const Protocol_ReqSync_T        SYNC;
    const uint32_t                  TIMEOUT; /* overwrite common timeout */
    // packet_size_t RESP_PAYLOAD_SIZE;  // 0 = variable (handler returns size)
}
Protocol_Req_T;

#define PROTOCOL_REQ(Id, ProcReqResp, ProcExt, ReqSync, ...) (Protocol_Req_T) \
    { .ID = (packet_id_t)Id, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSync, __VA_ARGS__ }

// #define PROTOCOL_REQ_EXT(Id, ProcReqResp, ProcExt, ReqSyncExt)
//     { .ID = (packet_id_t)Id, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSyncExt, }

/* Fast reverse map */
typedef Protocol_Req_T * (*Protocol_ReqMapper_T)(packet_id_t id);

/*! @return pointer to Req */
static inline const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    const Protocol_Req_T * p_req = NULL;
    for (uint8_t iReq = 0U; iReq < tableLength; iReq++) { if (p_reqTable[iReq].ID == id) { p_req = &p_reqTable[iReq]; break; } }
    return p_req;
}

/******************************************************************************/
/*!
    Private State Ids
*/
/******************************************************************************/
typedef enum Protocol_ReqState
{
    PROTOCOL_REQ_STATE_INACTIVE,
    PROTOCOL_REQ_STATE_WAIT_RX_ID,              /* Complete. await new initial request */
    PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE,        /* Await Rx. Wait for next ReqExt packet in stateful routine */ // tx → await next rx → call again
    PROTOCOL_REQ_STATE_WAIT_RX_SYNC,        // use nested to retain resume state
    PROTOCOL_REQ_STATE_PROCESS_REQ_EXT,         /* Wait for ReqExt process */
}
Protocol_ReqState_T;



