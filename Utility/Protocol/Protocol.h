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
#include "Config.h"
//#include "Datagram.h"

#include "Packet/Packet.h"
#include "Peripheral/Xcvr/Xcvr.h"
#include "Type/mux.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/******************************************************************************/
/*!
    OSI Layer 3,4
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Request / Request Table

    ProcReqResp - interface for Rx and Tx packets
    User provide functions to convert between Packet format and appInterface format

    User may config appInterface as
        appInterface => buffer once, unrestricted scope
        packetInterface => parse p_rxPacket to packetInterface, then call proc on buffered data => buffers twice
*/
/******************************************************************************/
/******************************************************************************/
/*
    Stateless Req, fit for simple read write.
    Shared ReqResp function allows function to maintain temporary local state
    @return txSize
*/
/******************************************************************************/
typedef packet_size_t(*Protocol_ProcReqResp_T)(void * p_appContext, uint8_t * p_txPacket, const uint8_t * p_rxPacket);
// typedef packet_size_t(*Protocol_ProcReqResp_T)(void * p_appContext, uint8_t * p_txPayload, const uint8_t * p_rxPayload);

/******************************************************************************/
/*
    Extended/Stateful Request - Support wait, loop, dynamic ack/nack and additional processes
*/
/******************************************************************************/
// alternatively,
// with payload,   builder header double buffers
// typedef void(*Protocol_ParseReq_T)(void * p_appContext, const uint8_t * p_rxPayload, const Protocol_HeaderMeta_T * p_rxMeta);
// typedef void(*Protocol_BuildResp_T)(void * p_appContext, uint8_t * p_txPayload, Protocol_HeaderMeta_T * p_txMeta);

// typedef void(*Protocol_ParseReq_T)(void * p_appContext, const uint8_t * p_rxPayload, const uint8_t * p_rxHeader);
// typedef void(*Protocol_BuildResp_T)(void * p_appContext, uint8_t * p_txPayload, uint8_t * p_txHeader);

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

    /* User Function Manual select next step */
    PROTOCOL_REQ_CODE_AWAIT_RX_CONTINUE,        /* Expecting Rx new packet */
    PROTOCOL_REQ_CODE_AWAIT_RX_SYNC,            /* Expecting static ack nack */
    PROTOCOL_REQ_CODE_TX_RESPONSE,
    PROTOCOL_REQ_CODE_TX_ACK,
    PROTOCOL_REQ_CODE_TX_NACK,

    /* Error */
    PROTOCOL_REQ_CODE_ERROR_ID,             /* ID not found */
    PROTOCOL_REQ_CODE_ERROR_RX_UNEXPECTED,  /* Out of sequence packet */
    PROTOCOL_REQ_CODE_ERROR_TIMEOUT,

    //remove
    // PROTOCOL_REQ_CODE_AWAIT_PROCESS,                 /* Child Protocol NonBlocking Wait ReqExt processing */
    // PROTOCOL_REQ_CODE_AWAIT_PROCESS_EXTEND_TIMER,    /* Child Protocol NonBlocking Wait, Extend timer for RxLost and Timeout */

    /* Special Context Functions, parent module mapped req table */
    //    PROTOCOL_RX_CODE_REQ_VAR,
    //    PROTOCOL_RX_CODE_REQ_DATAGRAM,
    //    PROTOCOL_RX_CODE_REQ_FLASH,
}
Protocol_ReqCode_T;

/*
    Stateful Req function, pass args collectively,
    alternatively pass by struct and let compiler handle
*/
/* ReqParameters */
typedef const struct Protocol_ReqContext
{
    void * p_SubState;
    uint32_t * p_SubStateIndex;
    const Protocol_HeaderMeta_T * p_RxMeta;
    const void * p_RxPacket;
    // const void * p_RxPayload;
    void * p_TxPacket;
    // void * p_TxPayload;
    packet_size_t * p_TxSize;
    // Protocol_HeaderMeta_T * p_TxMeta;
}
Protocol_ReqContext_T;

typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(void * p_appContext, Protocol_ReqContext_T * p_interface);
// typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(Protocol_ReqContext_T interface);
// typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(Protocol_T * p_base);

typedef void (*Protocol_ResetReqState_T)(void * p_subState);

/*
    Sync Options - Configure per Req, uses common timeout
    Stateless static string sync, ack, nack - Ack Nack Packet_BuildTxSync_T
    Dynamic ack nack string implementation use Protocol_ReqExtFunction_T
*/
typedef const union Protocol_ReqSync
{
    struct
    {
        uint32_t TX_ACK         : 1U;   /* Tx Ack after Rx initial Request*/
        uint32_t RX_ACK         : 1U;   /* Wait for Rx Ack after Tx Response */
        uint32_t NACK_REPEAT    : 3U;   /* Common setting. Number of repeat TxPacket on Rx Nack, Tx Nack on RxPacket error */
        uint32_t TX_ACK_EXT     : 1U;   /* Use for All Stateful Ext Request  */
        uint32_t RX_ACK_EXT     : 1U;
        uint32_t TX_ACK_ABORT   : 1U;   /* Use for All Stateful Ext Request  */
    };
    uint32_t ID;
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
}
Protocol_Req_T;

#define PROTOCOL_REQ(Id, ProcReqResp, ProcExt, ReqSync, ...) \
    { .ID = (packet_id_t)Id, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSync, __VA_ARGS__ }

// #define PROTOCOL_REQ_EXT(Id, ProcReqResp, ProcExt, ReqSyncExt) \
//     { .ID = (packet_id_t)Id, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSyncExt, }


/******************************************************************************/
/*!
    Tx Sync
    User supply function to build Tx Ack, Nack, etc
    alternatively combine rx sync with map
*/
/******************************************************************************/
typedef enum Protocol_TxSyncId
{
    PROTOCOL_TX_SYNC_ACK_REQ,
    PROTOCOL_TX_SYNC_NACK_REQ,
    PROTOCOL_TX_SYNC_ACK_ABORT,
    PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT,
    PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT,
    PROTOCOL_TX_SYNC_NACK_PACKET_META,
    PROTOCOL_TX_SYNC_NACK_PACKET_DATA,
    PROTOCOL_TX_SYNC_ACK_REQ_EXT,
    PROTOCOL_TX_SYNC_NACK_REQ_EXT,
}
Protocol_TxSyncId_T;


/******************************************************************************/
/*!
    Private State Ids
*/
/******************************************************************************/
typedef enum Protocol_RxState
{
    PROTOCOL_RX_STATE_INACTIVE,
    PROTOCOL_RX_STATE_WAIT_BYTE_1,
    PROTOCOL_RX_STATE_WAIT_PACKET,
    // PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
}
Protocol_RxState_T;

typedef enum Protocol_ReqState
{
    PROTOCOL_REQ_STATE_INACTIVE,
    PROTOCOL_REQ_STATE_WAIT_RX_ID,
    PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE,        /* Wait for next ReqExt packet in stateful routine */
    PROTOCOL_REQ_STATE_WAIT_RX_SYNC,
    PROTOCOL_REQ_STATE_PROCESS_REQ_EXT,         /* Wait for ReqExt process */
}
Protocol_ReqState_T;

// mux
// typedef struct Protocol_Mux
// {
//     const Protocol_Req_T * P_REQ_TABLE;
//     uint8_t REQ_TABLE_LENGTH;
//     // REQ_TIMEOUT

//     const volatile uint32_t * P_TIMER;

// #ifdef
//     const Packet_Class_T * const * P_PACKET_CLASS_TABLE;    /* Bound and verify specs selection. Array of pointers, Specs not necessarily in a contiguous array */
//     uint8_t PACKET_CLASS_COUNT;

//     /*  */
//     const Xcvr_T * const * P_XCVR_TABLE; /* array of struct, or pointers. todo move selection */
//     uint8_t XCVR_COUNT; /* number of Xcvr in table */
//     // alternatively fixed // const Xcvr_T * P_XCVR;
// }
// Protocol_Mux_T;

// typedef struct Protocol
// {
//     const Protocol_Req_T * P_REQ_TABLE;
//     uint8_t REQ_TABLE_LENGTH;
//     // REQ_TIMEOUT
//     const Packet_Class_T * P_PACKET_CLASS

//     const volatile uint32_t * P_TIMER;
// }
// Protocol_T;



/******************************************************************************/
/*!
*/
/******************************************************************************/

// static inline uint16_t Protocol_GetTxPacketCount(const Protocol_T * p_protocol) { return p_protocol->TxPacketCount; }
// static inline uint16_t Protocol_GetRxPacketCount(const Protocol_T * p_protocol) { return p_protocol->RxPacketErrorCount + p_protocol->RxPacketSuccessCount; }

// static inline void Protocol_ClearTxRxPacketCount(Protocol_T * p_protocol)
// {
//     p_protocol->TxPacketCount = 0U;
//     p_protocol->RxPacketSuccessCount = 0U;
//     p_protocol->RxPacketErrorCount = 0U;
// }



//todo datagram request
// typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)    (Datagram_T * p_datagram, uint8_t * p_txPacket,  packet_size_t * p_txSize, const uint8_t * p_rxPacket, packet_size_t rxSize);
// typedef void (*Protocol_DatagramInit_T) (void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, packet_size_t * p_txSize, const uint8_t * p_rxPacket, packet_size_t rxSize);
