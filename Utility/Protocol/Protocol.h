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
// typedef packet_size_t(*Protocol_ProcReqResp_T)(void * p_appContext, void * p_txPayload, const void * p_rxPayload);

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
// typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(void * p_appContext, Packet_RxContext_T, Packet_TxConontext_T, );

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
    // packet_size_t RESP_PAYLOAD_SIZE;  // 0 = variable (handler returns size)
}
Protocol_Req_T;

#define PROTOCOL_REQ(Id, ProcReqResp, ProcExt, ReqSync, ...) (Protocol_Req_T) \
    { .ID = (packet_id_t)Id, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSync, __VA_ARGS__ }

// #define PROTOCOL_REQ_EXT(Id, ProcReqResp, ProcExt, ReqSyncExt) \
//     { .ID = (packet_id_t)Id, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSyncExt, }

/* Faster reverse map */
typedef Protocol_Req_T * (*Protocol_ReqMapper_T)(void * p_app, packet_id_t id);

/*! @return pointer to Req */
static inline const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id)
{
    const Protocol_Req_T * p_req = NULL;
    for (uint8_t iChar = 0U; iChar < tableLength; iChar++) { if (p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; break; } }
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
// typedef struct Protocol
// {
    // Protocol_ReqMapper_T;
//     const Protocol_Req_T * P_REQ_TABLE;
//     uint8_t REQ_TABLE_LENGTH;
//     // REQ_TIMEOUT
//     const Packet_Format_T * P_PACKET_FORMAT;
//     const volatile uint32_t * P_TIMER;
// }
// Protocol_T;

// mux
// typedef struct Protocol_Mux
// {
//     const Packet_Format_T * const * P_PACKET_CLASS_TABLE;    /* Bound and verify specs selection. Array of pointers, Specs not necessarily in a contiguous array */
//     uint8_t PACKET_CLASS_COUNT;
//     const Xcvr_T * const * P_XCVR_TABLE; /* array of struct, or pointers. todo move selection */
//     uint8_t XCVR_COUNT; /* number of Xcvr in table */
// }
// Protocol_Mux_T;




/******************************************************************************/
/*!
    Sync
*/
/******************************************************************************/
/* Transport State */
typedef enum
{
    PROTOCOL_SYNC_IDLE,
    PROTOCOL_SYNC_WAIT_ACK,          // sent data, awaiting ack
    PROTOCOL_SYNC_WAIT_DATA,         // sent ack, awaiting next data
}
Protocol_SyncStateId_T;

typedef struct
{
    Protocol_SyncStateId_T State;
    uint8_t NackCount;
    uint8_t RetransmitCount;
    uint32_t TimeStart;
}
Protocol_SyncContext_T;

// Returns: data available, timeout, still waiting
typedef enum
{
    SYNC_RX_WAITING,            // still waiting (no data, or awaiting ack)
    SYNC_RX_DATA_READY,         // data packet delivered (auto-acked if configured)
    SYNC_RX_ACK_RECEIVED,       // ack received after our Tx
    SYNC_RX_ABORT,              // abort received
    SYNC_RX_TIMEOUT,            // request timeout
    SYNC_RX_ERROR,              // unexpected rx (nack in wrong state, etc.)
}
Protocol_SyncStatus_T;

// static Protocol_SyncStatus_T ProcSyncRx(const Socket_T * p_socket, Socket_State_T * p_state, Protocol_RxCode_T rxCode, const Protocol_ReqSync_T * p_syncOpts)
// {
//     Protocol_SyncContext_T * p_sync = &p_state->Sync;

//     switch (p_sync->State)
//     {
//         case SYNC_STATE_IDLE:
//             switch (rxCode)
//             {
//                 case PROTOCOL_RX_CODE_PACKET_COMPLETE:
//                     /* Auto-ack on reception if configured */
//                     if (p_syncOpts != NULL)
//                     {
//                         bool txAck = (p_state->ReqState == REQ_STATE_IDLE)
//                             ? p_syncOpts->TX_ACK        /* initial request */
//                             : p_syncOpts->TX_ACK_EXT;   /* continuation */
//                         if (txAck) { TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_REQ); }
//                     }
//                     p_sync->TimeStart = *p_socket->P_TIMER;
//                     return SYNC_RX_DATA_READY;

//                 case PROTOCOL_RX_CODE_ABORT:
//                     if (p_syncOpts != NULL && p_syncOpts->TX_ACK_ABORT)
//                         TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_ABORT);
//                     return SYNC_RX_ABORT;

//                 case PROTOCOL_RX_CODE_AWAIT_PACKET:
//                     return SYNC_RX_WAITING;

//                 default:    /* ACK/NACK in idle = unexpected */
//                     return SYNC_RX_ERROR;
//             }

//         case SYNC_STATE_WAIT_RX_ACK:
//             switch (rxCode)
//             {
//                 case PROTOCOL_RX_CODE_ACK:
//                     p_sync->State = SYNC_STATE_IDLE;
//                     p_sync->NackRetransmitCount = 0U;
//                     p_sync->TimeStart = *p_socket->P_TIMER;
//                     return SYNC_RX_ACK_RECEIVED;

//                 case PROTOCOL_RX_CODE_NACK:
//                     /* Retransmit buffered response */
//                     TxResp(p_socket, p_state);
//                     p_sync->NackRetransmitCount++;
//                     // TODO: check NACK_REPEAT limit
//                     return SYNC_RX_WAITING;

//                 case PROTOCOL_RX_CODE_ABORT:
//                     p_sync->State = SYNC_STATE_IDLE;
//                     if (p_syncOpts != NULL && p_syncOpts->TX_ACK_ABORT)    TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_ABORT);
//                     return SYNC_RX_ABORT;

//                 case PROTOCOL_RX_CODE_AWAIT_PACKET:
//                     return SYNC_RX_WAITING;

//                 default:    /* data packet while waiting for ack = unexpected */
//                     return SYNC_RX_ERROR;
//             }
//     }
//     return SYNC_RX_ERROR;
// }



// typedef const struct
// {
//     const Packet_Format_T * P_FORMAT;
//     uint8_t * P_BUFFER;
//     Protocol_HeaderMeta_T * P_META;
//     const Packet_BuildTxHeader_T  BUILD_TX_HEADER;  // symmetric with Phase 2
//     const Packet_BuildTxSync_T    BUILD_TX_SYNC;
// }
// Protocol_TxContext_T;

// typedef const struct
// {
//     const Packet_Format_T * P_FORMAT;
//     uint8_t * P_BUFFER;
//     Protocol_HeaderMeta_T * P_META;
//     Packet_RxParserState_T * P_RX_STATE;
//     const Packet_ParseRxFraming_T PARSE_RX_FRAMING;  // Phase 1: Id + Length
//     const Packet_ParseRxHeader_T  PARSE_RX_HEADER;   // Phase 2: checksum + fields extraction
// }
// Protocol_RxContext_T;

// // After Tx — optionally wait for ack
// Protocol_SyncStatus_T Sync_AfterTx(Protocol_SyncContext_T * p_sync, Protocol_RxCode_T rxCode, bool requireAck)
// // static void ProcSyncAfterTx(const Socket_T * p_socket, Socket_State_T * p_state, const Protocol_ReqSync_T * p_syncOpts)
// {
//     Sync_Context_T * p_sync = &p_state->Sync;

//     if (p_state->TxLength > 0U)
//     {
//         p_state->p_Specs->BUILD_TX_HEADER(p_socket->P_TX_PACKET_BUFFER, &p_state->TxMeta);
//         TxResp(p_socket, p_state);

//         bool rxAck = (p_state->DispatchState == DISPATCH_STATE_IDLE)  ? p_syncOpts->RX_ACK : p_syncOpts->RX_ACK_EXT;

//         if (rxAck)
//         {
//             p_sync->State = SYNC_STATE_WAIT_RX_ACK;
//             p_sync->NackRetransmitCount = 0U;
//         }
//     }
// }

// // After Rx — optionally send ack, deliver data
// Protocol_SyncStatus_T Sync_AfterRx(Protocol_SyncContext_T * p_sync, Protocol_RxCode_T rxCode, bool sendAck);

// // static bool ProcSyncTimeout(const Socket_T * p_socket, Socket_State_T * p_state)
// // {
// //     if (p_state->DispatchState != DISPATCH_STATE_IDLE)
// //     {
// //         if (*p_socket->P_TIMER - p_state->Sync.TimeStart > p_state->p_Specs->REQ_TIMEOUT)
// //         {
// //             TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT);
// //             p_state->Sync.State = SYNC_STATE_IDLE;
// //             p_state->DispatchState = DISPATCH_STATE_IDLE;
// //             return true;
// //         }
// //     }
// //     return false;
// // }



// static Protocol_ReqCode_T ProcDispatch(const Socket_T * p_socket, Socket_State_T * p_state)
// {
//     switch (p_state->DispatchState)
//     {
//         case DISPATCH_STATE_IDLE:
//             {
//                 /* ── Entry ── */
//                 p_state->p_ReqActive = _Protocol_SearchReqTable(p_socket->P_REQ_TABLE, p_socket->REQ_TABLE_LENGTH, p_state->RxMeta.Id);

//                 if (p_state->p_ReqActive == NULL)  return PROTOCOL_REQ_CODE_ERROR_ID;

//                 p_state->TxLength = 0U;

//                 if (p_state->p_ReqActive->PROC != NULL)
//                 {
//                     p_state->TxLength = p_state->p_ReqActive->PROC(p_socket->P_APP_CONTEXT, p_socket->P_TX_PACKET_BUFFER, p_socket->P_RX_PACKET_BUFFER);
//                 }

//                 if (p_state->p_ReqActive->PROC_EXT == NULL)                    return PROTOCOL_REQ_CODE_COMPLETE;

//                 /* Transition to Do loop */
//                 p_state->ReqSubStateIndex = 0U;
//                 p_state->DispatchState = DISPATCH_STATE_ACTIVE;

//                 /* PROC may have produced initial response — CONTINUE triggers tx */
//                 return (p_state->TxLength > 0U) ? PROTOCOL_REQ_CODE_CONTINUE : PROTOCOL_REQ_CODE_AWAIT_RX;
//             }

//         case DISPATCH_STATE_ACTIVE:
//             {
//                 /* ── Do ── */
//                 Protocol_ReqCode_T code = p_state->p_ReqActive->PROC_EXT(p_socket->P_APP_CONTEXT, &p_state->ReqCtx);

//                 if (code == PROTOCOL_REQ_CODE_COMPLETE || code == PROTOCOL_REQ_CODE_ABORT)
//                 {
//                     p_state->DispatchState = DISPATCH_STATE_IDLE;
//                     p_state->ReqSubStateIndex = 0U;
//                 }

//                 return code;
//             }
//     }
//     return PROTOCOL_REQ_CODE_ERROR_ID;
// }
// static void Socket_Proc(const Socket_T * p_socket, Socket_State_T * p_state)
// {
//     Protocol_RxCode_T rxCode = ProcRxParse(p_socket, p_state);

//     SyncResult_T syncResult = Sync_Proc(&p_state->Sync, rxCode); // additional transport processing

//     if (syncResult == SYNC_DATA_READY)
//     {
//         ReqCode_T reqCode = ProcDispatch(p_socket, p_state);

//         // Tx response if handler produced one
//         if (p_state->TxLength > 0U)
//         {
//             p_state->p_Specs->BUILD_TX_HEADER(p_socket->P_TX_PACKET_BUFFER, &p_state->TxMeta);
//             Xcvr_TxN(p_state->p_Xcvr, p_socket->P_TX_PACKET_BUFFER, p_state->TxLength);
//         }

//         // Sync wraps the Tx with ack if needed
//         if (reqCode == REQ_CODE_COMPLETE || reqCode == REQ_CODE_CONTINUE) Sync_AfterTx(&p_state->Sync, p_state->p_ActiveReq->SYNC);
//     }
// }

// void Socket_Proc(const Socket_T * p_socket, Socket_State_T * p_state)
// {
//     /* ── Layer 1: Transport ── move bytes */
//     ProcRxTransport(p_socket, p_state);

//     /* ── Layer 2: Parse ── interpret bytes */
//     Protocol_RxCode_T rxCode = ProcRxParse(p_socket, p_state);

//     /* ── Layer 3: Sync ── reliable delivery */
//     const Protocol_ReqSync_T * p_syncOpts =
//         (p_state->p_ReqActive != NULL) ? &p_state->p_ReqActive->SYNC : NULL;

//     /* Timeout check — before processing rx */
//     if (ProcSyncTimeout(p_socket, p_state)) return;

//     Sync_RxResult_T syncResult = ProcSyncRx(p_socket, p_state, rxCode, p_syncOpts);

//     switch (syncResult)
//     {
//         case SYNC_RX_DATA_READY:
//             {
//                 /* ── Layer 4: Dispatch ── call handler */
//                 Protocol_ReqCode_T reqCode = ProcDispatch(p_socket, p_state);

//                 switch (reqCode)
//                 {
//                     case PROTOCOL_REQ_CODE_CONTINUE:
//                         /* Handler produced response — tx it, sync wraps with ack */
//                         ProcSyncAfterTx(p_socket, p_state, p_syncOpts);
//                         break;

//                     case PROTOCOL_REQ_CODE_COMPLETE:
//                         /* Final response — tx it, then done */
//                         if (p_state->TxLength > 0U)
//                             ProcSyncAfterTx(p_socket, p_state, p_syncOpts);
//                         /* DispatchState already reset to IDLE by ProcDispatch */
//                         break;

//                     case PROTOCOL_REQ_CODE_AWAIT_RX:
//                         /* No response — wait for next packet */
//                         break;

//                     case PROTOCOL_REQ_CODE_ACK:
//                         /* Handler validated Rx, wants explicit ack (deferred ack) */
//                         TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_ACK_REQ_EXT);
//                         break;

//                     case PROTOCOL_REQ_CODE_NACK:
//                         /* Handler rejected Rx data */
//                         TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ_EXT);
//                         break;

//                     case PROTOCOL_REQ_CODE_ABORT:
//                         p_state->Sync.State = SYNC_STATE_IDLE;
//                         break;

//                     case PROTOCOL_REQ_CODE_ERROR_ID:
//                         TxSync(p_socket, p_state, PROTOCOL_TX_SYNC_NACK_REQ);
//                         break;
//                 }
//                 break;
//             }

//         case SYNC_RX_ACK_RECEIVED:
//             /* Remote acked our response — continue PROC_EXT if active */
//             if (p_state->DispatchState == DISPATCH_STATE_ACTIVE)
//             {
//                 Protocol_ReqCode_T reqCode = ProcDispatch(p_socket, p_state);
//                 if (reqCode == PROTOCOL_REQ_CODE_CONTINUE)
//                     ProcSyncAfterTx(p_socket, p_state, p_syncOpts);
//             }
//             break;

//         case SYNC_RX_ABORT:
//             p_state->DispatchState = DISPATCH_STATE_IDLE;
//             p_state->Sync.State = SYNC_STATE_IDLE;
//             break;

//         case SYNC_RX_WAITING:
//             break;

//         case SYNC_RX_TIMEOUT:
//         case SYNC_RX_ERROR:
//             p_state->DispatchState = DISPATCH_STATE_IDLE;
//             p_state->Sync.State = SYNC_STATE_IDLE;
//             break;
//     }
// }

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
Stateless (VarRead) — PROC only, SYNC_DISABLE

Transport:  bytes arrive → RxIndex advances
Parse:      PACKET_COMPLETE
Sync:       IDLE + PACKET_COMPLETE → DATA_READY (no auto-ack)
Dispatch:   IDLE → call PROC → TxLength set → return COMPLETE
Compose:    COMPLETE + TxLength > 0 → ProcSyncAfterTx → TxResp (no RX_ACK wait)
One main loop iteration. No state persists.
*/

/*
Stateless with Ack (Call) — PROC only, SYNC(TX_ACK=1, RX_ACK=1)

Iteration 1:
  Parse:    PACKET_COMPLETE
  Sync:     DATA_READY, auto Tx Ack (TX_ACK=1)
  Dispatch: call PROC → return COMPLETE
  Compose:  ProcSyncAfterTx → TxResp, enter WAIT_RX_ACK (RX_ACK=1)

Iteration 2..N:
  Parse:    AWAIT_PACKET
  Sync:     WAIT_RX_ACK + AWAIT → WAITING

Iteration M:
  Parse:    ACK received
  Sync:     WAIT_RX_ACK + ACK → ACK_RECEIVED, back to IDLE
  Compose:  DispatchState == IDLE → done
*/

/*
Stateful (DataModeRead) — PROC + PROC_EXT, SYNC_EXT(TX_ACK=1, TX_ACK_EXT=1)

Iteration 1:
  Parse:    PACKET_COMPLETE (DataModeRead request)
  Sync:     DATA_READY, auto Tx Ack (TX_ACK=1)
  Dispatch: IDLE → call PROC (init, build status resp) → PROC_EXT exists → ACTIVE, return CONTINUE
  Compose:  ProcSyncAfterTx → TxResp

Iteration 2:
  Parse:    PACKET_COMPLETE (host sends next request or ack)
  Sync:     DATA_READY, auto Tx Ack (TX_ACK_EXT=1)
  Dispatch: ACTIVE → call PROC_EXT → builds data chunk → return CONTINUE
  Compose:  ProcSyncAfterTx → TxResp

  ... repeat ...

Iteration N:
  Dispatch: ACTIVE → call PROC_EXT → last chunk → return COMPLETE
  Compose:  ProcSyncAfterTx, DispatchState → IDLE
*/

/*
Handler-Driven Ack (DataModeWrite) — handler validates before acking

  Table: SYNC_EXT(TX_ACK=0, TX_ACK_EXT=0)  ← no auto-ack

  Parse:    PACKET_COMPLETE (data chunk)
  Sync:     DATA_READY (no auto-ack, TX_ACK_EXT=0)
  Dispatch: ACTIVE → call PROC_EXT → validate data → return ACK (or NACK)
  Compose:  ACK → TxSync(ACK_REQ_EXT)
            NACK → TxSync(NACK_REQ_EXT)
*/
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



// typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)    (Datagram_T * p_datagram, uint8_t * p_txPacket,  packet_size_t * p_txSize, const uint8_t * p_rxPacket, packet_size_t rxSize);
// typedef void (*Protocol_DatagramInit_T) (void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, packet_size_t * p_txSize, const uint8_t * p_rxPacket, packet_size_t rxSize);

/* For Stateful DataMode Read/Write */
// typedef struct  Protocol_DataModeState
// {
//     uintptr_t Address;
//     size_t Size;
//     size_t Index;
//     //  Protocol_StateId_T StateId;
// }
// Protocol_DataModeState_T;



// Stateful handler — uses phase + sub - state:
// Protocol_ReqCode_T DataModeRead(const void * p_mc, Protocol_HandlerContext_T * p_ctx)
// {
//     MotProtocol_DataModeState_T * p_xfer = p_ctx->p_SubState;

//     switch (*p_ctx->p_Phase)
//     {
//         case 0:
//             { // Init — parse request, begin transfer
//                 const MotPacket_DataModeReq_T * p_req = p_ctx->p_RxPayload;
//                 p_xfer->DataModeAddress = p_req->AddressStart;
//                 p_xfer->DataModeSize = p_req->SizeBytes;
//                 p_xfer->DataIndex = 0;

//                 MotPacket_DataModeResp_T * p_resp = p_ctx->p_TxPayload;
//                 p_resp->Status = MOT_STATUS_SUCCESS;
//                 *p_ctx->p_TxPayloadSize = sizeof(MotPacket_DataModeResp_T);
//                 *p_ctx->p_Phase = 1;
//                 return PROTOCOL_REQ_CODE_TX_CONTINUE;
//             }
//         case 1:
//             { // Data — stream chunks
//                 uint16_t remaining = p_xfer->DataModeSize - p_xfer->DataIndex;
//                 uint16_t chunkSize = (remaining > 32) ? 32 : remaining;

//                 memcpy(p_ctx->p_TxPayload, (void *)(p_xfer->DataModeAddress + p_xfer->DataIndex), chunkSize);
//                 *p_ctx->p_TxPayloadSize = chunkSize;
//                 p_xfer->DataIndex += chunkSize;

//                 if (p_xfer->DataIndex >= p_xfer->DataModeSize)
//                     *p_ctx->p_Phase = 2;
//                 return PROTOCOL_REQ_CODE_TX_CONTINUE;
//             }
//         case 2:
//             { // Complete
//                 *p_ctx->p_Phase = 0;
//                 return PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
//             }
//     }
// }


// // Table entry
// PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ, DataModeReadInit, DataModeReadProc, SYNC_EXT(...))

// // PROC — Entry: parse request, validate, send initial response
// static packet_size_t DataModeReadInit(const MotorController_T * p_mc,
//     MotPacket_T * p_txPacket, const MotPacket_T * p_rxPacket)
// {
//     // Parse and validate the data mode request
//     const MotPacket_DataModeReq_T * p_req =
//         (const MotPacket_DataModeReq_T *)p_rxPacket->Payload;

//     // Validate address range, permissions, etc.
//     // Initialize state will happen via PROC_EXT first call

//     // Build initial ack response (DataModeResp with status)
//     MotPacket_DataModeResp_T * p_resp =
//         (MotPacket_DataModeResp_T *)p_txPacket->Payload;
//     p_resp->Status = MOT_STATUS_SUCCESS;

//     return MotPacket_DataModeReadResp_Build(p_txPacket, MOT_STATUS_SUCCESS);
// }

// // PROC_EXT — Do: stream data chunks until complete
// static Protocol_ReqCode_T DataModeReadProc(const MotorController_T * p_mc,
//     Protocol_ReqContext_T * p_ctx)
// {
//     MotProtocol_DataModeState_T * p_xfer = p_ctx->p_SubState;
//     MotPacket_T * p_tx = p_ctx->p_TxPacket;

//     switch (*p_ctx->p_SubStateIndex)
//     {
//         case 0:
//             { // stream chunks
//                 uint16_t remaining = p_xfer->DataModeSize - p_xfer->DataIndex;
//                 uint8_t chunkSize = (remaining > 32) ? 32 : (uint8_t)remaining;

//                 *p_ctx->p_TxSize = MotPacket_ByteData_Build(p_tx,
//                     (const uint8_t *)(p_xfer->DataModeAddress + p_xfer->DataIndex), chunkSize);
//                 p_xfer->DataIndex += chunkSize;

//                 if (p_xfer->DataIndex >= p_xfer->DataModeSize)
//                     *p_ctx->p_SubStateIndex = 1U;

//                 return PROTOCOL_REQ_CODE_TX_CONTINUE;
//             }
//         case 1: // final
//             *p_ctx->p_TxSize = MotPacket_DataModeReadResp_Build(p_tx, MOT_STATUS_SUCCESS);
//             return PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
//     }
// }
