/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @brief  Simple general configurable protocol
    @version V0
*/
/******************************************************************************/
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "Config.h"
//#include "Datagram.h"

// #if defined(CONFIG_PROTOCOL_XCVR_ENABLE)
#include "Peripheral/Xcvr/Xcvr.h"
// #endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/*!
    Rx Parser
    User provide function to determine completion of rx packet.
    returns reqid code and sets pointer upon completion.
*/
/******************************************************************************/
typedef uint8_t protocol_reqid_t; /* Index into Req Function Table P_REQ_TABLE */

/*
    User functions return status - communicate module handled behaviors
*/
typedef enum Protocol_RxCode_Tag
{
    PROTOCOL_RX_CODE_WAIT_PACKET,         /* Wait BuildRxPacket */
    PROTOCOL_RX_CODE_PACKET_COMPLETE,     /* Success Complete Req/ReqExt/Data Packet */
    PROTOCOL_RX_CODE_PACKET_ERROR,         /* Error Req Packet Checksum/CRC */
    PROTOCOL_RX_CODE_PACKET_TIMEOUT,     /* Error Timeout */

    PROTOCOL_RX_CODE_SYNC_ERROR,

    /* Sync packets allocated RxCode */
    PROTOCOL_RX_CODE_ACK,
    PROTOCOL_RX_CODE_NACK,
    PROTOCOL_RX_CODE_ABORT,

    /* Special Context Functions */
    //    PROTOCOL_RX_CODE_REQ_VAR,
    //    PROTOCOL_RX_CODE_REQ_DATAGRAM,
    //    PROTOCOL_RX_CODE_REQ_FLASH,
}
Protocol_RxCode_T;

typedef Protocol_RxCode_T(*Protocol_ParseRxMeta_T)(protocol_reqid_t * p_reqId, size_t * p_packetLength, const void * p_rxPacket, size_t rxCount);
// typedef protocol_reqid_t(*Protocol_ParseRxMetaExt_T)(void * p_subState, protocol_reqid_t reqId, const void * p_rxPacket, size_t rxCount);
// seperate?
// typedef Protocol_RxCode_T(*Protocol_ParseRxLength_T)(size_t * p_packetLength, const void * p_rxPacket, size_t rxCount);
// typedef Protocol_RxCode_T(*Protocol_ParseRxId_T)(protocol_reqid_t * p_reqId, const void * p_rxPacket, size_t rxCount);

//use stateful to parse respid mismatch or wait till req proc?
// typedef Protocol_RxCode_T(*Protocol_ParseRxMeta_T)(void * p_subState, protocol_reqid_t * p_reqId, size_t * p_packetLength, const void * p_rxPacket, size_t rxCount);
// pass arguments as contiguous memory?
// typedef const struct
// {
//     void * const P_APP_INTERFACE;
//     void * const P_SUBSTATE_BUFFER;
//     uint8_t * const P_TX_PACKET_BUFFER;
//     const uint8_t * const P_RX_PACKET_BUFFER;
//     //size_t * const P_TX_LENGTH;
//     //const size_t * const P_RX_INDEX;
// }
// Protocol_ArgsInterface_T;

/******************************************************************************/
/*!
    ProcReqResp - interface for Rx and Tx packets
    User provide functions to convert between Packet format and appInterface format

    Passing appInterface => buffer once
    Passing packetInterface, parse p_rxPacket to packetInterface, then call proc appInterface and tx Resp => buffers twice
*/
/******************************************************************************/

/* Stateless Req, simple read write. */
typedef void (*Protocol_ProcReqResp_T)(void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

//split? commander side no following tx
// typedef void (*Protocol_BuildTx_T)(uint8_t * p_txPacket, size_t * p_txLength, const void * p_appInterface);
// typedef void (*Protocol_ParseProcRx_T)(void * p_appInterface, const uint8_t * p_rxPacket, size_t rxLength);
// typedef Protocol_ReqCode_T(*Protocol_BuildTxExt_T)(void * p_subState, uint8_t * p_txPacket, size_t * p_txLength, const void * p_appInterface);
// typedef Protocol_ReqCode_T(*Protocol_ParseProcRxExt_T)(void * p_subState, void * p_appInterface, const uint8_t * p_rxPacket, size_t rxLength);


/*
    Extended stateful req - Support wait, loop, dynamic ack/nack and additional processes
    for wait process state, and template/format behavior
    process context support, flash, datagram, user provide
*/

/* Common Req from child protocol to supported general protocol control, predefined behaviors */
typedef enum Protocol_ReqCode_Tag
{
    PROTOCOL_REQ_CODE_WAIT_PROCESS,                 /* Child Protocol NonBlocking Wait ReqExt processing */
    PROTOCOL_REQ_CODE_WAIT_PROCESS_EXTEND_TIMER,    /* Child Protocol NonBlocking Wait, Extend timer for RxLost and Timeout */
    PROTOCOL_REQ_CODE_PROCESS_COMPLETE,             /* Exit nonblocking wait processing state upon reception */
    PROTOCOL_REQ_CODE_AWAIT_RX_REQ_EXT,             /* Expecting Rx new packet */
    PROTOCOL_REQ_CODE_AWAIT_RX_SYNC,                /* Expecting static ack nack */
    PROTOCOL_REQ_CODE_TX_RESPONSE,
    PROTOCOL_REQ_CODE_TX_ACK,
    PROTOCOL_REQ_CODE_TX_NACK,
}
Protocol_ReqCode_T;

/* Stateful Req function. */
typedef Protocol_ReqCode_T(*Protocol_ProcReqRespExt_T) (void * p_subState, void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

typedef void (*Protocol_ResetExt_T)(void * p_subState);



/*
    Sync Options Select
    Static string sync - Ack Nack Protocol_BuildTxSync_T
    Dynamic ack nack string implementation use Protocol_ReqExtFunction_T
    per req sync. does not includ timeout, and ExtReq initiated sync, as they do not need to be
*/
typedef const union Protocol_ReqSync_Tag
{
    struct
    {
        uint32_t TX_ACK : 1U;     /* Tx Ack on Rx Req Success, PROTOCOL_RX_CODE_PACKET_COMPLETE */
        uint32_t RX_ACK : 1U;     /* Wait for Ack on Req Complete, Tx Response */
        // uint32_t TX_ACK_EXT     : 1U;     /* Use for All Stateful Ext Packets  */
        // uint32_t RX_ACK_EXT     : 1U;
        uint32_t NACK_REPEAT : 3U;     /* Number of repeat TxPacket on Rx Nack, Tx Nack on RxPacket error */
    };
    uint32_t ID;
}
Protocol_ReqSync_T;

#define PROTOCOL_SYNC_ID_DISABLE (0U)
#define PROTOCOL_SYNC_ID_DEFINE(UseTxAck, UseRxAck, NackRepeat) ( (NackRepeat << 2U) | (UseRxAck << 1U) | (UseTxAck) )

/* Protocol_Req_T */
typedef const struct Protocol_Req_Tag
{
    const protocol_reqid_t              ID;
    const Protocol_ProcReqResp_T        PROC;
    const Protocol_ProcReqRespExt_T     PROC_EXT;
    const Protocol_ReqSync_T            SYNC;        /* Stateless ack nack */
    //    const uint32_t     TIMEOUT; /* overwrite common timeout */
}
Protocol_Req_T;

#define PROTOCOL_REQ_DEFINE(ReqId, ProcReqResp, ProcExt, ReqSyncId) { .ID = (protocol_reqid_t)ReqId, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqRespExt_T)ProcExt, .SYNC = { .ID = ReqSyncId, }, }

//todo datagram request
// typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)    (Datagram_T * p_datagram, uint8_t * p_txPacket,  size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);
// typedef void (*Protocol_DatagramInit_T) (void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

/******************************************************************************/
/*!
    Tx Sync
    User supply function to build Tx Ack, Nack, etc
*/
/******************************************************************************/
typedef enum Protocol_TxSyncId_Tag
{
    PROTOCOL_TX_SYNC_ACK_REQ_ID,
    PROTOCOL_TX_SYNC_NACK_REQ_ID,
    PROTOCOL_TX_SYNC_ABORT,
    PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT,
    PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT,
    PROTOCOL_TX_SYNC_NACK_PACKET_ERROR,
    PROTOCOL_TX_SYNC_ACK_REQ_EXT,
    PROTOCOL_TX_SYNC_NACK_REQ_EXT,
}
Protocol_TxSyncId_T;

typedef void (* const Protocol_BuildTxSync_T)(uint8_t * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId);

/*
    Packet Format Specs
*/
typedef const struct Protocol_Specs_Tag
{
    const uint8_t RX_LENGTH_MIN;                    /* Rx this many bytes before calling PARSE_RX */
    const uint8_t RX_LENGTH_MAX;
    const Protocol_ParseRxMeta_T PARSE_RX_META;     /* Parse Header for RxReqId and RxRemaining, and check data */
    const Protocol_BuildTxSync_T BUILD_TX_SYNC;     /* Build Sync Packets */

    /* Req Resp Table */
    const Protocol_Req_T * const P_REQ_TABLE;       /* Protocol_Req_T */
    const uint8_t REQ_TABLE_LENGTH;
    const Protocol_ResetExt_T REQ_EXT_RESET;        /* Alternatively, handle in req by user */

    /* Optional */
    const uint32_t RX_START_ID;             /* set to 0x00 for not applicable */
    const uint32_t RX_END_ID;
    const uint32_t BAUD_RATE_DEFAULT;
    const uint32_t RX_TIMEOUT;              /* Reset cumulative per packet, until user reset RX_TIMEOUT_PACKET */
    // const uint32_t RX_TIMEOUT_BYTE;      /* Reset per byte *///todo as delimeter
    const uint32_t REQ_TIMEOUT;             /* checked for stateful Req only */
    const uint8_t NACK_COUNT;
    const bool ENCODED;                        /* TODO Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */

    // alternative to PARSE_RX_META
    // const size_t RX_REQ_ID_INDEX;    65535 to disable, check RxIndex < RX_REQ_ID_INDEX
    // const size_t RX_LENGTH_INDEX;
    // const size_t RX_HEADER_LENGTH;     fixed header length known to include contain data length value

    /* Cmdr side only *///todo change for inheritance, runtime polymorphism not needed
    // const Protocol_Cmdr_BuildTxReq_T CMDR_BUILD_TX_REQ; /* Single build function is sufficient. Rx Share Protocol_Proc() and P_REQ_TABLE. Alternatively, function table */
}
Protocol_Specs_T;

typedef enum Protocol_RxState_Tag
{
    PROTOCOL_RX_STATE_INACTIVE,
    PROTOCOL_RX_STATE_AWAIT_BYTE_1,
    PROTOCOL_RX_STATE_AWAIT_PACKET,
    PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL,
}
Protocol_RxState_T;

typedef enum Protocol_ReqState_Tag
{
    PROTOCOL_REQ_STATE_INACTIVE,
    PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_ID,
    PROTOCOL_REQ_STATE_WAIT_RX_COMPLETE_EXT, /* Wait for next ReqExt packet in stateful routine */
    PROTOCOL_REQ_STATE_WAIT_RX_SYNC,
    PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL,
    PROTOCOL_REQ_STATE_WAIT_PROCESS,        /* Wait for ReqExt process */
}
Protocol_ReqState_T;

typedef struct __attribute__((aligned(2U))) Protocol_Params_Tag
{
    uint8_t XcvrId;
    uint8_t SpecsId;
    uint32_t WatchdogTime;    //Watchdog Time
    uint32_t BaudRate;
    //RxTimeOutPacket
    //RxTimeOutByte
    bool IsEnableOnInit;     /* enable on start up */
}
Protocol_Params_T;

/*

*/
typedef const struct Protocol_Config_Tag
{
    uint8_t * const P_RX_PACKET_BUFFER;
    uint8_t * const P_TX_PACKET_BUFFER;
    const uint8_t PACKET_BUFFER_LENGTH;                         /* Must be greater than Specs RX_LENGTH_MAX */
    void * const P_APP_INTERFACE;                               /* User app context for packet processing */
    void * const P_SUBSTATE_BUFFER;                             /* Child protocol control variables, may be seperate from app_interface, must be largest enough to hold substate context referred by specs */
    const Protocol_Specs_T * const * const PP_SPECS_TABLE;      /* Bound and verify specs selection. Pointer to table of pointers to Specs, Specs not necessarily in a contiguous array */
    const uint8_t SPECS_COUNT;
    const volatile uint32_t * const P_TIMER;
    const Protocol_Params_T * const P_PARAMS;
}
Protocol_Config_T;

#define _PROTOCOL_XCVR_INIT(p_XcvrTable, TableLength) .Xcvr = XCVR_INIT(p_XcvrTable, TableLength),

#define PROTOCOL_INIT(p_RxBuffer, p_TxBuffer, PacketBufferLength, p_AppInterface, p_SubstateBuffer, p_SpecsTable, SpecsCount, p_XcvrTable, XcvrCount, p_Timer, p_Params)    \
{                                                                \
    .CONFIG =                                                    \
    {                                                            \
        .P_RX_PACKET_BUFFER     = p_RxBuffer,                    \
        .P_TX_PACKET_BUFFER     = p_TxBuffer,                    \
        .PACKET_BUFFER_LENGTH   = PacketBufferLength,            \
        .P_APP_INTERFACE        = p_AppInterface,                \
        .P_SUBSTATE_BUFFER      = p_SubstateBuffer,              \
        .PP_SPECS_TABLE         = p_SpecsTable,                  \
        .SPECS_COUNT            = SpecsCount,                    \
        .P_TIMER                = p_Timer,                       \
        .P_PARAMS               = p_Params,                      \
    },                                                           \
    _PROTOCOL_XCVR_INIT(p_XcvrTable, XcvrCount)                  \
}

typedef struct Protocol_Tag
{
    const Protocol_Config_T CONFIG;
    Protocol_Params_T Params;
    Xcvr_T Xcvr;
    const Protocol_Specs_T * p_Specs;
    //    Datagram_T Datagram; //configurable broadcast

    Protocol_RxState_T RxState;
    Protocol_RxCode_T RxStatus;     /* Returned from child function, also return to caller. updated per proc */
    size_t RxIndex;                 /* AKA RxCount */
    size_t RxLength;                /* Rx Packet Total Length. From Parse with PARSE_RX_META */
    uint32_t RxTimeStart;

    Protocol_ReqState_T ReqState;
    Protocol_ReqCode_T ReqStatus;    /* Returned from child function, also return to caller. updated per proc */
    protocol_reqid_t ReqIdActive;    /* protocol_reqid_t values defined by child module. Index in P_REQ_TABLE, Parse with PARSE_RX_META  */
    Protocol_Req_T * p_ReqActive;    /* */
    uint32_t ReqTimeStart;           /* Set on Req Start and Complete */

    size_t TxLength;
    uint8_t NackCount;

    uint16_t TxPacketCount;
    uint16_t RxPacketSuccessCount;
    uint16_t RxPacketErrorCount;
}
Protocol_T;

static inline Protocol_RxCode_T Protocol_GetRxStatus(const Protocol_T * p_protocol) { return p_protocol->RxStatus; }
static inline Protocol_ReqCode_T Protocol_GetReqStatus(const Protocol_T * p_protocol) { return p_protocol->ReqStatus; }

static inline uint16_t Protocol_GetTxPacketCount(const Protocol_T * p_protocol) { return p_protocol->TxPacketCount; }
static inline uint16_t Protocol_GetRxPacketCount(const Protocol_T * p_protocol) { return p_protocol->RxPacketErrorCount + p_protocol->RxPacketSuccessCount; }
// static inline void Protocol_ClearTxPacketCount(Protocol_T * p_protocol) { p_protocol->TxPacketCount = 0U; }
static inline void Protocol_ClearTxRxPacketCount(Protocol_T * p_protocol)
{
    p_protocol->TxPacketCount = 0U;
    p_protocol->RxPacketSuccessCount = 0U;
    p_protocol->RxPacketErrorCount = 0U;
}


/*
    User must reboot. Does propagate set. Current settings remain active until reboot.
*/
static inline void Protocol_EnableOnInit(Protocol_T * p_protocol) { p_protocol->Params.IsEnableOnInit = true; }
static inline void Protocol_DisableOnInit(Protocol_T * p_protocol) { p_protocol->Params.IsEnableOnInit = false; }


/*
    Extern
*/
extern const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, protocol_reqid_t id);
extern void Protocol_Init(Protocol_T * p_protocol);
extern void Protocol_Proc(Protocol_T * p_protocol);
extern bool Protocol_CheckRxLost(Protocol_T * p_protocol);
extern void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId);
extern void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate);
extern void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t specsId);
extern bool Protocol_Enable(Protocol_T * p_protocol);
extern void Protocol_Disable(Protocol_T * p_protocol);

extern void Protocol_Debug(Protocol_T * p_protocol);


#endif

