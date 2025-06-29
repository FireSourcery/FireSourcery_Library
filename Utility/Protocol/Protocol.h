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
    @file   Protocol.h
    @author FireSourcery
    @brief  Protocol abstract class equivalent

*/
/******************************************************************************/
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "Config.h"
//#include "Datagram.h"

#include "Peripheral/Xcvr/Xcvr.h"
#include "Type/mux.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint8_t protocol_req_id_t;   /* Index into P_REQ_TABLE. Child module define */
typedef uint8_t protocol_size_t;    /* Packet Size */

/******************************************************************************/
/*!
    Rx Packet Meta/Header Parser
    User app provide child function to determine completion of rx packet.
    returns reqid code via pointer upon completion.
*/
/******************************************************************************/
/*
    User functions return status - communicate module handled behaviors
    overload to eliminate need of addtional parser functions
*/
typedef enum Protocol_RxCode
{
    PROTOCOL_RX_CODE_AWAIT_PACKET,       /* Wait CaptureRx */
    PROTOCOL_RX_CODE_PACKET_COMPLETE,   /* Success Complete Req/ReqExt Packet */

    // PROTOCOL_RX_CODE_ERROR,
    PROTOCOL_RX_CODE_ERROR_TIMEOUT,     /* Error Timeout */
    PROTOCOL_RX_CODE_ERROR_META,        /* Error Req Packet meta data */
    PROTOCOL_RX_CODE_ERROR_DATA,        /* Error Req Packet Checksum/CRC */

    /* Sync packets allocated RxCode */
    // PROTOCOL_RX_CODE_SYNC, /* Sync type for further parsing */
    PROTOCOL_RX_CODE_ACK,
    PROTOCOL_RX_CODE_NACK,
    PROTOCOL_RX_CODE_ABORT,

    /* Special Context Functions, parent module mapped req table */
    //    PROTOCOL_RX_CODE_REQ_VAR,
    //    PROTOCOL_RX_CODE_REQ_DATAGRAM,
    //    PROTOCOL_RX_CODE_REQ_FLASH,
}
Protocol_RxCode_T;

/* Parse with PARSE_RX_META */
typedef struct Protocol_HeaderMeta
{
    protocol_req_id_t ReqId;    /* protocol_req_id_t values defined by child module. Index into P_REQ_TABLE */
    protocol_size_t Length;     /* Rx Packet Total Length. */
}
Protocol_HeaderMeta_T;

typedef Protocol_RxCode_T(*Protocol_ParseRxMeta_T)(Protocol_HeaderMeta_T * p_rxMeta, const void * p_rxPacket, protocol_size_t rxCount);

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
typedef protocol_size_t(*Protocol_ProcReqResp_T)(void * p_appInterface, uint8_t * p_txPacket, const uint8_t * p_rxPacket);
// alternatively,
//req response
// typedef protocol_size_t (*MotProtocol_Fixed_T)(const void * p_context, MotPayload_T * p_txPayload, const MotPayload_T * p_rxPayload);
// typedef protocol_size_t (*MotProtocol_Variable_T)(const void * p_context, MotPayload_T * p_txPayload, const MotPayload_T * p_rxPayload, protocol_size_t rxLength);

// typedef protocol_size_t(*Protocol_ParseProcReq_T)(void * p_appInterface, const uint8_t * p_rxPayload);
// typedef protocol_size_t(*Protocol_BuildResponse_T)(void * p_appInterface, uint8_t * p_txPayload);

/******************************************************************************/
/*
    Extended/Stateful Request - Support wait, loop, dynamic ack/nack and additional processes
*/
/******************************************************************************/
/* Common Req from child protocol to supported general protocol control, predefined behaviors */
typedef enum Protocol_ReqCode
{
    // PROTOCOL_REQ_CODE_AWAIT_RX_REQ_INITIAL,
    PROTOCOL_REQ_CODE_TX_CONTINUE,             /* continue using default sync settings, wait for next packet */
    // PROTOCOL_REQ_CODE_PROCESS_AWAIT_RX,        /* Expecting Rx new packet */
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
}
Protocol_ReqCode_T;

/*
    Stateful Req function, pass args collectively,
    pass as literal and let compiler optimize?
*/
typedef const struct Protocol_ReqContext
{
    void * const p_SubState;
    uint32_t * const p_SubStateIndex;
    const Protocol_HeaderMeta_T * const p_RxMeta;
    const void * const p_RxPacket;
    void * const p_TxPacket;
    protocol_size_t * const p_TxSize;
}
Protocol_ReqContext_T;

typedef Protocol_ReqCode_T(*Protocol_ProcReqExt_T)(void * p_appInterface, Protocol_ReqContext_T * p_interface);
// static inline void Protocol_SetTxLength(Protocol_T * p_protocol, uint8_t tx) { return p_protocol->TxLength = tx; }

typedef void (*Protocol_ResetReqState_T)(void * p_subState);

/*
    Sync Options - Configure per Req, uses common timeout
    Stateless static string sync, ack, nack - Ack Nack Protocol_BuildTxSync_T
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
    const protocol_req_id_t         ID;
    const Protocol_ProcReqResp_T    PROC;
    const Protocol_ProcReqExt_T     PROC_EXT;
    const Protocol_ReqSync_T        SYNC;
    //    const uint32_t     TIMEOUT; /* overwrite common timeout */
}
Protocol_Req_T;

#define PROTOCOL_REQ(ReqId, ProcReqResp, ProcExt, ReqSync) \
    { .ID = (protocol_req_id_t)ReqId, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSync, }

#define PROTOCOL_REQ_EXT(ReqId, ProcReqResp, ProcExt, ReqSyncExt) \
    { .ID = (protocol_req_id_t)ReqId, .PROC = (Protocol_ProcReqResp_T)ProcReqResp, .PROC_EXT = (Protocol_ProcReqExt_T)ProcExt, .SYNC = ReqSyncExt, }




/******************************************************************************/
/*!
//todo datagram request
*/
/******************************************************************************/
// typedef Protocol_ReqCode_T (*Protocol_ReqDatagram_T)    (Datagram_T * p_datagram, uint8_t * p_txPacket,  protocol_size_t * p_txSize, const uint8_t * p_rxPacket, protocol_size_t rxSize);
// typedef void (*Protocol_DatagramInit_T) (void * p_app, void * p_datagramOptions, uint8_t * p_txPacket, protocol_size_t * p_txSize, const uint8_t * p_rxPacket, protocol_size_t rxSize);

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

typedef void (* const Protocol_BuildTxSync_T)(uint8_t * p_txPacket, protocol_size_t * p_txSize, Protocol_TxSyncId_T txId);
// typedef void (* const Protocol_BuildTxHeader_T)(uint8_t * p_txPacket, const uint8_t * p_txPayload, Protocol_TxSyncId_T txId);

/******************************************************************************/
/*!
    Packet Format Specs
    todo split Packet Class
*/
/******************************************************************************/
typedef const struct Protocol_PacketClass
{
    const uint8_t RX_LENGTH_MIN;                    /* Rx this many bytes before calling PARSE_RX */
    const uint8_t RX_LENGTH_MAX;
    const Protocol_ParseRxMeta_T PARSE_RX_META;     /* Parse Header for RxReqId and RxRemaining, and check data */
    const Protocol_BuildTxSync_T BUILD_TX_SYNC;     /* Build Sync Packets */
    // const Protocol_BuildTxHeader_T BUILD_TX_HEADER;     /* todo  */

    /* Optional */
    const uint32_t RX_START_ID;             /* 0x00 for not applicable */
    // const uint32_t RX_END_ID;

    // if all packets common
    const uint32_t RX_TIMEOUT;              /* Reset cumulative per packet */
    // const uint32_t RX_TIMEOUT_BYTE;      /* Reset per byte *///todo as delimeter
    const uint32_t REQ_TIMEOUT;             /* checked for stateful Req only */
    const uint8_t NACK_COUNT;

    //todo defaults over write in protocol Param
    // const uint32_t BAUD_RATE_DEFAULT;

    // alternative to PARSE_RX_META while building
    // const protocol_size_t RX_LENGTH_INDEX;
    // const protocol_size_t RX_REQ_ID_INDEX;
    // const protocol_size_t RX_HEADER_LENGTH;     /* fixed header length known to include contain data length value */

    // const bool ENCODED;                  /* Encoded data, non encoded use TIMEOUT only. No meta chars past first char. */

    /* Req Resp Table */
    const Protocol_Req_T * const P_REQ_TABLE;       /* Protocol_Req_T */
    const uint8_t REQ_TABLE_LENGTH;
    // const Protocol_ResetReqState_T REQ_EXT_RESET;   /* Alternatively, handle in req by user */
}
Protocol_PacketClass_T;

/******************************************************************************/
/*!

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
    PROTOCOL_REQ_STATE_WAIT_RX_CONTINUE,             /* Wait for next ReqExt packet in stateful routine */
    PROTOCOL_REQ_STATE_WAIT_RX_SYNC,
    PROTOCOL_REQ_STATE_PROCESS_REQ_EXT,         /* Wait for ReqExt process */
}
Protocol_ReqState_T;

typedef struct Protocol_Config
{
    uint8_t XcvrId;
    uint8_t SpecsId;
    uint32_t BaudRate;
    uint32_t WatchdogTimeout;
    //uint32_t RxTimeoutPacket
    //uint32_t RxTimeoutByte
    //uint32_t ReqExtTimeout
    //uint8_t NackLimit
    bool IsEnableOnInit;     /* enable on start up */
    // bool IsWatchdogOnInit;
}
Protocol_Config_T;

typedef const struct Protocol_Const
{
    uint8_t * const P_RX_PACKET_BUFFER;
    uint8_t * const P_TX_PACKET_BUFFER;
    const uint8_t PACKET_BUFFER_LENGTH;                         /* Must be greater than Specs RX_LENGTH_MAX */
    void * const P_APP_INTERFACE;                               /* User app context for packet processing */
    void * const P_REQ_STATE_BUFFER;                            /* Child protocol control variables, may be separate from app_interface, must be largest enough to hold substate context referred by specs */
    const volatile uint32_t * const P_TIMER;
    const Protocol_PacketClass_T * const * const P_SPECS_TABLE;      /* Bound and verify specs selection. Pointer to table of pointers to Specs, Specs not necessarily in a contiguous array */
    const uint8_t SPECS_COUNT;
    // const Protocol_Req_T * const P_REQ_TABLE;       /* Protocol_Req_T */
    // const uint8_t REQ_TABLE_LENGTH;
    const Xcvr_T * const * const P_XCVR_TABLE; /* array of struct, or pointers. todo move selection */
    const uint8_t XCVR_COUNT; /* number of Xcvr in table */
    const Protocol_Config_T * const P_CONFIG;
}
Protocol_Const_T;

typedef struct Protocol
{
    const Protocol_Const_T CONST;
    Protocol_Config_T Config;

    const Xcvr_T * p_Xcvr;
    const Protocol_PacketClass_T * p_Specs; /* Active protocol */
    // Datagram_T Datagram; // configurable broadcast

    /* Rx Parse Packet Meta */
    Protocol_HeaderMeta_T RxMeta;

    /* Rx */
    Protocol_RxState_T RxState;
    Protocol_RxCode_T RxStatus;     /* Returned from child function, also return to caller. updated per proc. store as way of retaining 2 return values */
    protocol_size_t RxIndex;        /* Index into P_RX_PACKET_BUFFER, number of bytes received */
    uint32_t RxTimeStart;

    protocol_size_t TxLength;

    /* Req/Response */
    Protocol_ReqState_T ReqState;
    Protocol_ReqCode_T ReqStatus;    /* Returned from child function, also return to caller. updated per proc. store as way of retaining 2 return values */
    Protocol_Req_T * p_ReqActive;    /* */
    uint32_t ReqTimeStart;           /* Set on Req Start and Complete */
    uint32_t ReqSubStateIndex; //track ext req index internally?
    Protocol_ReqContext_T ReqContext; // buffer for passing req parameters. Alternatively, pass const block and TxLength separately

    /* Common */
    uint8_t NackCount;
    uint8_t TxNackRxCount;
    uint8_t RxNackTxCount;

    /* */
    bool IsRxWatchdogEnable;

    /* Debug */
    // uint16_t TxPacketCount;
    // uint16_t RxPacketSuccessCount;
    // uint16_t RxPacketErrorCount;
    // uint16_t RxPacketErrorSync;
}
Protocol_T;


#define PROTOCOL_INIT(p_RxBuffer, p_TxBuffer, PacketBufferLength, p_AppInterface, p_SubStateBuffer, p_SpecsTable, SpecsCount, p_Xcvrs, XcvrCount, p_Timer, p_Config)    \
{                                                                \
    .CONST =                                                     \
    {                                                            \
        .P_RX_PACKET_BUFFER     = p_RxBuffer,                    \
        .P_TX_PACKET_BUFFER     = p_TxBuffer,                    \
        .PACKET_BUFFER_LENGTH   = PacketBufferLength,            \
        .P_APP_INTERFACE        = p_AppInterface,                \
        .P_REQ_STATE_BUFFER     = p_SubStateBuffer,              \
        .P_SPECS_TABLE         = p_SpecsTable,                  \
        .SPECS_COUNT            = SpecsCount,                    \
        .P_TIMER                = p_Timer,                       \
        .P_CONFIG               = p_Config,                      \
        .P_XCVR_TABLE           = p_Xcvrs,                       \
        .XCVR_COUNT             = XcvrCount,                     \
    },                                                           \
}

typedef enum Protocol_ConfigId
{
    PROTOCOL_CONFIG_XCVR_ID,
    PROTOCOL_CONFIG_SPECS_ID,
    PROTOCOL_CONFIG_WATCHDOG_TIME,
    PROTOCOL_CONFIG_BAUD_RATE, // On boot
    PROTOCOL_CONFIG_IS_ENABLED,
}
Protocol_ConfigId_T;

static inline Protocol_RxCode_T Protocol_GetRxStatus(const Protocol_T * p_protocol)     { return p_protocol->RxStatus; }
static inline Protocol_ReqCode_T Protocol_GetReqStatus(const Protocol_T * p_protocol)   { return p_protocol->ReqStatus; }

/*
    Watchdog
*/
/*!
    @return true if WatchdogTimeout reached, a successful Req has not occurred
*/
static inline bool Protocol_IsRxLost(Protocol_T * p_protocol)
{
    return ((p_protocol->IsRxWatchdogEnable == true) && (*p_protocol->CONST.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Config.WatchdogTimeout));
}

static inline void Protocol_EnableRxWatchdog(Protocol_T * p_protocol) { if (p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) { p_protocol->IsRxWatchdogEnable = true; } }
static inline void Protocol_DisableRxWatchdog(Protocol_T * p_protocol) { p_protocol->IsRxWatchdogEnable = false; }
static inline void Protocol_SetRxWatchdogOnOff(Protocol_T * p_protocol, bool isEnable) { (isEnable == true) ? Protocol_EnableRxWatchdog(p_protocol) : Protocol_DisableRxWatchdog(p_protocol); }

/*
    User must reboot. Does propagate set. Current settings remain active until reboot.
*/
static inline void Protocol_EnableOnInit(Protocol_T * p_protocol) { p_protocol->Config.IsEnableOnInit = true; }
static inline void Protocol_DisableOnInit(Protocol_T * p_protocol) { p_protocol->Config.IsEnableOnInit = false; }

/*
    Extern
*/
extern const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, protocol_req_id_t id);
extern void Protocol_Init(Protocol_T * p_protocol);
extern void Protocol_Proc(Protocol_T * p_protocol);
extern void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId);
extern void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate);
extern void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t specsId);
extern bool Protocol_Enable(Protocol_T * p_protocol);
extern void Protocol_Disable(Protocol_T * p_protocol);

extern int32_t Protocol_ConfigId_Get(const Protocol_T * p_protocol, Protocol_ConfigId_T id);
extern void Protocol_ConfigId_Set(Protocol_T * p_protocol, Protocol_ConfigId_T id, uint32_t value);

#endif


// static inline uint16_t Protocol_GetTxPacketCount(const Protocol_T * p_protocol) { return p_protocol->TxPacketCount; }
// static inline uint16_t Protocol_GetRxPacketCount(const Protocol_T * p_protocol) { return p_protocol->RxPacketErrorCount + p_protocol->RxPacketSuccessCount; }

// static inline void Protocol_ClearTxRxPacketCount(Protocol_T * p_protocol)
// {
//     p_protocol->TxPacketCount = 0U;
//     p_protocol->RxPacketSuccessCount = 0U;
//     p_protocol->RxPacketErrorCount = 0U;
// }

// typedef struct Socket
// {
//     Protocol_T * p_Protocol;
//     Xcvr_T * p_Xcvr;
// }
// Socket_T;