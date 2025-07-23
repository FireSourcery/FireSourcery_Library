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
    @file   Socket.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Protocol.h"

/******************************************************************************/
/*!
    Protocol_Socket
*/
/******************************************************************************/
typedef struct Socket_Config
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
Socket_Config_T;

typedef struct Socket_State
{
    /*
        Active Refs
    */
    const Xcvr_T * p_Xcvr;
    const PacketClass_T * p_Specs; /* Active protocol */
    // Datagram_T Datagram; // configurable broadcast

    /*
        Runtime State/StateMachines
    */
    /* Rx */
    Protocol_RxState_T RxState;
    Protocol_RxCode_T RxStatus;     /* Returned from child function, also return to caller. updated per proc. store as way of retaining 2 return values */
    packet_size_t RxIndex;          /* Index into P_RX_PACKET_BUFFER, number of bytes received */
    uint32_t RxTimeStart;
    Protocol_HeaderMeta_T RxMeta;   /* Rx Parse Packet Meta */

    /* Tx */
    packet_size_t TxLength;

    /* Req/Response */
    Protocol_ReqState_T ReqState;
    Protocol_ReqCode_T ReqStatus;    /* Returned from child function, also return to caller. updated per proc. store as way of retaining 2 return values */
    Protocol_Req_T * p_ReqActive;    /* */
    uint32_t ReqTimeStart;           /* Set on Req Start and Complete */
    uint32_t ReqSubStateIndex; // track ext req index internally?
    // Protocol_ReqContext_T ReqContext; // buffer for passing req parameters. Alternatively, pass const block and TxLength separately

    /* Protocol_CommState */
    uint8_t NackCount;
    uint8_t TxNackRxCount;
    uint8_t RxNackTxCount;

    /* */
    bool IsRxWatchdogEnable;

    Socket_Config_T Config;

    /* Debug */
    // uint16_t TxPacketCount;
    // uint16_t RxPacketSuccessCount;
    // uint16_t RxPacketErrorCount;
    // uint16_t RxPacketErrorSync;
}
Socket_State_T;

#define SOCKET_STATE_ALLOC() (&(Socket_State_T){0})

typedef const struct Socket
{
    /*
        Per Instance/Socket
    */
    Socket_State_T * P_SOCKET_STATE;
    uint8_t * P_RX_PACKET_BUFFER;
    uint8_t * P_TX_PACKET_BUFFER;
    uint8_t PACKET_BUFFER_LENGTH;           /* Must be greater than Specs RX_LENGTH_MAX */
    void * P_APP_CONTEXT;                   /* User app context for packet processing */
    void * P_REQ_STATE_BUFFER;              /* Session layer. Child protocol control variables, must be largest enough to hold substate context referred by specs */

    const Socket_Config_T * P_NVM_CONFIG;

    /*
        Protocol Context common
    */
   // alternatively Map overlaping sockets for selection
    // const Protocol_Base_T * P_PROTOCOL_CONTEXT;
    const Protocol_Req_T * P_REQ_TABLE;
    uint8_t REQ_TABLE_LENGTH;
    // REQ_TIMEOUT

    const PacketClass_T * const * P_PACKET_CLASS_TABLE;    /* Bound and verify specs selection. Array of pointers, Specs not necessarily in a contiguous array */
    uint8_t PACKET_CLASS_COUNT;

    /*  */
    const Xcvr_T * const * P_XCVR_TABLE; /* array of struct, or pointers. todo move selection */
    uint8_t XCVR_COUNT; /* number of Xcvr in table */
    // alternatively fixed // const Xcvr_T * P_XCVR;

    const volatile uint32_t * P_TIMER;
}
Socket_T;

#define SOCKET_INIT(p_State, p_RxBuffer, p_TxBuffer, BufferLength, p_AppContext, p_SubstateBuffer, p_Config, p_ReqTable, ReqCount, p_PacketClassTable, PacketClassCount, p_XcvrTable, XcvrCount, p_Timer) \
{ \
    .P_SOCKET_STATE         = p_State,                      \
    .P_RX_PACKET_BUFFER     = p_RxBuffer,                   \
    .P_TX_PACKET_BUFFER     = p_TxBuffer,                   \
    .PACKET_BUFFER_LENGTH   = BufferLength,                 \
    .P_APP_CONTEXT          = p_AppContext,                 \
    .P_REQ_STATE_BUFFER     = p_SubstateBuffer,             \
    .P_NVM_CONFIG           = p_Config,                     \
    .P_REQ_TABLE            = p_ReqTable,                   \
    .REQ_TABLE_LENGTH       = ReqCount,                     \
    .P_PACKET_CLASS_TABLE   = p_PacketClassTable,           \
    .PACKET_CLASS_COUNT     = PacketClassCount,             \
    .P_XCVR_TABLE           = p_XcvrTable,                  \
    .XCVR_COUNT             = XcvrCount,                    \
    .P_TIMER                = p_Timer,                      \
}

#define SOCKET_ALLOC(BufferLength, p_AppContext, p_SubstateBuffer, p_Config, p_ReqTable, ReqCount, p_PacketClassTable, PacketClassCount, p_XcvrTable, XcvrCount, p_Timer) \
    SOCKET_INIT(SOCKET_STATE_ALLOC(), (uint8_t[BufferLength]){0}, (uint8_t[BufferLength]){0}, BufferLength, p_AppContext, p_SubstateBuffer, p_Config, \
                p_ReqTable, ReqCount, p_PacketClassTable, PacketClassCount, p_XcvrTable, XcvrCount, p_Timer)


// #define PROTOCOL_INIT(p_RxBuffer, p_TxBuffer, PacketBufferLength, p_AppInterface, p_SubStateBuffer, p_SpecsTable, SpecsCount, p_Xcvrs, XcvrCount, p_Timer, p_Config)    \
// { \
//     .P_RX_PACKET_BUFFER     = p_RxBuffer,                    \
//     .P_TX_PACKET_BUFFER     = p_TxBuffer,                    \
//     .PACKET_BUFFER_LENGTH   = PacketBufferLength,            \
//     .P_APP_CONTEXT          = p_AppInterface,                \
//     .P_REQ_STATE_BUFFER     = p_SubStateBuffer,              \
//     .P_SPECS_TABLE          = p_SpecsTable,                  \
//     .SPECS_COUNT            = SpecsCount,                    \
//     .P_TIMER                = p_Timer,                       \
//     .P_CONFIG               = p_Config,                      \
//     .P_XCVR_TABLE           = p_Xcvrs,                       \
//     .XCVR_COUNT             = XcvrCount,                     \
// }

static inline Protocol_RxCode_T _Socket_GetRxStatus(const Socket_State_T * p_socket) { return p_socket->RxStatus; }
static inline Protocol_ReqCode_T _Socket_GetReqStatus(const Socket_State_T * p_socket) { return p_socket->ReqStatus; }

static inline void _Socket_EnableRxWatchdog(Socket_State_T * p_socket) { if (p_socket->ReqState != PROTOCOL_REQ_STATE_INACTIVE) { p_socket->IsRxWatchdogEnable = true; } }
static inline void _Socket_DisableRxWatchdog(Socket_State_T * p_socket) { p_socket->IsRxWatchdogEnable = false; }
static inline void _Socket_SetRxWatchdogOnOff(Socket_State_T * p_socket, bool isEnable) { (isEnable == true) ? _Socket_EnableRxWatchdog(p_socket) : _Socket_DisableRxWatchdog(p_socket); }

/*
    User must reboot. Does propagate set. Current settings remain active until reboot.
*/
static inline void _Socket_EnableOnInit(Socket_State_T * p_socket) { p_socket->Config.IsEnableOnInit = true; }
static inline void _Socket_DisableOnInit(Socket_State_T * p_socket) { p_socket->Config.IsEnableOnInit = false; }

/*
    Watchdog
*/
/*!
    @return true if WatchdogTimeout reached, a successful Req has not occurred
*/
static inline bool Socket_IsRxLost(const Socket_T * p_socket)
{
    const Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    return ((p_state->IsRxWatchdogEnable == true) && (*p_socket->P_TIMER - p_state->ReqTimeStart > p_state->Config.WatchdogTimeout));
}

/*
    Extern
*/
extern const Protocol_Req_T * _Protocol_SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, packet_id_t id);

extern void Socket_Init(const Socket_T * p_socket);
extern void Socket_Proc(const Socket_T * p_socket);

extern void Socket_SetXcvr(const Socket_T * p_socket, uint8_t xcvrId);
extern void Socket_ConfigXcvrBaudRate(const Socket_T * p_socket, uint32_t baudRate);
extern void Socket_SetSpecs(const Socket_T * p_socket, uint8_t specsId);
extern bool Socket_Enable(const Socket_T * p_socket);
extern void Socket_Disable(const Socket_T * p_socket);


typedef enum Protocol_ConfigId
{
    PROTOCOL_CONFIG_XCVR_ID,
    PROTOCOL_CONFIG_SPECS_ID,
    PROTOCOL_CONFIG_WATCHDOG_TIME,
    PROTOCOL_CONFIG_BAUD_RATE, // On boot
    PROTOCOL_CONFIG_IS_ENABLED,
}
Protocol_ConfigId_T;

extern int Socket_ConfigId_Get(const Socket_T * p_socket, Protocol_ConfigId_T id);
extern void Socket_ConfigId_Set(const Socket_T * p_socket, Protocol_ConfigId_T id, int value);