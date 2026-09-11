#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @brief  The instance. Binding, configuration, lifecycle.
*/
/******************************************************************************/
#include "Protocol.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Protocol.h is the engine: given a link, a service and a state, run one pass. It has no
    opinion about where those came from.

    This file is what a socket instance is - one Xcvr and one format, chosen from tables,
    with the buffers and the NVM configuration that belong to this port and not to the
    protocol. Nothing here parses, dispatches or acknowledges.

    The split matters because selection is runtime and the engine's inputs are const. The
    link is assembled per pass from the current selection, which costs a handful of stores
    and keeps Protocol.h free of any notion that a format can change. The request table and
    its context need no such view - they are const fields of Socket_T already, so they pass
    straight through.

        Socket_T        const, per instance   buffers, tables, timer
        Socket_State_T  mutable               selection, config, engine state
        Protocol_Base_T view, per pass        the transport binding in effect

    Selection is admitted only while the socket is idle. Swapping a format mid-exchange
    would leave a staged response built to one header shape and acked against another, so
    that is lifecycle reasoning and it belongs here rather than in the engine.
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Status - observation only. The socket does not read it back.
*/
/******************************************************************************/
typedef enum Socket_Status
{
    SOCKET_STATUS_DISABLED,
    SOCKET_STATUS_IDLE,             /* Enabled, nothing in flight */
    SOCKET_STATUS_RX_FRAME,         /* A frame is part way in */
    SOCKET_STATUS_BUSY,             /* An exchange is in flight */
}
Socket_Status_T;

/******************************************************************************/
/*!
    Config - NVM backed, writable at runtime through the socket's own protocol
*/
/******************************************************************************/
typedef struct Socket_Config
{
    uint8_t XcvrId;             /* Index into P_XCVR_TABLE */
    uint8_t FormatId;           /* Index into P_FORMAT_TABLE */
    uint32_t BaudRate;
    uint32_t RxTimeout;         /* Frame deadline */
    uint32_t ReqTimeout;        /* Exchange deadline */
    bool IsEnableOnInit;
}
Socket_Config_T;

/******************************************************************************/
/*!
    State
*/
/******************************************************************************/
typedef struct Socket_State
{
    /* Selection. Pointers into the instance's const tables. */
    const Xcvr_T * p_Xcvr;
    const Packet_Format_T * p_Format;

    Socket_Config_T Config;     /* Working copy, loaded from NVM at init */
    Protocol_State_T Protocol;  /* Parser + sync + request + counters */

    bool IsEnabled;
}
Socket_State_T;

/******************************************************************************/
/*!
    Instance
*/
/******************************************************************************/
typedef const struct Socket
{
    Socket_State_T * P_SOCKET_STATE;

    Protocol_Base_T PROTOCOL;

    /* Selectable bindings. Arrays of pointers - neither need be contiguous. */
    const Xcvr_T * const * P_XCVR_TABLE;
    uint8_t XCVR_COUNT;
    const Packet_Format_T * const * P_FORMAT_TABLE;
    uint8_t FORMAT_COUNT;

    const Socket_Config_T * P_NVM_CONFIG;   /* Initial config. The clock lives in PROTOCOL.P_TIMER. */
}
Socket_T;

/******************************************************************************/
/*!
    Query
*/
/******************************************************************************/
static inline bool Socket_IsEnabled(const Socket_T * p_socket) { return p_socket->P_SOCKET_STATE->IsEnabled; }

/*! true while an exchange occupies the socket. Selection is refused in this condition. */
static inline bool Socket_IsBusy(const Socket_T * p_socket)
{
    return Protocol_IsReqSyncActive(&p_socket->P_SOCKET_STATE->Protocol);
}

static inline Socket_Status_T Socket_StatusOf(const Socket_T * p_socket)
{
    const Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (p_state->IsEnabled == false)                                    { return SOCKET_STATUS_DISABLED; }
    if (Protocol_IsReqSyncActive(&p_state->Protocol) == true)                { return SOCKET_STATUS_BUSY; }
    if (Packet_IsRxWaiting(&p_state->Protocol.RxParser) == true) { return SOCKET_STATUS_RX_FRAME; }
    return SOCKET_STATUS_IDLE;
}

// /*
//     Watchdog
// */
// /*!
//     @return true if WatchdogTimeout reached, a successful Req has not occurred
// */
// static inline bool Socket_IsRxLost(const Socket_T * p_socket)
// {
//     const Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
//     return ((p_state->IsRxWatchdogEnable == true) && (*p_socket->P_TIMER - p_state->ReqTimeStart > p_state->Config.WatchdogTimeout));
// }

// // static inline bool _Socket_IsRxLost(const Socket_T * p_socket)
// // {
// //     return (*p_socket->P_TIMER - p_socket->P_SOCKET_STATE->ReqTimeStart > p_socket->P_SOCKET_STATE->Config.WatchdogTimeout);
// // }

// static inline void _Socket_EnableRxWatchdog(Socket_State_T * p_socket) { if (p_socket->ReqState != PROTOCOL_REQ_STATE_INACTIVE) { p_socket->IsRxWatchdogEnable = true; } }
// static inline void _Socket_DisableRxWatchdog(Socket_State_T * p_socket) { p_socket->IsRxWatchdogEnable = false; }
// static inline void _Socket_SetRxWatchdogOnOff(Socket_State_T * p_socket, bool isEnable) { if (isEnable == true) { _Socket_EnableRxWatchdog(p_socket); } else { _Socket_DisableRxWatchdog(p_socket); } }

// /*
//     User must reboot. Does propagate set. Current settings remain active until reboot.
// */
// static inline void _Socket_EnableOnInit(Socket_State_T * p_socket) { p_socket->Config.IsEnableOnInit = true; }
// static inline void _Socket_DisableOnInit(Socket_State_T * p_socket) { p_socket->Config.IsEnableOnInit = false; }

/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
static inline void Socket_Disable(const Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    Protocol_Reset(&p_state->Protocol, *p_socket->PROTOCOL.P_TIMER);
    p_state->IsEnabled = false;
}

/*!
    @return false when no binding is selected. An enabled socket always has both.
*/
static inline bool Socket_Enable(const Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if ((p_state->p_Xcvr == NULL) || (p_state->p_Format == NULL)) { return false; }

    Xcvr_ConfigBaudRate(p_state->p_Xcvr, p_state->Config.BaudRate);
    Protocol_Reset(&p_state->Protocol, *p_socket->PROTOCOL.P_TIMER);
    p_state->IsEnabled = true;
    return true;
}

/******************************************************************************/
/*!
    Selection

    Both refuse while busy: a format swap mid-exchange orphans the staged response, and an
    Xcvr swap orphans the bytes already in the frame buffer. Disable first to force it.
*/
/******************************************************************************/
static inline bool Socket_SetXcvr(const Socket_T * p_socket, uint8_t xcvrId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (xcvrId >= p_socket->XCVR_COUNT)     { return false; }
    if (Socket_IsBusy(p_socket) == true)    { return false; }

    p_state->Config.XcvrId = xcvrId;
    p_state->p_Xcvr = p_socket->P_XCVR_TABLE[xcvrId];
    Packet_ResetRx(&p_state->Protocol.RxParser);      /* bytes from the old port are not this frame */

    if (p_state->IsEnabled == true) { Xcvr_ConfigBaudRate(p_state->p_Xcvr, p_state->Config.BaudRate); }
    return true;
}

static inline bool Socket_SetFormat(const Socket_T * p_socket, uint8_t formatId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (formatId >= p_socket->FORMAT_COUNT) { return false; }
    if (Socket_IsBusy(p_socket) == true)    { return false; }

    /*
        formatId is the only runtime input here - it arrives from NVM or over the wire. The
        format's own fields are compile-time constants of a const table, so their bounds are
        the format author's to assert at the definition, not this function's to re-check on
        every selection. See the contract on Packet_Format_T.
    */
    p_state->Config.FormatId = formatId;
    p_state->p_Format = p_socket->P_FORMAT_TABLE[formatId];
    Packet_ResetRx(&p_state->Protocol.RxParser);      /* a partial frame has no meaning in the new shape */
    return true;
}

static inline bool Socket_SetBaudRate(const Socket_T * p_socket, uint32_t baudRate)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (Socket_IsBusy(p_socket) == true) { return false; }

    p_state->Config.BaudRate = baudRate;
    return (p_state->IsEnabled == true) ? Xcvr_ConfigBaudRate(p_state->p_Xcvr, baudRate) : true;
}

/******************************************************************************/
/*!
    Init
*/
/******************************************************************************/
static inline void Socket_Init(const Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (p_socket->P_NVM_CONFIG != NULL) { p_state->Config = *p_socket->P_NVM_CONFIG; }

    p_state->IsEnabled = false;
    p_state->p_Xcvr = NULL;
    p_state->p_Format = NULL;
    Protocol_Reset(&p_state->Protocol, *p_socket->PROTOCOL.P_TIMER);

    /* Select before enabling, so an out of range stored id leaves the socket down rather than bound to nothing. */
    (void)Socket_SetXcvr(p_socket, p_state->Config.XcvrId);
    (void)Socket_SetFormat(p_socket, p_state->Config.FormatId);

    if (p_state->Config.IsEnableOnInit == true) { (void)Socket_Enable(p_socket); }
}

/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*!
    @brief  One non-blocking pass. Single threaded.
            Cadence is the caller's: the engine reads the clock but never waits on it.

    The link is written out here, at the only place that needs it. Each field comes from its
    own source - selection from state, buffers from the instance, the deadline from config -
    so a builder would only hide where they came from. The request table, its context and its
    deadline pass individually; they are already fields of Socket_T and repackaging them
    would be the same data in a second shape.
*/
static inline void Socket_Proc(const Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (p_state->IsEnabled == false) { return; }

    Protocol_Proc(&p_socket->PROTOCOL, p_state->p_Xcvr, p_state->p_Format, &p_state->Protocol);
}

typedef enum Socket_ConfigId
{
    SOCKET_CONFIG_XCVR_ID,
    SOCKET_CONFIG_SPECS_ID,
    SOCKET_CONFIG_WATCHDOG_TIME,
    SOCKET_CONFIG_BAUD_RATE, // On boot
    SOCKET_CONFIG_IS_ENABLED,
}
Socket_ConfigId_T;

extern int Socket_ConfigId_Get(const Socket_T * p_socket, Socket_ConfigId_T id);
extern void Socket_ConfigId_Set(const Socket_T * p_socket, Socket_ConfigId_T id, int value);