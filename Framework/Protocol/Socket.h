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
    uint32_t WatchdogTimeout;    /* Watchdog deadline */
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
    const Packet_Codec_T * p_Format;

    Socket_Config_T Config;     /* Working copy, loaded from NVM at init */
    Protocol_State_T Protocol;  /* Parser + sync + request + counters */

    bool IsEnabled;
    bool IsRxWatchdogEnable;    /* Link-liveness watchdog, armed at runtime */
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
    const Packet_Codec_T * const * P_FORMAT_TABLE;
    uint8_t FORMAT_COUNT;

    const Socket_Config_T * P_NVM_CONFIG;   /* Initial config. The clock lives in PROTOCOL.P_TIMER. */
}
Socket_T;


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

/*
    Watchdog - link liveness, a third deadline at a different scale.

    RX_TIMEOUT bounds a frame and REQ_TIMEOUT bounds an exchange; both are transport
    concerns and both live in the engine. This one asks whether the host is still there at
    all, which only the application can act on - MotorController raises FaultFlags.RxLost
    from it. Hence config here rather than in Protocol_Base_T.

    It reads Protocol.ReqTimeStart, which advances only on a frame the engine actually
    delivered, so line noise cannot feed it.
*/
/*!
    @return true if WatchdogTimeout reached, a successful Req has not occurred
*/
static inline bool Socket_IsRxLost(const Socket_T * p_socket)
{
    const Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    return ((p_state->IsRxWatchdogEnable == true) && (*p_socket->PROTOCOL.P_TIMER - p_state->Protocol.ReqTimeStart > p_state->Config.WatchdogTimeout));
}

/*! Arming a disabled socket would fault immediately, since nothing can feed ReqTimeStart. */
static inline void _Socket_EnableRxWatchdog(Socket_State_T * p_socket) { if (p_socket->IsEnabled == true) { p_socket->IsRxWatchdogEnable = true; } }
static inline void _Socket_DisableRxWatchdog(Socket_State_T * p_socket) { p_socket->IsRxWatchdogEnable = false; }
static inline void _Socket_SetRxWatchdogOnOff(Socket_State_T * p_socket, bool isEnable) { if (isEnable == true) { _Socket_EnableRxWatchdog(p_socket); } else { _Socket_DisableRxWatchdog(p_socket); } }

/*
    User must reboot. Does propagate set. Current settings remain active until reboot.
*/
static inline void _Socket_EnableOnInit(Socket_State_T * p_socket) { p_socket->Config.IsEnableOnInit = true; }
static inline void _Socket_DisableOnInit(Socket_State_T * p_socket) { p_socket->Config.IsEnableOnInit = false; }


/******************************************************************************/
/*!
    Lifecycle and selection - defined in Socket.c

    Selection is admitted only while the socket is idle; both setters refuse while busy.
    Socket_Init selects before enabling, so an out of range stored id leaves the socket down
    rather than bound to nothing.
*/
/******************************************************************************/
extern void Socket_Init(const Socket_T * p_socket);
extern bool Socket_Enable(const Socket_T * p_socket);
extern void Socket_Disable(const Socket_T * p_socket);
extern bool Socket_SetXcvr(const Socket_T * p_socket, uint8_t xcvrId);
extern bool Socket_SetFormat(const Socket_T * p_socket, uint8_t formatId);
extern bool Socket_SetBaudRate(const Socket_T * p_socket, uint32_t baudRate);


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