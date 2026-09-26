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

    This file is what a socket instance is - one Xcvr, one [Protocol_T] and one
    [Protocol_Binding_T], with the NVM configuration that belongs to this port and not to the
    protocol. Nothing here parses, dispatches or acknowledges.

        Socket_T        const, per instance   PROTOCOL, BINDING, xcvr table, NVM config
        Socket_State_T  mutable               xcvr selection, config

    The codec, the request table and the app context are one selectable unit - a row's
    ID.FRAME_FORMAT names a shape the codec owns, so they cannot come apart. BINDING is a const
    field for now; making it selectable means a P_BINDING_TABLE here and a p_Binding in
    Socket_State_T, which is where Config.FormatId would then point.

    The Xcvr stays the one independently selected axis: the port is orthogonal to the protocol
    spoken on it. Selection is admitted only while the socket is idle - bytes already in the
    frame buffer do not belong to the new port - and that is lifecycle reasoning, so it lives
    here rather than in the engine.
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
    /* Selection. Pointer into the instance's const table. */
    const Xcvr_T * p_Xcvr;

    Socket_Config_T Config;     /* Working copy, loaded from NVM at init. NVM layout - do not reorder. */

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

    Protocol_T PROTOCOL;            /* This port's buffers, engine state and clock */
    Protocol_Binding_T BINDING;     /* Codec + request table + app context, as one unit */

    uint8_t PACKET_BUFFER_LENGTH;   /* What P_RX_BUFFER and P_TX_BUFFER were allocated with */

#ifdef SOCKET_XCVR_FIXED
    // Xcvr_T * const * P_XCVR; /* const binding */
#endif
    /* Selectable transports. Array of pointers - need not be contiguous. */
    Xcvr_T * const * P_XCVR_TABLE;
    uint8_t XCVR_COUNT;

    const Socket_Config_T * P_NVM_CONFIG;   /* Initial config. The clock is PROTOCOL.P_TIMER. */
}
Socket_T;

static inline Xcvr_T * _Socket_Xcvr(Socket_T * p_socket)
{
#ifdef SOCKET_XCVR_FIXED
    return (Xcvr_T *)p_socket->P_XCVR;
#else
    return p_socket->P_SOCKET_STATE->p_Xcvr;
#endif
}

/******************************************************************************/
/*!
    Proc
*/
/******************************************************************************/
/*!
    @brief  One non-blocking pass. Single threaded.
            Cadence is the caller's: the engine reads the clock but never waits on it.

    Everything the engine needs is already assembled in flash. Only the Xcvr is selected at
    runtime, so only the Xcvr is passed.
*/
static inline void Socket_Proc(Socket_T * p_socket)
{
    if (p_socket->P_SOCKET_STATE->IsEnabled == false) { return; }

    Protocol_Proc(&p_socket->PROTOCOL, &p_socket->BINDING, _Socket_Xcvr(p_socket));
}


/******************************************************************************/
/*!
    Query
*/
/******************************************************************************/
static inline bool Socket_IsEnabled(Socket_T * p_socket) { return p_socket->P_SOCKET_STATE->IsEnabled; }

/*! true while an exchange occupies the socket. Selection is refused in this condition. */
static inline bool Socket_IsBusy(Socket_T * p_socket) { return Protocol_IsReqSyncActive(p_socket->PROTOCOL.P_STATE); }

static inline Socket_Status_T Socket_StatusOf(Socket_T * p_socket)
{
    const Protocol_State_T * p_protocolState = p_socket->PROTOCOL.P_STATE;

    if (p_socket->P_SOCKET_STATE->IsEnabled == false) { return SOCKET_STATUS_DISABLED; }
    if (Protocol_IsReqSyncActive(p_protocolState) == true) { return SOCKET_STATUS_BUSY; }
    if (Packet_IsRxActive(&p_protocolState->RxParser) == true) { return SOCKET_STATUS_RX_FRAME; }
    return SOCKET_STATUS_IDLE;
}

/*
    Watchdog - link liveness, a third deadline at a different scale.

    RX_TIMEOUT bounds a frame and REQ_TIMEOUT bounds an exchange; both are transport
    concerns and both live in the engine. This one asks whether the host is still there at
    all, which only the application can act on - MotorController raises FaultFlags.RxLost
    from it. Hence config here rather than in [Protocol_Binding_T].
*/
/*!
    @return true if WatchdogTimeout reached, a successful Req has not occurred
*/
static inline bool Socket_IsRxLost(Socket_T * p_socket)
{
    const Socket_State_T * p_state = p_socket->P_SOCKET_STATE;
    return ((p_state->IsRxWatchdogEnable == true) &&
            (Protocol_RxLostTime(p_socket->PROTOCOL.P_STATE, Protocol_TimerNow(&p_socket->PROTOCOL)) > p_state->Config.WatchdogTimeout));
}

/*! Arming a disabled socket would fault immediately, since nothing can feed the base. */
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

    Config.FormatId is retained in NVM but selects nothing while BINDING is const.
*/
/******************************************************************************/
extern void Socket_Init(Socket_T * p_socket);
extern bool Socket_Enable(Socket_T * p_socket);
extern void Socket_Disable(Socket_T * p_socket);
extern bool Socket_SetXcvr(Socket_T * p_socket, uint8_t xcvrId);
extern bool Socket_SetBaudRate(Socket_T * p_socket, uint32_t baudRate);


typedef enum Socket_ConfigId
{
    SOCKET_CONFIG_XCVR_ID,
    SOCKET_CONFIG_SPECS_ID,
    SOCKET_CONFIG_WATCHDOG_TIME,
    SOCKET_CONFIG_BAUD_RATE, // On boot
    SOCKET_CONFIG_IS_ENABLED,
}
Socket_ConfigId_T;

extern int Socket_ConfigId_Get(Socket_T * p_socket, Socket_ConfigId_T id);
extern void Socket_ConfigId_Set(Socket_T * p_socket, Socket_ConfigId_T id, int value);