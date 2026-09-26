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
    @file   Socket.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Protocol.h"
#include "Socket.h"
#include <assert.h>
#include <string.h>


/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
void Socket_Disable(Socket_T * p_socket)
{
    Protocol_Reset(p_socket->PROTOCOL.P_STATE);
    p_socket->P_SOCKET_STATE->IsEnabled = false;
}

/*!
    @return false when no Xcvr is selected. BINDING is const, so it is always present.
*/
bool Socket_Enable(Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (p_state->p_Xcvr == NULL) { return false; }

    Xcvr_ConfigBaudRate(p_state->p_Xcvr, p_state->Config.BaudRate);
    Protocol_Reset(p_socket->PROTOCOL.P_STATE);
    /* The link starts measuring from now - otherwise the watchdog carries the age of the
       socket's last enable, and arming it on a long-idle socket faults immediately. */
    Protocol_Init(p_socket->PROTOCOL.P_STATE, Protocol_TimerNow(&p_socket->PROTOCOL));
    p_state->IsEnabled = true;
    return true;
}

/******************************************************************************/
/*!
    Selection

    Refuses while busy: an Xcvr swap orphans the bytes already in the frame buffer. Disable
    first to force it.
*/
/******************************************************************************/
bool Socket_SetXcvr(Socket_T * p_socket, uint8_t xcvrId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (xcvrId >= p_socket->XCVR_COUNT) { return false; }
    if (Socket_IsBusy(p_socket) == true) { return false; }

    p_state->Config.XcvrId = xcvrId;
    p_state->p_Xcvr = p_socket->P_XCVR_TABLE[xcvrId];
    Packet_ResetRx(&p_socket->PROTOCOL.P_STATE->RxParser);   /* bytes from the old port are not this frame */

    if (p_state->IsEnabled == true) { Xcvr_ConfigBaudRate(p_state->p_Xcvr, p_state->Config.BaudRate); }
    return true;
}

bool Socket_SetBaudRate(Socket_T * p_socket, uint32_t baudRate)
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
void Socket_Init(Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    assert(p_socket->PROTOCOL.P_RX_BUFFER != p_socket->PROTOCOL.P_TX_BUFFER); /* the handler's payloads are restrict */

    if (p_socket->P_NVM_CONFIG != NULL) { p_state->Config = *p_socket->P_NVM_CONFIG; }

    p_state->IsEnabled = false;
    p_state->p_Xcvr = NULL;
    Protocol_Init(p_socket->PROTOCOL.P_STATE, Protocol_TimerNow(&p_socket->PROTOCOL));

    /* Select before enabling, so an out of range stored id leaves the socket down rather than bound to nothing. */
    (void)Socket_SetXcvr(p_socket, p_state->Config.XcvrId);

    if (p_state->Config.IsEnableOnInit == true) { (void)Socket_Enable(p_socket); }
}


/******************************************************************************/
/*!
    Var Id interface
*/
/******************************************************************************/
int _Socket_ConfigId_Get(const Socket_State_T * p_socket, Socket_ConfigId_T id)
{
    int value = 0;
    switch (id)
    {
        case SOCKET_CONFIG_XCVR_ID:         value = p_socket->Config.XcvrId;            break;
        case SOCKET_CONFIG_SPECS_ID:        value = p_socket->Config.FormatId;          break;
        case SOCKET_CONFIG_WATCHDOG_TIME:   value = p_socket->Config.WatchdogTimeout;   break;
        case SOCKET_CONFIG_BAUD_RATE:       value = p_socket->Config.BaudRate;          break;
        case SOCKET_CONFIG_IS_ENABLED:      value = p_socket->Config.IsEnableOnInit;    break;
    }
    return value;
}


void _Socket_ConfigId_Set(Socket_State_T * p_socket, Socket_ConfigId_T id, int value)
{
    switch (id)
    {
        case SOCKET_CONFIG_XCVR_ID:         p_socket->Config.XcvrId = value;            break;
        case SOCKET_CONFIG_SPECS_ID:        p_socket->Config.FormatId = value;          break;
        case SOCKET_CONFIG_WATCHDOG_TIME:   p_socket->Config.WatchdogTimeout = value;   break;
        case SOCKET_CONFIG_BAUD_RATE:       p_socket->Config.BaudRate = value;          break;
        case SOCKET_CONFIG_IS_ENABLED:      p_socket->Config.IsEnableOnInit = value;    break;
    }
}

int Socket_ConfigId_Get(Socket_T * p_socket, Socket_ConfigId_T id)
{
    return (p_socket != NULL) ? _Socket_ConfigId_Get(p_socket->P_SOCKET_STATE, id) : 0;
}

void Socket_ConfigId_Set(Socket_T * p_socket, Socket_ConfigId_T id, int value)
{
    if (p_socket != NULL) { _Socket_ConfigId_Set(p_socket->P_SOCKET_STATE, id, value); }
}

