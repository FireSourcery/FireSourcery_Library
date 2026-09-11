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
#include <string.h>

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
void Socket_Proc(const Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (p_state->IsEnabled == false) { return; }

    Protocol_Proc(&p_socket->PROTOCOL, p_state->p_Xcvr, p_state->p_Format, &p_state->Protocol);
}

/******************************************************************************/
/*!
    Lifecycle
*/
/******************************************************************************/
void Socket_Disable(const Socket_T * p_socket)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    Protocol_Reset(&p_state->Protocol, *p_socket->PROTOCOL.P_TIMER);
    p_state->IsEnabled = false;
}

/*!
    @return false when no binding is selected. An enabled socket always has both.
*/
bool Socket_Enable(const Socket_T * p_socket)
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
bool Socket_SetXcvr(const Socket_T * p_socket, uint8_t xcvrId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (xcvrId >= p_socket->XCVR_COUNT) { return false; }
    if (Socket_IsBusy(p_socket) == true) { return false; }

    p_state->Config.XcvrId = xcvrId;
    p_state->p_Xcvr = p_socket->P_XCVR_TABLE[xcvrId];
    Packet_ResetRx(&p_state->Protocol.RxParser);      /* bytes from the old port are not this frame */

    if (p_state->IsEnabled == true) { Xcvr_ConfigBaudRate(p_state->p_Xcvr, p_state->Config.BaudRate); }
    return true;
}

bool Socket_SetFormat(const Socket_T * p_socket, uint8_t formatId)
{
    Socket_State_T * p_state = p_socket->P_SOCKET_STATE;

    if (formatId >= p_socket->FORMAT_COUNT) { return false; }
    if (Socket_IsBusy(p_socket) == true) { return false; }

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

bool Socket_SetBaudRate(const Socket_T * p_socket, uint32_t baudRate)
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
void Socket_Init(const Socket_T * p_socket)
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

int Socket_ConfigId_Get(const Socket_T * p_socket, Socket_ConfigId_T id)
{
    return (p_socket != NULL) ? _Socket_ConfigId_Get(p_socket->P_SOCKET_STATE, id) : 0;
}

void Socket_ConfigId_Set(const Socket_T * p_socket, Socket_ConfigId_T id, int value)
{
    if (p_socket != NULL) { _Socket_ConfigId_Set(p_socket->P_SOCKET_STATE, id, value); }
}


//todo
//static void ProcDatagram(Socket_T * p_socket)
//{
//    //* save room for 1 req packet
//    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_state->Datagram) +  p_socket->PACKET_BUFFER_LENGTH)
//    {
//        if (Datagram_Server_Proc(&p_state->Datagram))
//        {
//            PortTxString(p_state, p_state->Datagram.P_TX_BUFFER, p_state->Datagram.TxDataSizeActive + p_state->Datagram.HeaderSize);
//        }
//    }
//        //    if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_state->Datagram) +  p_socket->PACKET_BUFFER_LENGTH)
//        //    {
//                if (Datagram_Server_Proc(&p_state->Datagram))
//                {
//                    PortTxString(p_state, p_state->Datagram.P_TX_BUFFER, p_state->Datagram.TxDataSizeActive + p_state->Datagram.HeaderSize);
//                }
//        //    }
//}

