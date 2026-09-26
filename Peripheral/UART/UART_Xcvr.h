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
    @file   UART_Xcvr.h
    @author FireSourcery
    @brief  [UART_T] as an [Xcvr_T].
*/
/******************************************************************************/
#include "UART.h"
#include "../Xcvr/Xcvr.h"

/*
    No target, and no transaction to hold open, so SET_TARGET and SET_FLAGS stay absent.
*/
static const Xcvr_VTable_T UART_XCVR_VTABLE =
{
    .TX                 = (Xcvr_Tx_T)UART_SendN,
    .RX                 = (Xcvr_Rx_T)UART_RecvN,
    .CONFIG_BAUD_RATE   = (Xcvr_SetProperty_T)UART_ConfigBaudRate,
};

#define UART_XCVR_INIT(p_Uart) XCVR_INIT((p_Uart), &UART_XCVR_VTABLE)
