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
    @file   Serial_Xcvr.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Serial_Xcvr.h"

const Xcvr_VTable_T SERIAL_XCVR_VTABLE =
{
    .TX_BYTE = (Xcvr_TxByte_T)Serial_SendByte,
    .RX_BYTE = (Xcvr_RxByte_T)Serial_RecvByte,
    .TX_MAX = (Xcvr_TxMax_T)Serial_SendMax,
    .RX_MAX = (Xcvr_RxMax_T)Serial_RecvMax,
    .TX_N = (Xcvr_TxN_T)Serial_SendN,
    .RX_N = (Xcvr_RxN_T)Serial_RecvN,
    .GET_TX_EMPTY_COUNT = (Xcvr_GetCount_T)Serial_GetTxEmptyCount,
    .GET_RX_FULL_COUNT = (Xcvr_GetCount_T)Serial_GetRxFullCount,
    .INIT_BAUD_RATE = (Xcvr_SetConfig_T)Serial_ConfigBaudRate,
};


// static inline void Serial_Interface_(Xcvr_T * p_xcvr )
// {
//     p_xcvr->p_Xcvr.value
// }




