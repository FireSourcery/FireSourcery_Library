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
    @file   MotProtocol.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotPacket.h"
#include "Framework/Protocol/Extension/Protocol_FlashLoader.h"
#include "Framework/Protocol/Protocol.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

/******************************************************************************/
/*!
    Fixed flash loader - Flash_T bound directly, no generic DataMode interface.

    Handler shaped: register through PROTOCOL_REQ with the Flash_T as P_APP_CONTEXT. Typed
    parameters, cast once at the row. P_SUB_STATE must hold a Protocol_DataMode_State_T.
*/
/******************************************************************************/
extern Protocol_ReqCode_T MotProtocol_FlashLoader_Read(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * restrict p_rx, void * restrict p_resp);
extern Protocol_ReqCode_T MotProtocol_FlashLoader_Write(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const void * restrict p_rx, void * restrict p_resp);
extern Protocol_ReqCode_T MotProtocol_FlashLoader_Erase_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp);


