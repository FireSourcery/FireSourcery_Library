/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@file	Protocol.h
	@author FireSoucery
	@brief	Simple general configurable protocol
	@version V0
*/
/******************************************************************************/
#ifndef PROTOCOL_CMDR_H
#define PROTOCOL_CMDR_H

#include "Protocol.h"

/*
	Tx Packet using Protocol Xcvr
*/
extern void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
extern void Protocol_Cmdr_StartReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId);

extern bool Protocol_Cmdr_CheckTxIdle(Protocol_T * p_protocol);

#endif

/*
	extern
*/
// extern bool _Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
// extern bool _Protocol_Cmdr_StartReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId);
// extern size_t _Protocol_Cmdr_BuildTxReq(Protocol_T * p_protocol, protocol_reqid_t cmdId);
// extern size_t _Protocol_Cmdr_BuildTxReq_Overwrite(Protocol_T * p_protocol, protocol_reqid_t cmdId);
// extern bool _Protocol_Cmdr_PollTimeout(Protocol_T * p_protocol);
// extern bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol);
