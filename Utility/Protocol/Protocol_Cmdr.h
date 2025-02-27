/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Protocol.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef PROTOCOL_CMDR_H
#define PROTOCOL_CMDR_H

#include "Protocol.h"


    /* Cmdr side only *///todo wrap in cmdr, runtime polymorphism not needed
    // const Protocol_Cmdr_BuildTxReq_T CMDR_BUILD_TX_REQ; /* Single build function is sufficient. Rx Share Protocol_Proc() and P_REQ_TABLE. Alternatively, function table */

/*
    Extern
*/
/******************************************************************************/
/*!
    Cmdr Tx Req Start
*/
/******************************************************************************/
/* One function handle all cases. Alternatively, use function table */
// typedef void (*Protocol_Cmdr_BuildTxReq_T)(uint8_t * p_txPacket, size_t * p_txLength, const void * p_appInterface, protocol_req_id_t reqId); //size_t * p_rxRemainig,
// // typedef void (*Protocol_Cmdr_BuildReq_T)(uint8_t * p_txPacket, size_t * p_txLength, const void * p_appInterface); //size_t * p_rxRemainig,

// extern void Protocol_Cmdr_StartReq(Protocol_T * p_protocol, protocol_req_id_t cmdId);
// extern void Protocol_Cmdr_StartReq_Overwrite(Protocol_T * p_protocol, protocol_req_id_t cmdId);
// extern bool Protocol_Cmdr_CheckTxIdle(Protocol_T * p_protocol);

#endif

/*

*/
// extern size_t _Protocol_Cmdr_BuildTxReq(Protocol_T * p_protocol, protocol_req_id_t cmdId);
// extern size_t _Protocol_Cmdr_BuildTxReq_Overwrite(Protocol_T * p_protocol, protocol_req_id_t cmdId);
// extern bool _Protocol_Cmdr_PollTimeout(Protocol_T * p_protocol);
// extern bool _Protocol_Cmdr_ParseResp(Protocol_T * p_protocol);
