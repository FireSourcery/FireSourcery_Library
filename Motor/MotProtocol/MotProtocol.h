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
    @file   MotProtocol.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_PROTOCOL_H
#define MOT_PROTOCOL_H

#include "MotPacket.h"
#include "Utility/Protocol/Protocol.h"

#define MOT_PROTOCOL_BAUD_RATE_DEFAULT  (19200U)
#define MOT_PROTOCOL_TIMEOUT_RX         (2000U)     /* Timeout Rx Packet */
#define MOT_PROTOCOL_TIMEOUT_REQ        (5000U)     /* Timeout Req */

/* For flashloader only */
typedef struct MotProtocol_SubState_Tag
{
    uint8_t StateIndex;
    uint32_t DataModeAddress;
    uint16_t DataModeSize;
    uint16_t SequenceIndex;
    bool IsDataModeActive;
    // uint16_t WriteModeStatus;
    uint8_t OnceBuffer[8U];
}
MotProtocol_SubState_T;

static inline void MotProtocol_ResetSubState(MotProtocol_SubState_T * p_subState) { p_subState->StateIndex = 0U; }

extern void MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId);
extern void MotProtocol_ResetSubState(MotProtocol_SubState_T * p_subState);
extern Protocol_RxCode_T MotProtocol_ParseRxMeta(Protocol_HeaderMeta_T * p_rxMeta, const MotPacket_T * p_rxPacket, size_t rxCount);

#endif
