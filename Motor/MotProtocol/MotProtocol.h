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
	@file 	MotProtocol.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef MOT_PROTOCOL_H
#define MOT_PROTOCOL_H

#include "MotPacket.h"
#include "Utility/Protocol/Protocol_Cmdr.h"

#define MOT_PROTOCOL_BAUD_RATE_DEFAULT		(19200U)
#define MOT_PROTOCOL_TIMEOUT_MS				(2000U) 	/* Timeout packet / req */

typedef struct MotProtocol_Substate_Tag
{
	uint8_t StateId;
}
MotProtocol_Substate_T;

extern void MotProtocol_BuildTxSync(MotPacket_Sync_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId);
extern void MotProtocol_ResetExt(MotProtocol_Substate_T * p_subState);

/******************************************************************************/
/*!
	Cmdr side only
*/
/******************************************************************************/
extern Protocol_RxCode_T MotProtocol_CheckPacket(const MotPacket_T * p_rxPacket);

/******************************************************************************/
/*!
	Ctrlr side only
*/
/******************************************************************************/
extern Protocol_RxCode_T MotProtocol_ParseRxMeta(protocol_reqid_t * p_reqId, size_t * p_rxRemaining, const MotPacket_T * p_rxPacket, size_t rxCount);

#endif