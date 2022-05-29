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
	@file 	Protocol.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Protocol.h"
#include <string.h>

void Protocol_Init(Protocol_T * p_protocol)
{
	if(p_protocol->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_protocol->Params, p_protocol->CONFIG.P_PARAMS, sizeof(Protocol_Params_T));
	}
	else
	{
		p_protocol->Params.SpecsId = 0U;
		p_protocol->Params.IsEnableOnInit = 0U;
	}
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	Xcvr_Init(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
#endif
	Protocol_SetSpecs(p_protocol, p_protocol->Params.SpecsId);

	if(p_protocol->Params.IsEnableOnInit == true)	{ Protocol_Enable(p_protocol); }
	else											{ Protocol_Disable(p_protocol); }
}

/******************************************************************************/
/*! Start CONFIG_PROTOCOL_XCVR_ENABLE */
/******************************************************************************/
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE

#define RX_REMAINING_MAX (65535U)

/*
	Receive into P_RX_PACKET_BUFFER and run PARSE_RX_META for RxRemaining and ReqCode / Rx completion
*/
static inline Protocol_RxCode_T BuildRxPacket(Protocol_T * p_protocol)
{
	Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET;
	uint8_t rxLength;
	uint8_t rxLimit;

	while(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MAX)
	{
		/* Set rxLimit to check completion before reading byte from following packet */
		if(p_protocol->RxRemaining == RX_REMAINING_MAX) /* Check 1 byte (or upto RX_LENGTH_MIN at a time) until Length remaining is known */
		{
			rxLimit = (p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MIN) ? p_protocol->p_Specs->RX_LENGTH_MIN - p_protocol->RxIndex : 1U;
		}
		else /* Check RxRemaining */
		{
			rxLimit = p_protocol->RxRemaining;
		}

		rxLength = Xcvr_Rx(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], rxLimit); /* Rx up to rxLimit */

		if(rxLength > 0U)
		{
			p_protocol->RxIndex += rxLength;

			if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MIN)
			{
				rxStatus = p_protocol->p_Specs->PARSE_RX_META
				(
					&p_protocol->ReqIdActive, &p_protocol->RxRemaining,
					p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
				);
				if(rxStatus != PROTOCOL_RX_CODE_WAIT_PACKET) { break; } /* Packet is complete, Req, ReqExt or Sync */
			}
		}
		else /*  Rx Buffer empty, wait for Xcvr_Rx */
		{
			break;
		}
	}
	// if(p_protocol->RxIndex > p_protocol->p_Specs->RX_LENGTH_MAX) { rxStatus = PROTOCOL_RX_CODE_ERROR_PACKET; }

	return rxStatus;
}

static void ProcTxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
	if(p_protocol->p_Specs->BUILD_TX_SYNC != 0U)
	{
		p_protocol->p_Specs->BUILD_TX_SYNC(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, txId);
		Xcvr_Tx(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
	}
}

/*
	Handle Rx Packet
	Controls Rx Packet Parser, and Timeout Timer
	if run inside isr, need sync mechanism
*/
static inline void ProcRxState(Protocol_T * p_protocol)
{
	// Protocol_RxCode_T rxStatus;

	switch(p_protocol->RxState)
	{
		case PROTOCOL_RX_STATE_WAIT_BYTE_1: /* nonblocking wait state, no timer */
			if(Xcvr_Rx(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U], 1U) == true)
			{
				/*
					Use starting byte even if data is unencoded. first char can still be handled in separate state.
					Unencoded still discards Rx chars until starting ID
				*/
				if((p_protocol->p_Specs->RX_START_ID == 0x00U) || (p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] == p_protocol->p_Specs->RX_START_ID))
				{
					p_protocol->RxIndex = 1U;
					p_protocol->RxRemaining = RX_REMAINING_MAX;
					p_protocol->RxTimeStart = *(p_protocol->CONFIG.P_TIMER);
					p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
				}
			}
			break;

		case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
			if(*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
			{
				p_protocol->RxCode = BuildRxPacket(p_protocol);

				/*
					Continue BuildRxPacket while Req is processsing
					Rx can queue out of sequence. Invalid Rx sequence until timeout buffer flush

					Alternatively, pause BuildRxPacket during Req processing
					Incoming packet bytes wait in queue. Cannot miss packets (unless overflow)
					Cannot check for Abort without user signal, persistent wait process
				*/
				if (p_protocol->RxCode != PROTOCOL_RX_CODE_WAIT_PACKET)
				{
					p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
				}

				if(p_protocol->RxCode == PROTOCOL_RX_CODE_ERROR)
				{
					if(p_protocol->NackCount < p_protocol->p_Specs->SYNC.NACK_REPEAT)
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_ERROR);
						p_protocol->NackCount++;
						p_protocol->RxTimeStart = *(p_protocol->CONFIG.P_TIMER);
					}
					else
					{
						p_protocol->NackCount = 0U;
						//pass error to req?
						//wait for timeout
					}
				}

				// switch(p_protocol->RxCode)
				// {
				// 	case PROTOCOL_RX_CODE_WAIT_PACKET: 				break;
				// 	// case PROTOCOL_RX_CODE_WAIT_PACKET_REMAINING: 	break;
				// 	// case PROTOCOL_RX_CODE_COMPLETE: 		p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
				// 	// case PROTOCOL_RX_CODE_ACK: 				p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
				// 	// case PROTOCOL_RX_CODE_NACK: 			p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
				// 	// case PROTOCOL_RX_CODE_ABORT: 			p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
				// 	case PROTOCOL_RX_CODE_ERROR:
				// 		// use reqActive->SYNC if available
				// 		if(p_protocol->NackCount < p_protocol->p_Specs->SYNC.NACK_REPEAT)
				// 		{
				// 			ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_ERROR);
				// 			p_protocol->NackCount++;
				// 			p_protocol->RxTimeStart = *(p_protocol->CONFIG.P_TIMER);
				// 		}
				// 		else
				// 		{
				// 			//pass error to req?
				// 			//wait for timeout
				// 		}
				// 		// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
				// 		// p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
				// 		break;
				// 		// case PROTOCOL_RX_CODE_DATAGRAM_SETUP: break;
				// 	default:
				// 		break;
				// }

			}
			else
			{
				ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_RX_TIMEOUT);
				p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
				p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
				p_protocol->NackCount = 0U;
			}
			break;

		// case PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL:
		// 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
		// 	p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
		// 	// p_protocol->NackCount = 0U;
		// 	break;

		case PROTOCOL_RX_STATE_INACTIVE:
			break;

		default: break;
	}
}

static void ResetReqState(Protocol_T * p_protocol)
{
	p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID;
	if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
	p_protocol->NackCount = 0U;
}

static bool TxPacket(Protocol_T * p_protocol, const uint8_t * p_txBuffer, uint8_t length)
{
	return (length > 0U) ? Xcvr_Tx(&p_protocol->Xcvr, p_txBuffer, length) : false;
}

/*
	Common PROTOCOL_REQ_STATE_WAIT_RX_SYNC PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL
*/
static inline Protocol_RxCode_T ProcReqWaitRxSync(Protocol_T * p_protocol)
{
	if(p_protocol->RxCode == PROTOCOL_RX_CODE_ACK)
	{
		p_protocol->NackCount = 0U;
	}
	else if(p_protocol->RxCode == PROTOCOL_RX_CODE_NACK)
	{
		if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
		{
			TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength); /* Retransmit Packet */
			p_protocol->NackCount++;
			p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
		}
		else
		{
			ResetReqState(p_protocol);
		}
	}
	//handle unexpected req packet

	return p_protocol->RxCode;
}

/*! @return pointer to Req */
static const Protocol_Req_T * SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, protocol_reqid_t id)
{
	const Protocol_Req_T * p_req = 0U;
	for(uint8_t iChar = 0U; iChar < tableLength; iChar++) { if(p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; } }
	return p_req;
}

// static void ProcReqTxSyncNack(Protocol_T * p_protocol, Protocol_RxCode_T nackId)
// {
// 	if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
// 	{
// 		p_protocol->NackCount++;
// 		ProcTxSync(p_protocol, nackId);
// 	}
// }

/*
	Handle user Req/Cmd function
	ReqTimeStart resets for all expected behaviors
*/
static inline void ProcReqState(Protocol_T * p_protocol)
{
	Protocol_ReqCode_T reqStatus;

	/* States Common*/
	if((p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) && (p_protocol->ReqState != PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID))
	{
		if(*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT)
		{
			ResetReqState(p_protocol);
			ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_TIMEOUT);
		}
		else if(p_protocol->RxCode == PROTOCOL_RX_CODE_ABORT)
		{
			p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER); /* reset timer for RxLost  */
			ResetReqState(p_protocol);
		}
		else if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE)
		{
			p_protocol->NackCount = 0U;
		}
	}

	switch(p_protocol->ReqState)
	{
		case PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID:
			if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE)
			{
				p_protocol->p_ReqActive = SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, p_protocol->ReqIdActive);

				if(p_protocol->p_ReqActive != 0U)
				{
					p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER); /* Reset timer on all non error packets,  */

					if(p_protocol->p_ReqActive->SYNC.TX_ACK == true) { ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_ID); }

					if(p_protocol->p_ReqActive->PROC != 0U) /* Does not invoke state machine, no loop / nonblocking wait. */
					{
						p_protocol->p_ReqActive->PROC
						(
							p_protocol->CONFIG.P_APP_INTERFACE,
							p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
							p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
						);
						TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					}

					if(p_protocol->p_ReqActive->PROC_EXT != 0U)
					{
						if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
					}
					else if(p_protocol->p_ReqActive->SYNC.RX_ACK == true)
					{
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
				}
				else
				{
					ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_ID);
				}
			}
			else
			{
				//handle out of sequence packet / handle special context
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_REQ_EXT: /* Wait Rx Req Ext Continue */
			if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE)
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
				p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
			}
			else
			{
				//handle out of sequence packet
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
			if(ProcReqWaitRxSync(p_protocol) == PROTOCOL_RX_CODE_ACK)
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
				p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
			}
			else
			{
				//handle out of sequence packet
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL:
			if(ProcReqWaitRxSync(p_protocol) == PROTOCOL_RX_CODE_ACK)
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
				ResetReqState(p_protocol);
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_PROCESS:
			// if(p_protocol->RxCode != PROTOCOL_RX_CODE_WAIT_PACKET)
			// {
			// 	OutOfSequencePacket++
			// }
			p_protocol->TxLength = 0U; /* in case user does not set 0 */
			reqStatus = p_protocol->p_ReqActive->PROC_EXT
			(
				p_protocol->CONFIG.P_SUBSTATE_BUFFER, p_protocol->CONFIG.P_APP_INTERFACE,
				p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength,
				p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex
			);

			if(reqStatus != PROTOCOL_REQ_CODE_WAIT_PROCESS) { p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER); }
			switch(reqStatus)
			{
				case PROTOCOL_REQ_CODE_WAIT_PROCESS: break;			/* Timer ticking */
				case PROTOCOL_REQ_CODE_WAIT_PROCESS_EXTEND_TIMER: p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER); break;
				case PROTOCOL_REQ_CODE_COMPLETE:
					if(p_protocol->p_ReqActive->SYNC.RX_ACK == true) { p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL; }
					else { ResetReqState(p_protocol); }
					break;

				case PROTOCOL_REQ_CODE_AWAIT_RX_SYNC:				p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;					break;
				case PROTOCOL_REQ_CODE_AWAIT_RX_REQ_EXT:			p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_REQ_EXT;				break;

				/*
					Separate tx data state vs tx on user return txlength > 0
					User will will have to manage more return codes, but will not have to explicitly set txlength to zero.
					must save txlength in case of retransmit
				*/
				case PROTOCOL_REQ_CODE_TX_DATA: 	TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);	break;
				case PROTOCOL_REQ_CODE_TX_ACK:		ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_EXT);								break;
				case PROTOCOL_REQ_CODE_TX_NACK: /* Shared Nack */
					if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_EXT);
						p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
						p_protocol->NackCount++;
					}
					else
					{
						ResetReqState(p_protocol);
					}
					break;

				default: break;
			}
			break;

		case PROTOCOL_REQ_STATE_INACTIVE: break;

		default: break;

	}

	if((p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE))
	{
		p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
	}
}

//static void ProcDatagram(Protocol_T * p_protocol)
//{
//	//* save room for 1 req packet
//	if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
//	{
//		if (Datagram_Server_Proc(&p_protocol->Datagram))
//		{
//			PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//		}
//	}
//		//	if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
//		//	{
//				if (Datagram_Server_Proc(&p_protocol->Datagram))
//				{
//					PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//				}
//		//	}
//}

/*
	Controller Side, sequential rx req, proc, tx response
	Non blocking
	Single threaded only, RxIndex not protected
*/
void Protocol_Proc(Protocol_T * p_protocol)
{
	ProcRxState(p_protocol);
	ProcReqState(p_protocol);
//	if (p_protocol->State != PROTOCOL_STATE_INACTIVE) /* Enqueue Datagram for processing in parallel */
//	{
//		Protocol_Datagram_Proc(p_protocol);
//	}
}

/*! @return true if RxLostTime reached, a sucessful Req has not occured */
bool Protocol_CheckRxLost(Protocol_T * p_protocol)
{
	/* Check one, RxState or ReqState == STATE_INACTIVE */
	return (p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) ? (*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Params.RxLostTime) : false;
}

void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId)
{
	p_protocol->Params.XcvrId = xcvrId;
	Xcvr_Init(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
}

void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate)
{
	if(Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Params.XcvrId) == true)
	{
		Xcvr_ConfigBaudRate(&p_protocol->Xcvr, baudRate); //todo check valid baudrate
	}
}
#endif
/******************************************************************************/
/*! End CONFIG_PROTOCOL_XCVR_ENABLE */
/******************************************************************************/

void Protocol_SetSpecs(Protocol_T * p_protocol, uint8_t p_specsId)
{
	const Protocol_Specs_T * p_specs = (p_specsId < p_protocol->CONFIG.SPECS_COUNT) ? p_protocol->CONFIG.PP_SPECS_TABLE[p_specsId] : 0U;

	if(p_specs != 0U && p_specs->RX_LENGTH_MAX <= p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
	{
		p_protocol->p_Specs = p_specs;
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
		if(p_protocol->p_Specs->BAUD_RATE_DEFAULT != 0U)
		{
			Protocol_ConfigXcvrBaudRate(p_protocol, p_protocol->p_Specs->BAUD_RATE_DEFAULT);
		}
#endif
	}
}

static inline bool IsXcvrSet(Protocol_T * p_protocol)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	return Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
#else
	return false;
#endif
}

bool Protocol_Enable(Protocol_T * p_protocol)
{
	bool isEnable = ((IsXcvrSet(p_protocol) == true) && p_protocol->p_Specs != 0U);

	if(isEnable == true)
	{
		p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
		p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_REQ_ID;
		p_protocol->RxIndex = 0U;
		p_protocol->TxLength = 0U;
		p_protocol->p_ReqActive = 0U;
	}

	return isEnable;
}

void Protocol_Disable(Protocol_T * p_protocol)
{
	p_protocol->RxState = PROTOCOL_RX_STATE_INACTIVE;
	p_protocol->ReqState = PROTOCOL_REQ_STATE_INACTIVE;
}

/*
	User must reset. Does propagate set. Current settings remain active until reboot.
*/
void Protocol_EnableOnInit(Protocol_T * p_protocol) { p_protocol->Params.IsEnableOnInit = true; }
void Protocol_DisableOnInit(Protocol_T * p_protocol) { p_protocol->Params.IsEnableOnInit = false; }