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

static inline bool IsXcvrSet(Protocol_T * p_protocol)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	return Xcvr_CheckIsSet(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
#else
	return false;
#endif
}

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

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
static bool TxPacket(Protocol_T * p_protocol, const uint8_t * p_txBuffer, uint8_t length)
{
	return (length > 0U) ? Xcvr_Tx(&p_protocol->Xcvr, p_txBuffer, length) : false;
}

// uint32_t _Protocol_RxPacket(Protocol_T * p_protocol, uint8_t * p_rxBuffer, uint8_t length)
// {
// 	return Xcvr_RxPacket(&p_protocol->Xcvr, p_rxBuffer, length);
// }


// uint32_t _Protocol_RxBytes(Protocol_T * p_protocol, uint8_t * p_rxBuffer, uint8_t length)
// {
// 	return Xcvr_RxBytes(&p_protocol->Xcvr, p_rxBuffer, length);
// }
#endif

//static inline void XcvrFlushBuffers(Protocol_T * p_protocol)
//{
//	Serial_FlushBuffers(p_protocol->Params.p_Port);
//}

/*! @return pointer to Req */
static const Protocol_Req_T * SearchReqTable(Protocol_Req_T * p_reqTable, size_t tableLength, protocol_reqid_t id)
{
	const Protocol_Req_T * p_req = 0U;

	for(uint8_t iChar = 0U; iChar < tableLength; iChar++)
	{
		if(p_reqTable[iChar].ID == id) { p_req = &p_reqTable[iChar]; }
	}

	return p_req;
}


#define RX_REMAINING_MAX (65535U)

/*
	Parse for length remaining and ReqCode
	Receive into buffer and check for completion
	Packet must be 2 bytes minimum
*/
static inline Protocol_RxCode_T BuildRxPacket(Protocol_T * p_protocol)
{
	Protocol_RxCode_T rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET; // using status as substate, alternatively add state to RxStates
// rxStatus = PROTOCOL_RX_CODE_ERROR_PACKET;
	uint8_t rxLength;
	uint8_t rxLimit;

		// todo change to rx max Serial_RecvMax, when remaining char count is known
		// use rx length min to determine datalength byte

	if(p_protocol->RxRemaining != RX_REMAINING_MAX)
	{
		rxStatus = PROTOCOL_RX_CODE_WAIT_PACKET_REMAINING;
	}

	/*
		Check 1 byte at a time until Length remaining is known
		Rx maximum of Rx length before checking completion.
	*/
	while(p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MAX)
	// for(size_t iRxIndex = p_protocol->RxIndex; iRxIndex < p_protocol->p_Specs->RX_LENGTH_MAX; iRxIndex += rxLength)
	{
		if(rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET)
		{
			rxLimit = (p_protocol->RxIndex < p_protocol->p_Specs->RX_LENGTH_MIN) ? p_protocol->p_Specs->RX_LENGTH_MIN - p_protocol->RxIndex : 1U;
			// rxbytes?
			//loop this side only
		}
		else if(rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET_REMAINING)
		{
			rxLimit = p_protocol->RxRemaining;
			// rxpacket?
			// if(Xcvr_RxPacket(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], p_protocol->RxRemaining) == true)
			//get id
		}
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
		rxLength = Xcvr_Rx(&p_protocol->Xcvr, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], rxLimit); /* Rx Upto rxLimit */
#endif
		if(rxLength > 0U) /* Loop _Protocol_RxPacket if rxLength is 1 */
		{
			p_protocol->RxIndex += rxLength;

			if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MIN)
			{
				rxStatus = p_protocol->p_Specs->PARSE_RX_META(&p_protocol->ReqIdActive, &p_protocol->RxRemaining, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);

				if(((rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET) || (rxStatus == PROTOCOL_RX_CODE_WAIT_PACKET_REMAINING)) == false)
				{
					break;
				}
			}
		}
		else /* Break if rxLength is 0, wait for Xcvr_RX */
		{
			break;
		}
	}
	// if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MAX) { rxStatus = PROTOCOL_RX_CODE_ERROR_PACKET; }

	// while(_Protocol_RxPacket(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], 1U) == true)
	// {
	// 	p_protocol->RxIndex++;

	// 	if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MIN)
	// 	{
	// 		if(p_protocol->RxIndex <= p_protocol->p_Specs->RX_LENGTH_MAX)
	// 		{
	// 			rxStatus = p_protocol->p_Specs-> (p_protocol->CONFIG.P_SUBSTATE_BUFFER, &p_protocol->ReqIdActive, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
	// 			if(rxStatus != PROTOCOL_RX_CODE_WAIT_PACKET)
	// 			{
	// 				break;
	// 			}
	// 		}
	// 		else
	// 		{
	// 			rxStatus = PROTOCOL_RX_CODE_ERROR_PACKET;
	// 			break;
	// 		}
	// 	}
	// }

	return rxStatus;
}


static void ProcTxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
	if(p_protocol->p_Specs->BUILD_TX_SYNC != 0U)
	{
		p_protocol->p_Specs->BUILD_TX_SYNC(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, txId);
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
		Xcvr_Tx(&p_protocol->Xcvr, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
#endif
	}
}



/*
	Common Rx Req Timeout
	Timeout does not check NackCount
		rx timeour reset req?
*/
static void ProcTimeout(Protocol_T * p_protocol)
{
	ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
	if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
}

// static inline void ProcTimeout(Protocol_T * p_protocol)
// {
// 	ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
// 	p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_BYTE_1;
// 	p_protocol->RxCode 		= PROTOCOL_RX_CODE_WAIT_PACKET;

// 	p_protocol->TxNackCount = 0U;
// 	p_protocol->RxNackCount = 0U;
// 	/*
// 	 * Reset both states on timeout - must reset req state to break from unreceived sync
// 	 * Only intance RxState changes ReqState, to avoid 3 timer state machine
// 	 */
// 	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL;
// 	p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_COMPLETE;

// 	if (p_protocol->p_Specs->RESET_SUBSTATE != 0U)
// 	{
// 		p_protocol->p_Specs->RESET_SUBSTATE(p_protocol->CONFIG.P_SUBSTATE_BUFFER);
// 	}

// 	PortFlushBuffers(p_protocol);
// }


static void ProcRxTimeout(Protocol_T * p_protocol)
{
	ProcTimeout(p_protocol);
	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
}





		//static inline void ProcRxTransitionWaitByte1(Protocol_T * p_protocol)
		//{
		//	p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
		//	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
		//}
//
	//static inline void ProcRxTransitionWaitPacket(Protocol_T * p_protocol)
	//{
	//	p_protocol->RxTimeStart = *(p_protocol->CONFIG.P_TIMER);
	//	p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
	//	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
	//}
	// static void ProcRxTransitionWaitReq(Protocol_T * p_protocol)
	// {
	// 	//	if (p_protocol->p_Specs->USE_RX_PAUSE_ON_REQ == true)
	// 	//	{
	// 	//		p_protocol->RxTimeStart 	= *(p_protocol->CONFIG.P_TIMER);
	// 	//		p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL;
	// 	//	}
	// 	//	else
	// 	//	{
	// 	////		start  req timer
	// 	//	}
	// }


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
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
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
#endif
			break;

		case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
			if(*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->p_Specs->RX_TIMEOUT)  /* No need to check for overflow if using millis */
			{
				p_protocol->RxCode = BuildRxPacket(p_protocol);

				switch(p_protocol->RxCode)
				{
					case PROTOCOL_RX_CODE_WAIT_PACKET: break;
					case PROTOCOL_RX_CODE_COMPLETE: 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
					case PROTOCOL_RX_CODE_ACK: 				p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
					case PROTOCOL_RX_CODE_NACK: 			p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
					case PROTOCOL_RX_CODE_ABORT: 			p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL; break;
					case PROTOCOL_RX_CODE_ERROR:
						//cehck proc nack
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_ERROR);
						p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
						break;
						// case PROTOCOL_RX_CODE_DATAGRAM_SETUP: break;
					default:
						// ProcRxTransitionWaitReq(p_protocol);
						//assuming 2 state machines run in sync, only need 1 loop to process rxcode, before set back to wait packet
						break;
				}
			}
			else
			{
				ProcRxTimeout(p_protocol);
			}
			break;

		case PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL:
			p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
			p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
			/*
				pause build new RX packets  req processing, incoming packet bytes wait in queue.
				cannot miss packets (unless overflow)
				cannot check for cancel without user req signal, persistent wait process

				proc tx
				rx can queue out of sequence. invalid TxRx sequence until timeout buffer flush
			 */
			 //			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->REQ_TIMEOUT)
			 //			{
			 //				if (p_protocol->ReqCode == PROTOCOL_REQ_CODE_COMPLETE)
			 //			if(p_protocol->ReqCode != PROTOCOL_REQ_CODE_HOLD_RX)
			 //			{
			// p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
			// if(p_protocol->ReqState != PROTOCOL_REQ_CODE_SIGNAL_RX)
			// {
			// 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
			// }

			//restart xcvr rx if needed
//			}
//				else if ((p_protocol->ReqCode == PROTOCOL_REQ_CODE_AWAIT_RX_DATA) || (p_protocol->ReqCode == PROTOCOL_REQ_CODE_AWAIT_RX_SYNC))
//				{
//					p_protocol->RxIndex 	= 0U;
//					ProcRxTransitionWaitPacket(p_protocol);
//				}
//			}
//			else
//			{
//				ProcTimeout(p_protocol);
//			}
			break;

		case PROTOCOL_RX_STATE_INACTIVE:
			break;

		default: break;
	}
}

// static void ProcReqReset(Protocol_T * p_protocol)
// {
// 	// p_protocol->TxNackCount = 0U;
// 	// p_protocol->RxNackCount = 0U;
// 	// p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL;
// 	// p_protocol->ReqCode = PROTOCOL_REQ_CODE_COMPLETE;
// }

static void ProcReqReset(Protocol_T * p_protocol)
{
	p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL;
	p_protocol->NackCount = 0U;
	ProcTimeout(p_protocol);
}


// static void SignalRx(Protocol_T * p_protocol)
// {
// 	p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
// 	p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
// }

static inline Protocol_RxCode_T ProcReqWaitRxSyncCommon(Protocol_T * p_protocol)
{
	if(p_protocol->RxCode == PROTOCOL_RX_CODE_ACK)
	{
		p_protocol->NackCount = 0U;
	}
	else if(p_protocol->RxCode == PROTOCOL_RX_CODE_NACK)
	{
		if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
		{
			p_protocol->NackCount++; //RxNackCount
			TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);			/* Retransmit Packet */
			p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
		}
		else
		{
			//			p_protocol->NackCount = 0U;
			ProcReqReset(p_protocol); //3 state return
		}
	}
	//handle unexpected req packet

	return p_protocol->RxCode;
}

// static void ProcReqTxSyncNack(Protocol_T * p_protocol)
// {
// 	if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
// 	{
// 		p_protocol->NackCount++;
// 		ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK);
// 	}
// }

/*
	Handle user Req/Cmd function
*/
static inline void ProcReqState(Protocol_T * p_protocol)
{
	Protocol_ReqCode_T reqStatus;

	/* Common States */
	if((p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) && (p_protocol->ReqState != PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL))
	{
		if(*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->p_Specs->REQ_TIMEOUT)
		{
			ProcReqReset(p_protocol);
		}
		else if(p_protocol->RxCode == PROTOCOL_RX_CODE_ABORT)
		{
			ProcReqReset(p_protocol);
		}
	}

	if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE)
	{
		p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER); /* reset timer on packet reception */
	}

	switch(p_protocol->ReqState)
	{
		case PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL:
			if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE) //PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL
			{
				p_protocol->p_ReqActive = SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, p_protocol->ReqIdActive);

				if(p_protocol->p_ReqActive != 0U)
				{
					if(p_protocol->p_ReqActive->SYNC.TX_ACK == true)
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ_ID);
					}

					if(p_protocol->p_ReqActive->PROC != 0U) /* does not invoke state machine, no loop / nonblocking wait. */
					{
						p_protocol->p_ReqActive->PROC(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
						TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					}

					if(p_protocol->p_ReqActive->PROC_EXT != 0U)
					{
						if(p_protocol->p_Specs->REQ_EXT_RESET != 0U) { p_protocol->p_Specs->REQ_EXT_RESET(p_protocol->CONFIG.P_SUBSTATE_BUFFER); }
						// p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
						// p_protocol->ReqCode = PROTOCOL_REQ_CODE_WAIT_PROCESS;
						// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
					}
					else if(p_protocol->p_ReqActive->SYNC.RX_ACK == true)
					{
						// p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
						// p_protocol->ReqCode = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
						// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
					else
					{
						// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						// p_protocol->ReqCode = PROTOCOL_REQ_CODE_COMPLETE;
					}
				}
				else
				{
					// p_protocol->ReqCode = PROTOCOL_REQ_CODE_COMPLETE;
					// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
					ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_ID);
				}

				// p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
				// p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
			}
			else
			{
				//handle out of sequence packet / handle special context
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_DATA: /* Wait Rx Ext Continue */
			if(p_protocol->RxCode == PROTOCOL_RX_CODE_COMPLETE)
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER); /* reset timer on packet reception */
				// p_protocol->ReqCode = PROTOCOL_REQ_CODE_WAIT_PROCESS;
				p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
			}
			else // still PROTOCOL_REQ_STATE_WAIT_RX_DATA
			{
				//handle out of sequence packet, send nack
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
			if(ProcReqWaitRxSyncCommon(p_protocol) == PROTOCOL_RX_CODE_ACK) //3 state return
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);

				// p_protocol->ReqCode = PROTOCOL_REQ_CODE_WAIT_PROCESS;
				p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_PROCESS;
			} //else still PROTOCOL_REQ_CODE_AWAIT_RX_SYNC
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL:
			if(ProcReqWaitRxSyncCommon(p_protocol) == PROTOCOL_RX_CODE_ACK)
			{
				// p_protocol->ReqCode = PROTOCOL_REQ_CODE_COMPLETE;
				p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL;
			} //else still PROTOCOL_REQ_CODE_AWAIT_RX_SYNC
			//handle out of sequence packet
			break;

		case PROTOCOL_REQ_STATE_WAIT_PROCESS:
			//			if(p_protocol->RxCode != PROTOCOL_RX_CODE_WAIT_PACKET) || wa
			//			{
			//				OutOfSequencePacket++
			//			}
			p_protocol->TxLength = 0U; /* in case user does not set 0 */
			reqStatus = p_protocol->p_ReqActive->PROC_EXT(p_protocol->CONFIG.P_SUBSTATE_BUFFER, p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);

			switch(reqStatus)
			{
				case PROTOCOL_REQ_CODE_WAIT_PROCESS: //if length >0, check
					//waiting timer
					break;

				case PROTOCOL_REQ_CODE_COMPLETE:
					if(p_protocol->p_ReqActive->SYNC.RX_ACK == true)
					{
						// p_protocol->ReqCode = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
					else
					{
						// p_protocol->TxNackCount = 0U;
						p_protocol->NackCount = 0U;
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL;
					}
					break;

				case PROTOCOL_REQ_CODE_AWAIT_RX_SYNC:
					p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC; //user ensures P_SYNC != 0
					break;

				case PROTOCOL_REQ_CODE_AWAIT_RX_DATA:
					p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_DATA;
					break;

				case PROTOCOL_REQ_CODE_EXTEND_TIMER:
					p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
					break;

				case PROTOCOL_REQ_CODE_TX_DATA:
					/*
						Separate tx data state vs tx on user return txlength > 0
						User will will have to manage more return codes, but will not have to explicitly set txlength to zero.
						must save txlength in case of retransmit
					*/
					TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					break;

				case PROTOCOL_REQ_CODE_TX_ACK:
					ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_DATA); //user initiated, no need to check if option is set
					break;

				case PROTOCOL_REQ_CODE_TX_NACK:
					if(p_protocol->NackCount < p_protocol->p_ReqActive->SYNC.NACK_REPEAT)
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_DATA);
						p_protocol->NackCount++;
					}
					else
					{
						ProcReqReset(p_protocol);
					}
					break;

				default:
					break;
			}
			break;

		case PROTOCOL_REQ_STATE_INACTIVE:
			break;

		default: break;

	}
}

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

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
void Protocol_SetXcvr(Protocol_T * p_protocol, uint8_t xcvrId)
{
	p_protocol->Params.XcvrId = xcvrId;
	Xcvr_Init(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
}

void Protocol_ConfigXcvrBaudRate(Protocol_T * p_protocol, uint32_t baudRate)
{
	if(IsXcvrSet(p_protocol) == true)
	{
		Xcvr_ConfigBaudRate(&p_protocol->Xcvr, baudRate); //todo check valid baudrate
	}
}
#endif

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

bool Protocol_Enable(Protocol_T * p_protocol)
{
	bool isEnable = ((IsXcvrSet(p_protocol) == true) && p_protocol->p_Specs != 0U);

	if(isEnable == true)
	{
		p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
		p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SIGNAL;
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
	User must reset. Do not propagate set, so current settings remain active until reboot.
*/
void Protocol_EnableOnInit(Protocol_T * p_protocol) { p_protocol->Params.IsEnableOnInit = true; }
void Protocol_DisableOnInit(Protocol_T * p_protocol) { p_protocol->Params.IsEnableOnInit = false; }