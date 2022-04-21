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
    @file
    @author FireSoucery
    @brief 	Simple general configurable protocol
    @version V0
*/
/******************************************************************************/
#include "Protocol.h"

#include <string.h>

static inline bool IsXcvrSet(Protocol_T * p_protocol)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	return (p_protocol->Xcvr.p_Xcvr != 0U); //todo sub Xcvr.p_Xcvr if validate xcvr id
#elif defined(CONFIG_PROTOCOL_XCVR_SERIAL)
	return (p_protocol->Params.p_Serial != 0U);
#endif
}

static inline void ConfigSpecsBaudRate(Protocol_T * p_protocol)
{
	if (p_protocol->Params.p_Specs->BAUD_RATE_DEFAULT != 0U)
	{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
		Xcvr_ConfigBaudRate(&p_protocol->Xcvr, p_protocol->Params.p_Specs->BAUD_RATE_DEFAULT);
#elif defined(CONFIG_PROTOCOL_XCVR_SERIAL)
		Serial_ConfigBaudRate(p_protocol->Params.p_Serial, p_protocol->Params.p_Specs->BAUD_RATE_DEFAULT);
#endif
	}
}

void Protocol_Init(Protocol_T * p_protocol)
{
	if(p_protocol->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_protocol->Params, p_protocol->CONFIG.P_PARAMS, sizeof(Protocol_Params_T));
	}

#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	Xcvr_Init(&p_protocol->Xcvr, p_protocol->Params.XcvrId);
#endif

	if(IsXcvrSet(p_protocol) && (p_protocol->Params.p_Specs != 0U) && (p_protocol->Params.IsEnable == true))
	{
		ConfigSpecsBaudRate(p_protocol);
		Protocol_Enable(p_protocol);
	}
	else
	{
		Protocol_Disable(p_protocol);
	}

	p_protocol->RxIndex 	= 0U;
	p_protocol->TxLength 	= 0U;
	p_protocol->p_ReqActive = 0U;
}

void Protocol_ConfigSpecsBaudRate(Protocol_T * p_protocol)
{
	if(IsXcvrSet(p_protocol) && (p_protocol->Params.p_Specs != 0U))
	{
		ConfigSpecsBaudRate(p_protocol);
	}
}

void Protocol_SetSpecs(Protocol_T * p_protocol, const Protocol_Specs_T * p_specs) //validate its on the list
{
	if(p_specs->RX_LENGTH_MAX < p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
	{
		p_protocol->Params.p_Specs = p_specs;
//		ConfigSpecsBaudRate(p_protocol);
	}
}

void Protocol_SetXcvr(Protocol_T * p_protocol, void * p_transceiver)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE

#elif defined(CONFIG_PROTOCOL_XCVR_SERIAL)
	// cannot validate pointer without xcvr module
	p_protocol->Params.p_Serial = p_transceiver;
#endif
}

bool Protocol_Enable(Protocol_T * p_protocol)
{
	bool isEnable = (IsXcvrSet(p_protocol) && p_protocol->Params.p_Specs != 0U);

	if (isEnable == true)
	{
		p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_BYTE_1;
		p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_REQ;
	}

	return isEnable;
}

//check need critical
void Protocol_Disable(Protocol_T * p_protocol)
{
	p_protocol->RxState 	= PROTOCOL_RX_STATE_INACTIVE;
	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_INACTIVE;
	p_protocol->RxIndex 	= 0U;
	p_protocol->TxLength 	= 0U;
	p_protocol->p_ReqActive = 0U;
}

static inline bool TxPacket(Protocol_T * p_protocol, const uint8_t * p_txBuffer, uint8_t length)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	return (length > 0U) ? Xcvr_Tx(&p_protocol->Xcvr, p_txBuffer, length) : false;
#else
	return (length > 0U) ? Serial_Send(p_protocol->Params.p_Serial, p_txBuffer, length) : false;
#endif
}

static inline uint32_t RxPacket(Protocol_T * p_protocol, uint8_t * p_rxBuffer, uint8_t length)
{
#ifdef CONFIG_PROTOCOL_XCVR_ENABLE
	return Xcvr_Rx(&p_protocol->Xcvr, p_rxBuffer, length);
#else
	return Serial_Recv(p_protocol->Params.p_Serial, p_rxBuffer, length);
#endif
}


//static inline void PortFlushBuffers(Protocol_T * p_protocol)
//{
//	Serial_FlushBuffers(p_protocol->Params.p_Port);
//}

/*
 * Parse for length remaining and ReqCode
 * Receive into buffer and check for completion
 * Packet must be 2 bytes minimum
 */
static inline Protocol_RxCode_T BuildRxPacket(Protocol_T * p_protocol)
{
	Protocol_RxCode_T status = PROTOCOL_RX_CODE_WAIT_PACKET;

//	uint8_t rxLength;

	//todo change to rx max Serial_RecvBytes, when remaining char count is known
	//use rx length min to determine datalength byte
//	if((p_protocol->RxCode == waitheader) || (p_protocol->RxCode == PROTOCOL_RX_CODE_WAIT_PACKET))
//	{
//		rxLength = 1U;
//	}
//	else
//	{
////		rxLength = count remaining - p_protocol->RxIndex;
//	}

//	while (PortRxPacket(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], rxLength) > 0U)
//	{
//		p_protocol->RxIndex+rxcount;
//
//		if (p_protocol->RxIndex >= p_protocol->Params.p_Specs->RX_LENGTH_MIN)
//		{
//			if (p_protocol->RxIndex <= p_protocol->Params.p_Specs->RX_LENGTH_MAX)
//			{
//				status = p_protocol->Params.p_Specs->PARSE_RX(p_protocol->CONFIG.P_SUBSTATE_BUFFER, &p_protocol->ReqIdActive, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
//
//				//if wwait data rxLength = count remaining - p_protocol->RxIndex;
//
//				if (status != PROTOCOL_RX_CODE_WAIT_PACKET)
//				{
//					break;
//				}
//			}
//			else
//			{
//				status = PROTOCOL_RX_CODE_ERROR_PACKET;
//				break;
//			}
//		}
//	}


	while (RxPacket(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex], 1U) == true)
	{
		p_protocol->RxIndex++;

		if (p_protocol->RxIndex >= p_protocol->Params.p_Specs->RX_LENGTH_MIN)
		{
			if (p_protocol->RxIndex <= p_protocol->Params.p_Specs->RX_LENGTH_MAX)
			{
				status = p_protocol->Params.p_Specs->PARSE_RX(p_protocol->CONFIG.P_SUBSTATE_BUFFER, &p_protocol->ReqIdActive, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
				if (status != PROTOCOL_RX_CODE_WAIT_PACKET)
				{
					break;
				}
			}
			else
			{
				status = PROTOCOL_RX_CODE_ERROR_PACKET;
				break;
			}
		}
	}

	return status;
}


static inline void ProcTxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
	if (p_protocol->Params.p_Specs->BUILD_TX_SYNC != 0U)
	{
		p_protocol->Params.p_Specs->BUILD_TX_SYNC(p_protocol->CONFIG.P_SUBSTATE_BUFFER, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, txId);
		TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
	}
}

//static inline void ProcTimeout(Protocol_T * p_protocol)
//{
//	ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
//	p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_BYTE_1;
//	p_protocol->RxCode 	= PROTOCOL_RX_CODE_WAIT_PACKET;
//
//	p_protocol->TxNackCount = 0U;
//	p_protocol->RxNackCount = 0U;
//	/*
//	 * Reset both states on timeout - must reset req state to break from unreceived sync
//	 * Only intance RxState changes ReqState, to avoid 3 timer state machine
//	 */
//	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_REQ;
//	p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_COMPLETE;
//
//	if (p_protocol->Params.p_Specs->RESET_SUBSTATE != 0U)
//	{
//		p_protocol->Params.p_Specs->RESET_SUBSTATE(p_protocol->CONFIG.P_SUBSTATE_BUFFER);
//	}
//
//	PortFlushBuffers(p_protocol);
//}




static inline void ProcRxTimeout(Protocol_T * p_protocol)
{
	ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
	p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_BYTE_1;
	p_protocol->RxCode 		= PROTOCOL_RX_CODE_WAIT_PACKET;
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
static inline void ProcRxTransitionWaitReq(Protocol_T * p_protocol)
{
//	if (p_protocol->Params.p_Specs->USE_RX_PAUSE_ON_REQ == true)
//	{
//		p_protocol->RxTimeStart 	= *(p_protocol->CONFIG.P_TIMER);
//		p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL;
//	}
//	else
//	{
////		start  req timer
//	}
}


/*
 * Handle protocol control
 * Controls Rx Packet Parser, and Timeout Timer
 * if run inside isr , need sync mechanism
 * Encoded and non encoded checks
 */
static inline void ProcRxState(Protocol_T * p_protocol)
{
 	switch (p_protocol->RxState)
	{
		case PROTOCOL_RX_STATE_WAIT_BYTE_1: /* nonblocking wait state, no timer */
			if (RxPacket(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U], 1U) == true)
			{
				/*
				 * Use starting byte even if data is unencoded. first char can still be handled in separate state.
				 */
				if ((p_protocol->Params.p_Specs->RX_START_ID == 0x00U) || (p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] == p_protocol->Params.p_Specs->RX_START_ID))
				{
					p_protocol->RxIndex = 1U;
//					ProcRxTransitionWaitPacket(p_protocol);
					p_protocol->RxTimeStart = *(p_protocol->CONFIG.P_TIMER);
//					p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
					p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
				}
			}
			break;

		case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->RxTimeStart < p_protocol->Params.p_Specs->RX_TIMEOUT)  /* no need to check for overflow if using millis */
			{
				p_protocol->RxCode = BuildRxPacket(p_protocol);

				switch(p_protocol->RxCode)
				{
					case PROTOCOL_RX_CODE_WAIT_PACKET:
						break;

					case PROTOCOL_RX_CODE_ERROR_PACKET:
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_ERROR);
						p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						break;

					case PROTOCOL_RX_CODE_ERROR_PACKET_ECC:
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_PACKET_ECC_ERROR);
						p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						break;

//					case PROTOCOL_RX_CODE_REQ_ID:
//						ProcRxTransitionWaitReq(p_protocol);
//						break;
//
//					case PROTOCOL_RX_CODE_ACK:
//						ProcRxTransitionWaitReq(p_protocol);
//						break;
//
//					case PROTOCOL_RX_CODE_NACK:
//						ProcRxTransitionWaitReq(p_protocol);
//						break;

//					case PROTOCOL_RX_CODE_DATAGRAM_SETUP: break;
//					case PROTOCOL_RX_CODE_CONTEXT_DATAGRAM: //pass in context
					default:
						ProcRxTransitionWaitReq(p_protocol);
						//assuming 2 state machines run in sync, only need 1 loop to process rxcode, before set back to wait packet
						p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL;
						break;
				}
			}
			else
			{
				ProcRxTimeout(p_protocol);
			}
			break;

		case PROTOCOL_RX_STATE_WAIT_REQ_SIGNAL:
			/*
			 * pause RX packets during processing, incoming packet bytes wait in queue.
			 * cannot miss packets (unless overflow)
			 * however
			 * cannot check for cancel without user req signal, persistent wait process
			 * rx can queue out of sequence. invalid TxRx sequence until timeout buffer flush
			 */
//			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->Params.p_Specs->REQ_TIMEOUT)
//			{
//				if (p_protocol->ReqCode == PROTOCOL_REQ_CODE_COMPLETE)
//			if(p_protocol->ReqCode != PROTOCOL_REQ_CODE_HOLD_RX)
//			{
				p_protocol->RxCode = PROTOCOL_RX_CODE_WAIT_PACKET;
				p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
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



static inline void ProcReqReset(Protocol_T * p_protocol)
{

	p_protocol->TxNackCount = 0U;
	p_protocol->RxNackCount = 0U;
	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_REQ;
	p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_COMPLETE;

}

static inline void ProcReqTimeout(Protocol_T * p_protocol)
{
	ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
	ProcReqReset(p_protocol);
	if (p_protocol->Params.p_Specs->RESET_SUBSTATE != 0U)
	{
		p_protocol->Params.p_Specs->RESET_SUBSTATE(p_protocol->CONFIG.P_SUBSTATE_BUFFER);
	}
}

static inline Protocol_ReqEntry_T * SearchReqTable(Protocol_ReqEntry_T * p_reqTable, size_t tableLength, uint32_t id)
{
	Protocol_ReqEntry_T * p_response = 0U;

	for(uint8_t iChar = 0U; iChar < tableLength; iChar++)
	{
		if (p_reqTable[iChar].ID == id)
		{
			p_response = &p_reqTable[iChar];
		}
	}

	return p_response;
}

static inline bool ProcReqWaitRxSyncCommon(Protocol_T * p_protocol)
{
	bool isAck = false;

	if (p_protocol->RxCode == PROTOCOL_RX_CODE_ACK)
	{
		isAck = true;
	}
	else if (p_protocol->RxCode == PROTOCOL_RX_CODE_NACK)
	{
		if (p_protocol->TxNackCount < p_protocol->p_ReqActive->P_SYNC->WAIT_RX_NACK_REPEAT)
		{
			p_protocol->TxNackCount++; //RxNackCount
			TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
			p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
			isAck = false;
		}
		else
		{
//			p_protocol->NackCount = 0U;
			isAck = false;
			ProcReqTimeout(p_protocol); //3 state return
		}
	}

	//handle unexpected req packet

	return isAck;
}


/*
 * Handle user Req/Cmd function
 */
static inline void ProcReqState(Protocol_T * p_protocol)
{

	if((p_protocol->ReqState != PROTOCOL_REQ_STATE_INACTIVE) && (p_protocol->ReqState != PROTOCOL_REQ_STATE_WAIT_RX_REQ))
	{
		if(*p_protocol->CONFIG.P_TIMER - p_protocol->ReqTimeStart > p_protocol->Params.p_Specs->REQ_TIMEOUT)
		{
			ProcReqTimeout(p_protocol);
		}
		else if(p_protocol->RxCode == PROTOCOL_RX_CODE_ABORT)
		{
			ProcReqReset(p_protocol);
		}
	}

	switch (p_protocol->ReqState)
	{
		case PROTOCOL_REQ_STATE_WAIT_RX_REQ:
			if (p_protocol->RxCode == PROTOCOL_RX_CODE_REQ_ID)
			{
				p_protocol->p_ReqActive = SearchReqTable(p_protocol->Params.p_Specs->P_REQ_TABLE, p_protocol->Params.p_Specs->REQ_TABLE_LENGTH, p_protocol->ReqIdActive);

				if (p_protocol->p_ReqActive != 0U)
				{
					if ((p_protocol->p_ReqActive->P_SYNC != 0U) && (p_protocol->p_ReqActive->P_SYNC->USE_TX_ACK_REQ == true))
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ);
					}

					if (p_protocol->p_ReqActive->FAST != 0U) //does not invoke state machine, no loop / nonblocking wait.
					{
						p_protocol->p_ReqActive->FAST(p_protocol->CONFIG.P_APP_CONTEXT, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
						TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					}

					if ((p_protocol->p_ReqActive->P_EXT != 0U) && (p_protocol->p_ReqActive->P_EXT->PROCESS != 0U))
					{
						p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
						p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_WAIT_PROCESS;
						p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
					}
					else if ((p_protocol->p_ReqActive->P_SYNC != 0U) && (p_protocol->p_ReqActive->P_SYNC->USE_WAIT_RX_ACK_COMPLETE == true))
					{
						p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);
						p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
						p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
					else
					{
						p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_COMPLETE;
					}
				}
				else
				{
					p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_COMPLETE;
					ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_ID);
				}
			}
//			else if (p_protocol->RxCode == PROTOCOL_RX_CODE_REQ_VAR)

			//handle special context
			//handle out of sequence ack
			break;

			//combine   wait?
		case PROTOCOL_REQ_STATE_WAIT_RX_DATA:
			if (p_protocol->RxCode == PROTOCOL_RX_CODE_DATA)
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);

				p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_WAIT_PROCESS;
				p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
			} //else still PROTOCOL_REQ_STATE_WAIT_RX_DATA
			//handle out of sequence ack
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
			if (ProcReqWaitRxSyncCommon(p_protocol) == true) //3 state return
			{
				p_protocol->ReqTimeStart = *(p_protocol->CONFIG.P_TIMER);

				p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_WAIT_PROCESS;
				p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
			} //else still PROTOCOL_REQ_CODE_AWAIT_RX_SYNC
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL:
			if (ProcReqWaitRxSyncCommon(p_protocol) == true)
			{
				p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_COMPLETE;
				p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_REQ;
				p_protocol->TxNackCount = 0U;
				p_protocol->RxNackCount = 0U;
			} //else still PROTOCOL_REQ_CODE_AWAIT_RX_SYNC
			//handle out of sequence packet
			break;

		case PROTOCOL_REQ_STATE_WAIT_PROCESS:
//			if(p_protocol->RxCode != PROTOCOL_RX_CODE_WAIT_PACKET)
//			{
//				OutOfSequencePacket++
//			}
			p_protocol->TxLength = 0U; //in case user does not set 0

			p_protocol->ReqCode = p_protocol->p_ReqActive->P_EXT->PROCESS
				(
					p_protocol->CONFIG.P_SUBSTATE_BUFFER,
					p_protocol->CONFIG.P_APP_CONTEXT,
					p_protocol->CONFIG.P_TX_PACKET_BUFFER,
					&p_protocol->TxLength,
					p_protocol->CONFIG.P_RX_PACKET_BUFFER,
					p_protocol->RxIndex
				);

			switch(p_protocol->ReqCode)
			{
				case PROTOCOL_REQ_CODE_WAIT_PROCESS: //if length >0, check
//					PortTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					break;

				case PROTOCOL_REQ_CODE_COMPLETE:
					if ((p_protocol->p_ReqActive->P_SYNC != 0U) && (p_protocol->p_ReqActive->P_SYNC->USE_WAIT_RX_ACK_COMPLETE == true))
					{
						p_protocol->ReqCode 	= PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
						p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
					else
					{
						p_protocol->TxNackCount = 0U;
						p_protocol->RxNackCount = 0U;
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_REQ;
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
					 * Separate tx data state vs tx on user return txlength > 0
					 * User will will have to manage more states, but will not have to explicitly set txlength to zero.
					 * must save txlength in case of retransmit
					 */
					TxPacket(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					break;

				case PROTOCOL_REQ_CODE_TX_ACK:
					ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_DATA); //user initiated, no need to check if option is set
					break;

				case PROTOCOL_REQ_CODE_TX_NACK:
					if(p_protocol->RxNackCount < p_protocol->p_ReqActive->P_SYNC->TX_NACK_REPEAT)
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_DATA);
						p_protocol->TxNackCount++;
					}
					else
					{
						ProcReqTimeout(p_protocol);
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
 * Slave Mode, sequential receive/response
 *
 * non blocking
 * single threaded only, RxIndex not protected
 *
 */
void Protocol_Slave_Proc(Protocol_T * p_protocol)
{
//	Protocol_Status_T status = 0U;
//	uint32_t reqId;
	ProcRxState(p_protocol);
	ProcReqState(p_protocol);
//	if (p_protocol->State != PROTOCOL_STATE_INACTIVE)
//	{
////		Protocol_Datagram_Proc(p_protocol);
//		//	if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
//		//	{
//				if (Datagram_Server_Proc(&p_protocol->Datagram))
//				{
//					PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//				}
//		//	}
//	}
}



//void Protocol_Datagram_Proc(Protocol_T * p_protocol)
//{
//	//* save room for 1 req packet
//	if (Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
//	{
//		if (Datagram_Server_Proc(&p_protocol->Datagram))
//		{
//			PortTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//		}
//	}
//}


/*
 * Master Mode, sequential cmd/Rx
 *
 * non blocking
 * single threaded only,
 *
 */
//Protocol_Status_T Protocol_Master_Begin(Protocol_T * p_protocol, cmd)
//{
//	p_protocol->State = PROTOCOL_STATE_SEND_CMD;
//}
//
//Protocol_Status_T Protocol_Master_Proc(Protocol_T * p_protocol)
//{
//	switch (p_protocol->State)
//	{
//		case PROTOCOL_STATE_SEND_CMD:
//
//		case PROTOCOL_STATE_PACKET_RX: //wait state
//			if (p_protocol->p_Time - p_protocol->TimeStart < p_protocol->TimeOut)
//			{
//				if (RxPacket(p_protocol))
//				{
//					p_protocol->State = PROTOCOL_STATE_PACKET_CHECK;
//				}
//			}
//			else
//			{
//				p_protocol->State = PROTOCOL_STATE_SEND_CMD;
//				//send txtimeout
//			}
//
//			break;
//
//		case PROTOCOL_STATE_CMD_PROCESS:
//			p_protocol->p_ReqFunction(p_protocol->Params.p_Specs->P_BYTES_BUFFER);
//			p_protocol->State = PROTOCOL_STATE_INACTIVE;
//			break;
//
//		case PROTOCOL_STATE_INACTIVE: break;
//		default: break;
//	}
//}

