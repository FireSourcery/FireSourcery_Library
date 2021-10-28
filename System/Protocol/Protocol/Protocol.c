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

#include "Peripheral/Serial/Serial.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void Protocol_Init(Protocol_T * p_protocol, const Protocol_Specs_T * p_specs, void * p_transceiver)
{
	Protocol_Init_Void(p_protocol);
	Protocol_SetSpecs(p_protocol, p_specs);
	Protocol_SetPort(p_protocol, p_transceiver);
}

void Protocol_Init_Void(Protocol_T * p_protocol)
{
	p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_BYTE_1;
	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_REQ;
	p_protocol->TimeStart 	= 0U;
	p_protocol->RxIndex 	= 0U;
	p_protocol->TxLength 	= 0U;
	p_protocol->p_ReqActive = 0U;
}

void Protocol_SetSpecs(Protocol_T * p_protocol, const Protocol_Specs_T * p_specs)
{
	if (p_specs->RX_LENGTH_MAX < p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
	{
		p_protocol->p_Specs = p_specs;
	}
}

void Protocol_SetPort(Protocol_T * p_protocol, void * p_transceiver)
{
	p_protocol->p_Port = p_transceiver;
}

static inline bool PortRxByte(Protocol_T * p_protocol, uint8_t * p_rxChar)
{
	return Serial_RecvChar(p_protocol->p_Port, p_rxChar);
}

static inline bool PortTxString(Protocol_T * p_protocol, const volatile uint8_t * p_string, uint16_t length)
{
	return (length > 0U) ? Serial_SendString(p_protocol->p_Port, p_string, length) : false;
}

//Receive into buffer and check for completion
static inline Protocol_RxStatus_T BuildRxPacket(Protocol_T * p_protocol)
{
	Protocol_RxStatus_T status = PROTOCOL_RX_STATUS_WAIT;

	while (PortRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex]) == true) //skip checking sw buffer
	{
		p_protocol->RxIndex++;
//		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER); //reset byte timeout

		if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MIN)
		{
			if(p_protocol->RxIndex <= p_protocol->p_Specs->RX_LENGTH_MAX)
			{
				status = p_protocol->p_Specs->PARSE_RX_REQ(p_protocol->CONFIG.P_SUBSTATE_BUFFER, &p_protocol->ReqIdActive, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
				if (status != PROTOCOL_RX_STATUS_WAIT)
				{
					break;
				}
			}
			else
			{
				status = PROTOCOL_RX_STATUS_ERROR_REQ;
				break;
			}
		}

//		if((p_protocol->p_Specs->ENCODED == true) && (p_protocol->p_Specs->RX_START_ID != 0U))
//		{
//			if (p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex - 1U] == p_protocol->p_Specs->RX_START_ID) //Received starting char before packet complete
//			{
//				p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] = p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex];
//				p_protocol->RxIndex = 1U;
//			}
//		}
	}

//	if ((p_protocol->p_Specs->ENCODED == true) && (p_protocol->p_Specs->END_ID != 0U))
//	{
//		while (PortRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex]) == true) //skip checking sw buffer
//		{
//			p_protocol->RxIndex++;
//
//			if (p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex - 1U] == p_protocol->p_Specs->END_ID)
//			{
//				(p_protocol->p_Specs->CHECK_RX_COMPLETE(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex)
//				{
//					isComplete = true;
//				}
//			}
//			else if (p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex - 1U] == p_protocol->p_Specs->START_ID) //Received starting char before packet complete
//			{
//				p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] = p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex];
//				p_protocol->RxIndex = 1U;
//			}
//		}
//	}
//	else
//	{
//
//	}

	return status;
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

static inline void ProcTxSync(Protocol_T * p_protocol, Protocol_TxSyncId_T txId)
{
	p_protocol->p_Specs->BUILD_TX_SYNC(p_protocol->CONFIG.P_SUBSTATE_BUFFER, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength, txId);
	PortTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
}

void ProcRxState(Protocol_T * p_protocol)
{
//	p_protocol->RxStatus = PROTOCOL_RX_STATUS_WAIT;

	switch (p_protocol->RxState)
	{
		case PROTOCOL_RX_STATE_WAIT_BYTE_1: /* nonblocking wait state, no timer */
			p_protocol->RxStatus = PROTOCOL_RX_STATUS_WAIT;
			if (PortRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U]) == true)
			{
				/*
				 * Use starting byte even if data segment is unencoded. first char in separate state.
				 */
				if (((p_protocol->p_Specs->RX_START_ID == 0x00U) || (p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] == p_protocol->p_Specs->RX_START_ID)) == true)
				{
					p_protocol->RxIndex = 1U;
					p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
					p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_PACKET;
				}
			}
			break;

		case PROTOCOL_RX_STATE_WAIT_PACKET: /* nonblocking wait state, timer started */
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->RX_TIMEOUT)  /* no need to check for overflow if using millis */
			{
				p_protocol->RxStatus = BuildRxPacket(p_protocol);

//				if(p_protocol->ReqStatus == PROCESSING)
//				{
////					p_protocol->State = ERROR_HANDLING
//				}
				switch(p_protocol->RxStatus)
				{
					case PROTOCOL_RX_STATUS_WAIT:
						break;

					case PROTOCOL_RX_STATUS_ERROR_REQ:
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_REQ_ID);
						p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						break;

					case PROTOCOL_RX_STATUS_ERROR_DATA:
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_DATA_ERROR);
						p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						break;

					case PROTOCOL_RX_STATUS_SUCCESS:
						p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_REQ; //if no pipelining
						p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
						break;

					case PROTOCOL_RX_STATUS_ACK:
						if (p_protocol->ReqStatus == PROTOCOL_REQ_STATUS_AWAIT_SYNC)
						{
							p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_REQ;
							p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
						}
						else
						{
							p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						} //out of sequence error handle
						break;

					case PROTOCOL_RX_STATUS_NACK:
						if(p_protocol->ReqStatus == PROTOCOL_REQ_STATUS_AWAIT_SYNC)
						{
							p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_REQ;
							p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
						}
						else
						{
							p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
						} //out of sequence error handle
						break;

					default: break;
				}
			}
			else
			{
				ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
				p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
			}
			break;

		case PROTOCOL_RX_STATE_WAIT_REQ:
			p_protocol->RxStatus = PROTOCOL_RX_STATUS_WAIT;

//			p_protocol->p_ReqActive->P_EXT->TIMEOUT
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->REQ_TIMEOUT)
			{
				if (p_protocol->ReqStatus == PROTOCOL_REQ_STATUS_COMPLETE)
				{
					p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
				}
				else if ((p_protocol->ReqStatus == PROTOCOL_REQ_STATUS_AWAIT_RX) || (p_protocol->ReqStatus == PROTOCOL_REQ_STATUS_AWAIT_SYNC))
				{
					p_protocol->RxState 	= PROTOCOL_RX_STATE_WAIT_PACKET;
					p_protocol->RxIndex 	= 0U;
					p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
				}
			}
			else
			{
				ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_NACK_TIMEOUT);
				p_protocol->RxState = PROTOCOL_RX_STATE_WAIT_BYTE_1;
			}
			break;

		case PROTOCOL_RX_STATE_INACTIVE:
			break;

		default: break;
	}
}

static inline bool ProcWaitRxSync(Protocol_T * p_protocol)
{
	bool isComplete = false;

	if (p_protocol->RxStatus == PROTOCOL_RX_STATUS_ACK)
	{
		isComplete = true;
	}
	else if (p_protocol->RxStatus == PROTOCOL_RX_STATUS_NACK)
	{
		if (p_protocol->NackCount < p_protocol->p_ReqActive->P_SYNC->WAIT_RX_NACK_REPEAT)
		{
			p_protocol->NackCount++;
			if (p_protocol->p_ReqActive->P_SYNC->USE_TX_RETRANSMIT_ON_NACK == true)
			{
				PortTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
			}
			isComplete = false;
		}
		else
		{
			p_protocol->NackCount = 0U;
			isComplete = true;
		}
	}

	return isComplete;
}

//static inline void SetStateWaitForSync(Protocol_T * p_protocol)
//{
//	p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_AWAIT_SYNC;
//	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
////	p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
//}
//
//static inline void SetStateWaitExtProcess(Protocol_T * p_protocol)
//{
//	p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_PROCESSING;
//	p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
////	p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
//}

void ProcReqState(Protocol_T * p_protocol)
{
	//use interface args for easy extension
	const Protocol_ReqExtProcessArgs_T args =
	{
		.p_SubState = p_protocol->CONFIG.P_SUBSTATE_BUFFER,
		.p_AppInterface = p_protocol->CONFIG.P_APP_INTERFACE,
		.p_TxPacket =  p_protocol->CONFIG.P_TX_PACKET_BUFFER,
		.p_TxSize = &p_protocol->TxLength,
		.p_RxPacket = p_protocol->CONFIG.P_RX_PACKET_BUFFER,
		.RxSize = p_protocol->RxIndex,
		.Option = 0u,
	};

	switch(p_protocol->ReqState)
	{
		case PROTOCOL_REQ_STATE_WAIT_RX_REQ:
//			p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_COMPLETE;
			if(p_protocol->RxStatus == PROTOCOL_RX_STATUS_SUCCESS)
			{
				p_protocol->p_ReqActive = SearchReqTable(p_protocol->p_Specs->P_REQ_TABLE, p_protocol->p_Specs->REQ_TABLE_LENGTH, p_protocol->ReqIdActive);

				if (p_protocol->p_ReqActive != 0U)
				{
					if ((p_protocol->p_ReqActive->P_SYNC != 0U) && (p_protocol->p_ReqActive->P_SYNC->USE_TX_ACK_REQ == true))
					{
						ProcTxSync(p_protocol, PROTOCOL_TX_SYNC_ACK_REQ);
					}

					if (p_protocol->p_ReqActive->FAST != 0U)
					{
						p_protocol->p_ReqActive->FAST(p_protocol->CONFIG.P_APP_INTERFACE, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex);
						PortTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
					}

					if ((p_protocol->p_ReqActive->P_EXT != 0U) && (p_protocol->p_ReqActive->P_EXT->PROCESS != 0U))
					{
						p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_WAIT;
						p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
					}
					else if ((p_protocol->p_ReqActive->P_SYNC != 0U) && (p_protocol->p_ReqActive->P_SYNC->USE_WAIT_RX_ACK_COMPLETE == true))
					{
						p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_AWAIT_SYNC;
						p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
					else
					{
						p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_COMPLETE;
					}
				}
			}
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL:
			if(ProcWaitRxSync(p_protocol) == true)
			{
				p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_COMPLETE;
				p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_REQ;
			} //else still PROTOCOL_REQ_STATUS_AWAIT_SYNC
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_SYNC:
			if(ProcWaitRxSync(p_protocol) == true)
			{
				p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_WAIT;
				p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
			}//else still PROTOCOL_REQ_STATUS_AWAIT_SYNC
			break;

		case PROTOCOL_REQ_STATE_WAIT_RX_DATA:
			if(p_protocol->RxStatus == PROTOCOL_RX_STATUS_SUCCESS)
			{
				p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_WAIT;
				p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_PROCESS;
			} //else still PROTOCOL_REQ_STATE_WAIT_RX_DATA
			break;

		case PROTOCOL_REQ_STATE_WAIT_PROCESS:
			p_protocol->ReqStatus = p_protocol->p_ReqActive->P_EXT->PROCESS(args);
			PortTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);

			switch(p_protocol->ReqStatus)
			{
				case PROTOCOL_REQ_STATUS_WAIT:
					break;

				case PROTOCOL_REQ_STATUS_EXTEND_TIMER:
					p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
					break;

				case PROTOCOL_REQ_STATUS_COMPLETE:
					if((p_protocol->p_ReqActive->P_SYNC != 0U) && (p_protocol->p_ReqActive->P_SYNC->USE_WAIT_RX_ACK_COMPLETE == true))
					{
						p_protocol->ReqStatus 	= PROTOCOL_REQ_STATUS_AWAIT_SYNC;
						p_protocol->ReqState 	= PROTOCOL_REQ_STATE_WAIT_RX_SYNC_FINAL;
					}
					else
					{
						p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_REQ;
					}
					break;

				case PROTOCOL_REQ_STATUS_AWAIT_SYNC:
					p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_SYNC;
//					p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
					break;

				case PROTOCOL_REQ_STATUS_AWAIT_RX:
					p_protocol->ReqState = PROTOCOL_REQ_STATE_WAIT_RX_DATA;
//					p_protocol->TimeStart 	= *(p_protocol->CONFIG.P_TIMER);
					break;

				default:
					break;
			}
			break;

		default:
			break;

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

//	if(p_protocol->State != PROTOCOL_STATE_INACTIVE)
//	{
////		Protocol_Datagram_Proc(p_protocol);
//		//	if(Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
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
//	if(Port_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
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
//			p_protocol->p_ReqFunction(p_protocol->p_Specs->P_BYTES_BUFFER);
//			p_protocol->State = PROTOCOL_STATE_INACTIVE;
//			break;
//
//		case PROTOCOL_STATE_INACTIVE: break;
//		default: break;
//	}
//}

