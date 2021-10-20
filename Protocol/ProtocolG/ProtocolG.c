// Simple General configurable protocol

#include "ProtocolG.h"

#include "Peripheral/Serial/Serial.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


void ProtocolG_Init(ProtocolG_T * p_protocol)
{
	p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
	p_protocol->TimeStart = 0U;
	p_protocol->RxIndex = 0U;
	p_protocol->TxLength = 0U;
	p_protocol->p_ReqActive = 0U;
	ProtocolG_SetSpecs(p_protocol, p_protocol->CONFIG.P_INIT_SPECS);
	ProtocolG_SetDataLink(p_protocol, p_protocol->CONFIG.P_INIT_DATA_LINK, p_protocol->CONFIG.INIT_DATA_LINK_TYPE);
}

void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs)
{
	if (p_specs->RX_LENGTH_MAX < p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
	{
		p_protocol->p_Specs = p_specs;
	}
}

void ProtocolG_SetDataLink(ProtocolG_T * p_protocol, void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType)
{
	p_protocol->DataLinkType = dataLinkType;
	p_protocol->p_DataLink = p_dataLink;

	switch(p_protocol->DataLinkType)
	{
		case PROTOCOL_DATA_LINK_MODE_SERIAL:
			p_protocol->p_Serial = (Serial_T *)p_dataLink;
			Serial_ConfigBaudRate(p_protocol->p_Serial, p_protocol->p_Specs->BAUD_RATE_DEFAULT);
			break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:	p_dataLink;		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:	p_dataLink;		break;
		default: break;
	}
}

	static bool DataLinkRxByte(ProtocolG_T * p_protocol, uint8_t * p_rxChar)
	{
		bool rxChar;

		switch(p_protocol->DataLinkType)
		{
			case PROTOCOL_DATA_LINK_MODE_SERIAL:	rxChar = Serial_RecvChar(p_protocol->p_Serial, p_rxChar);		break;
	//		case PROTOCOL_DATA_LINK_MODE_I2C:		rxChar = I2C_RecvChar(p_protocol->p_Serial);		break;
	//		case PROTOCOL_DATA_LINK_MODE_CAN:		rxChar = CanBus_RecvChar(p_protocol->p_Serial);		break;
			default: break;
		}

		return rxChar;
	}

	static void DataLinkTxString(ProtocolG_T * p_protocol, const uint8_t * p_string, uint16_t length)
	{
		switch(p_protocol->DataLinkType)
		{
			case PROTOCOL_DATA_LINK_MODE_SERIAL:	 Serial_SendString(p_protocol->p_Serial, p_string, length);		break;
	//		case PROTOCOL_DATA_LINK_MODE_I2C:		 I2C_RecvChar(p_protocol->p_Serial);		break;
	//		case PROTOCOL_DATA_LINK_MODE_CAN:		 CanBus_RecvChar(p_protocol->p_Serial);		break;
			default: break;
		}
	}

//Receive into buffer and check for completion
//PollRxPacket
static bool BuildRxPacket(ProtocolG_T * p_protocol)
{
//	uint8_t rxChar;
	bool isComplete = false;

	while (DataLinkRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex]) == true) //skip checking sw buffer
	{
		p_protocol->RxIndex++;
//		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER); //todo byte timeout //reset  byte timeout

		//seperate check for header?
		if(p_protocol->p_Specs->CHECK_RX_COMPLETE(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex) == true)
		{
			isComplete = true;
			break;
		}
		else if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MAX)
		{
			isComplete = true;
			break;
		}
		else if (p_protocol->p_Specs->ENCODED == true)
		{
			if (p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex - 1U] == p_protocol->p_Specs->END_ID)
			{
				isComplete = true;
				break;
			}
			else if (p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex - 1U] == p_protocol->p_Specs->START_ID) //Received starting char before packet complete
			{
				p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] = p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex];
				p_protocol->RxIndex = 1U;
			}
		}
	}

	if(isComplete)
	{
		p_protocol->RxIndex = 0;
		//reset packet timeout
	}

	return isComplete;
}

static ProtocolG_ReqEntry_T * SearchReqTable(ProtocolG_T * p_protocol, uint32_t id)
{
	ProtocolG_ReqEntry_T * p_response = 0U;

	for(uint8_t iChar = 0U; iChar < p_protocol->p_Specs->REQ_TABLE_LENGTH; iChar++)
	{
		if (p_protocol->p_Specs->P_REQ_TABLE[iChar].ID == id)
		{
			p_response = &p_protocol->p_Specs->P_REQ_TABLE[iChar];
		}
	}

	return p_response;
}


static void ProcReq(ProtocolG_T * p_protocol)
{
	ProtocolG_State_T nextState;

	//								if(p_protocol->p_ReqActive->P_EXT != 0U)
	//								{
	//									p_protocol->State = ProcReqExt(p_protocol, p_protocol->p_ReqActive->P_EXT, p_protocol->p_ReqActive->FAST);
	//
	//									// if datagram enable and throttled signal
	//								}
	//								else
	//								{
	//									if (p_protocol->p_ReqActive->FAST != 0U)
	//									{
	//										p_protocol->p_ReqActive->FAST(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->CONFIG.P_WRITE_REGS, p_protocol->CONFIG.P_READ_REGS);
	//									}
	//								}
//		if(p_reqExt->P_TX_ACK_RECEPTION_STRING != 0U)
//		{
//			DataLinkTxString(p_protocol, p_reqExt->P_TX_ACK_RECEPTION_STRING, p_reqExt->TX_ACK_RECEPTION_LENGTH);
//		}
//
//	//	if (fastFunction != 0U)
//	//	{
//	//		fastFunction(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->CONFIG.P_WRITE_REGS, p_protocol->CONFIG.P_READ_REGS);
//	//		DataLinkTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
//	//	}
//
//		if(p_reqExt->PARSE_RX != 0U)
//		{
//			p_reqExt->PARSE_RX(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->CONFIG.P_WRITE_REGS, p_reqExt->P_PROCESS_CONTEXT);
//		}
//
//		if(p_reqExt->PROCESS != 0U)
//		{
//			nextState = PROTOCOL_STATE_WAIT_PROCESS;
//			//WAIT_PROCESS_RX_TIME_OUT
//		}
//	//	else if(p_reqExt->P_WAIT_RX_ACK_STRING != 0U)
//	//	{
//	//		p_protocol->RxIndex = 0U;
//	//		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
//	//		nextState = PROTOCOL_STATE_WAIT_ACK;
//	//	}
//		else
//		{
//			if(p_reqExt->BUILD_TX != 0U)
//			{
//				p_reqExt->BUILD_TX(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_READ_REGS, p_reqExt->P_PROCESS_CONTEXT);
//				DataLinkTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
//			}
//			nextState = PROTOCOL_STATE_WAIT_RX_BYTE_1;
//
//		}

	return nextState;
}

static bool CheckSetWaitForAck(ProtocolG_T * p_protocol)
{
	bool isWaitForAck;
	if ((p_protocol->p_ReqActive->P_EXT_SYNC != 0U) && (p_protocol->p_ReqActive->P_EXT_SYNC->P_WAIT_RX_ACK_STRING != 0U))
	{
		p_protocol->RxIndex = 0U;
		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
		p_protocol->State = PROTOCOL_STATE_WAIT_ACK;
		isWaitForAck = true;
	}
	else
	{
		p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
		isWaitForAck = false;
	}

	return isWaitForAck;
}

/*
 * Slave Mode, sequential receive/response
 *
 * non blocking
 * single threaded only, RxIndex not protected
 *
 */
ProtocolG_Status_T ProtocolG_Slave_Proc(ProtocolG_T * p_protocol)
{
	ProtocolG_Status_T status = 0U;

	uint32_t reqId;

	switch (p_protocol->State)
	{
		case PROTOCOL_STATE_WAIT_RX_BYTE_1: /* nonblocking wait state, no timer */
			if (DataLinkRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U]) == true)
			{
				/*
				 * Use starting byte even if data segment is unencoded. first char in separate state.
				 */
				if (!((p_protocol->p_Specs->START_ID != 0x00U) && (p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] != p_protocol->p_Specs->START_ID)))
				{
					p_protocol->RxIndex = 1U;
					p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
					p_protocol->State = PROTOCOL_STATE_WAIT_RX_PACKET;
				}
			}
			break;

		case PROTOCOL_STATE_WAIT_RX_PACKET: /* nonblocking wait state, timer started */
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->RX_TIME_OUT)  /* no need to check for overflow if using millis */
			{
				if (BuildRxPacket(p_protocol) == true)
				{
					if (p_protocol->p_Specs->CHECK_RX_CORRECT(p_protocol->CONFIG.P_RX_PACKET_BUFFER) == true)
					{
						reqId = p_protocol->p_Specs->PARSE_RX_REQ_ID(p_protocol->CONFIG.P_RX_PACKET_BUFFER);
						p_protocol->p_ReqActive = SearchReqTable(p_protocol, reqId);

						if (p_protocol->p_ReqActive != 0U)
						{
							ProcReq(p_protocol);
						}
						else
						{
							DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_REQ, p_protocol->p_Specs->TX_NACK_REQ_LENGTH);
							p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
						}
					}
					else
					{
						DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_DATA, p_protocol->p_Specs->TX_NACK_DATA_LENGTH);
						p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
					}
				}
			}
			else
			{
				DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_TIME, p_protocol->p_Specs->TX_NACK_TIME_LENGTH);
				p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
			}
			break;

		case PROTOCOL_STATE_WAIT_PROCESS:
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_ReqActive->P_EXT_PROCESS->WAIT_PROCESS_TIME)
			{
				switch(p_protocol->p_ReqActive->P_EXT_PROCESS->WAIT_PROCESS(p_protocol->p_ReqActive->P_EXT_CONTEXT,  p_protocol->CONFIG.P_INTERFACE))
				{
					case PROTOCOLG_REQ_RETURN_WAIT:
						break;

					case PROTOCOLG_REQ_RETURN_COMPLETE:
						if(p_protocol->p_ReqActive->P_EXT_PROCESS->BUILD_TX_PROCESS != 0U)
						{
							p_protocol->p_ReqActive->P_EXT_PROCESS->BUILD_TX_PROCESS(p_protocol->p_ReqActive->P_EXT_CONTEXT, p_protocol->CONFIG.P_INTERFACE, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength);

							if(p_protocol->TxLength > 0U)
							{
								DataLinkTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
							}
						}

						 CheckSetWaitForAck(p_protocol);
//						if ((p_protocol->p_ReqActive->P_EXT_SYNC != 0U) && (p_protocol->p_ReqActive->P_EXT_SYNC->P_WAIT_RX_ACK_STRING != 0U))
//						{
//							p_protocol->RxIndex = 0U;
//							p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
//							p_protocol->State = PROTOCOL_STATE_WAIT_ACK;
//						}
//						else
//						{
//							p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
//						}

						break;

					case PROTOCOLG_REQ_RETURN_REPEAT: //REPEAT PROCESS
						p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
						break;

					case PROTOCOLG_REQ_RETURN_REPEAT_AFTER_ACK: //PREPEAT PROCESS_ACK
						 CheckSetWaitForAck(p_protocol);
//						if ((p_protocol->p_ReqActive->P_EXT_SYNC != 0U) && (p_protocol->p_ReqActive->P_EXT_SYNC->P_WAIT_RX_ACK_STRING != 0U))
//						{
//							p_protocol->RxIndex = 0U;
//							p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
//							p_protocol->State = PROTOCOL_STATE_WAIT_ACK;
//						}
//						else
//						{
//
//						}

						p_protocol->RepeatFlag = PROTOCOLG_REQ_RETURN_REPEAT_AFTER_ACK;

						break;

					case PROTOCOLG_REQ_RETURN_REPEAT_RX_PROCESS_ACK: //REAT RX
						 CheckSetWaitForAck(p_protocol);
						 p_protocol->RepeatFlag = PROTOCOLG_REQ_RETURN_REPEAT_RX_PROCESS_ACK;
						break;
					default: break;
				}
			}
			else
			{
				DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_TIME, p_protocol->p_Specs->TX_NACK_TIME_LENGTH);
				p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
			}
			break;

		case PROTOCOL_STATE_WAIT_ACK: 	/* wait for ack, recognize Ack, as ack has special behavior over Req entry */
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_ReqActive->P_EXT_SYNC->WAIT_RX_ACK_TIME)
			{
				while (DataLinkRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex]) == true)
				{
					p_protocol->RxIndex++;

					if(p_protocol->RxIndex == p_protocol->p_ReqActive->P_EXT_SYNC->WAIT_RX_ACK_LENGTH)
					{
						if(strncmp(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->p_ReqActive->P_EXT_SYNC->P_WAIT_RX_ACK_STRING, p_protocol->p_ReqActive->P_EXT_SYNC->WAIT_RX_ACK_LENGTH) == 0)
						{
							if(p_protocol->p_ReqActive->P_EXT_PROCESS->BUILD_TX_PROCESS != 0U)
							{
								p_protocol->p_ReqActive->P_EXT_PROCESS->BUILD_TX_PROCESS(p_protocol->p_ReqActive->P_EXT_CONTEXT, p_protocol->CONFIG.P_INTERFACE, p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength);
								DataLinkTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);
							}

							if (p_protocol->RepeatFlag == true)
							{
								p_protocol->State = PROTOCOL_STATE_WAIT_PROCESS;
								p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
							}
							else
							{
								p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
							}
						}
						break;
					}
					else if(p_protocol->RxIndex == p_protocol->p_ReqActive->P_EXT_SYNC->WAIT_RX_NACK_LENGTH)
					{
						if(strncmp(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->p_ReqActive->P_EXT_SYNC->P_WAIT_RX_NACK_STRING, p_protocol->p_ReqActive->P_EXT_SYNC->WAIT_RX_NACK_LENGTH) == 0)
						{
							if (p_protocol->NackCount < p_protocol->p_ReqActive->P_EXT_SYNC->RX_NACK_REPEAT)
							{
								p_protocol->NackCount++;
								DataLinkTxString(p_protocol, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength); //retransmit
							}
							else
							{
								p_protocol->NackCount = 0U;
								p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
							}
						}
						break;
					}
					else if(p_protocol->RxIndex >= p_protocol->p_Specs->RX_LENGTH_MAX) //error unexpected rx
					{
						p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
						break;
					}
				}
			}
//			else
//			{
//				DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_TIME, p_protocol->p_Specs->TX_NACK_TIME_LENGTH);
//				p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
//			}
			break;



		case PROTOCOL_STATE_INACTIVE:
			break;

		default: break;
	}


	if(p_protocol->State != PROTOCOL_STATE_INACTIVE)
	{
//		Protocol_Datagram_Proc(p_protocol);
		//	if(DataLink_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
		//	{
				if (Datagram_Server_Proc(&p_protocol->Datagram))
				{
					DataLinkTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
				}
		//	}
	}



	return status;
}


//void Protocol_Datagram_Proc(ProtocolG_T * p_protocol)
//{
//	//* save room for 1 req packet
////	if(DataLink_GetTxEmpty() > Datagram_GetPacketSize(&p_protocol->Datagram) +  p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
////	{
//		if (Datagram_Server_Proc(&p_protocol->Datagram))
//		{
//			DataLinkTxString(p_protocol, p_protocol->Datagram.P_TX_BUFFER, p_protocol->Datagram.TxDataSizeActive + p_protocol->Datagram.HeaderSize);
//		}
////	}
//}



/*
 * Master Mode, sequential cmd/Rx
 *
 * non blocking
 * single threaded only,
 *
 */
//ProtocolG_Status_T Protocol_Master_Begin(ProtocolG_T * p_protocol, cmd)
//{
//	p_protocol->State = PROTOCOL_STATE_SEND_CMD;
//}
//
//ProtocolG_Status_T Protocol_Master_Proc(ProtocolG_T * p_protocol)
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

