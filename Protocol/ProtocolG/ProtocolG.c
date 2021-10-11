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

static bool DataLinkRxByte(ProtocolG_T * p_protocol)
{
	bool rxChar;

	switch(p_protocol->DataLinkType)
	{
		case PROTOCOL_DATA_LINK_MODE_SERIAL:	rxChar = Serial_RecvChar(p_protocol->p_Serial,  &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex]);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		rxChar = I2C_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		rxChar = CanBus_RecvChar(p_protocol->p_Serial);		break;
		default: break;
	}

	return rxChar;
}

static void DataLinkTxPacket(ProtocolG_T * p_protocol)
{
	switch(p_protocol->DataLinkType)
	{
		case PROTOCOL_DATA_LINK_MODE_SERIAL:	 Serial_SendString(p_protocol->p_Serial, p_protocol->CONFIG.P_TX_PACKET_BUFFER, p_protocol->TxLength);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		 I2C_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		 CanBus_RecvChar(p_protocol->p_Serial);		break;
		default: break;
	}
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

//static void RestartRx(ProtocolG_T * p_protocol)
//{
//	p_protocol->RxIndex = 0;
//	p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
//}

//Receive into buffer and check for completion
static bool RxPacket(ProtocolG_T * p_protocol)
{
//	uint8_t rxChar;
	bool isComplete = false;

	//todo dynamic packet timeout
	while (DataLinkRxByte(p_protocol) == true) //skip checking sw buffer
	{
		p_protocol->RxIndex++;
//		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);  //reset  byte timeout

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


static ProtocolG_State_T ProcReqExt(ProtocolG_T * p_protocol, ProtocolG_ReqExt_T * p_reqExt, ProtocolG_ReqFastReadWrite_T fastFunction)
{
	ProtocolG_State_T nextState;

	if(p_reqExt->P_TX_ACK_PACKET_STRING != 0U)
	{
		DataLinkTxString(p_protocol, p_reqExt->P_TX_ACK_PACKET_STRING, p_reqExt->TX_ACK_PACKET_LENGTH);
	}

	if (fastFunction != 0U)
	{
		fastFunction(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->CONFIG.P_WRITE_REGS, p_protocol->CONFIG.P_READ_REGS);
		DataLinkTxPacket(p_protocol);
	}

	if(p_reqExt->PARSE_RX != 0U)
	{
		p_reqExt->PARSE_RX(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->CONFIG.P_WRITE_REGS, p_reqExt->P_PROCESS_CONTEXT);
	}

	if(p_reqExt->PROCESS != 0U)
	{
		nextState = PROTOCOL_STATE_WAIT_PROCESS;
		//WAIT_PROCESS_RX_TIME_OUT
	}
//	else if(p_reqExt->P_WAIT_RX_ACK_STRING != 0U)
//	{
//		p_protocol->RxIndex = 0U;
//		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
//		nextState = PROTOCOL_STATE_WAIT_ACK;
//	}
	else
	{
		if(p_reqExt->BUILD_TX != 0U)
		{
			p_reqExt->BUILD_TX(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_READ_REGS, p_reqExt->P_PROCESS_CONTEXT);
			DataLinkTxPacket(p_protocol);
		}
		nextState = PROTOCOL_STATE_WAIT_RX_BYTE_1;
	}

	return nextState;
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

	uint32_t responseId;

	switch (p_protocol->State)
	{
		case PROTOCOL_STATE_WAIT_RX_BYTE_1: //wait state, no timer
			p_protocol->RxIndex = 0U;
			if (DataLinkRxByte(p_protocol) == true)
			{
//				if (p_protocol->p_Specs->ENCODED == true) //discard all except start byte if using encoding
				if ((p_protocol->p_Specs->START_ID != 0x00U) && (p_protocol->CONFIG.P_RX_PACKET_BUFFER[0U] != p_protocol->p_Specs->START_ID))
				{
					p_protocol->RxIndex = 0U;
				}
				else
				{
					p_protocol->RxIndex = 1U;
					p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
					p_protocol->State = PROTOCOL_STATE_WAIT_RX_PACKET;
				}
			}
			break;

		case PROTOCOL_STATE_WAIT_RX_PACKET: //wait state, timer started
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->RX_TIME_OUT)  //no need to check for overflow if using millis
			{
				if (RxPacket(p_protocol) == true)
				{
					if (p_protocol->p_Specs->CHECK_RX_CORRECT(p_protocol->CONFIG.P_RX_PACKET_BUFFER) == true)
					{
						responseId = p_protocol->p_Specs->PARSE_RX_REQ_ID(p_protocol->CONFIG.P_RX_PACKET_BUFFER);
						p_protocol->p_ReqActive = SearchReqTable(p_protocol, responseId);

						if (p_protocol->p_ReqActive != 0U)
						{
							if(p_protocol->p_ReqActive->P_EXT != 0U)
							{
								p_protocol->State = ProcReqExt(p_protocol, p_protocol->p_ReqActive->P_EXT, p_protocol->p_ReqActive->FAST);
							}
							else
							{
								p_protocol, p_protocol->p_ReqActive->FAST(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->CONFIG.P_WRITE_REGS, p_protocol->CONFIG.P_READ_REGS);
							}
						}
						else
						{
							DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_CMD, p_protocol->p_Specs->TX_NACK_CMD_LENGTH);
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
				DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_TIMEOUT, p_protocol->p_Specs->TX_NACK_TIMEOUT_LENGTH);
				p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
			}
			break;

		case PROTOCOL_STATE_WAIT_PROCESS:
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_ReqActive->P_EXT->WAIT_PROCESS_TIME_OUT)
			{
				if(p_protocol->p_ReqActive->P_EXT->PROCESS(p_protocol->p_ReqActive->P_EXT->P_PROCESS_CONTEXT) == p_protocol->p_ReqActive->P_EXT->WAIT_PROCESS_CODE)
				{
					if(p_protocol->p_ReqActive->P_EXT->BUILD_TX != 0U)
					{
						p_protocol->p_ReqActive->P_EXT->BUILD_TX(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, p_protocol->CONFIG.P_READ_REGS, p_protocol->p_ReqActive->P_EXT->P_PROCESS_CONTEXT);
						DataLinkTxPacket(p_protocol);
					}
					p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
				}
			}
			else
			{
				DataLinkTxString(p_protocol, p_protocol->p_Specs->P_TX_NACK_TIMEOUT, p_protocol->p_Specs->TX_NACK_TIMEOUT_LENGTH);
				p_protocol->State = PROTOCOL_STATE_WAIT_RX_BYTE_1;
			}

//			if(p_protocol->p_ReqActive->P_EXT != 0U)
//			{
//				p_protocol->State = ProcReqExtProcess(p_protocol, p_protocol->p_ReqActive->P_EXT);
//			}

			break;

//		case PROTOCOL_STATE_WAIT_ACK:
////			if(p_protocol->p_ReqActive->P_EXT != 0U)
////			{
////				p_protocol->State = ProcReqWaitForAck(p_protocol, p_protocol->p_ReqActive->P_EXT);
////			}
////			else
////			{
////				p_protocol->State = ProcReqWaitForAck(p_protocol, &p_protocol->p_Specs->REQ_EXT_DEFAULT);
////			}

			break;

//		case PROTOCOL_STATE_SEQUENCE:

		case PROTOCOL_STATE_INACTIVE:
			break;

		default: break;
	}

	return status;
}



//ProtocolG_Status_T Protocol_Datagram_Proc(ProtocolG_T * p_protocol)
//{
//	for (uint8_t iData; iData < p_protocol->TxLength; iData++)
//	{
//if(TxEmpty>p_protocol->TxLength - save space for 1 reply)
//		p_protocol->CONFIG.P_TX_PACKET_BUFFER[iData] = *p_protocol->p_DatagramVarAddressTable[iData];
//	}
//
//	DataLinkTxPacket(p_protocol);
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

