// Simple General configurable protocol
// start or end char, 1 ack sequence

#include "ProtocolG.h"

#include "Peripheral/Serial/Serial.h"

#include <stdint.h>
#include <stdbool.h>

void ProtocolG_Init(ProtocolG_T * p_protocol)
{
//	p_protocol->p_Config = p_config;
	ProtocolG_SetSpecs(p_protocol, p_protocol->CONFIG.P_INIT_SPECS);
	ProtocolG_SetDataLink(p_protocol, p_protocol->CONFIG.P_INIT_DATA_LINK, p_protocol->CONFIG.INIT_DATA_LINK_TYPE);
}

//void ProtocolG_InitSet(ProtocolG_T * p_protocol, const ProtocolG_Config_T * p_config, const void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType, const ProtocolG_Specs_T * p_specs)
//{
//	p_protocol->p_Config = p_config;
//	ProtocolG_SetSpecs(p_protocol, p_specs);
//	ProtocolG_SetDataLink(p_protocol, p_dataLink, dataLinkType);
//}

void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs)
{
	if (p_specs->PACKET_LENGTH_MAX < p_protocol->CONFIG.PACKET_BUFFER_LENGTH)
	{
		p_protocol->p_Specs = p_specs;
	}
}

void ProtocolG_SetDataLink(ProtocolG_T * p_protocol, const void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType)
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

static void RestartRx(ProtocolG_T * p_protocol)
{
	p_protocol->RxIndex = 0;
	p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
}

//Receive into buffer and check for completion
static bool RxPacket(ProtocolG_T * p_protocol)
{
//	uint8_t rxChar;
	bool isComplete = false;

	//todo dynamic packet timeout
	while (DataLinkRxByte(p_protocol) != false) //skip checking sw buffer
	{
		p_protocol->RxIndex++;
		p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);  //reset  byte timeout

		//seperate check for header?
		if(p_protocol->p_Specs->CHECK_RX_COMPLETE(p_protocol->CONFIG.P_RX_PACKET_BUFFER, p_protocol->RxIndex))
		{
			isComplete = true;
			break;
		}
		else if(p_protocol->RxIndex >= p_protocol->p_Specs->PACKET_LENGTH_MAX)
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

//run inside isr if nonblocking super loop speed is too slow
//Encoded and non encoded checks
//static bool RxPacket_ISR(ProtocolG_T * p_protocol)
//{
////	uint8_t rxChar;
//	bool isComplete = false;
//
//	case SUBSTATE_1:
//		if (DataLinkRxByte(p_protocol, &p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex]) != false)
//		{
//			p_protocol->RxIndex++;
//			p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);  //reset  byte timeout
//		}
//
//	case SUBSTATE_2:
////		if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->TIME_OUT_PACKET) //packet time out if needed
//
//
//			if(isComplete)
//			{
//				p_protocol->RxIndex = 0;
//				//reset packet timeout
//			}
//		}
//		else
//		{
////			set SUBSTATE_1
//			p_protocol->RxIndex = 0;
//			//send txtimeout
//		}
//
//	return isComplete;
//}

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

bool ProcProcess(ProtocolG_T * p_protocol)
{
	bool isComplete;

	switch(p_protocol->p_ReqActive->TYPE)
	{
		case PROTOCOLG_REQ_EXT:
			isComplete = p_protocol->p_ReqActive->P_EXTENDED->PROCESS(p_protocol->p_ReqActive->P_EXTENDED->P_PROCESS_CONTEXT);
			break;
		case PROTOCOLG_REQ_FLASH:
			isComplete = Flash_PollWrite(p_protocol->FlashContext.p_Flash);
			break;

		default: break;
	}
	return isComplete;
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
		case PROTOCOL_STATE_PACKET_WAIT_RX_BYTE_1: //wait state, no timer
			if (DataLinkRxByte(p_protocol) == true)
			{
				p_protocol->RxIndex++;
				p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);  //reset  byte timeout

				if (p_protocol->p_Specs->ENCODED == true) //discard all except start byte if using encoding
				{
					if (p_protocol->CONFIG.P_RX_PACKET_BUFFER[p_protocol->RxIndex - 1U] != p_protocol->p_Specs->START_ID)
					{
						p_protocol->RxIndex = 0;
						p_protocol->TimeStart = *(p_protocol->CONFIG.P_TIMER);
						break;
					}
				}

				p_protocol->State = PROTOCOL_STATE_WAIT_RX_PACKET;
			}
			break;

		case PROTOCOL_STATE_WAIT_RX_PACKET: //wait state, timer started
			if (*p_protocol->CONFIG.P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->TIME_OUT)  //no need to check for overflow if using millis
			{
				if (RxPacket(p_protocol) == true)
				{
					if (p_protocol->p_Specs->CHECK_RX_CORRECT(p_protocol->CONFIG.P_RX_PACKET_BUFFER) == true)
					{
						if (p_protocol->p_Specs->BUILD_TX_ACK_PACKET != 0U)
						{
							p_protocol->p_Specs->BUILD_TX_ACK_PACKET(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength);
						}

						responseId = p_protocol->p_Specs->PARSE_RX_REQ_ID(p_protocol->CONFIG.P_RX_PACKET_BUFFER);
						p_protocol->p_ReqActive = SearchReqTable(p_protocol, responseId);
						if (p_protocol->p_ReqActive != 0U)
						{
							switch(p_protocol->p_ReqActive->TYPE)
							{
								case PROTOCOLG_REQ_FAST_READ_WRITE:
									break;
								case PROTOCOLG_REQ_EXT:
									if (p_protocol->p_ReqActive->P_EXTENDED == 0U)
									{
										if(p_protocol->p_ReqActive->P_EXTENDED->WaitForProcess == true)
										{
											p_protocol->State = PROTOCOL_STATE_WAIT_PROCESS;
										}
										else
										{
											p_protocol->p_ReqActive->P_EXTENDED->PROCESS(p_protocol->p_ReqActive->P_EXTENDED->P_PROCESS_CONTEXT);
										}
									}
									else
									{
										p_protocol->State = PROTOCOL_STATE_PACKET_WAIT_RX_BYTE_1;
									}
									break;
								case PROTOCOLG_REQ_FLASH:
//										typedef uint32_t (*ProtocolG_ReqFlashParseRx_T)		(volatile const uint8_t * p_rxPacket,  Protocol_FlashContext_T * p_flashContext);
//										typedef uint32_t (*ProtocolG_ReqFlashBuildTx_T)		(uint8_t * p_txPacket, uint8_t * p_txSize,  Protocol_FlashContext_T * p_flashContext);
//										p_protocol->p_ReqActive->P_FLASH_OP->PARSE_RX(p_protocol->CONFIG.P_RX_PACKET_BUFFER, &p_protocol->FlashContext);
//										p_protocol->p_ReqActive->P_FLASH_OP->PARSE_RX(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength, &p_protocol->FlashContext);

									break;

								default: break;
							}
						}
						else
						{
							if (p_protocol->p_Specs->BUILD_TX_NACK_CMD != 0U)
							{
								p_protocol->p_Specs->BUILD_TX_NACK_CMD(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength);
							}
							p_protocol->State = PROTOCOL_STATE_PACKET_WAIT_RX_BYTE_1;
						}

					}
					else
					{
						if (p_protocol->p_Specs->BUILD_TX_NACK_DATA != 0U)
						{
							p_protocol->p_Specs->BUILD_TX_NACK_DATA(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength);
						}
						p_protocol->State = PROTOCOL_STATE_PACKET_WAIT_RX_BYTE_1;
					}
				}
			}
			else
			{
				if (p_protocol->p_Specs->BUILD_TX_NACK_TIMEOUT != 0U)
				{
					p_protocol->p_Specs->BUILD_TX_NACK_TIMEOUT(p_protocol->CONFIG.P_TX_PACKET_BUFFER, &p_protocol->TxLength);
				}
				p_protocol->State = PROTOCOL_STATE_PACKET_WAIT_RX_BYTE_1;
			}
			break;

		case PROTOCOL_STATE_WAIT_PROCESS:
			if (ProcProcess(p_protocol))
			{
//				if()
//				{
//					buildTxpacket
//					DataLinkTxPacket(p_protocol);
//					p_protocol->State = PROTOCOL_STATE_PACKET_WAIT_RX_PACKET;
//
//				}
//				else
//				{
//					p_protocol->State = PROTOCOL_STATE_WAIT_ACK;
//				}
			}

		case PROTOCOL_STATE_WAIT_ACK:
			//if(ack)  buildTxpacket DataLinkTxPacket(p_protocol);

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

