// Simple General configurable protocol
// start or end char, 1 ack sequence

#include "ProtocolG.h"

#include "Peripheral/Serial/Serial.h"

#include <stdint.h>
#include <stdbool.h>

void ProtocolG_Init(ProtocolG_T * p_protocol, const ProtocolG_Config_T * p_config)
{
	p_protocol->p_Config = p_config;
	ProtocolG_SetSpecs(p_protocol, p_config->P_INIT_SPECS);
	ProtocolG_SetDataLink(p_protocol, p_config->P_INIT_DATA_LINK, p_config->INIT_DATA_LINK_TYPE);
}

void ProtocolG_InitSet(ProtocolG_T * p_protocol, const ProtocolG_Config_T * p_config, const void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType, const ProtocolG_Specs_T * p_specs)
{
	p_protocol->p_Config = p_config;
	ProtocolG_SetSpecs(p_protocol, p_specs);
	ProtocolG_SetDataLink(p_protocol, p_dataLink, dataLinkType);
}

void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs)
{
	if (p_specs->PACKET_LENGTH_MAX < p_protocol->p_Config->PACKET_BUFFER_LENGTH)
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
			Serial_SetBaudRate(p_protocol->p_Serial, p_protocol->p_Specs->BAUD_RATE_DEFAULT);
			break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:	p_dataLink;		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:	p_dataLink;		break;
		default: break;
	}
}


static uint8_t RxByte(ProtocolG_T * p_protocol)
{
	uint8_t rxChar;

	switch(p_protocol->DataLinkType)
	{
		case PROTOCOL_DATA_LINK_MODE_SERIAL:	rxChar = Serial_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		rxChar = I2C_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		rxChar = CanBus_RecvChar(p_protocol->p_Serial);		break;
		default: break;
	}

	return rxChar;
}

//static uint8_t RxBuffer(ProtocolG_T * p_protocol)
//{
//	uint8_t count;
//
//	switch(p_protocol->DataLinkType)
//	{
////		case PROTOCOL_DATA_LINK_MODE_SERIAL:	count = Serial_Recv(p_protocol->p_Serial);		break;
////		case PROTOCOL_DATA_LINK_MODE_I2C:		count = I2C_RecvChar(p_protocol->p_Serial);		break;
////		case PROTOCOL_DATA_LINK_MODE_CAN:		count = CanBus_RecvChar(p_protocol->p_Serial);		break;
//		default: break;
//	}
//
//	return count;
//}

static uint8_t GetAvailableRx(ProtocolG_T * p_protocol)
{
	uint8_t rxAvailabe;

	switch(p_protocol->DataLinkType)
	{
		case PROTOCOL_DATA_LINK_MODE_SERIAL:	rxAvailabe = Serial_GetAvailableRx(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		rxChar = I2C_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		rxChar = CanBus_RecvChar(p_protocol->p_Serial);		break;
		default: break;
	}

	return rxAvailabe;
}

static void RestartRx(ProtocolG_T * p_protocol)
{
	p_protocol->RxIndex = 0;
	p_protocol->TimeStart = *(p_protocol->p_Config->P_TIMER);
}

//Receive into buffer and check for completion
static bool RxPacket(ProtocolG_T * p_protocol)
{
	uint8_t rxChar;
	bool isComplete = false;

//	Serial_Recv(p_protocol->p_Serial, p_protocol->p_destBuffer, p_protocol->p_Specs->LENGTH_MAX - p_protocol->RxIndex)
	//p_protocol->RxIndex += revclength

	while (GetAvailableRx(p_protocol) != 0U)
	{
		rxChar = RxByte(p_protocol);

		p_protocol->p_Config->P_RX_PACKET_BUFFER[p_protocol->RxIndex] = rxChar;
		p_protocol->RxIndex++;

		if(p_protocol->p_Specs->CHECK_RX_COMPLETE(p_protocol->p_Config->P_RX_PACKET_BUFFER, p_protocol->RxIndex))
		{
			isComplete = true;
		}
		else if(p_protocol->RxIndex >= p_protocol->p_Specs->PACKET_LENGTH_MAX)
		{
			isComplete = true;
		}
		// generalized conditions enable/disable
		else if(rxChar == p_protocol->p_Specs->END_ID)
		{
			isComplete = true;
		}
		else if(rxChar == p_protocol->p_Specs->START_ID) //Received starting char before packet complete
		{
			p_protocol->RxIndex = 0;
			p_protocol->p_Config->P_RX_PACKET_BUFFER[p_protocol->RxIndex] = rxChar;
			p_protocol->RxIndex++;
		}


//		else //(p_protocol->RxIndex < ETS_PACKET_LENGTH_MAX)
//		{
//
//		}

		if(isComplete) { break; }
	};
}


static void TxPacket(ProtocolG_T * p_protocol)
{
	switch(p_protocol->DataLinkType)
	{
		case PROTOCOL_DATA_LINK_MODE_SERIAL:	 Serial_SendString(p_protocol->p_Serial, p_protocol->p_Config->P_TX_PACKET_BUFFER, p_protocol->TxLength);		break;
//		case PROTOCOL_DATA_LINK_MODE_I2C:		 I2C_RecvChar(p_protocol->p_Serial);		break;
//		case PROTOCOL_DATA_LINK_MODE_CAN:		 CanBus_RecvChar(p_protocol->p_Serial);		break;
		default: break;
	}
}

static ProtocolG_ResponseEntry_T * SearchResponseTable(ProtocolG_T * p_protocol, uint32_t id)
{
	ProtocolG_ResponseEntry_T * p_response = 0U;

	for(uint8_t iChar = 0U; iChar < p_protocol->p_Specs->RESPONSE_TABLE_LENGTH; iChar++)
	{
		if (p_protocol->p_Specs->P_RESPONSE_TABLE[iChar].ID == id)
		{
			p_response = &p_protocol->p_Specs->P_RESPONSE_TABLE[iChar];
		}
	}

	return p_response;
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
	uint32_t responseId;
	ProtocolG_ResponseEntry_T * p_response;

	if (*p_protocol->p_Config->P_TIMER - p_protocol->TimeStart < p_protocol->p_Specs->TIME_OUT)
	{
		if (RxPacket(p_protocol) == true)
		{
			RestartRx(p_protocol);

			if (p_protocol->p_Specs->CHECK_RX_CORRECT(p_protocol->p_Config->P_RX_PACKET_BUFFER) == true)
			{
				responseId = p_protocol->p_Specs->GET_RESPONSE_ID(p_protocol->p_Config->P_RX_PACKET_BUFFER);
				p_response = SearchResponseTable(p_protocol, responseId);
				if (p_response != 0U)
				{
					switch(p_response->TYPE)
					{
						case PROTOCOLG_RESPONSE_READ:
							((ProtocolG_ResponseFunctionRead_T)p_response->FUNCTION)(p_protocol->p_Config->P_TX_PACKET_BUFFER, p_protocol->p_Config->P_RX_PACKET_BUFFER, p_protocol->p_Config->P_READ_REGS);
							break;
						case PROTOCOLG_RESPONSE_WRITE:
							//if (p_protocol.WriteEnabled)
							((ProtocolG_ResponseFunctionWrite_T)p_response->FUNCTION)(p_protocol->p_Config->P_TX_PACKET_BUFFER, p_protocol->p_Config->P_RX_PACKET_BUFFER, p_protocol->p_Config->P_WRITE_REGS);
							break;
						case PROTOCOLG_RESPONSE_READ_WRITE:
							((ProtocolG_ResponseFunctionReadWrite_T)p_response->FUNCTION)(p_protocol->p_Config->P_TX_PACKET_BUFFER, p_protocol->p_Config->P_RX_PACKET_BUFFER, p_protocol->p_Config->P_WRITE_REGS, p_protocol->p_Config->P_READ_REGS);
							break;
						case PROTOCOLG_RESPONSE_EXECUTE:
							((ProtocolG_ResponseFunctionExecute_T)p_response->FUNCTION)(p_protocol->p_Config->P_TX_PACKET_BUFFER, p_protocol->p_Config->P_RX_PACKET_BUFFER, p_protocol->p_Config->P_EXECUTE_CONTEXT, p_protocol->p_Config->P_WRITE_REGS, p_protocol->p_Config->P_READ_REGS);
							break;
						case PROTOCOLG_RESPONSE_CONFIG:
							break;
						default: break;
					}

					if ((p_response->REPLY == true)) //||(p_response->TYPE == PROTOCOLG_RESPONSE_READ)
					{
						p_protocol->TxLength = p_protocol->p_Specs->GET_TX_LENGTH(p_protocol->p_Config->P_TX_PACKET_BUFFER, p_protocol->p_Config->P_RX_PACKET_BUFFER);
						TxPacket(p_protocol);
					}
				}
				else
				{
					RestartRx(p_protocol);
//					p_protocol->p_Specs->REPLY_CMD_INVALID(p_protocol->p_Config->P_TX_PACKET_BUFFER);
					//send invalid cmd
				}
			}
			else
			{
				RestartRx(p_protocol);
				//sned packet error
			}
		}
	}
	else
	{
		RestartRx(p_protocol);
		//send txtimeout
	}
}

//ProtocolG_Status_T Protocol_Slave_Proc(ProtocolG_T * p_protocol)
//{
//	ProtocolG_Status_T status = 0U;
//
//	switch (p_protocol->State)
//	{
//		case PROTOCOL_STATE_RESTART:
//			p_protocol->RxIndex = 0;
//			p_protocol->TimeStart = *(p_protocol->p_Time);
//			p_protocol->State = PROTOCOL_STATE_PACKET_RX;
//			status = PROTOCOL_STATUS_OK;
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
//				p_protocol->State = PROTOCOL_STATE_RESTART;
//				//send txtimeout
//			}
//
//			status = PROTOCOL_STATUS_OK;
//			break;
//
//		case PROTOCOL_STATE_PACKET_CHECK:
//			if (CheckRx(p_protocol))
//			{
//				p_protocol->State = PROTOCOL_STATE_PACKET_PARSE;
//			}
//			else
//			{
//				p_protocol->State = PROTOCOL_STATE_RESTART;
//				//			sned error
//			}
//			status = PROTOCOL_STATUS_OK;
//			break;
//
//		case PROTOCOL_STATE_PACKET_PARSE:
////			ParseRx(p_protocol->p_Specs->P_BYTES_BUFFER, p_protocol->p_InputRegs);
//			p_protocol->ResponseId = p_protocol->p_Specs->PARSE(p_protocol->p_Specs->P_BYTES_BUFFER);
//			p_protocol->State = PROTOCOL_STATE_CMD_SEARCH;
//			status = PROTOCOL_STATUS_OK;
//			break;
//
//		case PROTOCOL_STATE_CMD_SEARCH:	//function table based
//			p_protocol->p_ResponseFunction = SearchResponseFunctionTable(p_protocol, p_protocol->ResponseId);
//			if (p_protocol->p_ResponseFunction != 0U)
//			{
//				p_protocol->State = PROTOCOL_STATE_CMD_PROCESS;
////				status =  PROTOCOL_STATUS_CMD_ACCEPTED;
//			}
//			else
//			{
//
//				p_protocol->State = PROTOCOL_STATE_RESTART;
//				//send invalid
////				status = PROTOCOL_STATUS_CMD_INVALID;
//			}
//			break;
//
//		case PROTOCOL_STATE_CMD_PROCESS:
//			p_protocol->p_ResponseFunction(p_protocol->p_Specs->P_BYTES_BUFFER);
//			break;
//
//		case PROTOCOL_STATE_INACTIVE:
//			break;
//
//		default: break;
//	}
//
//	return status;
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
//			p_protocol->p_ResponseFunction(p_protocol->p_Specs->P_BYTES_BUFFER);
//			p_protocol->State = PROTOCOL_STATE_INACTIVE;
//			break;
//
//		case PROTOCOL_STATE_INACTIVE: break;
//		default: break;
//	}
//}

//static bool CheckCompleteRx(ProtocolG_T * p_protocol)
//{
////	generalize completition
////	if( )
//
////	else if(p_protocol->RxIndex > ETS_PACKET_LENGTH_MAX - 1U) // buffer full
//
//	//by end char
//	//by start char
//	//by user provided function
//}

//static bool CheckErrorRx(ProtocolG_T * p_protocol)
//{
//	return p_protocol->p_Specs->CHECK_RX_CORRECT(p_protocol->p_Specs->P_BYTES_BUFFER);
//}
