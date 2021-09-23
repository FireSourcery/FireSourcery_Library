#ifndef PROTOCOL_G_H
#define PROTOCOL_G_H

#include "Peripheral/Serial/Serial.h"

#include <stdint.h>
#include <stdbool.h>


/*
 * Function Table
 */
//typedef void (*ProtocolG_ResponseFunction_T)(); //generic function pointer type
typedef void * ProtocolG_ResponseFunction_T; //generic function pointer type
typedef uint32_t (*ProtocolG_ResponseFunctionRead_T)		(uint8_t * p_txPacket, volatile const uint8_t * p_rxPacket, volatile const void * p_readAccess);
typedef uint32_t (*ProtocolG_ResponseFunctionWrite_T)		(uint8_t * p_txPacket, volatile const uint8_t * p_rxPacket, void * p_writeAccess);
typedef uint32_t (*ProtocolG_ResponseFunctionReadWrite_T)	(uint8_t * p_txPacket, volatile const uint8_t * p_rxPacket, void * p_writeAccess, volatile const void * p_readAccess);
typedef uint32_t (*ProtocolG_ResponseFunctionExecute_T)		(uint8_t * p_txPacket, volatile const uint8_t * p_rxPacket, void * p_execute, void * p_writeAccess, volatile const void * p_readAccess);
//typedef uint32_t (*ProtocolG_ResponseFunctionFull_T)		(uint8_t * p_txPacket, volatile const uint8_t * p_rxPacket, void * p_writeAccess, volatile const void * p_readAccess);
//typedef uint32_t (*ProtocolG_ResponseFunctionConfig_T)	(ProtocolG_T * p_protocol, uint32_t option); //config baud rate, disable, change mode, etc

typedef enum
{
	PROTOCOLG_RESPONSE_READ,
	PROTOCOLG_RESPONSE_WRITE,
	PROTOCOLG_RESPONSE_READ_WRITE,
	PROTOCOLG_RESPONSE_EXECUTE,
	PROTOCOLG_RESPONSE_CONFIG
}
ProtocolG_ResponseType_T;

typedef const struct
{
	const uint32_t ID;
	const ProtocolG_ResponseFunction_T FUNCTION;
	const ProtocolG_ResponseType_T TYPE;
	const bool REPLY;
//	void * P_WRITE_ACCESS;
//	void * P_READ_ACCESS;
}
ProtocolG_ResponseEntry_T;

/*
 * Packet Specs
 */
typedef const struct
{
	const uint32_t TIME_OUT;
	const uint32_t BAUD_RATE_DEFAULT;

	//packet spec
	const uint8_t PACKET_LENGTH_MAX;
	bool (*const CHECK_RX_COMPLETE)(volatile const uint8_t * p_rxPacket, uint8_t rxIndex);
	bool (*const CHECK_RX_CORRECT)(volatile const uint8_t * p_rxPacket);
	uint32_t (*const GET_RESPONSE_ID)(volatile const uint8_t * p_rxPacket); //parse cmd id
	uint32_t (*const GET_TX_LENGTH)(volatile const uint8_t * p_txPacket, volatile const uint8_t * p_RxPacket); //parse cmd id

	const ProtocolG_ResponseEntry_T * const P_RESPONSE_TABLE;
	const uint8_t RESPONSE_TABLE_LENGTH;

	//optional
	const uint32_t START_ID; //set to 0x00 or 0xff by default for bus idle state
//	const uint8_t START_ID_LENGTH; multichar start/end id

	const uint32_t END_ID;
//	const uint8_t END_ID_LENGTH;

	const uint32_t FUNCTION_ID; //can use generalize module function if fixed
//	const uint8_t FUNCTION_LENGTH;
//
//	const uint8_t DATA_LENGTH;
//	const uint8_t DATA_INDEX;

//	const uint8_t ADDRESS_LENGTH;
//	const uint8_t ADDRESS_INDEX;
//
//	const uint8_t ERROR_CHECK_LENGTH;
//	const uint8_t ERROR_CHECK_INDEX;
} ProtocolG_Specs_T;

typedef enum
{
	PROTOCOL_STATUS_OK
}
ProtocolG_Status_T;

//typedef enum
//{
//	PROTOCOL_STATE_RESTART,
//	PROTOCOL_STATE_PACKET_RX,
//	PROTOCOL_STATE_PACKET_CHECK,
//	PROTOCOL_STATE_PACKET_PARSE,
//
//	PROTOCOL_STATE_RESPONSE_SEARCH,
//	PROTOCOL_STATE_RESPONSE_PROCESS,
//
//	PROTOCOL_STATE_INACTIVE,
//}
//ProtocolG_State_T;

typedef enum
{
	PROTOCOL_DATA_LINK_MODE_SERIAL,
	PROTOCOL_DATA_LINK_MODE_I2C,
	PROTOCOL_DATA_LINK_MODE_CAN,
	PROTOCOL_DATA_LINK_MODE_X,
}
ProtocolG_DataLinkType_T;

typedef const struct
{
	//user map to packet parser
	volatile uint8_t * const P_RX_PACKET_BUFFER;
	volatile uint8_t * const P_TX_PACKET_BUFFER;
	const uint8_t PACKET_BUFFER_LENGTH; //if shared buffer, must be greater than PACKET_LENGTH_MAX

	//user map to app data interface
	void * P_WRITE_REGS;
	volatile const void * P_READ_REGS;
	void * P_EXECUTE_CONTEXT;

	volatile const uint32_t * P_TIMER;

	// Can be set after init. 0 to set later
	ProtocolG_Specs_T * P_INIT_SPECS;
	void * P_INIT_DATA_LINK;
	ProtocolG_DataLinkType_T INIT_DATA_LINK_TYPE;
}
ProtocolG_Config_T;

typedef struct
{
	//compile time consts config
	const ProtocolG_Config_T * p_Config;

	//run time config

	const ProtocolG_Specs_T * p_Specs;

	union
	{
		void * p_DataLink;
		Serial_T * p_Serial;
	};

	ProtocolG_DataLinkType_T DataLinkType;
	//	uint32_t BaudRate; /if need to configure multiple per spec
	//	uint32_t TimeOut;

//	void * p_WriteAccess;
//	void * p_ReadAccess;

	//proc variables
//	volatile ProtocolG_State_T State;
	volatile uint32_t TimeStart;
	volatile uint8_t RxIndex;
	volatile uint8_t TxLength;

	//function table based
//	volatile uint32_t ResponseId;
//	ProtocolG_ResponseFunction_T * volatile p_ResponseFunction;

	//indexed in/out data regs
//	uint32_t * p_InputRegs;
//	uint8_t InputRegsCount;
//	uint32_t * p_OutputRegs;
//	uint8_t OutputRegsCount;
}
ProtocolG_T;

extern void ProtocolG_Init(ProtocolG_T * p_protocol, const ProtocolG_Config_T * p_config);
extern void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs);
extern void ProtocolG_SetDataLink(ProtocolG_T * p_protocol, const void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType);
extern ProtocolG_Status_T ProtocolG_Slave_Proc(ProtocolG_T * p_protocol);

#endif
