#ifndef PROTOCOL_G_H
#define PROTOCOL_G_H

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Flash/Flash.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct ProtocolG_Tag ProtocolG_T;
//typedef struct Protocol_ControlContext_Tag Protocol_ControlContext_T;
/*
 * Function Table
 */
//typedef void (*ProtocolG_ReqFunction_T)(); //generic function pointer type
//typedef void * ProtocolG_ReqFunction_T;
//typedef uint32_t (*ProtocolG_ReqFunctionExecute_T)	(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const uint8_t * p_rxPacket, void * p_execute, void * p_writeRegs, volatile const void * p_readRegs);
//typedef uint32_t (*ProtocolG_ReqFunctionRead_T)		(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const uint8_t * p_rxPacket, volatile const void * p_readRegs);
//typedef uint32_t (*ProtocolG_ReqFunctionWrite_T)		(volatile const uint8_t * p_rxPacket, void * p_writeRegs);
//typedef uint32_t (*ProtocolG_ReqFunctionReadWrite_T)	(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const uint8_t * p_rxPacket, void * p_writeRegs, volatile const void * p_readRegs);
//typedef uint32_t (*ProtocolG_ReqFunctionFull_T)		(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const uint8_t * p_rxPacket, void * p_writeRegs, volatile const void * p_readRegs, Protocol_ControlContext_T * p_control);

//typedef uint32_t (*ProtocolG_ControlFunction_T)		(Protocol_ControlContext_T * p_control);

//typedef union
//{
//	ProtocolG_ReqFunctionReadWrite_T;
//	ProtocolG_ReqFunctionFull_T;
//} ProtocolG_ReqFunction_T;

typedef enum
{
//	PROTOCOLG_REQ_READ,
//	PROTOCOLG_REQ_WRITE,
//	PROTOCOLG_REQ_READ_WRITE,
//	PROTOCOLG_REQ_EXECUTE,
//	PROTOCOLG_REQ_FULL,

	PROTOCOLG_REQ_FAST_READ_WRITE,
	PROTOCOLG_REQ_EXT,
	PROTOCOLG_REQ_FLASH,
}
ProtocolG_ReqType_T;

typedef uint32_t (*ProtocolG_ReqFastReadWrite_T)	(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const uint8_t * p_rxPacket, void * p_writeRegs, volatile const void * p_readRegs);
typedef uint32_t (*ProtocolG_ReqParseRx_T)		(volatile const uint8_t * p_rxPacket, void * p_writeRegs, void * p_processContext);
typedef bool 	 (*ProtocolG_ReqProcess_T)		(void * p_processContext);
typedef uint32_t (*ProtocolG_ReqBuildTx_T)		(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const void * p_readRegs, void * p_processContext);
typedef uint32_t (*ProtocolG_ReqOnComplete_T)	(void * p_writeRegs, void * p_processContext);

typedef const struct Protocol_ReqExt_Tag
{

//	uint32_t TimeOut;
//	bool SendAck;
//	bool SendNack;
	//subcommand

	bool Repeat;
	bool WaitForProcess; //nonblocking wait state
//	bool WaitForAck; //nonblocking wait state
	uint8_t * p_WaitForAckString;

	const ProtocolG_ReqParseRx_T 	PARSE_RX;
	const ProtocolG_ReqProcess_T 	PROCESS;
	const ProtocolG_ReqBuildTx_T 	BUILD_TX;
	volatile void * P_PROCESS_CONTEXT;

//	void (*OnCompleteConversion)(volatile void * p_userData); 		/* On conversion group complete */
//	volatile void *p_OnCompleteUserData;

//	uint8_t Options[];
	//indexed in/out data regs
//	uint32_t * p_InputRegs;
//	uint8_t InputRegsCount;
//	uint32_t * p_OutputRegs;
//	uint8_t OutputRegsCount;

//	bool PassFlashContext; //special case send flash context

} Protocol_ReqExtended_T;

typedef struct
{
	Flash_T * p_Flash;
//	uint8_t VarWritten;
} Protocol_FlashContext_T;

typedef uint32_t (*ProtocolG_ReqFlashParseRx_T)		(volatile const uint8_t * p_rxPacket,  Protocol_FlashContext_T * p_flashContext);
typedef uint32_t (*ProtocolG_ReqFlashBuildTx_T)		(uint8_t * p_txPacket, uint8_t * p_txSize,  Protocol_FlashContext_T * p_flashContext);

typedef const struct Protocol_ReqFlash_Tag
{
	bool Repeat;
	bool WaitForProcess; //nonblocking wait state
	uint8_t * p_WaitForAckString;
	const ProtocolG_ReqFlashParseRx_T 	PARSE_RX;
	const ProtocolG_ReqFlashBuildTx_T 	BUILD_TX;
	const ProtocolG_ReqOnComplete_T 	ON_COMPLETE;
	bool Start; //special case send flash context
} Protocol_ReqFlash_T;

typedef const struct
{
	const uint32_t ID;
	const ProtocolG_ReqType_T TYPE;
	const ProtocolG_ReqFastReadWrite_T 	FAST_READ_WRITE;
	const Protocol_ReqExtended_T * const P_EXTENDED;
	const Protocol_ReqFlash_T * const P_FLASH_OP;
}
ProtocolG_ReqEntry_T;

/*
 * Packet Specs
 */
typedef const struct
{
	const uint32_t TIME_OUT;
//	const uint32_t TIME_OUT_BYTE;		//reset if byte is not recived
//	const uint32_t TIME_OUT_PACKET;		//reset if packet is not complete
	const uint8_t PACKET_LENGTH_MAX;

	//primary response entry

	bool (*const CHECK_RX_COMPLETE)(volatile const uint8_t * p_rxPacket, uint8_t rxCount);
	bool (*const CHECK_RX_CORRECT)(volatile const uint8_t * p_rxPacket);
	uint32_t (*const PARSE_RX_REQ_ID)(volatile const uint8_t * p_rxPacket);
//	bool (*const CHECK_RX_PROCESS_CANCEL)(volatile const uint8_t * p_rxPacket);
	const ProtocolG_ReqEntry_T * const P_REQ_TABLE;
	const uint8_t REQ_TABLE_LENGTH;

	//optional

	//pre parse rx id
	const (*const BUILD_TX_ACK_PACKET)(volatile uint8_t * p_txPacket, uint8_t * p_txSize);
	const (*const BUILD_TX_NACK_DATA)(volatile uint8_t * p_txPacket, uint8_t * p_txSize);
	const (*const BUILD_TX_NACK_CMD)(volatile uint8_t * p_txPacket, uint8_t * p_txSize);
	const (*const BUILD_TX_NACK_TIMEOUT)(volatile uint8_t * p_txPacket, uint8_t * p_txSize);

	const union
	{
		const struct
		{
			const uint32_t BAUD_RATE_DEFAULT;
		};
		const struct
		{

		};
	};

	//encoded protocol only
	const bool ENCODED;	//non encoded use time out only, no meta chars
	const uint32_t START_ID; //set to 0x00 or 0xff by default for bus idle state
//	const uint8_t START_ID_LENGTH; multichar start/end id
	const uint32_t END_ID;
//	const uint8_t END_ID_LENGTH;

} ProtocolG_Specs_T;

typedef enum
{
	PROTOCOL_STATUS_OK
}
ProtocolG_Status_T;

typedef enum
{
	PROTOCOL_STATE_PACKET_WAIT_RX_BYTE_1,
	PROTOCOL_STATE_WAIT_RX_PACKET,
	PROTOCOL_STATE_WAIT_PROCESS,
	PROTOCOL_STATE_WAIT_ACK,

	PROTOCOL_STATE_INACTIVE,
}
ProtocolG_State_T;

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
	volatile void * const P_WRITE_REGS;
	volatile const void * const P_READ_REGS;

	volatile const uint32_t * P_TIMER;

//	volatile const void * volatile * const pp_DatagramVarAddressTable;
//	volatile uint8_t * p_DatagramVarSizeTable;
//	[DATAGRAM_VARS_MAX]

	// Can be set after init. 0 to set later
	ProtocolG_Specs_T * P_INIT_SPECS;
	void * P_INIT_DATA_LINK;
	ProtocolG_DataLinkType_T INIT_DATA_LINK_TYPE;

//	volatile void * P_REQ_EXT_CONTEXT_DEFAULT;
}
ProtocolG_Config_T;



struct ProtocolG_Tag
{
	//compile time consts config
	const ProtocolG_Config_T CONFIG;

	//run time config

	const ProtocolG_Specs_T * p_Specs;

	union
	{
		void * p_DataLink;
		Serial_T * p_Serial;
	};

	ProtocolG_DataLinkType_T DataLinkType;

	Protocol_FlashContext_T FlashContext;

//	volatile const void * volatile p_DatagramVarAddressTable[DATAGRAM_VARS_MAX];
//	volatile uint8_t DatagramVarSizeTable[DATAGRAM_VARS_MAX];

	//proc variables
	volatile ProtocolG_State_T State;
	volatile uint32_t TimeStart;
	volatile uint8_t RxIndex;
	volatile uint8_t TxLength;

	volatile ProtocolG_ReqEntry_T * p_ReqActive;


//	const ProtocolG_ReqEntry_T * const p_ReqSequence;
//	const uint8_t ReqSequence;

//	volatile Protocol_ControlContext_T ControlContext;
};

//extern void ProtocolG_Init(ProtocolG_T * p_protocol, const ProtocolG_Config_T * p_config);
//extern void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs);
//extern void ProtocolG_SetDataLink(ProtocolG_T * p_protocol, const void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType);
//extern ProtocolG_Status_T ProtocolG_Slave_Proc(ProtocolG_T * p_protocol);

#endif
