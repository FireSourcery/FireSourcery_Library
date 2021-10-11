#ifndef PROTOCOL_G_H
#define PROTOCOL_G_H

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Flash/Flash.h"

#include <stdint.h>
#include <stdbool.h>

//typedef struct ProtocolG_Tag ProtocolG_T;

/*
	ProtocolG_Req_T
 */
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

typedef uint32_t (*ProtocolG_ReqFastReadWrite_T)	(uint8_t * p_txPacket, uint8_t * p_txSize, const uint8_t * p_rxPacket, volatile void * p_writeRegs, const volatile void * p_readRegs);

/*
	ProtocolG_ReqExt_T
 */
typedef struct
{
	uint8_t Options[1]; //buffer;
//	volatile ProtocolG_ReqEntry_T * p_ReqActive;
}
ProtocolG_ControlContext_T;

typedef void (*ProtocolG_ReqParseRx_T)		(const uint8_t * p_rxPacket, volatile void * p_writeRegs, ProtocolG_ControlContext_T * p_processContext);
typedef void (*ProtocolG_ReqBuildTx_T)		(uint8_t * p_txPacket, uint8_t * p_txSize,  const volatile void * p_readRegs, ProtocolG_ControlContext_T * p_processContext);
//typedef uint32_t (*ProtocolG_ReqOnComplete_T)	(void * p_writeRegs, ProtocolG_ControlContext_T * p_processContext);
typedef uint32_t protocolg_returncode_t;
typedef protocolg_returncode_t (*ProtocolG_ReqProcess_T) (ProtocolG_ControlContext_T * p_processContext);

//default version in protocol specs, post req id specs, default and per req
typedef const struct Protocol_ReqExt_Tag
{
	const uint8_t * const P_TX_ACK_PACKET_STRING;
	const uint8_t TX_ACK_PACKET_LENGTH;

	const ProtocolG_ReqParseRx_T PARSE_RX;

	const ProtocolG_ReqProcess_T PROCESS;
	volatile void * P_PROCESS_CONTEXT;
	const protocolg_returncode_t WAIT_PROCESS_CODE; //exit nonblocking wait state upon reception
	const uint8_t WAIT_PROCESS_TIME_OUT;
	const uint8_t * const P_TX_NACK_PROCESS_STRING;
	const uint8_t TX_NACK_PROCESS_LENGTH;

	const ProtocolG_ReqBuildTx_T BUILD_TX;

//remove
//	const uint8_t * const P_WAIT_RX_ACK_STRING;
//	const uint8_t WAIT_RX_ACK_LENGTH;
//	const uint8_t WAIT_RX_ACK_TIME_OUT;
//	const uint8_t * const P_TX_NACK_WAIT_STRING;  //when wait for ack times out
//	const uint8_t TX_NACK_WAIT_LENGTH;

	//	uint8_t REPEAT;
} ProtocolG_ReqExt_T;

typedef uint32_t protocolg_reqid_t;

typedef const struct
{
	const protocolg_reqid_t ID;
	const ProtocolG_ReqType_T TYPE;
	const ProtocolG_ReqFastReadWrite_T 	FAST;
	const ProtocolG_ReqExt_T * const 	P_EXT;
//	const Protocol_ReqFlash_T * const P_FLASH_OP;
}
ProtocolG_ReqEntry_T;

#define PROTOCOL_G_REQ_ENTRY(ReqId, ReqType, ReqFast, p_ReqExt) { (.ID = ReqId), (.TYPE = ReqType), (.FAST_READ_WRITE = ReqFast), (.P_EXT = p_ReqExt) }

/*
 * Packet Format Specs
 */
typedef const struct
{
	//pre Id Settings
	const uint32_t RX_TIME_OUT;		//reset if packet is not complete
//	const uint32_t TIME_OUT_BYTE;		//reset if byte is not received
	const uint8_t RX_LENGTH_MAX;

	bool (*const CHECK_RX_COMPLETE)		(const void * p_rxPacket, uint8_t rxCount);
	bool (*const CHECK_RX_CORRECT)		(const void * p_rxPacket);
	protocolg_reqid_t (*const PARSE_RX_REQ_ID)	(const void * p_rxPacket);
//	bool (*const CHECK_RX_PROCESS_CANCEL)	(volatile const uint8_t * p_rxPacket);

	const ProtocolG_ReqEntry_T * const P_REQ_TABLE;
	const uint8_t REQ_TABLE_LENGTH;

	//optional
//	const uint8_t * const P_TX_ACK_PACKET;		const uint8_t TX_ACK_PACKET_LENGTH; //preparse ack?
	const uint8_t * const P_TX_NACK_TIMEOUT;	const uint8_t TX_NACK_TIMEOUT_LENGTH;
	const uint8_t * const P_TX_NACK_DATA;		const uint8_t TX_NACK_DATA_LENGTH;
	const uint8_t * const P_TX_NACK_CMD;		const uint8_t TX_NACK_CMD_LENGTH;

	//todo remove
	ProtocolG_ReqExt_T REQ_EXT_DEFAULT; //default request format

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

	const uint32_t START_ID; //set to 0x00 or 0xff by default for bus idle state
//	const uint8_t START_ID_LENGTH; multichar start/end id
	const uint32_t END_ID;
//	const uint8_t END_ID_LENGTH;
	const bool ENCODED;	//encoded data, non encoded use time out only, past first char, no meta chars
} ProtocolG_Specs_T;

typedef enum
{
	PROTOCOL_STATUS_OK
}
ProtocolG_Status_T;

typedef enum
{
	PROTOCOL_STATE_WAIT_RX_BYTE_1,
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
	uint8_t * const P_RX_PACKET_BUFFER;
	uint8_t * const P_TX_PACKET_BUFFER;
	const uint8_t PACKET_BUFFER_LENGTH; //  must be greater than PACKET_LENGTH_MAX
	//user map to app data interface
	volatile void * const P_WRITE_REGS;
	const volatile void * const P_READ_REGS;
	Flash_T * const P_FLASH;

	const volatile uint32_t * P_TIMER;

//	volatile const void * volatile * const pp_DatagramVarAddressTable;
//	volatile uint8_t * p_DatagramVarSizeTable;
//	[DATAGRAM_VARS_MAX]

	// Can be set after init. 0 to set later
	ProtocolG_Specs_T * P_INIT_SPECS;
	void * P_INIT_DATA_LINK;
	ProtocolG_DataLinkType_T INIT_DATA_LINK_TYPE;
}
ProtocolG_Config_T;



typedef struct ProtocolG_Tag
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

//	volatile const void * volatile p_DatagramVarAddressTable[DATAGRAM_VARS_MAX];
//	volatile uint8_t DatagramVarSizeTable[DATAGRAM_VARS_MAX];

	//proc variables
	volatile ProtocolG_State_T State;
	volatile uint32_t TimeStart;
	volatile uint8_t RxIndex;
	volatile uint8_t TxLength;
	volatile ProtocolG_ReqEntry_T * p_ReqActive;

	volatile ProtocolG_ControlContext_T Control;
}
ProtocolG_T;

extern void ProtocolG_Init(ProtocolG_T * p_protocol);
extern void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs);
extern void ProtocolG_SetDataLink(ProtocolG_T * p_protocol, void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType);
extern ProtocolG_Status_T ProtocolG_Slave_Proc(ProtocolG_T * p_protocol);

#endif


//section pass to user function
//typedef const struct
//{
//	volatile uint8_t * const P_RX_PACKET_BUFFER;
//	volatile uint8_t * const P_TX_PACKET_BUFFER;
//	const uint8_t PACKET_BUFFER_LENGTH;
//	volatile void * const P_WRITE_REGS;
//	volatile const void * const P_READ_REGS;
//	Flash_T * const P_FLASH;
//}
//ProtocolG_UserContext_T;

//typedef uint32_t (*ProtocolG_ReqOpContext_T) (ProtocolG_UserContext_T * p_context, ProtocolG_ControlContext_T * p_buffer);

//typedef const struct Protocol_ReqExt_Tag
//{

//	const ProtocolG_ReqOpContext_T 	PRE_PROCESS;
//
//	const ProtocolG_ReqOpContext_T 	PROCESS;

//	volatile void * P_PROCESS_CONTEXT;
//const protocolg_returncode_t WAIT_PROCESS_CODE; //nonblocking wait state
//const uint8_t WAIT_PROCESS_TIME_OUT;
//const uint8_t * const P_TX_NACK_PROCESS_STRING;
//const uint8_t TX_NACK_PROCESS_LENGTH;
//
//const uint8_t * const P_WAIT_RX_ACK_STRING;
//const uint8_t WAIT_RX_ACK_LENGTH;
//const uint8_t WAIT_RX_ACK_TIME_OUT;
//const uint8_t * const P_TX_NACK_WAIT_ST

//	const ProtocolG_ReqOpContext_T 	POST_PROCESS;

//} ProtocolG_ReqExtOpContext_T;




//typedef struct
//{
//		uint8_t VarWritten;
//} Protocol_FlashContext_T;

//typedef Flash_T Protocol_FlashContext_T;
//
//typedef uint32_t (*ProtocolG_ReqFlashParseRx_T)						(volatile const uint8_t * p_rxPacket, volatile void * p_writeRegs, Protocol_FlashContext_T * p_flashContext);
//typedef protocolg_returncode_t (*ProtocolG_ReqFlashProcess_T)		(Protocol_FlashContext_T * p_processContext);
//typedef uint32_t (*ProtocolG_ReqFlashBuildTx_T)						(uint8_t * p_txPacket, uint8_t * p_txSize, volatile const void * p_readRegs, Protocol_FlashContext_T * p_flashContext);

//typedef const struct Protocol_ReqFlash_Tag
//{
//	bool REPEAT;
//	bool WAIT_FOR_PROCESS;
//	uint8_t * P_WAIT_FOR_ACK_STRING;
//	const ProtocolG_ReqFlashParseRx_T 	PARSE_RX;
//	const ProtocolG_ReqFlashBuildTx_T 	BUILD_TX;
//	const ProtocolG_ReqOnComplete_T 	ON_COMPLETE;
//	bool Start;
//} Protocol_ReqFlash_T;
