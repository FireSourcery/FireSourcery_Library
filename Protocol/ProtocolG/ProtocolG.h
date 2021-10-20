#ifndef PROTOCOL_G_H
#define PROTOCOL_G_H

#include "Datagram.h"

#include "Peripheral/Serial/Serial.h"
#include "Peripheral/Flash/Flash.h"

#include <stdint.h>
#include <stdbool.h>

//typedef struct ProtocolG_Tag ProtocolG_T;

/*
	ProtocolG_Req_T
 */
typedef uint32_t protocolg_reqid_t;

typedef enum
{
//	PROTOCOLG_REQ_READ,
//	PROTOCOLG_REQ_WRITE,
//	PROTOCOLG_REQ_READ_WRITE,
//	PROTOCOLG_REQ_EXECUTE,
//	PROTOCOLG_REQ_FULL,
	PROTOCOLG_REQ_FAST_READ_WRITE,
	PROTOCOLG_REQ_EXT, 		//passes user defined P_PROCESS_CONTEXT
	PROTOCOLG_REQ_FLASH, 	//passes special control context
	PROTOCOLG_REQ_DATAGRAM,	//passes special control context
//	PROTOCOLG_REQ_DATAGRAM_CONTROL,	//when datagram uses non unique acks, need seperate state
}
ProtocolG_ReqType_T;

typedef uint32_t (*ProtocolG_ReqFastReadWrite_T)	(volatile void * p_appInterface, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize);

/*
 *
 */

typedef enum
{
	PROTOCOLG_REQ_RETURN_WAIT,
	PROTOCOLG_REQ_RETURN_COMPLETE, //exit nonblocking wait state upon reception
	PROTOCOLG_REQ_RETURN_REPEAT,
	PROTOCOLG_REQ_RETURN_REPEAT_AFTER_ACK,

	PROTOCOLG_REQ_RETURN_REPEAT_PROCESS,
	PROTOCOLG_REQ_RETURN_REPEAT_PROCESS_ACK,
	PROTOCOLG_REQ_RETURN_REPEAT_RX_PROCESS_ACK,
}
ProtocolG_ReqReturn_T;

//typedef uint32_t protocolg_retcode_t;

typedef void 					(*ProtocolG_ReqParseRx_T)	(volatile void * p_processContext, volatile void * p_appInterface, const volatile uint8_t * p_rxPacket, size_t rxSize);
typedef void 					(*ProtocolG_ReqBuildTx_T)	(volatile void * p_processContext, volatile void * p_appInterface, volatile uint8_t * p_txPacket, volatile size_t * p_txSize);
typedef ProtocolG_ReqReturn_T 	(*ProtocolG_ReqProcess_T) 	(volatile void * p_processContext, volatile void * p_appInterface);

/*
	for wait process state, and template/format behavior
	process context support, flash, datagram, user provide
 */
typedef const struct
{
	const ProtocolG_ReqParseRx_T PARSE_RX;
	const ProtocolG_ReqBuildTx_T BUILD_TX_ACK_REQ;	//dynamic ack using parsed info

	const ProtocolG_ReqProcess_T WAIT_PROCESS;
//	const protocolg_retcode_t WAIT_PROCESS_COMPLETE; //exit nonblocking wait state upon reception
	const uint8_t WAIT_PROCESS_TIME;

	const ProtocolG_ReqBuildTx_T BUILD_TX_PROCESS;

	// dynamic ack nack response using parsed info
	//	const ProtocolG_ReqProcess_T ON_RX_ACK_PROCESS;
	//	const ProtocolG_ReqProcess_T ON_RX_NACK_PROCESS;
	//
	//	const ProtocolG_ReqBuildTx_T ON_RX_ACK_BUILD_TX;
	//	const ProtocolG_ReqBuildTx_T ON_RX_NACK_BUILD_TX;
	//
	//	const ProtocolG_ReqBuildTx_T RX_ACK_ERROR_BUILD_TX;
	//	const ProtocolG_ReqBuildTx_T RX_ACK_TIME_OUT_BUILD_TX;

} ProtocolG_ReqExtProcess_T;

typedef const struct
{
	//pre process
	const uint8_t * const P_TX_ACK_REQ_STRING; //static pre parse ack
	const uint8_t TX_ACK_REQ_LENGTH;

	//post process
	//	bool (*const WAIT_RX_ACK) (const void * p_rxPacket, uint8_t rxCount);  //fp faster over string compare?
	const uint8_t WAIT_RX_ACK_TIME;
	const uint8_t * const P_WAIT_RX_ACK_STRING;		const uint8_t WAIT_RX_ACK_LENGTH;
	const uint8_t * const P_WAIT_RX_NACK_STRING;	const uint8_t WAIT_RX_NACK_LENGTH;

//	const uint8_t * const P_TX_WAIT_RX_ACK_TIME_OUT_STRING;  //when wait for ack times out
//	const uint8_t P_TX_WAIT_RX_ACK_TIME_OUT_LENGTH;

	const uint8_t RX_NACK_REPEAT; //stay in wait for ack state
}
ProtocolG_ReqExtSync_T;

typedef const struct
{
	const protocolg_reqid_t 			ID;
	const ProtocolG_ReqType_T 			TYPE; /* module cmd code, corresponds to context passed to functions */
	const ProtocolG_ReqFastReadWrite_T 	FAST;
	volatile void * P_EXT_CONTEXT;
	const ProtocolG_ReqExtProcess_T * const P_EXT_PROCESS;
	const ProtocolG_ReqExtSync_T 	* const P_EXT_SYNC;
}
ProtocolG_ReqEntry_T;

#define PROTOCOL_G_REQ_ENTRY(ReqId, ReqType, ReqFast, p_ReqExt) { (.ID = ReqId), (.TYPE = ReqType), (.FAST_READ_WRITE = ReqFast), (.P_EXT = p_ReqExt) }


/*
 * Packet Format Specs
 */
typedef const struct
{
	/* required to identify rx cmd */
	const uint32_t RX_TIME_OUT;			//reset if packet is not complete
//	const uint32_t TIME_OUT_BYTE;		//reset if byte is not received
	const uint8_t RX_LENGTH_MIN;
	const uint8_t RX_LENGTH_MAX;
	bool (*const CHECK_RX_COMPLETE)	(const volatile void * p_rxPacket, uint8_t rxCount);
	bool (*const CHECK_RX_CORRECT)	(const volatile void * p_rxPacket);
	protocolg_reqid_t (*const PARSE_RX_REQ_ID) (const volatile void * p_rxPacket);

//	bool (*const CHECK_RX_COMPLETE)	(const volatile void * p_rxPacket, uint8_t rxCount, void* p_appInterface);

	const ProtocolG_ReqEntry_T * const P_REQ_TABLE;
	const uint8_t REQ_TABLE_LENGTH;

	//optional
	const uint8_t * const P_TX_NACK_TIME;	const uint8_t TX_NACK_TIME_LENGTH;
	const uint8_t * const P_TX_NACK_DATA;	const uint8_t TX_NACK_DATA_LENGTH;
	const uint8_t * const P_TX_NACK_REQ;	const uint8_t TX_NACK_REQ_LENGTH;

//	bool (*const CHECK_ACK)	(const void * p_txPacket, uint8_t * p_txCount, protocolg_reqid_t activeReq);

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
	PROTOCOL_STATE_DATAGRAM_CONTROL,
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

/*
	ProtocolG_ReqExt_T
 */
typedef struct
{
	//	uint8_t Options[1]; //buffer;
	//	uint32_t StatusCode;
	//	uint32_t EnableDatagramMode;
	//	volatile ProtocolG_ReqEntry_T * p_ReqActive;
	//	size_t TxLength;
	//	repeat
	//	wait for ack
}
ProtocolG_Control_T; // control interface registers

typedef const struct
{
	const volatile uint32_t * const P_TIMER;
	const uint8_t PACKET_BUFFER_LENGTH; // must be greater than RX_LENGTH_MAX

	volatile uint8_t * const P_RX_PACKET_BUFFER;
	volatile uint8_t * const P_TX_PACKET_BUFFER;
	// user map to app data interface
//	volatile void * const P_WRITE_REGS;
//	const volatile void * const P_READ_REGS;
	volatile void * const P_INTERFACE;

	Flash_T * const P_FLASH;
	Datagram_T * const P_DATAGRAM;

	// Can be set after init. 0 to set later
	const ProtocolG_Specs_T * const P_INIT_SPECS;
	void * const P_INIT_DATA_LINK;
	const ProtocolG_DataLinkType_T INIT_DATA_LINK_TYPE;
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

	Datagram_T Datagram;
//	ProtocolG_Control_T Control;

	//proc variables
	volatile ProtocolG_State_T State;
	volatile uint32_t TimeStart;
	volatile size_t RxIndex;
	volatile size_t TxLength;
	volatile uint8_t NackCount;

	volatile bool RepeatFlag; /* for repeat after ack */

	//todo concurrent req wait process, non unique acks procs first in buffer
	volatile ProtocolG_ReqEntry_T * p_ReqActive;
}
ProtocolG_T;

extern void ProtocolG_Init(ProtocolG_T * p_protocol);
extern void ProtocolG_SetSpecs(ProtocolG_T * p_protocol, const ProtocolG_Specs_T * p_specs);
extern void ProtocolG_SetDataLink(ProtocolG_T * p_protocol, void * p_dataLink, ProtocolG_DataLinkType_T dataLinkType);
extern ProtocolG_Status_T ProtocolG_Slave_Proc(ProtocolG_T * p_protocol);

#endif
