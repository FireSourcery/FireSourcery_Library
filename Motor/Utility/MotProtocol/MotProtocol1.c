//#include "Protocol/ProtocolG/ProtocolG.h"
//
//#include <stdint.h>
//#include <stdbool.h>
//

//typedef struct
//{
//	/*
//	 * MISRA violation, use of union. Rationale: provide named fields to array
//	 */
//	union
//	{
//		struct
//		{
//			uint8_t Start;
//			uint8_t Cmd;
//			uint8_t DataLength;
//			uint8_t Data[20];
//		};
//
//		uint8_t Bytes[22];
//	};
//} MotorProtocol1_Packet1_T;

//uint32_t MotorProtocol1_FunctionWriteX(uint8_t * p_txPacket, volatile const MotorProtocol1_Packet1_T * p_rxPacket, MotorInterface_Input_T * p_motorInput)
//{
//	p_motorInput->InputSwitchBrake = p_rxPacket->Data[1];
//
//	p_txPacket[0U] = MP1_ACK;
//}

//
//const ProtocolG_ResponseEntry_T MOTOR_PROTOCOL_FUNCTION_TABLE[] =
//{
//	{0x11U, MotorProtocol_Function1, PROTOCOLG_RESPONSE_READ},
//
//};
//
//ProtocolG_Specs_T MOTOR_PROTOCOL_1_SPECS =
//{
//	.TIME_OUT = 5000U,
//	.BAUD_RATE_DEFAULT = 19200U,
//	.PACKET_LENGTH_MAX = 20U,
//
//	.P_RESPONSE_TABLE = MOTOR_PROTOCOL_FUNCTION_TABLE,
//	.RESPONSE_TABLE_LENGTH = sizeof(MOTOR_PROTOCOL_FUNCTION_TABLE)/sizeof(ProtocolG_ResponseEntry_T),
//
//	.CHECK_COMPLETE = 00,
//	.CHECK_CORRECT	= 00,
//	.GET_RESPONSE_ID = 00,
//
//	.START_ID = 0x00U,
//	.END_ID = '\n',
//	.FUNCTION_INDEX = 0U,
//};
