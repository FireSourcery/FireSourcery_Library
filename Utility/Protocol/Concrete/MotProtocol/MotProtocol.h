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
	@file 	MotProtocol.h
	@author FireSoucery
	@brief 	MotProtocol Packets and Interface Defs
	@version V0
*/
/******************************************************************************/
#ifndef MOTPROTOCOL_H
#define MOTPROTOCOL_H

#include "MotProtocolVarId.h"

#include "Utility/Protocol/Protocol.h"

#include <stdint.h>
#include <stdbool.h>

#define MOTPROTOCOL_VERSION_OPT 	(0U)
#define MOTPROTOCOL_VERSION_MAJOR 	(0U)
#define MOTPROTOCOL_VERSION_MINOR 	(0U)
#define MOTPROTOCOL_VERSION_BUGFIX 	(1U)

#define MOTPROTOCOL_PACKET_HEADER_LENGTH	(6U)
#define MOTPROTOCOL_PACKET_PAYLOAD_MAX		(32U)
#define MOTPROTOCOL_PACKET_LENGTH_MIN 		(2U)
#define MOTPROTOCOL_PACKET_LENGTH_MAX 		(MOTPROTOCOL_PACKET_PAYLOAD_MAX + MOTPROTOCOL_PACKET_HEADER_LENGTH)
#define MOTPROTOCOL_BAUD_RATE_DEFAULT		(19200U)
#define MOTPROTOCOL_TIMEOUT_MS				(2000U) 	/* Timeout packet */
#define MOTPROTOCOL_START_BYTE				(0xA5U)

/*
	Packet and Correspondence type. Per unique packet structure, parsing/processing pattern
	Limited 8-bit value allocated. May directly include cmd, or extended packet type
*/
typedef enum
{
	/* 2 Byte Packet */
	MOTPROTOCOL_STOP_MOTOR = 0x00,	 /* if first char after stop is 0x00 */
	MOTPROTOCOL_PING = 0x11,
	MOTPROTOCOL_SYNC_ACK = 0x12U,
	MOTPROTOCOL_SYNC_NACK = 0x13U,
	MOTPROTOCOL_SYNC_ABORT = 0x14U,

	/* Extended Types. Additional Cmd Id */
	MOTPROTOCOL_CMD_MONITOR_TYPE = 0xA1,
	MOTPROTOCOL_CMD_CONTROL_TYPE = 0xA2,

	/* Special Def */
	/* 1 Header byte Cmd Id */
	// MotProtocol_CMD_MONITOR_SPEED_RPM = 0xB1,	/* RPM */
	// MotProtocol_CMD_MONITOR_SPEED_RAW = 0xB2,	/* RPM */
	// MotProtocol_CMD_CONTROL_THROTTLE = 0xC1,
	// MotProtocol_CMD_CONTROL_BRAKE = 0xC2,
	// MotProtocol_CMD_CONTROL_DIRECTION = 0xC3, 	/* Include Neutral */
	// MotProtocol_CMD_CONTROL_FLOAT = 0xC4, 		/* Stop */

	MOTPROTOCOL_CMD_REBOOT = 0xC1U,
	MOTPROTOCOL_CMD_CALL = 0xC2U,

	/* General Data */
	MOTPROTOCOL_CMD_READ_IMMEDIATE = 0xD1U, /* Read Single Var */
	MOTPROTOCOL_CMD_WRITE_IMMEDIATE = 0xD2U, /* Write Single Var */
	// MotProtocol_CMD_READ_VARS = 0xD3U, /* Read 4 */
	// MotProtocol_CMD_WRITE_VARS = 0xD4U, /* Write 4 */
	MOTPROTOCOL_CMD_READ_VAR16 = 0xD3U, /* Up to 16 Ids, for 16 uint16_t values, or 8 uint32_t values */
	MOTPROTOCOL_CMD_WRITE_VAR16 = 0xD4U,

	MOTPROTOCOL_CMD_READ_MEMORY = 0xD5U, /* Read Address */
	MOTPROTOCOL_CMD_WRITE_MEMORY = 0xD6U, /* Write Address */

	MOTPROTOCOL_CMD_DATA_MODE_READ = 0xDAU, /* Stateful Read */
	MOTPROTOCOL_CMD_DATA_MODE_WRITE = 0xDBU, /* Stateful Write */
	MOTPROTOCOL_DATA_MODE_TYPE = 0xDD,

	MOTPROTOCOL_CMD_SAVE_NVM = 0xDFU,

	MOTPROTOCOL_EXT_CMD = 0xE1U,	/* Extended Header Modes */
	MOTPROTOCOL_EXT_RSVR2 = 0xE2U,
	MOTPROTOCOL_ID_RESERVED_255 = 0xFFU,
}
MotProtocol_HeaderId_T;

typedef enum
{
	MOTPROTOCOL_STATUS_OK = 0x00,
 	MOTPROTOCOL_STATUS_RESERVED = 0xFF,
}
MotProtocol_Status_T;

typedef struct
{
	uint8_t Start;
	uint8_t SyncId; /* MotProtocol_HeaderId_T */
}
MotProtocol_SyncPacket_T;

typedef struct
{
	uint8_t Start;
	uint8_t TypeId; /* MotProtocol_HeaderId_T - Cmd / Descriptor of packet contents */
	uint8_t Length; /* Payload Length */
	uint8_t Status; /* Optional Status */
	uint16_t Crc;
}
MotProtocol_Header_T;

typedef union
{
	uint16_t U16s[16U];
	uint32_t U32s[8U];
	uint8_t U8s[32U];
	int16_t S16s[16U];
	int32_t S32s[8U];
	int8_t S8s[32U];
}
MotProtocol_Registers_T;

typedef union
{
	struct
	{
		MotProtocol_Header_T Header;
		union
		{
			uint8_t Payload[MOTPROTOCOL_PACKET_PAYLOAD_MAX];
			MotProtocol_Registers_T Registers;
		};
	};
	uint8_t Bytes[MOTPROTOCOL_PACKET_LENGTH_MAX];
}
MotProtocol_Packet_T;

/*
	Common Status Response
*/
typedef struct { uint16_t Status; } MotProtocol_RespPayload_Status_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_Status_T Response; } MotProtocol_RespPacket_Status_T;

/******************************************************************************/
/*!
	General Cmds
*/
/******************************************************************************/

/******************************************************************************/
/*!	Read Immediate by Id */
/******************************************************************************/
typedef struct { uint16_t VarId; } 																	MotProtocol_ReqPayload_ReadImmediate_T;
typedef struct { uint32_t Value; uint16_t Status; } 												MotProtocol_RespPayload_ReadImmediate_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_ReqPayload_ReadImmediate_T CmdRead; } 	MotProtocol_ReqPacket_ReadImmediate_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_ReadImmediate_T ReadVar; } 	MotProtocol_RespPacket_ReadImmediate_T;

/******************************************************************************/
/*!	Write Immediate Var by Id */
/******************************************************************************/
typedef struct { uint16_t VarId; uint32_t Value; } 													MotProtocol_ReqPayload_WriteImmediate_T;
typedef MotProtocol_RespPayload_Status_T 															MotProtocol_RespPayload_WriteImmediate_T;	/* one 16-bit optional status reponse */
typedef struct { MotProtocol_Header_T Header; MotProtocol_ReqPayload_WriteImmediate_T CmdWrite; } 	MotProtocol_ReqPacket_WriteImmediate_T;
typedef MotProtocol_RespPacket_Status_T 															MotProtocol_RespPacket_WriteImmediate_T;

/******************************************************************************/
/*!
	Read Multiple Vars
	MotProtocolVarId_T is type uint16_t

	Only 8 Ids should be used for uint32_t read
	Id 0xFFFF for null
*/
/******************************************************************************/
// typedef struct { uint8_t VarSize; uint8_t VarCount; MotProtocolVarId_T VarIds[15U]; } 				MotProtocol_Payload_ReadVars_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ReadVars_T CmdReadVars; } 			MotProtocol_Packet_ReadVars_T;
// typedef struct { union { uint32_t Value32[7U]; uint16_t Value16[15U];  }; } 							MotProtocol_Payload_ReadVars_Response_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ReadVars_Response_T Response; } 	MotProtocol_Packet_ReadVars_Response_T;

typedef struct { uint16_t VarIds[16U]; } 															MotProtocol_RespPayload_ReadVar16_T;
typedef struct { union { uint16_t Value16[16U]; uint32_t Value32[8U]; }; } 							MotProtocol_RespPayload_ReadVar16_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_ReadVar16_T CmdReadVar16; } 	MotProtocol_RespPacket_ReadVar16_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_ReadVar16_T Vars; } 			MotProtocol_RespPacket_ReadVar16_T;

/******************************************************************************/
/*!
	Write Multiple Vars
	MotProtocolVarId_T is type uint16_t

	VarCount range [0:8]
*/
/******************************************************************************/
// typedef struct { uint8_t VarSize; uint8_t VarCount; MotProtocolVarId_T VarIds[7U]; } 				MotProtocol_Payload_ReadVars_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ReadVars_T CmdReadVars; } 			MotProtocol_Packet_ReadVars_T;
// typedef struct { union { uint32_t Value32[8U]; uint16_t Value16[16U]; uint8_t Value8[32U]; }; } 		MotProtocol_Payload_ReadVars_Response_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ReadVars_Response_T Response; } 	MotProtocol_Packet_ReadVars_Response_T;

typedef struct { uint16_t VarIds[8U]; union { uint16_t Value16[8U]; uint32_t Value32[4U]; }; } 		MotProtocol_ReqPayload_WriteVar16_T;
typedef struct { uint16_t MainStatus; uint16_t Status[8U]; } 										MotProtocol_RespPayload_WriteVar16_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_ReqPayload_WriteVar16_T CmdWriteVar16; } 	MotProtocol_ReqPacket_WriteVar16_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_WriteVar16_T Status; } 		MotProtocol_RespPacket_WriteVar16_T;

/******************************************************************************/
/*!
	Special Cmds
*/
/******************************************************************************/

/******************************************************************************/
/*!	Control */
/******************************************************************************/
/*
	Default units in the client motor controller native format
*/
typedef enum
{
	MOTPROTOCOL_CONTROL_STOP = 0x00U,
	MOTPROTOCOL_CONTROL_THROTTLE = 0x01U,	/* 0:65536 */
	MOTPROTOCOL_CONTROL_BRAKE = 0x02U,		/* 0:65536 */

	MOTPROTOCOL_CONTROL_VOLTAGE = 0x03U, 	/* +/-32768 */
	MOTPROTOCOL_CONTROL_TORQUE = 0x04U,	/* +/-32768 */
	MOTPROTOCOL_CONTROL_SPEED = 0x05U,		/* +/-32768 */
	MOTPROTOCOL_CONTROL_ANGLE = 0x06U,		/*   */

	MOTPROTOCOL_CONTROL_DIRECTION = 0x10U,

	// MOTPROTOCOL_CONTROL_VOLTAGE_VPWM = 0x83U, /* DutyCycle 65536 */
	// MOTPROTOCOL_CONTROL_VOLTAGE_VOLTS = 0x83U,
	// MOTPROTOCOL_CONTROL_TORQUE_AMPS = 0x84U,
	MOTPROTOCOL_CONTROL_SPEED_RPM = 0x85U,
	// MOTPROTOCOL_CONTROL_ANGLE = 0x85U,

	MOTPROTOCOL_CONTROL_BATCH_1 = 0x71U,
	MOTPROTOCOL_CONTROL_MODE_RESERVED = 0xFFU,
}
MotProtocol_ControlId_T;

/*
	All Control Type Packets default 16-bit variables
	Speed, Torque, Voltage is signed. Throttle, Brake is unsigned

	Up to 15 parameters, determined by MotProtocol_ControlId_T
*/
typedef struct { uint8_t ControlId; uint8_t RsvrExt; union { int16_t ValueS16s[15U]; uint16_t ValueU16s[15U]; }; } 			MotProtocol_ReqPayload_Control_T;
typedef struct { uint16_t MainStatus; uint16_t OptStatus[15U]; } 															MotProtocol_RespPayload_Control_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_ReqPayload_Control_T CmdControl; } 								MotProtocol_ReqPacket_Control_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_Control_T Status; } 									MotProtocol_RespPacket_Control_T;

// typedef struct { MotProtocol_ControlId_T Id; } MotProtocol_Payload_ControlType0_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ControlType0_T Control; } MotProtocol_Packet_ControlType0_T;
// typedef MotProtocol_RespPayload_Status_T 	MotProtocol_Payload_ControlType0_Response_T;
// typedef MotProtocol_RespPacket_Status_T 	MotProtocol_Packet_ControlType0_Response_T;

// /* Control Type 1 */
// typedef struct { MotProtocol_ControlId_T Id; uint8_t IdExt; union { int16_t Signed; uint16_t Unsigned; }; } MotProtocol_Payload_ControlType1_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ControlType1_T Control; } MotProtocol_Packet_ControlType1_T;
// typedef MotProtocol_RespPayload_Status_T 	MotProtocol_Payload_ControlType1_Response_T;
// typedef MotProtocol_RespPacket_Status_T 	MotProtocol_Packet_ControlType1_Response_T;

// typedef struct { uint16_t Value; } MotProtocol_Payload_ControlThrottle_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ControlThrottle_T Throttle; } MotProtocol_Packet_ControlThrottle_T;
// typedef MotProtocol_RespPayload_Status_T MotProtocol_Payload_ControlThrottle_Response_T;
// typedef MotProtocol_RespPacket_Status_T MotProtocol_Packet_ControlThrottle_Response_T;

// typedef struct { uint16_t Value; } MotProtocol_Payload_ControlBrake_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_ControlBrake_T Brake; } MotProtocol_Packet_ControlBrake_T;
// typedef MotProtocol_RespPayload_Status_T MotProtocol_Payload_ControlBrake_Response_T;
// typedef MotProtocol_RespPacket_Status_T MotProtocol_Packet_ControlBrake_Response_T;
typedef struct { uint8_t ControlId; uint8_t RsvrExt; uint16_t ThrottleValue; } 		MotProtocol_ReqPayload_Control_Throttle_T;
typedef struct { uint16_t Status; } 												MotProtocol_RespPayload_Control_Throttle_T;
typedef struct { uint8_t ControlId; uint8_t RsvrExt; uint16_t BrakeValue; } 		MotProtocol_ReqPayload_Control_Brake_T;
typedef struct { uint16_t Status; } 												MotProtocol_RespPayload_Control_Brake_T;

/******************************************************************************/
/*!	Monitor */
/******************************************************************************/
typedef enum
{
	MOTPROTOCOL_MONITOR_SPEED = 0x01U,		/* Speed in Q1.15 */
	MOTPROTOCOL_MONITOR_I_PHASES = 0x02U,	/* Iabc in Q1.15 */
	MOTPROTOCOL_MONITOR_I_FOC = 0x03U,		/* Idq, Ialphabeta in Q1.15 */
	MOTPROTOCOL_MONITOR_V_PHASES = 0x04U,
	MOTPROTOCOL_MONITOR_ANGLES = 0x05U,
	MOTPROTOCOL_MONITOR_STATUS_FLAGS = 0x0FU,

	MOTPROTOCOL_MONITOR_ANALOG_USER = 0x10U,
	MOTPROTOCOL_MONITOR_HALL = 0x11U,
	MOTPROTOCOL_MONITOR_SIN_COS = 0x12U,
	MOTPROTOCOL_MONITOR_ENCODER = 0x13U,

	MOTPROTOCOL_MONITOR_V_SENSORS = 0x21U, 		/* VSupply, ~5V, ~12V */
	MOTPROTOCOL_MONITOR_HEAT = 0x22U,			/* In ADCU, Lower is higher */

	MOTPROTOCOL_MONITOR_METERS = 0x31U, /* Speed, Angles */

	MOTPROTOCOL_MONITOR_SPEED_RPM = 0x81U,	/* Speed in RPM */

	MOTPROTOCOL_MONITOR_ADC_BATCH_MSB = 0xE1U,

	// MotProtocol_MONITOR_ADC_MOTOR_COMMON,
	// MotProtocol_MONITOR_ADC_MOTOR_0,
	// MotProtocol_MONITOR_ADC_MOTOR_1,
	// MotProtocol_MONITOR_V_SENSORS_MV = 0x00U,
	// MotProtocol_MONITOR_HEAT_DEG_C = 0x00U,

	MOTPROTOCOL_MONITOR_RESERVED = 0xFFU,
}
MotProtocol_MonitorId_T;

typedef struct { uint8_t MonitorId; } 															MotProtocol_ReqPayload_Monitor_T;
typedef MotProtocol_Registers_T 																MotProtocol_RespPayload_Monitor_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_ReqPayload_Monitor_T CmdMonitor; } 	MotProtocol_ReqPacket_Monitor_T;
typedef struct { MotProtocol_Header_T Header; MotProtocol_RespPayload_Monitor_T Values; } 		MotProtocol_RespPacket_Monitor_T;

// typedef struct { MotProtocol_MonitorId_T Id; } 														MotProtocol_Payload_MonitorPhases_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_MonitorPhases_T Phases; } 			MotProtocol_Packet_MonitorPhases_T;
// typedef struct { MotProtocol_MonitorId_T Id; } 														MotProtocol_Payload_Monitor1_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_Monitor1_T Monitor; } 				MotProtocol_Packet_Monitor1_T;
// typedef struct { union { int16_t Signed; uint16_t Unsigned; }; } 									MotProtocol_Payload_Monitor1_Response_T;
// typedef struct { MotProtocol_Header_T Header; MotProtocol_Payload_Monitor1_Response_T Response; }  	MotProtocol_Packet_Monitor1_Response_T;

typedef struct{	int16_t PhaseA; int16_t PhaseB; int16_t PhaseC; } 		MotProtocol_RespPayload_Monitor_IPhases_T;
typedef struct{	int32_t Speed; } 										MotProtocol_RespPayload_Monitor_Speed_T;

typedef struct
{
 	union
	{
		MotProtocol_Registers_T Registers;

		MotProtocol_ReqPayload_Control_T ReqControl;
		MotProtocol_ReqPayload_Control_Throttle_T ReqThrottle;
		MotProtocol_ReqPayload_Monitor_T ReqMonitor;
		MotProtocol_ReqPayload_ReadImmediate_T ReqReadImmediate;

		MotProtocol_RespPayload_ReadImmediate_T RespReadImmediate;
		MotProtocol_RespPayload_Monitor_T RespMonitor;
		MotProtocol_RespPayload_Monitor_Speed_T Speed;
		MotProtocol_RespPayload_Monitor_IPhases_T IPhases;
	};
}
MotProtocol_Interface_T;


/******************************************************************************/
/*!
	Ext Cmds
*/
/******************************************************************************/
// typedef struct
// {
// 	uint8_t ExtId;	/* Extended Id */
// 	uint8_t ExtLength;
// 	uint8_t Opt0;
// 	uint8_t Opt1;
// }
// MotProtocol_Payload_ExtHeader_T;

// typedef enum
// {
// 	MOTPROTOCOL_EXT_CMD_1 = 0x01U,
// }
// MotProtocol_ExtCmdId_T;

/******************************************************************************/
/*!	Extern */
/******************************************************************************/

/******************************************************************************/
/*!	Common */
/******************************************************************************/
extern uint8_t _MotProtocol_SyncPacket_Build(MotProtocol_SyncPacket_T * p_txPacket, MotProtocol_HeaderId_T syncId);
extern uint8_t MotProtocol_SyncPacket_Build(MotProtocol_SyncPacket_T * p_txPacket, MotProtocol_HeaderId_T syncId);
// extern uint8_t MotProtocol_GetControlRespLength(MotProtocol_ControlId_T controlId);
// extern uint8_t MotProtocol_GetMonitorRespLength(MotProtocol_MonitorId_T id);
// extern uint8_t MotProtocol_GetRespLength(const MotProtocol_Interface_T * p_interface, MotProtocol_HeaderId_T id);

/******************************************************************************/
/*!	Cmdr side */
/******************************************************************************/
// extern uint8_t MotProtocol_ReqPacket_Control_BuildThrottle(MotProtocol_ReqPacket_Control_T * p_reqPacket, uint16_t throttleValue);
// extern uint8_t MotProtocol_ReqPacket_Control_BuildBrake(MotProtocol_ReqPacket_Control_T * p_reqPacket, uint16_t brakeValue);
extern uint8_t MotProtocol_ReqPacket_Control_Build(MotProtocol_ReqPacket_Control_T * p_reqPacket, const MotProtocol_Interface_T * p_interface);
// extern uint8_t MotProtocol_ReqPacket_Monitor_BuildSpeed(MotProtocol_ReqPacket_Monitor_T * p_reqPacket);
extern uint8_t MotProtocol_ReqPacket_Monitor_Build(MotProtocol_ReqPacket_Monitor_T * p_reqPacket, const MotProtocol_Interface_T * p_interface);
// extern void MotProtocol_RespPacket_ParseMonitorSpeed(int32_t * p_speed_Frac16, const MotProtocol_RespPacket_Monitor_T * p_rxPacket);
extern bool MotProtocol_RespPacket_Parse(MotProtocol_Interface_T * p_interface, const MotProtocol_Packet_T * p_rxPacket);

/******************************************************************************/
/*!	Ctrlr side */
/******************************************************************************/
// extern uint8_t MotProtocol_RespPacket_Monitor_Build(MotProtocol_RespPacket_Monitor_T * p_respPacket, const MotProtocol_Interface_T * p_interface, MotProtocol_MonitorId_T id);

#endif
