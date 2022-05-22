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
	@file 	MotPacket.h
	@author FireSoucery
	@brief 	MotPacket Packets and Interface Defs
	@version V0
*/
/******************************************************************************/
#ifndef MOT_PACKET_H
#define MOT_PACKET_H

#include "MotVarId.h"
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
#define MOTPROTOCOL_START_BYTE				(0xA5U)

/*
	Packet and Correspondence type. Per unique packet structure, parsing/processing pattern
	Limited 8-bit value allocated. May directly include cmd, or extended packet type
*/
//todo rename MOT_PACKET_
typedef enum MotPacket_HeaderId_Tag
{
	/* 2 Byte Packet */
	MOTPROTOCOL_STOP_MOTORS = 0x00,	 /* if first char after stop is 0x00 */
	MOTPROTOCOL_PING = 0x11,
	MOTPROTOCOL_SYNC_ACK = 0x12U,
	MOTPROTOCOL_SYNC_NACK = 0x13U,
	MOTPROTOCOL_SYNC_ABORT = 0x14U,

	/* Extended Types. Additional Cmd Id */
	MOTPROTOCOL_CMD_MONITOR_TYPE = 0xA1,
	MOTPROTOCOL_CMD_CONTROL_TYPE = 0xA2,

	/* Special Def */
	/* 1 Header byte Cmd Id */
	// MotPacket_CMD_MONITOR_SPEED_RPM = 0xB1,	/* RPM */
	// MotPacket_CMD_MONITOR_SPEED_RAW = 0xB2,	/* RPM */
	// MotPacket_CMD_CONTROL_THROTTLE = 0xC1,
	// MotPacket_CMD_CONTROL_BRAKE = 0xC2,
	// MotPacket_CMD_CONTROL_DIRECTION = 0xC3, 	/* Include Neutral */
	// MotPacket_CMD_CONTROL_FLOAT = 0xC4, 		/* Stop */

	MOTPROTOCOL_CMD_REBOOT = 0xC1U,
	MOTPROTOCOL_CMD_CALL = 0xC2U,

	/* General Data */
	MOTPROTOCOL_CMD_READ_IMMEDIATE = 0xD1U, /* Read Single Var */
	MOTPROTOCOL_CMD_WRITE_IMMEDIATE = 0xD2U, /* Write Single Var */
	// MotPacket_CMD_READ_VARS = 0xD3U, /* Read 4 */
	// MotPacket_CMD_WRITE_VARS = 0xD4U, /* Write 4 */
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
MotPacket_HeaderId_T;

typedef enum MotPacket_Status_Tag
{
	MOTPROTOCOL_STATUS_OK = 0x00,
 	MOTPROTOCOL_STATUS_RESERVED = 0xFF,
}
MotPacket_HeaderStatus_T;

typedef struct MotPacket_Sync_Tag
{
	uint8_t Start;
	uint8_t SyncId; /* MotPacket_HeaderId_T */
}
MotPacket_Sync_T;

typedef struct MotPacket_Header_Tag
{
	uint8_t Start;
	uint8_t TypeId; /* MotPacket_HeaderId_T - Cmd / Descriptor of packet contents */
	uint8_t Length; /* Payload Length */
	uint8_t Status; /* Optional Status */
	uint16_t Crc;
}
MotPacket_Header_T;

typedef union MotPacket_Registers_Tag
{
	uint16_t U16s[16U];
	uint32_t U32s[8U];
	uint8_t U8s[32U];
	int16_t S16s[16U];
	int32_t S32s[8U];
	int8_t S8s[32U];
}
MotPacket_Registers_T;

typedef union MotPacket_Packet_Tag
{
	struct
	{
		MotPacket_Header_T Header;
		union
		{
			uint8_t Payload[MOTPROTOCOL_PACKET_PAYLOAD_MAX];
			MotPacket_Registers_T Registers;
		};
	};
	uint8_t Bytes[MOTPROTOCOL_PACKET_LENGTH_MAX];
}
MotPacket_T;


//todo rename type order

/*
	Common Status Response - one 16-bit optional status reponse
*/
typedef enum MotPacket_StatusResp_Id_Tag
{
	MOTPROTOCOL_STATUS_RESP_OK = 0x00,
 	MOTPROTOCOL_STATUS_RESP_RESERVED = 0xFFFF,
}
MotPacket_StatusResp_Id_T;

typedef struct MotPacket_StatusResp_Payload_Tag { uint16_t Id; } 												MotPacket_StatusResp_Payload_T;
typedef struct MotPacket_StatusResp_Tag { MotPacket_Header_T Header; MotPacket_StatusResp_Payload_T Status; } 	MotPacket_StatusResp_T;

/******************************************************************************/
/*!
	General Cmds
*/
/******************************************************************************/

/******************************************************************************/
/*!	Ping */
/******************************************************************************/
typedef MotPacket_Sync_T 																					MotPacket_PingReq_T;
typedef struct MotPacket_PingResp_Payload_Tag { uint8_t Bytes[4U]; } 										MotPacket_PingResp_Payload_T;
typedef struct MotPacket_PingResp_Tag { MotPacket_Header_T Header; MotPacket_PingResp_Payload_T Version; } 	MotPacket_PingResp_T;

/******************************************************************************/
/*!	Stop */
/******************************************************************************/
typedef MotPacket_Sync_T 				MotPacket_StopReq_T;
typedef MotPacket_StatusResp_Payload_T 	MotPacket_StopResp_Payload_T;
typedef MotPacket_StatusResp_T 			MotPacket_StopResp_T;

/******************************************************************************/
/*!	Read Immediate by Id */
/******************************************************************************/
typedef struct MotPacket_ReqPayload_ReadImmediate_Tag { uint16_t MotVarId; } 													MotPacket_ReqPayload_ReadImmediate_T;
typedef struct MotPacket_RespPayload_ReadImmediate_Tag { uint32_t Value; uint16_t Status; } 									MotPacket_RespPayload_ReadImmediate_T;
typedef struct MotPacket_Req_ReadImmediate_Tag { MotPacket_Header_T Header; MotPacket_ReqPayload_ReadImmediate_T CmdRead; } 	MotPacket_Req_ReadImmediate_T;
typedef struct MotPacket_Resp_ReadImmediate_Tag { MotPacket_Header_T Header; MotPacket_RespPayload_ReadImmediate_T ReadVar; } 	MotPacket_Resp_ReadImmediate_T;

// typedef struct MotPacket_ReadImmediateReq_Payload_Tag { uint16_t MotVarId; } 													MotPacket_ReadImmediateReq_Payload_T;
// typedef struct MotPacket_ReadImmediateReq_Tag { MotPacket_Header_T Header; MotPacket_ReadImmediateReq_Payload_T CmdRead; } 		MotPacket_ReadImmediateReq_T;
// typedef struct MotPacket_ReadImmediateResp_Payload_Tag { uint32_t Value; uint16_t Status; } 									MotPacket_ReadImmediateResp_Payload_T;
// typedef struct MotPacket_ReadImmediateResp_Tag { MotPacket_Header_T Header; MotPacket_ReadImmediateResp_Payload_T ReadVar; } 	MotPacket_ReadImmediateResp_T;


/******************************************************************************/
/*!	Write Immediate Var by Id */
/******************************************************************************/
typedef struct MotPacket_ReqPayload_WriteImmediate_Tag { uint16_t MotVarId; uint32_t Value; } 										MotPacket_ReqPayload_WriteImmediate_T;
typedef MotPacket_StatusResp_Payload_T 																							MotPacket_RespPayload_WriteImmediate_T;
typedef struct MotPacket_Req_WriteImmediate_Tag { MotPacket_Header_T Header; MotPacket_ReqPayload_WriteImmediate_T CmdWrite; } 	MotPacket_Req_WriteImmediate_T;
typedef MotPacket_StatusResp_T 																								MotPacket_Resp_WriteImmediate_T;

/******************************************************************************/
/*!
	Read Multiple Vars
	MotVarId_T is type uint16_t

	Only 8 Ids should be used for uint32_t read
	Id 0xFFFF for null
*/
/******************************************************************************/
typedef struct MotPacket_ReqPayload_ReadVar16_Tag { uint16_t MotVarIds[16U]; } 															MotPacket_ReqPayload_ReadVar16_T;
typedef struct MotPacket_RespPayload_ReadVar16_Tag { union { uint16_t Value16[16U]; uint32_t Value32[8U]; }; } 							MotPacket_RespPayload_ReadVar16_T;
typedef struct MotPacket_Req_ReadVar16_Tag { MotPacket_Header_T Header; MotPacket_RespPayload_ReadVar16_T CmdReadVar16; } 	MotPacket_Req_ReadVar16_T;
typedef struct MotPacket_Resp_ReadVar16_Tag { MotPacket_Header_T Header; MotPacket_RespPayload_ReadVar16_T Vars; } 			MotPacket_Resp_ReadVar16_T;

/******************************************************************************/
/*!
	Write Multiple Vars
	MotVarId_T is type uint16_t

	VarCount range [0:8]
*/
/******************************************************************************/
typedef struct MotPacket_ReqPayload_WriteVar16_Tag { uint16_t MotVarIds[8U]; union { uint16_t Value16[8U]; uint32_t Value32[4U]; }; } 		MotPacket_ReqPayload_WriteVar16_T;
typedef struct MotPacket_RespPayload_WriteVar16_Tag { uint16_t MainStatus; uint16_t Status[8U]; } 										MotPacket_RespPayload_WriteVar16_T;
typedef struct MotPacket_Req_WriteVar16_Tag { MotPacket_Header_T Header; MotPacket_ReqPayload_WriteVar16_T CmdWriteVar16; } 	MotPacket_Req_WriteVar16_T;
typedef struct MotPacket_Resp_WriteVar16_Tag { MotPacket_Header_T Header; MotPacket_RespPayload_WriteVar16_T Status; } 		MotPacket_Resp_WriteVar16_T;

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
typedef enum MotPacket_ControlId_Tag
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
MotPacket_ControlId_T;

/*
	All Control Type Packets default 16-bit variables
	Speed, Torque, Voltage is signed. Throttle, Brake is unsigned

	Up to 15 parameters, determined by MotPacket_ControlId_T
*/
typedef struct MotPacket_ReqPayload_Control_Tag { uint8_t ControlId; uint8_t RsvrExt; union { int16_t ValueS16s[15U]; uint16_t ValueU16s[15U]; }; } 		MotPacket_ReqPayload_Control_T;
typedef struct MotPacket_RespPayload_Control_Tag { uint16_t MainStatus; uint16_t OptStatus[15U]; } 														MotPacket_RespPayload_Control_T;
typedef struct MotPacket_Req_Control_Tag { MotPacket_Header_T Header; MotPacket_ReqPayload_Control_T CmdControl; } 								MotPacket_Req_Control_T;
typedef struct MotPacket_Resp_Control_Tag { MotPacket_Header_T Header; MotPacket_RespPayload_Control_T Status; } 								MotPacket_Resp_Control_T;

typedef struct MotPacket_ReqPayload_Control_Throttle_Tag { uint8_t ControlId; uint8_t RsvrExt; uint16_t ThrottleValue; } 	MotPacket_ReqPayload_Control_Throttle_T;
typedef MotPacket_StatusResp_Payload_T																					MotPacket_RespPayload_Control_Throttle_T;
typedef struct MotPacket_ReqPayload_Control_Brake_Tag { uint8_t ControlId; uint8_t RsvrExt; uint16_t BrakeValue; } 	MotPacket_ReqPayload_Control_Brake_T;
typedef MotPacket_StatusResp_Payload_T																				MotPacket_RespPayload_Control_Brake_T;

/******************************************************************************/
/*!	Monitor */
/******************************************************************************/
typedef enum MotPacket_MonitorId_Tag
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

	// MotPacket_MONITOR_ADC_MOTOR_COMMON,
	// MotPacket_MONITOR_ADC_MOTOR_0,
	// MotPacket_MONITOR_ADC_MOTOR_1,
	// MotPacket_MONITOR_V_SENSORS_MV = 0x00U,
	// MotPacket_MONITOR_HEAT_DEG_C = 0x00U,

	MOTPROTOCOL_MONITOR_RESERVED = 0xFFU,
}
MotPacket_MonitorId_T;

typedef struct MotPacket_ReqPayload_Monitor_Tag { uint8_t MonitorId; } 												MotPacket_ReqPayload_Monitor_T;
typedef MotPacket_Registers_T 																						MotPacket_RespPayload_Monitor_T;
typedef struct MotPacket_Req_Monitor_Tag { MotPacket_Header_T Header; MotPacket_ReqPayload_Monitor_T CmdMonitor; } 	MotPacket_Req_Monitor_T;
typedef struct MotPacket_Resp_Monitor_Tag { MotPacket_Header_T Header; MotPacket_RespPayload_Monitor_T Values; } 	MotPacket_Resp_Monitor_T;

typedef struct MotPacket_RespPayload_Monitor_IPhases_Tag { int16_t PhaseA; int16_t PhaseB; int16_t PhaseC; } 		MotPacket_RespPayload_Monitor_IPhases_T;
typedef struct MotPacket_RespPayload_Monitor_Speed_Tag { int32_t Speed; } 											MotPacket_RespPayload_Monitor_Speed_T;


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
// MotPacket_Payload_ExtHeader_T;

// typedef enum
// {
// 	MOTPROTOCOL_EXT_CMD_1 = 0x01U,
// }
// MotPacket_ExtCmdId_T;

/******************************************************************************/
/*!	Extern */
/******************************************************************************/

/******************************************************************************/
/*!	Common */
/******************************************************************************/
extern bool MotPacket_CheckChecksum(const MotPacket_T * p_packet);
extern uint8_t _MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId);
extern uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId);
extern uint8_t MotPacket_GetControlRespLength(MotPacket_ControlId_T controlId);
extern uint8_t MotPacket_GetMonitorRespLength(MotPacket_MonitorId_T monitorId);

// extern uint8_t MotPacket_GetRespLength(const MotPacket_Interface_T * p_interface, MotPacket_HeaderId_T headerId);

/******************************************************************************/
/*!	Cmdr side */
/******************************************************************************/
extern uint8_t MotPacket_StopReq_Build(MotPacket_Sync_T * p_reqPacket);
extern uint8_t MotPacket_Req_Control_BuildThrottle(MotPacket_Req_Control_T * p_reqPacket, uint16_t throttleValue);
extern uint8_t MotPacket_Req_Control_BuildBrake(MotPacket_Req_Control_T * p_reqPacket, uint16_t brakeValue);
extern uint8_t MotPacket_Req_Monitor_BuildSpeed(MotPacket_Req_Monitor_T * p_reqPacket);

// extern uint8_t MotPacket_Req_Control_Build(MotPacket_Req_Control_T * p_reqPacket, const MotPacket_Interface_T * p_interface);
// extern uint8_t MotPacket_Req_Monitor_Build(MotPacket_Req_Monitor_T * p_reqPacket, const MotPacket_Interface_T * p_interface);
// extern bool MotPacket_Resp_Parse(MotPacket_Interface_T * p_interface, const MotPacket_T * p_rxPacket);

/******************************************************************************/
/*!	Ctrlr side */
/******************************************************************************/
extern uint8_t MotPacket_PingResp_Build(MotPacket_T * p_respPacket);
extern uint8_t MotPacket_StopResp_Build(MotPacket_Sync_T * p_respPacket);

extern uint8_t MotPacket_Resp_Monitor_Speed_Build(MotPacket_Resp_Monitor_T * p_respPacket, int32_t speed);
extern uint8_t MotPacket_Resp_Monitor_IPhases_Build(MotPacket_Resp_Monitor_T * p_respPacket, int16_t ia, int16_t ib, int16_t ic);
// extern MotPacket_StatusResp_Id_T MotPacket_Req_Monitor_ParseId(MotPacket_MonitorId_T * p_monitorId, const MotPacket_Req_Monitor_T * p_reqPacket);

#endif


// typedef struct MotPacket_Interface_Tag
// {
//  	union
// 	{
// 		MotPacket_Registers_T Registers;

// 		MotPacket_ReqPayload_Control_T ReqControl;
// 		MotPacket_ReqPayload_Control_Throttle_T ReqThrottle;
// 		MotPacket_ReqPayload_Monitor_T ReqMonitor;
// 		MotPacket_ReqPayload_ReadImmediate_T ReqReadImmediate;

// 		MotPacket_RespPayload_ReadImmediate_T RespReadImmediate;
// 		MotPacket_RespPayload_Monitor_T RespMonitor;
// 		MotPacket_RespPayload_Monitor_Speed_T Speed;
// 		MotPacket_RespPayload_Monitor_IPhases_T IPhases;
// 	};


// }
// MotPacket_Interface_T;

// typedef struct MotPacket_Interface_Tag
// {
// 	union
// 	{
// 		MotPacket_ReqPayload_Control_T ReqControl;
// 		MotPacket_ReqPayload_Control_Throttle_T ReqThrottle;
// 		MotPacket_ReqPayload_Monitor_T ReqMonitor;
// 		MotPacket_ReqPayload_ReadImmediate_T ReqReadImmediate;
// 	};
// 	struct
// 	{
// 		MotPacket_RespPayload_ReadImmediate_T RespReadImmediate;
// 		MotPacket_RespPayload_Monitor_T RespMonitor;
// 		MotPacket_RespPayload_Monitor_Speed_T Speed;
// 		MotPacket_RespPayload_Monitor_IPhases_T IPhases;
// 	};
// }
// MotPacket_Interface_T;