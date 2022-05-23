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

#define MOT_PROTOCOL_VERSION_OPT 		(0U)
#define MOT_PROTOCOL_VERSION_MAJOR 		(0U)
#define MOT_PROTOCOL_VERSION_MINOR 		(0U)
#define MOT_PROTOCOL_VERSION_BUGFIX 	(1U)

#define MOT_PACKET_HEADER_LENGTH	(6U)
#define MOT_PACKET_PAYLOAD_MAX		(32U)
#define MOT_PACKET_LENGTH_MIN 		(2U)
#define MOT_PACKET_LENGTH_MAX 		(MOT_PACKET_PAYLOAD_MAX + MOT_PACKET_HEADER_LENGTH)
#define MOT_PACKET_START_BYTE		(0xA5U)

/*
	Packet and Correspondence type. Per unique packet structure, parsing/processing pattern
	Limited 8-bit value allocated. May directly include cmd, or extended packet type
*/
typedef enum MotPacket_HeaderId_Tag
{
	/* 2 Byte Packet */
	MOT_PROTOCOL_STOP_MOTORS = 0x00,	 /* if first char after stop is 0x00 */
	MOT_PROTOCOL_PING = 0x11,
	MOT_PROTOCOL_SYNC_ACK = 0x12U,
	MOT_PROTOCOL_SYNC_NACK = 0x13U,
	MOT_PROTOCOL_SYNC_ABORT = 0x14U,

	/* Extended Types. Additional Cmd Id */
	MOT_PROTOCOL_CMD_MONITOR_TYPE = 0xA1,
	MOT_PROTOCOL_CMD_CONTROL_TYPE = 0xA2,

	MOT_PROTOCOL_CMD_REBOOT = 0xC1U,
	MOT_PROTOCOL_CMD_CALL = 0xC2U,

	/* General Data */
	MOT_PROTOCOL_CMD_READ_IMMEDIATE = 0xD1U, /* Read Single Var */
	MOT_PROTOCOL_CMD_WRITE_IMMEDIATE = 0xD2U, /* Write Single Var */

	MOT_PROTOCOL_CMD_READ_VAR16 = 0xD3U, /* Up to 16 Ids, for 16 uint16_t values, or 8 uint32_t values */
	MOT_PROTOCOL_CMD_WRITE_VAR16 = 0xD4U,

	MOT_PROTOCOL_CMD_READ_MEMORY = 0xD5U, /* Read Address */
	MOT_PROTOCOL_CMD_WRITE_MEMORY = 0xD6U, /* Write Address */

	MOT_PROTOCOL_CMD_DATA_MODE_READ = 0xDAU, /* Stateful Read */
	MOT_PROTOCOL_CMD_DATA_MODE_WRITE = 0xDBU, /* Stateful Write */
	MOT_PROTOCOL_DATA_MODE_TYPE = 0xDD,

	MOT_PROTOCOL_CMD_SAVE_NVM = 0xDFU,

	MOT_PROTOCOL_EXT_CMD = 0xE1U,	/* Extended Header Modes */
	MOT_PROTOCOL_EXT_RSVR2 = 0xE2U,
	MOT_PROTOCOL_ID_RESERVED_255 = 0xFFU,
}
MotPacket_HeaderId_T;

typedef struct MotPacket_Sync_Tag
{
	uint8_t Start;
	uint8_t SyncId; /* MotPacket_HeaderId_T */
}
MotPacket_Sync_T;

typedef enum MotPacket_Status_Tag
{
	MOT_PROTOCOL_HEADER_STATUS_OK = 0x00,
 	MOT_PROTOCOL_HEADER_STATUS_RESERVED = 0xFF,
}
MotPacket_HeaderStatus_T;

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
			uint8_t Payload[MOT_PACKET_PAYLOAD_MAX];
			MotPacket_Registers_T Registers;
		};
	};
	uint8_t Bytes[MOT_PACKET_LENGTH_MAX];
}
MotPacket_T;

/*
	Common Status Response - one 16-bit optional status reponse
*/
typedef enum MotPacket_StatusResp_Id_Tag
{
	MOT_PROTOCOL_STATUS_RESP_OK = 0x00,
 	MOT_PROTOCOL_STATUS_RESP_RESERVED = 0xFFFF,
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
typedef struct MotPacket_ReadImmediateReq_Payload_Tag { uint16_t MotVarId; } 													MotPacket_ReadImmediateReq_Payload_T;
typedef struct MotPacket_ReadImmediateReq_Tag { MotPacket_Header_T Header; MotPacket_ReadImmediateReq_Payload_T CmdRead; } 		MotPacket_ReadImmediateReq_T;
typedef struct MotPacket_ReadImmediateResp_Payload_Tag { uint32_t Value; uint16_t Status; } 									MotPacket_ReadImmediateResp_Payload_T;
typedef struct MotPacket_ReadImmediateResp_Tag { MotPacket_Header_T Header; MotPacket_ReadImmediateResp_Payload_T ReadVar; } 	MotPacket_ReadImmediateResp_T;

/******************************************************************************/
/*!	Write Immediate Var by Id */
/******************************************************************************/
typedef struct MotPacket_WriteImmediateReq_Payload_Tag { uint16_t MotVarId; uint32_t Value; } 									MotPacket_WriteImmediateReq_Payload_T;
typedef struct MotPacket_WriteImmediateReq_Tag { MotPacket_Header_T Header; MotPacket_WriteImmediateReq_Payload_T CmdWrite; } 	MotPacket_WriteImmediateReq_T;
typedef MotPacket_StatusResp_Payload_T 																							MotPacket_WriteImmediateResp_Payload_T;
typedef MotPacket_StatusResp_T 																									MotPacket_WriteImmediateResp_T;

/******************************************************************************/
/*!
	Read Multiple Vars
	MotVarId_T is type uint16_t

	Only 8 Ids should be used for uint32_t read
	Id 0xFFFF for null
*/
/******************************************************************************/
typedef struct MotPacket_ReadVar16Req_Payload_Tag { uint16_t MotVarIds[16U]; } 												MotPacket_ReadVar16Req_Payload_T;
typedef struct MotPacket_ReadVar16Req_Tag { MotPacket_Header_T Header; MotPacket_ReadVar16Req_Payload_T CmdReadVar16; } 	MotPacket_ReadVar16Req_T;
typedef struct MotPacket_ReadVar16Resp_Payload_Tag { union { uint16_t Value16[16U]; uint32_t Value32[8U]; }; } 				MotPacket_ReadVar16Resp_Payload_T;
typedef struct MotPacket_ReadVar16Resp_Tag { MotPacket_Header_T Header; MotPacket_ReadVar16Resp_Payload_T Vars; } 			MotPacket_ReadVar16Resp_T;

/******************************************************************************/
/*!
	Write Multiple Vars
	MotVarId_T is type uint16_t

	VarCount range [0:8]
*/
/******************************************************************************/
typedef struct MotPacket_WriteVar16Req_Payload_Tag { uint16_t MotVarIds[8U]; union { uint16_t Value16[8U]; uint32_t Value32[4U]; }; } 	MotPacket_WriteVar16Req_Payload_T;
typedef struct MotPacket_WriteVar16Req_Tag { MotPacket_Header_T Header; MotPacket_WriteVar16Req_Payload_T CmdWriteVar16; } 				MotPacket_WriteVar16Req_T;
typedef struct MotPacket_WriteVar16Resp_Payload_Tag { uint16_t MainStatus; uint16_t Status[8U]; } 										MotPacket_WriteVar16Resp_Payload_T;
typedef struct MotPacket_WriteVar16Resp_Tag { MotPacket_Header_T Header; MotPacket_WriteVar16Resp_Payload_T Status; } 					MotPacket_WriteVar16Resp_T;

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
	MOT_PROTOCOL_CONTROL_STOP = 0x00U,
	MOT_PROTOCOL_CONTROL_THROTTLE = 0x01U,	/* 0:65536 */
	MOT_PROTOCOL_CONTROL_BRAKE = 0x02U,		/* 0:65536 */

	MOT_PROTOCOL_CONTROL_VOLTAGE = 0x03U, 	/* +/-32768 */
	MOT_PROTOCOL_CONTROL_TORQUE = 0x04U,	/* +/-32768 */
	MOT_PROTOCOL_CONTROL_SPEED = 0x05U,		/* +/-32768 */
	MOT_PROTOCOL_CONTROL_ANGLE = 0x06U,		/*   */

	MOT_PROTOCOL_CONTROL_DIRECTION = 0x10U,

	// MOT_PROTOCOL_CONTROL_VOLTAGE_VPWM = 0x83U, /* DutyCycle 65536 */
	// MOT_PROTOCOL_CONTROL_VOLTAGE_VOLTS = 0x83U,
	// MOT_PROTOCOL_CONTROL_TORQUE_AMPS = 0x84U,
	MOT_PROTOCOL_CONTROL_SPEED_RPM = 0x85U,
	// MOT_PROTOCOL_CONTROL_ANGLE = 0x85U,

	MOT_PROTOCOL_CONTROL_BATCH_1 = 0x71U,
	MOT_PROTOCOL_CONTROL_MODE_RESERVED = 0xFFU,
}
MotPacket_ControlId_T;

/*
	All Control Type Packets default 16-bit variables
	Speed, Torque, Voltage is signed. Throttle, Brake is unsigned

	Up to 15 parameters, determined by MotPacket_ControlId_T
*/
typedef struct MotPacket_ControlReq_Payload_Tag { uint8_t ControlId; uint8_t RsvrExt; union { int16_t ValueS16s[15U]; uint16_t ValueU16s[15U]; }; } 	MotPacket_ControlReq_Payload_T;
typedef struct MotPacket_ControlReq_Tag { MotPacket_Header_T Header; MotPacket_ControlReq_Payload_T CmdControl; } 										MotPacket_ControlReq_T;
typedef struct MotPacket_ControlResp_Payload_Tag { uint16_t MainStatus; uint16_t OptStatus[15U]; } 														MotPacket_ControlResp_Payload_T;
typedef struct MotPacket_ControlResp_Tag { MotPacket_Header_T Header; MotPacket_ControlResp_Payload_T Status; } 										MotPacket_ControlResp_T;

typedef struct MotPacket_ControlReq_Throttle_Payload_Tag { uint8_t ControlId; uint8_t RsvrExt; uint16_t ThrottleValue; } 	MotPacket_ControlReq_Throttle_Payload_T;
typedef MotPacket_StatusResp_Payload_T																						MotPacket_ControlResp_Throttle_Payload_T;
typedef struct MotPacket_ControlReq_Brake_Payload_Tag { uint8_t ControlId; uint8_t RsvrExt; uint16_t BrakeValue; } 			MotPacket_ControlReq_Brake_Payload_T;
typedef MotPacket_StatusResp_Payload_T																						MotPacket_ControlResp_Brake_Payload_T;

/******************************************************************************/
/*!	Monitor */
/******************************************************************************/
typedef enum MotPacket_MonitorId_Tag
{
	MOT_PROTOCOL_MONITOR_SPEED = 0x01U,		/* Speed in Q1.15 */
	MOT_PROTOCOL_MONITOR_I_PHASES = 0x02U,	/* Iabc in Q1.15 */
	MOT_PROTOCOL_MONITOR_I_FOC = 0x03U,		/* Idq, Ialphabeta in Q1.15 */
	MOT_PROTOCOL_MONITOR_V_PHASES = 0x04U,
	MOT_PROTOCOL_MONITOR_ANGLES = 0x05U,
	MOT_PROTOCOL_MONITOR_STATUS_FLAGS = 0x0FU,

	MOT_PROTOCOL_MONITOR_ANALOG_USER = 0x10U,
	MOT_PROTOCOL_MONITOR_HALL = 0x11U,
	MOT_PROTOCOL_MONITOR_SIN_COS = 0x12U,
	MOT_PROTOCOL_MONITOR_ENCODER = 0x13U,

	MOT_PROTOCOL_MONITOR_V_SENSORS = 0x21U, 		/* VSupply, ~5V, ~12V */
	MOT_PROTOCOL_MONITOR_HEAT = 0x22U,			/* In ADCU, Lower is higher */

	MOT_PROTOCOL_MONITOR_METERS = 0x31U, /* Speed, Angles */

	// MOT_PROTOCOL_MONITOR_SPEED_RPM = 0x81U,	/* Speed in RPM */

	MOT_PROTOCOL_MONITOR_ADC_BATCH_MSB = 0xE1U,
	// MotPacket_MONITOR_ADC_MOTOR_COMMON,
	// MotPacket_MONITOR_ADC_MOTOR_0,
	// MotPacket_MONITOR_ADC_MOTOR_1,
	// MotPacket_MONITOR_V_SENSORS_MV = 0x00U,
	// MotPacket_MONITOR_HEAT_DEG_C = 0x00U,

	MOT_PROTOCOL_MONITOR_RESERVED = 0xFFU,
}
MotPacket_MonitorId_T;

typedef struct MotPacket_MonitorReq_Payload_Tag { uint8_t MonitorId; } 												MotPacket_MonitorReq_Payload_T;
typedef struct MotPacket_MonitorReq_Tag { MotPacket_Header_T Header; MotPacket_MonitorReq_Payload_T CmdMonitor; } 	MotPacket_MonitorReq_T;
typedef MotPacket_Registers_T 																						MotPacket_MonitorResp_Payload_T;
typedef struct MotPacket_MonitorResp_Tag { MotPacket_Header_T Header; MotPacket_MonitorResp_Payload_T Values; } 	MotPacket_MonitorResp_T;

typedef struct MotPacket_MonitorResp_IPhases_Payload_Tag { int16_t PhaseA; int16_t PhaseB; int16_t PhaseC; } 		MotPacket_MonitorResp_IPhases_Payload_T;
typedef struct MotPacket_MonitorResp_Speed_Payload_Tag { int32_t Speed; } 											MotPacket_MonitorResp_Speed_Payload_T;


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
extern uint8_t MotPacket_ControlReq_Throttle_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t throttleValue);
extern uint8_t MotPacket_ControlReq_Brake_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t brakeValue);
extern uint8_t MotPacket_MonitorReq_Speed_Build(MotPacket_MonitorReq_T * p_reqPacket);

/******************************************************************************/
/*!	Ctrlr side */
/******************************************************************************/
extern uint8_t MotPacket_PingResp_Build(MotPacket_T * p_respPacket);
extern uint8_t MotPacket_StopResp_Build(MotPacket_Sync_T * p_respPacket);
extern uint8_t MotPacket_ControlResp_MainStatus_Build(MotPacket_ControlResp_T * p_respPacket, MotPacket_StatusResp_Id_T status);
extern uint8_t MotPacket_MonitorResp_Speed_Build(MotPacket_MonitorResp_T * p_respPacket, int32_t speed);
extern uint8_t MotPacket_MonitorResp_IPhases_Build(MotPacket_MonitorResp_T * p_respPacket, int16_t ia, int16_t ib, int16_t ic);

#endif


// extern uint8_t MotPacket_Req_Control_Build(MotPacket_ControlReq_T * p_reqPacket, const MotPacket_Interface_T * p_interface);
// extern uint8_t MotPacket_MonitorReq_Build(MotPacket_MonitorReq_T * p_reqPacket, const MotPacket_Interface_T * p_interface);
// extern bool MotPacket_Resp_Parse(MotPacket_Interface_T * p_interface, const MotPacket_T * p_rxPacket);

// typedef struct MotPacket_Interface_Tag
// {
//  	union
// 	{
// 		MotPacket_Registers_T Registers;

// 		MotPacket_ControlReq_Payload_T ReqControl;
// 		MotPacket_ControlReq_Throttle_Payload_T ReqThrottle;
// 		MotPacket_MonitorReq_Payload_T ReqMonitor;
// 		MotPacket_ReadImmediateReq_Payload_T ReqReadImmediate;

// 		MotPacket_ReadImmediateResp_Payload_T RespReadImmediate;
// 		MotPacket_MonitorResp_Payload_T RespMonitor;
// 		MotPacket_MonitorResp_Speed_Payload_T Speed;
// 		MotPacket_MonitorResp_IPhases_Payload_T IPhases;
// 	};


// }
// MotPacket_Interface_T;

// typedef struct MotPacket_Interface_Tag
// {
// 	union
// 	{
// 		MotPacket_ControlReq_Payload_T ReqControl;
// 		MotPacket_ControlReq_Throttle_Payload_T ReqThrottle;
// 		MotPacket_MonitorReq_Payload_T ReqMonitor;
// 		MotPacket_ReadImmediateReq_Payload_T ReqReadImmediate;
// 	};
// 	struct
// 	{
// 		MotPacket_ReadImmediateResp_Payload_T RespReadImmediate;
// 		MotPacket_MonitorResp_Payload_T RespMonitor;
// 		MotPacket_MonitorResp_Speed_Payload_T Speed;
// 		MotPacket_MonitorResp_IPhases_Payload_T IPhases;
// 	};
// }
// MotPacket_Interface_T;