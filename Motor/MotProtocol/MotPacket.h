/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     MotPacket.h
    @author FireSourcery
    @brief     MotPacket Packets and Interface Defs
    @version V0
*/
/******************************************************************************/
#ifndef MOT_PACKET_H
#define MOT_PACKET_H

#include "MotVarId.h"
#include <stdint.h>
#include <stdbool.h>

#define MOT_PACKET_VERSION_OPT          (254U)
#define MOT_PACKET_VERSION_MAJOR        (0U)
#define MOT_PACKET_VERSION_MINOR        (6U)
#define MOT_PACKET_VERSION_BUGFIX       (10U)

#define MOT_PACKET_HEADER_LENGTH        (8U)
#define MOT_PACKET_PAYLOAD_MAX          (32U)
#define MOT_PACKET_LENGTH_MIN           (2U)
#define MOT_PACKET_LENGTH_MAX           (MOT_PACKET_PAYLOAD_MAX + MOT_PACKET_HEADER_LENGTH)
#define MOT_PACKET_START_BYTE           (0xA5U)
#define MOT_PACKET_LENGTH_BYTE_INDEX    (3U)

/*
    Packet and correspondence type. Per unique packet structure/parsing/processing pattern
    May process directly as cmd, or lead extended cmd id
*/
typedef enum MotPacket_HeaderId_Tag
{
    /*
        2-Byte Packets
        Response Packet must use different ID - Length compare determined by Id
    */
    MOT_PACKET_STOP_ALL = 0x00U,    /* If first char after Start Byte is 0x00. */
    MOT_PACKET_PING = 0x11U,        /* */
    MOT_PACKET_SYNC_ACK = 0x12U,
    MOT_PACKET_SYNC_NACK = 0x13U,
    MOT_PACKET_SYNC_ABORT = 0x14U,

    /* 8-Byte Header Packets */
    MOT_PACKET_STATUS = 0x20U,        /* Header Status Only */
    MOT_PACKET_VERSION = 0x21U,

    /*  */
    MOT_PACKET_REBOOT = 0xA1U,
    MOT_PACKET_CALL = 0xA2U,

    /* General Data */
    /*
        Read/Write Var: Real-Time Variable, NvMemory Parameters; call Functions with 1 argument
    */
    MOT_PACKET_READ_VAR = 0xD1U,            /* Read Single Var */
    MOT_PACKET_WRITE_VAR = 0xD2U,           /* Write Single Var */
    MOT_PACKET_READ_VARS16 = 0xD3U,         /* Up to 16 uint16_t values */
    MOT_PACKET_WRITE_VARS16 = 0xD4U,        /* Up to 8 uint16_t values */
    MOT_PACKET_READ_VARS32 = 0xD5U,         /* Up to 8 uint32_t values */
    MOT_PACKET_WRITE_VARS32 = 0xD6U,        /* Up to 4 uint32_t values */
    // MOT_PACKET_READ_VARS_FLEX = 0xD7U,          /*  */
    // MOT_PACKET_WRITE_VARS_FLEX  = 0xD8U,        /*  */
    MOT_PACKET_SAVE_NVM = 0xD9U,            /* Save Non-Volatile Parameters. Must run after params write. */

    /*
       Direct Address Read/Write
    */
    MOT_PACKET_READ_ADDRESS = 0xDAU,        /* Read Address */
    MOT_PACKET_WRITE_ADDRESS = 0xDBU,       /* Write Address */
    MOT_PACKET_DATA_MODE_DATA = 0xDDU,      /* Data Mode Data */
    MOT_PACKET_DATA_MODE_READ = 0xDEU,      /* Stateful NvMemory Read using Address */
    MOT_PACKET_DATA_MODE_WRITE = 0xDFU,     /* Stateful NvMemory Write using Address */

    /* Extended Header Modes */
    MOT_PACKET_EXT_CMD = 0xE1U,             /* ExtId Batch - Predefined Sequences */
    MOT_PACKET_ID_RESERVED_255 = 0xFFU,
}
MotPacket_HeaderId_T;
typedef enum MotPacket_Status_Tag
{
    MOT_PACKET_HEADER_STATUS_OK = 0x00U,
    MOT_PACKET_HEADER_STATUS_ERROR = 0x01U,
    MOT_PACKET_HEADER_STATUS_ERROR_WRITE_VAR_READ_ONLY = 0x11U,
    // MOT_PACKET_HEADER_STATUS_ERROR_NVM_ = 0x21U,
    MOT_PACKET_HEADER_STATUS_RESERVED = 0xFFU,
}
MotPacket_HeaderStatus_T;

/* 2-Byte Sync Packet */
typedef struct MotPacket_Sync_Tag
{
    uint8_t Start;
    uint8_t SyncId; /* MotPacket_HeaderId_T */
}
MotPacket_Sync_T;

typedef struct __attribute__((packed)) MotPacket_Header_Tag
{
    uint8_t Start;
    uint8_t HeaderId;       /* MotPacket_HeaderId_T - Cmd / Descriptor of packet contents */
    uint8_t TotalLength;    /* Packet TotalLength */
    uint8_t Status;         /* MotPacket_HeaderStatus_T - Optional Status */
    union { uint8_t Opt[2U]; uint16_t OptFull; }; //Immediate8[2U], Immediate16
    uint16_t Crc;
}
MotPacket_Header_T;

typedef union { uint32_t Unsigned; int32_t Signed; } var32_t;
typedef union { uint16_t Unsigned; int16_t Signed; } var16_t;
typedef union { uint8_t Unsigned; int8_t Signed; } var8_t;
typedef union MotPacket_Data_Tag
{
    uint8_t Bytes[32U]; var8_t Var8s[32U]; var16_t Var16s[16U]; var32_t Var32s[8U];
}
MotPacket_Data_T;

typedef union  __attribute__((packed)) MotPacket_Packet_Tag
{
    struct
    {
        MotPacket_Header_T Header;
        union { uint8_t Payload[MOT_PACKET_PAYLOAD_MAX]; MotPacket_Data_T Data; };
    };
    uint8_t Bytes[MOT_PACKET_LENGTH_MAX];
}
MotPacket_T;

static inline uint8_t MotPacket_ParsePayloadLength(const MotPacket_T * p_packet) { return p_packet->Header.TotalLength - sizeof(MotPacket_Header_T); }
static inline uint8_t MotPacket_ParseTotalLength(const MotPacket_T * p_packet) { return p_packet->Header.TotalLength; }
static inline MotPacket_HeaderStatus_T MotPacket_ParseHeaderStatus(const MotPacket_T * p_packet) { return p_packet->Header.Status; }

/******************************************************************************/
/*! Ping */
/******************************************************************************/
typedef MotPacket_Sync_T    MotPacket_PingReq_T;
typedef MotPacket_Sync_T    MotPacket_PingResp_T;

/******************************************************************************/
/*! Stop - Emergency Stop All */
/******************************************************************************/
typedef MotPacket_Sync_T                                                 MotPacket_StopReq_T;
typedef struct MotPacket_StopResp_Tag { MotPacket_Header_T Header; }     MotPacket_StopResp_T;

/******************************************************************************/
/*! Version */
/******************************************************************************/
typedef struct MotPacket_VersionResp_Payload_Tag { uint8_t Version[4U]; }                                               MotPacket_VersionResp_Payload_T;
typedef MotPacket_Header_T                                                                                              MotPacket_VersionReq_T;
typedef struct MotPacket_VersionResp_Tag { MotPacket_Header_T Header; MotPacket_VersionResp_Payload_T VersionResp; }    MotPacket_VersionResp_T;

/******************************************************************************/
/*! Read/Write Operations by Var Id. or indicator for a single arg function */
/******************************************************************************/

/******************************************************************************/
/*!
    Read Var by Id
    Req     [Start, Id, Length, Status, [VarId_L],[VarId_H], Check_L, Check_H]
    Resp    [Start, Id, Length, Status, [VarId_L],[VarId_H], Check_L, Check_H] [Value32]
*/
/******************************************************************************/
// typedef struct MotPacket_ReadVarReq_Payload_Tag { }                                                                          MotPacket_ReadVarReq_Payload_T;
typedef struct __attribute__((packed)) MotPacket_ReadVarResp_Payload_Tag { union { uint16_t Value16; uint32_t Value32; }; }     MotPacket_ReadVarResp_Payload_T;
typedef struct MotPacket_ReadVarReq_Tag { MotPacket_Header_T Header; }                                                          MotPacket_ReadVarReq_T;
typedef struct MotPacket_ReadVarResp_Tag { MotPacket_Header_T Header; MotPacket_ReadVarResp_Payload_T ReadVarResp; }            MotPacket_ReadVarResp_T;

/******************************************************************************/
/*!
    Write Var by Id
    Req     [Start, Id, Length, Status, [VarId_L],[VarId_H], Check_L, Check_H] [Value32]
    Resp    [Start, Id, Length, Status, [VarId_L],[VarId_H], Check_L, Check_H]
*/
/******************************************************************************/
typedef struct __attribute__((packed)) MotPacket_WriteVarReq_Payload_Tag { union { uint16_t Value16; uint32_t Value32; }; }     MotPacket_WriteVarReq_Payload_T;
// typedef struct MotPacket_WriteVarResp_Payload_Tag { }                                                                        MotPacket_WriteVarResp_Payload_T;
typedef struct MotPacket_WriteVarReq_Tag { MotPacket_Header_T Header; MotPacket_WriteVarReq_Payload_T WriteVarReq; }            MotPacket_WriteVarReq_T;
typedef struct MotPacket_WriteVarResp_Tag { MotPacket_Header_T Header; }                                                        MotPacket_WriteVarResp_T;


/******************************************************************************/
/*!
    Read Var16s - Up to 16 uint16_t
    Req     [Start, Id, Length, Status, [ ],[ ], Check_L, Check_H] [MotVarIds[16]]
    Resp    [Start, Id, Length, Status, [ ],[ ], Check_L, Check_H]
*/
/******************************************************************************/
typedef struct MotPacket_ReadVars16Req_Payload_Tag { uint16_t MotVarIds[16U]; }                                                 MotPacket_ReadVars16Req_Payload_T;
typedef struct MotPacket_ReadVars16Resp_Payload_Tag { uint16_t Vars16[16U]; }                                                   MotPacket_ReadVars16Resp_Payload_T;
typedef struct MotPacket_ReadVars16Req_Tag { MotPacket_Header_T Header; MotPacket_ReadVars16Req_Payload_T ReadVars16Req; }      MotPacket_ReadVars16Req_T;
typedef struct MotPacket_ReadVars16Resp_Tag { MotPacket_Header_T Header; MotPacket_ReadVars16Resp_Payload_T ReadVars16Resp; }   MotPacket_ReadVars16Resp_T;

/******************************************************************************/
/*!
    Write Var16s - Up to 8 uint16_t
    Req     [Start, Id, Length, Status, [ ],[ ], Check_L, Check_H]
    Resp    [Start, Id, Length, Status, [ ],[VarIdCheckSum], Check_L, Check_H]
*/
/******************************************************************************/
typedef struct MotPacket_WriteVars16Req_Payload_Tag { uint16_t MotVarIds[8U]; uint16_t Vars16[8U]; }                                MotPacket_WriteVars16Req_Payload_T;
typedef struct MotPacket_WriteVars16Resp_Payload_Tag { uint8_t Status[8U]; }                                                        MotPacket_WriteVars16Resp_Payload_T;
typedef struct MotPacket_WriteVars16Req_Tag { MotPacket_Header_T Header; MotPacket_WriteVars16Req_Payload_T WriteVars16Req; }       MotPacket_WriteVars16Req_T;
typedef struct MotPacket_WriteVars16Resp_Tag { MotPacket_Header_T Header; MotPacket_WriteVars16Resp_Payload_T WriteVars16Resp; }    MotPacket_WriteVars16Resp_T;

/******************************************************************************/
/*!
    Read Var32s - Up to 8 uint32_t
*/
/******************************************************************************/
// typedef struct MotPacket_ReadVars32Req_Payload_Tag { uint16_t MotVarIds[16U]; }                                                  MotPacket_ReadVars16Req_Payload_T;
// typedef struct MotPacket_ReadVars32Req_Tag { MotPacket_Header_T Header; MotPacket_ReadVars16Req_Payload_T ReadVars16Req; }       MotPacket_ReadVars16Req_T;
// typedef struct MotPacket_ReadVars32Resp_Payload_Tag { union { uint16_t Value16[16U]; uint32_t Value32[8U]; }; }                  MotPacket_ReadVars16Resp_Payload_T;
// typedef struct MotPacket_ReadVars32Resp_Tag { MotPacket_Header_T Header; MotPacket_ReadVars16Resp_Payload_T ReadVars16Resp; }    MotPacket_ReadVars16Resp_T;

/******************************************************************************/
/*!
    Write Var32s - Up to 5 uint32_t
*/
/******************************************************************************/
// typedef struct MotPacket_WriteVars16Req_Payload_Tag { uint16_t MotVarIds[5U]; uint32_t Value32[5U]; }                                MotPacket_WriteVars16Req_Payload_T;
// typedef struct MotPacket_WriteVars16Req_Tag { MotPacket_Header_T Header; MotPacket_WriteVars16Req_Payload_T WriteVars8Req; }         MotPacket_WriteVars16Req_T;
// typedef struct MotPacket_WriteVars16Resp_Payload_Tag { uint16_t Status[8U]; }                                                        MotPacket_WriteVars16Resp_Payload_T;
// typedef struct MotPacket_WriteVars16Resp_Tag { MotPacket_Header_T Header; MotPacket_WriteVars16Resp_Payload_T WriteVars8Resp; }      MotPacket_WriteVars16Resp_T;

/******************************************************************************/
/*! Save Nvm */ // todo specify segments
/******************************************************************************/
// typedef struct MotPacket_SaveNvmReq_Payload_Tag { }                          MotPacket_SaveNvmReq_Payload_T;
// typedef struct MotPacket_SaveNvmResp_Payload_Tag { }                         MotPacket_SaveNvmResp_Payload_T;
typedef struct MotPacket_SaveNvmReq_Tag { MotPacket_Header_T Header; }          MotPacket_SaveNvmReq_T;
typedef struct MotPacket_SaveNvmResp_Tag { MotPacket_Header_T Header; }         MotPacket_SaveNvmResp_T;

/******************************************************************************/
/*!
    Stateful Sequence
*/
/******************************************************************************/
/******************************************************************************/
/*! Data Mode Read */
/******************************************************************************/
typedef struct MotPacket_ReadDataReq_Payload_Tag { uint32_t AddressStart; uint32_t SizeBytes; }                             MotPacket_ReadDataReq_Payload_T;
// typedef struct MotPacket_ReadDataResp_Payload_Tag {  }                                                                   MotPacket_ReadDataResp_Payload_T;
typedef struct MotPacket_ReadDataReq_Tag { MotPacket_Header_T Header; MotPacket_ReadDataReq_Payload_T ReadDataReq; }        MotPacket_ReadDataReq_T;
typedef struct MotPacket_ReadDataResp_Tag { MotPacket_Header_T Header; }                                                    MotPacket_ReadDataResp_T;

/******************************************************************************/
/*! Data Mode Write */
/******************************************************************************/
typedef struct MotPacket_WriteDataReq_Payload_Tag { uint32_t AddressStart; uint32_t SizeBytes; }                            MotPacket_WriteDataReq_Payload_T;
// typedef struct MotPacket_WriteDataResp_Payload_Tag { }                                                                   MotPacket_WriteDataResp_Payload_T;
typedef struct MotPacket_WriteDataReq_Tag { MotPacket_Header_T Header; MotPacket_WriteDataReq_Payload_T WriteDataReq; }     MotPacket_WriteDataReq_T;
typedef struct MotPacket_WriteDataResp_Tag { MotPacket_Header_T Header; }                                                   MotPacket_WriteDataResp_T;

/******************************************************************************/
/*! Data Mode Raw Data Packet */
/******************************************************************************/
typedef struct MotPacket_DataReqResp_Payload_Tag { uint8_t Data[MOT_PACKET_PAYLOAD_MAX]; }       MotPacket_DataReqResp_Payload_T;
typedef struct MotPacket_DataReqResp_Tag { MotPacket_Header_T Header; MotPacket_Data_T Data; }   MotPacket_DataReqResp_T;

/******************************************************************************/
/*!
    Ext Batch Sequence
*/
/******************************************************************************/
/*
    Default units in the client motor controller native format
*/
typedef enum MotPacket_ExtId_Tag
{
    // MOT_PACKET_BATCH_INIT_UNITS = 0x00U,

    MOT_PACKET_CONTROL_RELEASE = 0x00U,
    MOT_PACKET_CONTROL_DIRECTION_FORWARD = 0x01U,        /* Main direction */
    MOT_PACKET_CONTROL_DIRECTION_REVERSE = 0x02U,        /* Main direction */
    MOT_PACKET_CONTROL_DIRECTION_NEUTRAL = 0x03U,        /* Main direction */

    MOT_PACKET_CONTROL_THROTTLE = 0x11U,    /* Value [0:65535] */
    MOT_PACKET_CONTROL_BRAKE = 0x12U,        /* Value [0:65535] */
    MOT_PACKET_CONTROL_VOLTAGE = 0x13U,     /* Value [-32768:32767] */
    MOT_PACKET_CONTROL_CURRENT = 0x14U,        /* Value [-32768:32767] */
    MOT_PACKET_CONTROL_SPEED = 0x15U,        /* Value [-32768:32767] */
    MOT_PACKET_CONTROL_ANGLE = 0x16U,        /*   */

    MOT_PACKET_CONTROL_BUZZER = 0x21U,        /* 2 Var params  */

    MOT_PACKET_CONTROL_MONITOR = 0x51U,        /*   */

    // MOT_PACKET_CONTROL_VOLTAGE_VPWM = 0x83U, /* DutyCycle 65536 */
    // MOT_PACKET_CONTROL_VOLTAGE_VOLTS = 0x83U,
    // MOT_PACKET_CONTROL_CURRENT_AMPS = 0x84U,
    // MOT_PACKET_CONTROL_SPEED_RPM = 0x85U,
    // MOT_PACKET_CONTROL_ANGLE = 0x85U,

    MOT_PACKET_CONTROL_BATCH_1 = 0x71U,         /* Rx Throttle, Brake, Direction. Tx: Speed, IPhase, Battery, Error,  */
    MOT_PACKET_CONTROL_MODE_RESERVED = 0xFFU,

    MOT_PACKET_MONITOR_SPEED = 0x01U,            /* Speed in Q1.15 */
    // MOT_PACKET_MONITOR_I_PHASES = 0x02U,        /* Iabc in Q1.15 */
    // MOT_PACKET_MONITOR_I_FOC = 0x03U,        /* Idq, Ialphabeta in Q1.15 */
    // MOT_PACKET_MONITOR_V_PHASES = 0x04U,
    // MOT_PACKET_MONITOR_ANGLES = 0x05U,
    // MOT_PACKET_MONITOR_STATUS_FLAGS = 0x0FU,

    MOT_PACKET_MONITOR_ANALOG_USER = 0x10U,
    MOT_PACKET_MONITOR_HALL = 0x11U,
    MOT_PACKET_MONITOR_SIN_COS = 0x12U,
    MOT_PACKET_MONITOR_ENCODER = 0x13U,

    MOT_PACKET_MONITOR_V_SENSORS = 0x21U,         /* VSource, ~5V, ~12V */
    MOT_PACKET_MONITOR_HEAT = 0x22U,            /* In ADCU, Lower is higher */

    MOT_PACKET_MONITOR_METERS = 0x31U,             /* Speed, Angles */

    MOT_PACKET_MONITOR_I_FOC = 0x03U,            /* Iabc, Ialphabeta, Idq in Q1.15 */

    // MOT_PACKET_MONITOR_SPEED_RPM = 0x81U,    /* Speed in RPM */

    MOT_PACKET_MONITOR_ADC_BATCH_MSB = 0xE1U,
    // MotPacket_MONITOR_ADC_MOTOR_COMMON,
    // MotPacket_MONITOR_ADC_MOTOR_0,
    // MotPacket_MONITOR_ADC_MOTOR_1,
    // MotPacket_MONITOR_V_SENSORS_MV = 0x00U,
    // MotPacket_MONITOR_HEAT_DEG_C = 0x00U,

    MOT_PACKET_MONITOR_RESERVED = 0xFFU,
}
MotPacket_ExtId_T;

/******************************************************************************/
/*!
    Control - Pre-Assigned Batch Read/Write, Monitor, Sequence,

    All Control Type Packets default 16-bit variables
    Throttle, Brake is unsigned. Speed, Torque, Voltage is signed.
    Up to 15 parameters, determined by MotPacket_ExtId_T
*/
/******************************************************************************/
typedef struct MotPacket_ControlReq_Payload_Tag { uint8_t ExtId; uint8_t RsvrByte; union { int16_t ValueS16s[15U]; uint16_t ValueU16s[15U]; }; }     MotPacket_ControlReq_Payload_T;
typedef struct MotPacket_ControlReq_Tag { MotPacket_Header_T Header; MotPacket_ControlReq_Payload_T ControlReq; }                                         MotPacket_ControlReq_T;
typedef struct MotPacket_ControlResp_Payload_Tag { uint16_t MainStatus; uint16_t OptStatus[15U]; }                                                         MotPacket_ControlResp_Payload_T;
typedef struct MotPacket_ControlResp_Tag { MotPacket_Header_T Header; MotPacket_ControlResp_Payload_T ControlResp; }                                     MotPacket_ControlResp_T;

// typedef struct MotPacket_ControlReq_Throttle_Payload_Tag { uint8_t ExtId; uint8_t RsvrByte; uint16_t ThrottleValue; }     MotPacket_ControlReq_Throttle_Payload_T;
// // typedef struct MotPacket_ControlResp_Throttle_Payload_Tag { }                                                             MotPacket_ControlResp_Throttle_Payload_T;
// typedef struct MotPacket_ControlReq_Brake_Payload_Tag { uint8_t ExtId; uint8_t RsvrByte; uint16_t BrakeValue; }         MotPacket_ControlReq_Brake_Payload_T;
// // typedef struct MotPacket_ControlResp_Brake_Payload_Tag { }                                                                 MotPacket_ControlResp_Brake_Payload_T;
// typedef struct MotPacket_ControlReq_Direction_Payload_Tag { uint8_t ExtId; uint8_t RsvrByte; }                             MotPacket_ControlReq_Direction_Payload_T;
// typedef struct MotPacket_ControlReq_Release_Payload_Tag { uint8_t ExtId; uint8_t RsvrByte; }                             MotPacket_ControlReq_Release_Payload_T;

// typedef struct MotPacket_MonitorResp_Speed_Payload_Tag { int32_t Speed; }                                                 MotPacket_MonitorResp_Speed_Payload_T;
// // typedef struct MotPacket_MonitorResp_Phases3_Payload_Tag { int16_t PhaseA; int16_t PhaseB; int16_t PhaseC; }             MotPacket_MonitorResp_Phases3_Payload_T;
// typedef struct MotPacket_MonitorResp_IFoc_Payload_Tag
// {
//     int16_t Ia; int16_t Ib; int16_t Ic;
//     int16_t Ialpha; int16_t Ibeta;
//     int16_t Id; int16_t Iq;
// }
// MotPacket_MonitorResp_IFoc_Payload_T;

// typedef struct MotPacket_InitUnitsReq_Payload_Tag {uint8_t InitUnitsId; uint8_t RsvrByte;}                                 MotPacket_InitUnitsReq_Payload_T;
// typedef struct MotPacket_InitUnitsReq_Tag { MotPacket_Header_T Header; }                                                     MotPacket_InitUnitsReq_T;
// typedef MotPacket_Data_T                                                                                                    MotPacket_InitUnitsResp_Payload_T;
// typedef struct MotPacket_InitUnitsResp_Tag { MotPacket_Header_T Header; MotPacket_InitUnitsResp_Payload_T InitUnitsResp; }     MotPacket_InitUnitsResp_T;
// typedef struct MotPacket_BatchReq_InitUnits_Payload_Tag
// {
//     speedRef_Rpm;     iRef_Amp;     vRef_Volts;     vSupply_R1;     vSupply_R2;     vSense_R1;     vSense_R2;     vAcc_R1;     vAcc_R2;
// }     MotPacket_BatchReq_InitUnits_Payload_T;



/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/

/******************************************************************************/
/*! Common */
/******************************************************************************/
extern bool MotPacket_CheckChecksum(const MotPacket_T * p_packet);
extern uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_HeaderId_T syncId);
// extern uint8_t MotPacket_GetRespLength(MotPacket_HeaderId_T headerId);

/******************************************************************************/
/*!
    Ctrlr side
*/
/******************************************************************************/
extern uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket);
extern uint8_t MotPacket_VersionResp_Build(MotPacket_VersionResp_T * p_respPacket);
extern uint8_t MotPacket_StopResp_Build(MotPacket_StopResp_T * p_respPacket);
extern uint8_t MotPacket_SaveNvmResp_Build(MotPacket_SaveNvmResp_T * p_respPacket, MotPacket_HeaderStatus_T status);

extern MotVarId_T MotPacket_ReadVarReq_ParseVarId(const MotPacket_ReadVarReq_T * p_reqPacket);
extern uint8_t MotPacket_ReadVarResp_Build(MotPacket_ReadVarResp_T * p_respPacket, uint32_t value);

extern MotVarId_T MotPacket_WriteVarReq_ParseVarId(const MotPacket_WriteVarReq_T * p_reqPacket);
extern uint32_t MotPacket_WriteVarReq_ParseVarValue(const MotPacket_WriteVarReq_T * p_reqPacket);
extern uint8_t MotPacket_WriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, MotPacket_HeaderStatus_T status);

extern const MotVarId_T * MotPacket_ReadVars16Req_ParsePtrVarIds(const MotPacket_ReadVars16Req_T * p_reqPacket);
extern uint8_t MotPacket_ReadVars16Req_ParseVarIdsCount(const MotPacket_ReadVars16Req_T * p_reqPacket);
extern uint8_t MotPacket_ReadVars16Resp_Build(MotPacket_ReadVars16Resp_T * p_respPacket, uint16_t * p_values, uint8_t varsCount);
extern void MotPacket_ReadVars16Resp_BuildIndexVar(MotPacket_ReadVars16Resp_T * p_respPacket, uint8_t index, uint16_t value);
extern uint8_t MotPacket_ReadVars16Resp_BuildHeader(MotPacket_ReadVars16Resp_T * p_respPacket, uint8_t varsCount);

extern const MotVarId_T * MotPacket_WriteVars16Req_ParsePtrVarIds(const MotPacket_WriteVars16Req_T * p_reqPacket);
extern uint8_t MotPacket_WriteVars16Req_ParseVarIdsCount(const MotPacket_WriteVars16Req_T * p_reqPacket);
extern uint8_t MotPacket_WriteVars16Resp_Build(MotPacket_WriteVars16Resp_T * p_respPacket, MotPacket_HeaderStatus_T status, uint8_t varsCount);
extern void MotPacket_WriteVars16Resp_BuildIndexStatus(MotPacket_WriteVars16Resp_T * p_respPacket, uint8_t index, MotPacket_HeaderStatus_T status);
extern uint8_t MotPacket_WriteVars16Resp_BuildHeader(MotPacket_WriteVars16Resp_T * p_respPacket, MotPacket_HeaderStatus_T status, uint8_t varsCount);

extern void MotPacket_ReadDataReq_Parse(const MotPacket_ReadDataReq_T * p_reqPacket, uint32_t * p_addressStart, uint32_t * p_sizeBytes);
extern uint8_t MotPacket_ReadDataResp_Build(MotPacket_ReadDataResp_T * p_respPacket, MotPacket_HeaderStatus_T status);
extern uint32_t MotPacket_ReadDataReq_ParseAddress(const MotPacket_ReadDataReq_T * p_reqPacket);
extern uint32_t MotPacket_ReadDataReq_ParseSize(const MotPacket_ReadDataReq_T * p_reqPacket);

extern void MotPacket_WriteDataReq_Parse(const MotPacket_WriteDataReq_T * p_reqPacket, uint32_t * p_addressStart, uint32_t * p_sizeBytes);
extern uint8_t MotPacket_WriteDataResp_Build(MotPacket_WriteDataResp_T * p_respPacket, MotPacket_HeaderStatus_T status);
extern uint32_t MotPacket_WriteDataReq_ParseAddress(const MotPacket_WriteDataReq_T * p_reqPacket);
extern uint32_t MotPacket_WriteDataReq_ParseSize(const MotPacket_WriteDataReq_T * p_reqPacket);

extern uint8_t MotPacket_Data_Build(MotPacket_DataReqResp_T * p_dataPacket, const uint8_t * p_address, uint8_t sizeData);
extern const uint8_t * MotPacket_Data_ParsePtrPayload(const MotPacket_DataReqResp_T * p_dataPacket);
extern uint8_t MotPacket_Data_ParseSize(const MotPacket_DataReqResp_T * p_dataPacket);

// extern uint8_t MotPacket_ControlResp_MainStatus_Build(MotPacket_ControlResp_T * p_respPacket, uint16_t status);
// extern uint8_t MotPacket_MonitorResp_Speed_Build(MotPacket_MonitorResp_T * p_respPacket, int32_t speed);
// extern uint8_t MotPacket_MonitorResp_IPhases_Build(MotPacket_MonitorResp_T * p_respPacket, int16_t ia, int16_t ib, int16_t ic);
// extern uint8_t MotPacket_MonitorResp_IFoc_Build
// (
//     MotPacket_MonitorResp_T * p_respPacket,
//     int16_t ia, int16_t ib, int16_t ic,
//     int16_t ialpha, int16_t ibeta,
//     int16_t id, int16_t iq
// );
// extern uint8_t MotPacket_InitUnitsResp_Build
// (
//     MotPacket_InitUnitsResp_T * p_respPacket,
//     uint16_t speedRef_Rpm, uint16_t iRef_Amp, uint16_t vRef_Volts, /* Frac16 conversions */
//     uint16_t vSupply_R1, uint16_t vSupply_R2,    /* Adcu <-> Volts conversions */
//     uint16_t vSense_R1, uint16_t vSense_R2,
//     uint16_t vAcc_R1, uint16_t vAcc_R2
// );

/******************************************************************************/
/*!
    Cmdr side
*/
/******************************************************************************/
// extern uint8_t MotPacket_PingReq_GetRespLength(void);
extern uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket);
extern MotPacket_HeaderId_T MotPacket_PingResp_Parse(const MotPacket_PingResp_T * p_respPacket);

// extern uint8_t MotPacket_StopReq_GetRespLength(void);
extern uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket);
extern MotPacket_HeaderStatus_T MotPacket_StopResp_Parse(const MotPacket_StopResp_T * p_respPacket);

// extern uint8_t MotPacket_ReadVarReq_GetRespLength(void);
extern uint8_t MotPacket_ReadVarReq_Build(MotPacket_ReadVarReq_T * p_reqPacket, MotVarId_T motVarId);
extern uint32_t MotPacket_ReadVarResp_Parse(const MotPacket_ReadVarResp_T * p_respPacket, uint32_t * p_value);

// extern uint8_t MotPacket_WriteVarReq_GetRespLength(void);
extern uint8_t MotPacket_WriteVarReq_Build(MotPacket_WriteVarReq_T * p_reqPacket, MotVarId_T motVarId, uint32_t value);
extern MotPacket_HeaderStatus_T MotPacket_WriteVarResp_Parse(const MotPacket_WriteVarResp_T * p_respPacket);

// extern uint8_t MotPacket_SaveNvmReq_GetRespLength(void);
extern uint8_t MotPacket_SaveNvmResp_Build(MotPacket_SaveNvmResp_T * p_respPacket, MotPacket_HeaderStatus_T status);
extern MotPacket_HeaderStatus_T MotPacket_SaveNvmReq_Parse(const MotPacket_SaveNvmReq_T * p_respPacket);

// extern uint8_t MotPacket_ControlReq_Release_Build(MotPacket_ControlReq_T * p_reqPacket);
// extern uint8_t MotPacket_ControlReq_DirectionForward_Build(MotPacket_ControlReq_T * p_reqPacket);
// extern uint8_t MotPacket_ControlReq_DirectionReverse_Build(MotPacket_ControlReq_T * p_reqPacket);
// extern uint8_t MotPacket_ControlReq_DirectionNeutral_Build(MotPacket_ControlReq_T * p_reqPacket);
// extern uint8_t MotPacket_ControlReq_Throttle_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t throttleValue);
// extern uint8_t MotPacket_ControlReq_Brake_Build(MotPacket_ControlReq_T * p_reqPacket, uint16_t brakeValue);
// extern uint8_t MotPacket_ControlReq_GetRespLength(MotPacket_ExtId_T ExtId);

// extern uint8_t MotPacket_MonitorReq_Build(MotPacket_MonitorReq_T * p_reqPacket, MotPacket_ExtId_T monitorId);
// // extern uint8_t MotPacket_MonitorReq_Speed_Build(MotPacket_MonitorReq_T * p_reqPacket);
// extern uint8_t MotPacket_MonitorReq_GetRespLength(MotPacket_ExtId_T monitorId);
// extern void MotPacket_MonitorResp_Speed_Parse(int32_t * p_speed_Fixed32, const MotPacket_MonitorResp_T * p_respPacket);
// extern void MotPacket_MonitorResp_IFoc_Parse
// (
//     int16_t * p_ia, int16_t * p_ib, int16_t * p_ic,
//     int16_t * p_ialpha, int16_t * p_ibeta, int16_t * p_id, int16_t * p_iq,
//     const MotPacket_MonitorResp_T * p_respPacket
// );
// extern uint8_t MotPacket_InitUnitsReq_Build(MotPacket_InitUnitsReq_T * p_reqPacket);
// extern uint8_t MotPacket_InitUnitsReq_GetRespLength(void);
// extern void MotPacket_InitUnitsResp_Parse
// (
//     uint16_t * p_speedRef_Rpm, uint16_t * p_iRef_Amp, uint16_t * p_vRef_Volts, /* Frac16 conversions */
//     uint16_t * p_vSupply_R1, uint16_t * p_vSupply_R2,    /* Adcu <-> Volts conversions */
//     uint16_t * p_vSense_R1, uint16_t * p_vSense_R2,
//     uint16_t * p_vAcc_R1, uint16_t * p_vAcc_R2,
//     const MotPacket_InitUnitsResp_T * p_respPacket
// );

#endif
