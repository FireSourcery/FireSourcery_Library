/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotPacket.h
    @author FireSourcery
    @brief  MotPacket Packets and Interface Defs
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
        Short 2-Byte Sync Packets
        Response Packet must use different ID - Length compare determined by Id
    */
    MOT_PACKET_STOP_ALL = 0x00U,    /* If first char after Start Byte is 0x00. */
    MOT_PACKET_PING = 0x11U,        /* */
    MOT_PACKET_SYNC_ACK = 0x12U,
    MOT_PACKET_SYNC_NACK = 0x13U,
    MOT_PACKET_SYNC_ABORT = 0x14U,

    /* Full 8-Byte Header Packets */
    MOT_PACKET_STATUS = 0x20U,      /* Header Status Only */
    MOT_PACKET_VERSION = 0x21U,

    /*  */
    MOT_PACKET_REBOOT = 0xA1U,
    MOT_PACKET_CALL = 0xA2U,

    /*
        General Data
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

    /* Extended Id Modes */
    MOT_PACKET_EXT_CMD = 0xE1U,             /* ExtId Batch - Predefined Sequences */
    MOT_PACKET_ID_RESERVED_255 = 0xFFU,
}
MotPacket_HeaderId_T;
typedef enum MotPacket_Status_Tag
{
    MOT_PACKET_HEADER_STATUS_OK = 0x00U,
    MOT_PACKET_HEADER_STATUS_ERROR = 0x01U,
    MOT_PACKET_HEADER_STATUS_ERROR_WRITE_VAR_READ_ONLY = 0x11U,
    MOT_PACKET_HEADER_STATUS_ERROR_NVM = 0x21U,
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
    union { uint8_t Immediate[2U]; uint16_t Immediate16; };  /* Immediate8[2U], Immediate16 */
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
/*! Read/Write Operations by Var Id, or a function with 1-arg  */
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
    Resp    [Start, Id, Length, Status, [VarIdCheckSum],[ ], Check_L, Check_H]
*/
/******************************************************************************/
typedef struct MotPacket_ReadVars16Req_Payload_Tag { uint16_t MotVarIds[16U]; }                                                 MotPacket_ReadVars16Req_Payload_T;
typedef struct MotPacket_ReadVars16Resp_Payload_Tag { uint16_t Value16[16U]; }                                                  MotPacket_ReadVars16Resp_Payload_T;
typedef struct MotPacket_ReadVars16Req_Tag { MotPacket_Header_T Header; MotPacket_ReadVars16Req_Payload_T ReadVars16Req; }      MotPacket_ReadVars16Req_T;
typedef struct MotPacket_ReadVars16Resp_Tag { MotPacket_Header_T Header; MotPacket_ReadVars16Resp_Payload_T ReadVars16Resp; }   MotPacket_ReadVars16Resp_T;

/******************************************************************************/
/*!
    Write Var16s - Up to 8 uint16_t
    Req     [Start, Id, Length, Status, [VarIdsCount], [ ], Check_L, Check_H] [MotVarIds_0, Value16_0...MotVarIds_15, Value16_15]
    Resp    [Start, Id, Length, Status, [VarIdCheckSum], [ ], Check_L, Check_H]
*/
/******************************************************************************/
typedef struct MotPacket_WriteVars16Req_Payload_Tag { struct { uint16_t MotVarId; uint16_t Value16; } Vars[8U]; }                   MotPacket_WriteVars16Req_Payload_T;
typedef struct MotPacket_WriteVars16Resp_Payload_Tag { uint8_t Status[8U]; }                                                        MotPacket_WriteVars16Resp_Payload_T;
typedef struct MotPacket_WriteVars16Req_Tag { MotPacket_Header_T Header; MotPacket_WriteVars16Req_Payload_T WriteVars16Req; }       MotPacket_WriteVars16Req_T;
typedef struct MotPacket_WriteVars16Resp_Tag { MotPacket_Header_T Header; MotPacket_WriteVars16Resp_Payload_T WriteVars16Resp; }    MotPacket_WriteVars16Resp_T;

/******************************************************************************/
/*!
    Read Var32s - Up to 8 uint32_t
*/
/******************************************************************************/
// typedef struct MotPacket_ReadVars32Req_Payload_Tag { uint16_t MotVarIds[16U]; }                                                  MotPacket_ReadVars32Req_Payload_T;
// typedef struct MotPacket_ReadVars32Req_Tag { MotPacket_Header_T Header; MotPacket_ReadVars32Req_Payload_T ReadVars32Req; }       MotPacket_ReadVars32Req_T;
// typedef struct MotPacket_ReadVars32Resp_Payload_Tag { union { uint16_t Value16[16U]; uint32_t Value32[8U]; }; }                  MotPacket_ReadVars32Resp_Payload_T;
// typedef struct MotPacket_ReadVars32Resp_Tag { MotPacket_Header_T Header; MotPacket_ReadVars32Resp_Payload_T ReadVars32Resp; }    MotPacket_ReadVars32Resp_T;

/******************************************************************************/
/*!
    Write Var32s - Up to 5 uint32_t
*/
/******************************************************************************/
// typedef struct MotPacket_WriteVars32Req_Payload_Tag { uint16_t MotVarIds[5U]; uint32_t Value32[5U]; }                                MotPacket_WriteVars32Req_Payload_T;
// typedef struct MotPacket_WriteVars32Req_Tag { MotPacket_Header_T Header; MotPacket_WriteVars32Req_Payload_T WriteVars8Req; }         MotPacket_WriteVars32Req_T;
// typedef struct MotPacket_WriteVars32Resp_Payload_Tag { uint16_t Status[8U]; }                                                        MotPacket_WriteVars32Resp_Payload_T;
// typedef struct MotPacket_WriteVars32Resp_Tag { MotPacket_Header_T Header; MotPacket_WriteVars32Resp_Payload_T WriteVars8Resp; }      MotPacket_WriteVars32Resp_T;

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
    Req     [Start, Id, Length, Status, [ExtId], [ ], Check_L, Check_H]
    Resp    [Start, Id, Length, Status, [ ], [ ], Check_L, Check_H]
*/
/******************************************************************************/
typedef enum MotPacket_ExtId_Tag
{
    MOT_PACKET_BATCH_RESV_0 = 0x00U,
    MOT_PACKET_BATCH_MANUFACTURER_READ = 0x01U,
    MOT_PACKET_BATCH_MANUFACTURER_WRITE = 0x02U,
    // MOT_PACKET_BATCH_MANUFACTURER_WRITE_0 = 0x02U,
    // MOT_PACKET_BATCH_MANUFACTURER_WRITE_1 = 0x02U,
    MOT_PACKET_BATCH_INIT_UNITS = 0x10U,
    MOT_PACKET_BATCH_SEQUENCE_1 = 0x11U,    /* Rx Throttle, Brake, Direction. Tx: Speed, IPhase, Battery, Error,  */
    // MOT_PACKET_BATCH_CONTROL_1 = 0x11U,  /* Rx Throttle, Brake, Direction. Tx: Speed, IPhase, Battery, Error,  */
    MOT_PACKET_BATCH_ADC_MSB = 0x21U,
    MOT_PACKET_BATCH_RESV_255 = 0xFFU,
}
MotPacket_ExtId_T;

typedef struct MotPacket_BatchReq_Payload_Tag { MotPacket_Data_T Data; }                                        MotPacket_BatchReq_Payload_T;
typedef struct MotPacket_BatchResp_Payload_Tag { MotPacket_Data_T Data; }                                       MotPacket_BatchResp_Payload_T;
typedef struct MotPacket_BatchReq_Tag { MotPacket_Header_T Header; MotPacket_BatchReq_Payload_T BatchReq; }     MotPacket_BatchReq_T;
typedef struct MotPacket_BatchResp_Tag { MotPacket_Header_T Header; MotPacket_BatchResp_Payload_T BatchResp; }  MotPacket_BatchResp_T;

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

extern uint8_t MotPacket_ReadVars16Req_ParseVarIdsCount(const MotPacket_ReadVars16Req_T * p_reqPacket);
extern MotVarId_T MotPacket_ReadVars16Req_ParseVarId(const MotPacket_ReadVars16Req_T * p_reqPacket, uint8_t index);
// extern const MotVarId_T * MotPacket_ReadVars16Req_ParsePtrVarIds(const MotPacket_ReadVars16Req_T * p_reqPacket);s
// extern uint8_t MotPacket_ReadVars16Resp_Build(MotPacket_ReadVars16Resp_T * p_respPacket, uint16_t * p_values, uint8_t varsCount);
extern void MotPacket_ReadVars16Resp_BuildVarValue(MotPacket_ReadVars16Resp_T * p_respPacket, uint8_t index, uint16_t value);
extern uint8_t MotPacket_ReadVars16Resp_BuildHeader(MotPacket_ReadVars16Resp_T * p_respPacket, uint8_t varsCount);

extern uint8_t MotPacket_WriteVars16Req_ParseVarIdsCount(const MotPacket_WriteVars16Req_T * p_reqPacket);
// extern const MotVarId_T * MotPacket_WriteVars16Req_ParsePtrVarIds(const MotPacket_WriteVars16Req_T * p_reqPacket);
// extern const uint16_t * MotPacket_WriteVars16Req_ParsePtrVarValues(const MotPacket_WriteVars16Req_T * p_reqPacket);
extern MotVarId_T MotPacket_WriteVars16Req_ParseVarId(const MotPacket_WriteVars16Req_T * p_reqPacket, uint8_t index);
extern uint16_t MotPacket_WriteVars16Req_ParseVarValue(const MotPacket_WriteVars16Req_T * p_reqPacket, uint8_t index);
// extern uint8_t MotPacket_WriteVars16Resp_Build(MotPacket_WriteVars16Resp_T * p_respPacket, MotPacket_HeaderStatus_T status, uint8_t varsCount);
extern void MotPacket_WriteVars16Resp_BuildVarStatus(MotPacket_WriteVars16Resp_T * p_respPacket, uint8_t index, MotPacket_HeaderStatus_T status);
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

/******************************************************************************/
/*!
    Cmdr side
*/
/******************************************************************************/
// extern uint8_t MotPacket_GetRespLength(MotPacket_HeaderId_T headerId);
// extern uint8_t MotPacket_PingReq_GetRespLength(void);
// extern uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket);
// extern MotPacket_HeaderId_T MotPacket_PingResp_Parse(const MotPacket_PingResp_T * p_respPacket);

// // extern uint8_t MotPacket_StopReq_GetRespLength(void);
// extern uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket);
// extern MotPacket_HeaderStatus_T MotPacket_StopResp_Parse(const MotPacket_StopResp_T * p_respPacket);

// // extern uint8_t MotPacket_ReadVarReq_GetRespLength(void);
// extern uint8_t MotPacket_ReadVarReq_Build(MotPacket_ReadVarReq_T * p_reqPacket, MotVarId_T motVarId);
// extern uint32_t MotPacket_ReadVarResp_Parse(const MotPacket_ReadVarResp_T * p_respPacket, uint32_t * p_value);

// // extern uint8_t MotPacket_WriteVarReq_GetRespLength(void);
// extern uint8_t MotPacket_WriteVarReq_Build(MotPacket_WriteVarReq_T * p_reqPacket, MotVarId_T motVarId, uint32_t value);
// extern MotPacket_HeaderStatus_T MotPacket_WriteVarResp_Parse(const MotPacket_WriteVarResp_T * p_respPacket);

// // extern uint8_t MotPacket_SaveNvmReq_GetRespLength(void);
// extern uint8_t MotPacket_SaveNvmResp_Build(MotPacket_SaveNvmResp_T * p_respPacket, MotPacket_HeaderStatus_T status);
// extern MotPacket_HeaderStatus_T MotPacket_SaveNvmReq_Parse(const MotPacket_SaveNvmReq_T * p_respPacket);

// extern uint8_t MotPacket_BatchReq_GetRespLength(MotPacket_ExtId_T ExtId);

#endif
