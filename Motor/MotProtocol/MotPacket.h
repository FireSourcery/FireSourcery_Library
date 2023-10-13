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

#include <stdint.h>
#include <stdbool.h>

#define MOT_PACKET_VERSION_OPT          (254U)
#define MOT_PACKET_VERSION_MAJOR        (0U)
#define MOT_PACKET_VERSION_MINOR        (6U)
#define MOT_PACKET_VERSION_BUGFIX       (10U)

#define MOT_PACKET_LENGTH_MAX           (40U)
#define MOT_PACKET_LENGTH_MIN           (2U)
#define MOT_PACKET_HEADER_LENGTH        (4U)
#define MOT_PACKET_PAYLOAD_MAX          (MOT_PACKET_LENGTH_MAX - MOT_PACKET_HEADER_LENGTH)
#define MOT_PACKET_START_BYTE           (0xA5U)
#define MOT_PACKET_LENGTH_BYTE_INDEX    (2U)

/*
    Packet and correspondence type. Per unique packet structure/parsing/processing pattern
    May process directly as cmd, or lead extended cmd id
    Essential the packets "OpCode"
*/
typedef enum MotPacket_Id
{
    /*
        2-Byte Sync Packets
        Response Packet must use different ID - Length compare determined by Id
    */
    MOT_PACKET_PING = 0x11U,        /* */
    MOT_PACKET_FEED_WATCHDOG = 0x15U,
    MOT_PACKET_SYNC_ACK = 0x12U,
    MOT_PACKET_SYNC_NACK = 0x13U,
    MOT_PACKET_SYNC_ABORT = 0x14U,

    // 4-byte status wihout checksum

    // 8-byte immediates with 16 checksum
    // MOT_PACKET_ACK_SEQ,
    // MOT_PACKET_STATUS_OK = 0x15U,
    // MOT_PACKET_STATUS_ERROR = 0x16U,
    // MOT_PACKET_STATUS_EXT = 0x17U,

    /* Fixed Length */
    MOT_PACKET_STOP_ALL = 0x00U,
    MOT_PACKET_VERSION = 0x21U,
    MOT_PACKET_REBOOT = 0xC1U,
    MOT_PACKET_CALL = 0xC2U,
    // /* Fixed Length */
    // MOT_PACKET_READ_VAR = 0xD1U,            /* Read Single Var */
    // MOT_PACKET_WRITE_VAR = 0xD2U,           /* Write Single Var */

    /*
        Read/Write by VarId:
        Real-Time Variable, NvMemory Parameters, Call functions passing 1 argument
    */
    /* Configurable Length */
    MOT_PACKET_VAR_READ = 0xD3U,         /* Up to 16 uint16_t values *//* Up to 8 uint32_t values */
    MOT_PACKET_VAR_WRITE = 0xD4U,        /* Up to 8 uint16_t values *//* Up to 4 uint32_t values */

    /* Read/Write by Address */
    MOT_PACKET_MEM_READ = 0xDAU,            /* Read Memory Address */
    MOT_PACKET_MEM_WRITE = 0xDBU,           /* Write Memory Address */

    /* Stateful Read/Write */
    MOT_PACKET_DATA_MODE_DATA = 0xDDU,      /* Data Mode Data */
    MOT_PACKET_DATA_MODE_READ = 0xDEU,      /* Stateful NvMemory Read using Address */
    MOT_PACKET_DATA_MODE_WRITE = 0xDFU,     /* Stateful NvMemory Write using Address */
    MOT_PACKET_DATA_MODE_ABORT = MOT_PACKET_SYNC_ABORT,


    /* Extended Id Modes */
    MOT_PACKET_EXT_CMD = 0xE1U,             /* ExtId Batch - Predefined Sequences */
    // MOT_PACKET_READ_ONCE = F1U,
    // MOT_PACKET_WRITE_ONCE = F2U,
    MOT_PACKET_ID_RESERVED_255 = 0xFFU,
    ///Datagram

}
MotPacket_Id_T;

typedef uint8_t checksum_t;

/* 2-Byte Sync Packet */
typedef struct __attribute__((packed)) MotPacket_Sync
{
    uint8_t Start;  /* MOT_PACKET_START_BYTE */
    uint8_t SyncId; /* MotPacket_HeaderId_T */
}
MotPacket_Sync_T;

// typedef struct __attribute__((packed)) MotPacket_Fixed
// {
//     uint8_t Start;      /* MOT_PACKET_START_BYTE */
//     uint8_t FixedId;    /* MotPacket_HeaderId_T */
//     uint16_t Checksum;
//     uint32_t Immediate;
// }
// MotPacket_Fixed_T;

// typedef struct __attribute__((packed)) MotPacket_Header
// {
//     uint8_t Start;      /* MOT_PACKET_START_BYTE */
//     uint8_t Id;         /* MotPacket_HeaderId_T - Cmd / Descriptor of packet contents */
//     uint16_t Checksum;   /* Checksum */
//     uint8_t Length;     /* Packet Length */
//     uint8_t Resv;     /* Packet Length */
//     uint16_t Imm16;
// }
// MotPacket_Header_T;

/*   */
typedef struct __attribute__((packed)) MotPacket_Header
{
    uint8_t Start;      /* MOT_PACKET_START_BYTE */
    uint8_t Id;         /* MotPacket_HeaderId_T - Cmd / Descriptor of packet contents */
    uint8_t Length;     /* Packet Length */
    uint8_t Checksum;   /* Checksum */
}
MotPacket_Header_T;

typedef union  __attribute__((packed)) MotPacket_Packet
{
    struct
    {
        MotPacket_Header_T Header;
        union { uint8_t Payload[MOT_PACKET_PAYLOAD_MAX]; };
    };
    uint8_t Bytes[MOT_PACKET_LENGTH_MAX];
}
MotPacket_T;

static inline uint8_t MotPacket_ParsePayloadLength(const MotPacket_T * p_packet)            { return p_packet->Header.Length - sizeof(MotPacket_Header_T); }
static inline uint8_t MotPacket_ParseTotalLength(const MotPacket_T * p_packet)              { return p_packet->Header.Length; }
static inline uint8_t MotPacket_BuildLength(MotPacket_T * p_packet, uint8_t payloadLength)  { p_packet->Header.Length = payloadLength + sizeof(MotPacket_Header_T); }

// static inline uint8_t MotPacket_ParseTotalLength(const MotPacket_T * p_packet, uint8_t rxCount)
// { return (rxCount > 2U) ? 0U : p_packet->Header.Length; }

/******************************************************************************/
/*! Common Resp */
/******************************************************************************/
typedef struct __attribute__((packed)) MotPacket_StatusResp_Payload
{
    uint16_t ReqState;
    uint16_t Status;
    uint8_t  AppStatus[MOT_PACKET_PAYLOAD_MAX - 4U];
}
MotPacket_StatusResp_Payload_T;

typedef struct MotPacket_StatusResp { MotPacket_Header_T Header; uint16_t StatusResp; } MotPacket_StatusResp_T;

/******************************************************************************/
/*! Ping */
/******************************************************************************/
typedef MotPacket_Sync_T MotPacket_PingReq_T;
typedef MotPacket_Sync_T MotPacket_PingResp_T;

/******************************************************************************/
/*! Version - Static Response */
/******************************************************************************/
typedef MotPacket_Header_T                                                                                                      MotPacket_VersionReq_T;
typedef struct MotPacket_VersionResp_Payload { uint8_t Version[4U]; uint32_t Library; uint32_t Main; uint32_t Board; }      MotPacket_VersionResp_Payload_T;
typedef struct MotPacket_VersionResp { MotPacket_Header_T Header; MotPacket_VersionResp_Payload_T VersionResp; }            MotPacket_VersionResp_T;

/******************************************************************************/
/*! Stop - Stop All */
/******************************************************************************/
typedef MotPacket_Header_T MotPacket_StopReq_T;
typedef struct MotPacket_StopResp_Payload { uint16_t Status; }                                          MotPacket_StopResp_Payload_T;
typedef struct MotPacket_StopResp { MotPacket_Header_T Header; MotPacket_StopResp_Payload_T StopResp; } MotPacket_StopResp_T;

/******************************************************************************/
/*! Call - collect blocking functions */
/******************************************************************************/
typedef struct MotPacket_CallReq_Payload { uint16_t Id; uint16_t Arg; }                                 MotPacket_CallReq_Payload_T;
typedef struct MotPacket_CallResp_Payload { uint16_t Id; uint16_t Status; }                             MotPacket_CallResp_Payload_T;
typedef struct MotPacket_CallReq { MotPacket_Header_T Header; MotPacket_CallReq_Payload_T CallReq; }    MotPacket_CallReq_T;
typedef struct MotPacket_CallResp { MotPacket_Header_T Header; MotPacket_CallResp_Payload_T CallResp; } MotPacket_CallResp_T;

/******************************************************************************/
/*!
    Var
    Read/Write Operations by Var Id, or a function with 1-arg
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Read Vars - flex length extensible
    Req     [Start, Id, Length, Checksum], [IdChecksum, Flags16],   [MotVarIds][16]
    Resp    [Start, Id, Length, Checksum], [IdChecksum, Status16],  [Value16][16]

    Read Var x1
            [MotVarId16][Flags16 = 0]
            [MotVarId16][Status16], [Value16]

    Alt 32-bit
    Req     [Start, Id, Length, Checksum], [IdChecksum, Flags16_Alt],   [MotVarIds][8]
    Resp    [Start, Id, Length, Checksum], [IdChecksum, Status16],      [Value32][8]
*/
/******************************************************************************/
typedef struct MotPacket_VarReadReq_Payload { uint16_t IdChecksum; uint16_t Flags16; uint16_t MotVarIds[16U]; } MotPacket_VarReadReq_Payload_T;
typedef struct MotPacket_VarReadResp_Payload { uint16_t IdChecksum; uint16_t Status16; uint16_t Value16[16U]; } MotPacket_VarReadResp_Payload_T;
typedef struct MotPacket_VarReadReq { MotPacket_Header_T Header; MotPacket_VarReadReq_Payload_T VarReadReq; }    MotPacket_VarReadReq_T;
typedef struct MotPacket_VarReadResp { MotPacket_Header_T Header; MotPacket_VarReadResp_Payload_T VarReadResp; } MotPacket_VarReadResp_T;

/******************************************************************************/
/*!
    Write Vars - extensible flex Up to 8 uint16_t, 4 uint32_t
    Req     [Start, Id, Length, Checksum], [IdChecksum, Flags16],    [MotVarIds, Value16][8]
    Resp    [Start, Id, Length, Checksum], [IdChecksum, Status16],   [VarStatus8][8]

    Write Var x1
        [MotVarId16][Flags16 = 0] [id = 0, Value16]
        [MotVarId16][Status16],

    Alt 32-bit
    Req     [Start, Id, Length, Checksum], [IdChecksum, Flags16],   [MotVarId16, MotVarId16, Value32, Value32][2]
    Resp    [Start, Id, Length, Checksum], [IdChecksum, Status16],  [Status8][4]
*/
/******************************************************************************/
typedef struct MotPacket_VarWriteReq_Payload { uint16_t IdChecksum; uint16_t Flags16; struct { uint16_t MotVarId; uint16_t Value16; } Pairs[8U]; }  MotPacket_VarWriteReq_Payload_T;
typedef struct MotPacket_VarWriteResp_Payload { uint16_t IdChecksum; uint16_t Status16; uint8_t VarStatus[8U]; }                                    MotPacket_VarWriteResp_Payload_T;
typedef struct MotPacket_VarWriteReq { MotPacket_Header_T Header; MotPacket_VarWriteReq_Payload_T VarWriteReq; }       MotPacket_VarWriteReq_T;
typedef struct MotPacket_VarWriteResp { MotPacket_Header_T Header; MotPacket_VarWriteResp_Payload_T VarWriteResp; }    MotPacket_VarWriteResp_T;

/******************************************************************************/
/*!

*/
/******************************************************************************/
/******************************************************************************/
/*! Mem Address Read */
/******************************************************************************/
typedef struct MotPacket_MemReadReq_Payload { uint32_t AddressStart; uint16_t SizeBytes; uint16_t ConfigFlags; }                    MotPacket_MemReadReq_Payload_T;
typedef struct MotPacket_MemReadResp_Payload { uint16_t Checksum; uint16_t Status; uint8_t ByteData[MOT_PACKET_PAYLOAD_MAX - 8U]; } MotPacket_MemReadResp_Payload_T;
typedef struct MotPacket_MemReadReq { MotPacket_Header_T Header; MotPacket_MemReadReq_Payload_T MemReadReq; }        MotPacket_MemReadReq_T;
typedef struct MotPacket_MemReadResp { MotPacket_Header_T Header; MotPacket_MemReadResp_Payload_T MemReadResp; }     MotPacket_MemReadResp_T;

/******************************************************************************/
/*! Mem Address Write */
/******************************************************************************/
typedef struct MotPacket_MemWriteReq_Payload { uint32_t AddressStart; uint16_t SizeBytes; uint16_t ConfigFlags; uint16_t Checksum; uint8_t ByteData[MOT_PACKET_PAYLOAD_MAX - 10U]; }    MotPacket_MemWriteReq_Payload_T;
typedef struct MotPacket_MemWriteResp_Payload { uint16_t State;  uint16_t Status; }                                                                                                     MotPacket_MemWriteResp_Payload_T;
typedef struct MotPacket_MemWriteReq { MotPacket_Header_T Header; MotPacket_MemWriteReq_Payload_T MemWriteReq; }     MotPacket_MemWriteReq_T;
typedef struct MotPacket_MemWriteResp { MotPacket_Header_T Header; MotPacket_MemWriteResp_Payload_T MemWriteResp; }  MotPacket_MemWriteResp_T;


/******************************************************************************/
/*!
    Stateful Sequence
*/
/******************************************************************************/
/******************************************************************************/
/*! Data Mode Read/Write Control/Framing Common */
/******************************************************************************/
typedef struct MotPacket_DataModeReq_Payload { uint32_t AddressStart; uint32_t SizeBytes; uint32_t ConfigFlags; }   MotPacket_DataModeReq_Payload_T;
typedef struct MotPacket_DataModeResp_Payload { uint16_t Status; }                                                  MotPacket_DataModeResp_Payload_T;
typedef struct MotPacket_DataModeReq { MotPacket_Header_T Header; MotPacket_DataModeReq_Payload_T DataModeReq; }    MotPacket_DataModeReq_T;
typedef struct MotPacket_DataModeResp { MotPacket_Header_T Header; MotPacket_DataModeResp_Payload_T DataModeResp; } MotPacket_DataModeResp_T;

/******************************************************************************/
/*! Data Mode Raw Data Packet */
/******************************************************************************/
typedef struct MotPacket_DataMode_Payload { uint16_t Checksum; uint16_t Sequence; uint8_t ByteData[MOT_PACKET_PAYLOAD_MAX - 4U]; }  MotPacket_DataMode_Payload_T;
typedef struct MotPacket_DataMode { MotPacket_Header_T Header; MotPacket_DataMode_Payload_T DataMode; }                             MotPacket_DataMode_T;


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
/******************************************************************************/
/*! Common */
/******************************************************************************/
extern bool MotPacket_CheckChecksum(const MotPacket_T * p_packet);
extern uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId);

/******************************************************************************/
/*! Ctrlr side */
/******************************************************************************/
extern uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket);
extern uint8_t MotPacket_VersionResp_Build(MotPacket_VersionResp_T * p_respPacket, uint32_t library, uint32_t main, uint32_t board);
extern uint8_t MotPacket_StopResp_Build(MotPacket_StopResp_T * p_respPacket, uint16_t status);
extern uint8_t MotPacket_CallResp_Build(MotPacket_CallResp_T * p_respPacket, uint16_t id, uint16_t status);

extern uint8_t MotPacket_VarReadReq_ParseVarIdCount(const MotPacket_VarReadReq_T * p_reqPacket);
extern uint16_t MotPacket_VarReadReq_ParseVarId(const MotPacket_VarReadReq_T * p_reqPacket, uint8_t index);
extern void MotPacket_VarReadResp_BuildVarValue(MotPacket_VarReadResp_T * p_respPacket, uint8_t index, uint16_t value);
extern void MotPacket_VarReadResp_BuildInnerHeader(MotPacket_VarReadResp_T * p_respPacket, uint16_t idChecksum, uint16_t status16);
extern uint8_t MotPacket_VarReadResp_BuildHeader(MotPacket_VarReadResp_T * p_respPacket, uint8_t varsCount);

extern uint8_t MotPacket_VarWriteReq_ParseVarCount(const MotPacket_VarWriteReq_T * p_reqPacket);
extern uint16_t MotPacket_VarWriteReq_ParseVarId(const MotPacket_VarWriteReq_T * p_reqPacket, uint8_t index);
extern uint16_t MotPacket_VarWriteReq_ParseVarValue(const MotPacket_VarWriteReq_T * p_reqPacket, uint8_t index);
extern void MotPacket_VarWriteResp_BuildVarStatus(MotPacket_VarWriteResp_T * p_respPacket, uint8_t index, uint16_t status);
extern void MotPacket_VarWriteResp_BuildInnerHeader(MotPacket_VarWriteResp_T * p_respPacket, uint16_t idChecksum, uint16_t status16);
extern uint8_t MotPacket_VarWriteResp_BuildHeader(MotPacket_VarWriteResp_T * p_respPacket, uint8_t varsCount);

// extern uint32_t MotPacket_ReadDataReq_ParseAddress(const MotPacket_DataModeReq_T * p_reqPacket);
// extern uint32_t MotPacket_ReadDataReq_ParseSize(const MotPacket_DataModeReq_T * p_reqPacket);
// extern uint32_t MotPacket_WriteDataReq_ParseAddress(const MotPacket_DataModeReq_T * p_reqPacket);
// extern uint32_t MotPacket_WriteDataReq_ParseSize(const MotPacket_DataModeReq_T * p_reqPacket);
extern void MotPacket_DataModeReq_Parse(const MotPacket_DataModeReq_T * p_reqPacket, uint32_t * p_addressStart, uint32_t * p_sizeBytes);
extern uint32_t MotPacket_DataModReq_ParseAddress(const MotPacket_DataModeReq_T * p_reqPacket);
extern uint32_t MotPacket_DataModReq_ParseSize(const MotPacket_DataModeReq_T * p_reqPacket);
extern uint8_t MotPacket_DataModeReadResp_Build(MotPacket_DataModeResp_T * p_respPacket, uint16_t status);
extern uint8_t MotPacket_DataModeWriteResp_Build(MotPacket_DataModeResp_T * p_respPacket, uint16_t status);

extern void MotPacket_DataRead_BuildStatus(MotPacket_DataMode_T * p_dataPacket, uint16_t sequence, uint16_t checksum);
extern uint8_t MotPacket_DataRead_BuildData(MotPacket_DataMode_T * p_dataPacket, const uint8_t * p_address, uint8_t sizeData);
extern uint16_t MotPacket_DataWrite_ParseChecksum(const MotPacket_DataMode_T * p_dataPacket);
extern uint8_t MotPacket_DataWrite_ParseDataSize(const MotPacket_DataMode_T * p_dataPacket);
extern const uint8_t * MotPacket_DataWrite_ParsePtrData(const MotPacket_DataMode_T * p_dataPacket);


/******************************************************************************/
/*! Cmdr side */
/******************************************************************************/
// extern uint8_t MotPacket_GetRespLength(MotPacket_HeaderId_T headerId);
// extern uint8_t MotPacket_PingReq_GetRespLength(void);
// extern uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket);
// extern MotPacket_HeaderId_T MotPacket_PingResp_Parse(const MotPacket_PingResp_T * p_respPacket);

// // extern uint8_t MotPacket_StopReq_GetRespLength(void);
// extern uint8_t MotPacket_StopReq_Build(MotPacket_StopReq_T * p_reqPacket);
// extern MotPacket_HeaderStatus_T MotPacket_StopResp_Parse(const MotPacket_StopResp_T * p_respPacket);

// // extern uint8_t MotPacket_ReadVarReq_GetRespLength(void);
// extern uint8_t MotPacket_ReadVarReq_Build(MotPacket_ReadVarReq_T * p_reqPacket, uint16_t motVarId);
// extern uint32_t MotPacket_ReadVarResp_Parse(const MotPacket_ReadVarResp_T * p_respPacket, uint32_t * p_value);

// // extern uint8_t MotPacket_WriteVarReq_GetRespLength(void);
// extern uint8_t MotPacket_WriteVarReq_Build(MotPacket_WriteVarReq_T * p_reqPacket, uint16_t motVarId, uint32_t value);
// extern MotPacket_HeaderStatus_T MotPacket_WriteVarResp_Parse(const MotPacket_WriteVarResp_T * p_respPacket);

// // extern uint8_t MotPacket_CallReq_GetRespLength(void);
// extern uint8_t MotPacket_CallResp_Build(MotPacket_CallResp_T * p_respPacket, MotPacket_HeaderStatus_T status);
// extern MotPacket_HeaderStatus_T MotPacket_CallReq_Parse(const MotPacket_CallReq_T * p_respPacket);

// extern uint8_t MotPacket_BatchReq_GetRespLength(MotPacket_ExtId_T ExtId);

#endif

// /******************************************************************************/
// /*!
//     Read Var by Id - flex length extendable
//     Req     [Start, Id, Length, Check8], [MotVarId16]
//     Resp    [Start, Id, Length, Check8], [MotVarId16][Status16], [Value32]
// */
// /******************************************************************************/
// typedef struct MotPacket_ReadVarReq_Payload { uint16_t MotVarId; }                                                  MotPacket_ReadVarReq_Payload_T;
// typedef struct MotPacket_ReadVarResp_Payload { uint16_t MotVarId; uint16_t Status; uint32_t Value32; }              MotPacket_ReadVarResp_Payload_T;
// typedef struct MotPacket_ReadVarReq { MotPacket_Header_T Header; MotPacket_ReadVarReq_Payload_T ReadVarReq; }       MotPacket_ReadVarReq_T;
// typedef struct MotPacket_ReadVarResp { MotPacket_Header_T Header; MotPacket_ReadVarResp_Payload_T ReadVarResp; }    MotPacket_ReadVarResp_T;

// /******************************************************************************/
// /*!
//     Write Var by Id
//     Req     [Start, Id, Length, Check8], [MotVarId16][Flags16], [Value32]
//     Resp    [Start, Id, Length, Check8], [MotVarId16][Status16],
// */
// /******************************************************************************/
// typedef struct MotPacket_WriteVarReq_Payload { uint16_t MotVarId; uint16_t Flags; uint32_t Value32; }               MotPacket_WriteVarReq_Payload_T;
// typedef struct MotPacket_WriteVarResp_Payload { uint16_t MotVarId; uint16_t Status; }                               MotPacket_WriteVarResp_Payload_T;
// typedef struct MotPacket_WriteVarReq { MotPacket_Header_T Header; MotPacket_WriteVarReq_Payload_T WriteVarReq; }    MotPacket_WriteVarReq_T;
// typedef struct MotPacket_WriteVarResp { MotPacket_Header_T Header; MotPacket_WriteVarResp_Payload_T WriteVarResp; } MotPacket_WriteVarResp_T;


// extern uint16_t MotPacket_ReadVarReq_ParseVarId(const MotPacket_ReadVarReq_T * p_reqPacket);
// extern uint8_t MotPacket_ReadVarResp_Build(MotPacket_ReadVarResp_T * p_respPacket, uint32_t value);

// extern uint16_t MotPacket_WriteVarReq_ParseVarId(const MotPacket_WriteVarReq_T * p_reqPacket);
// extern uint32_t MotPacket_WriteVarReq_ParseVarValue(const MotPacket_WriteVarReq_T * p_reqPacket);
// extern uint8_t MotPacket_WriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, uint16_t status);