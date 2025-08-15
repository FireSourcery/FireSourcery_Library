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

*/
/******************************************************************************/
#ifndef MOT_PACKET_H
#define MOT_PACKET_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MOT_PACKET_VERSION_OPT          (255U)
#define MOT_PACKET_VERSION_MAJOR        (24U)
#define MOT_PACKET_VERSION_MINOR        (16U)
#define MOT_PACKET_VERSION_FIX          (5U)
#define MOT_PACKET_VERSION_WORD32       ((MOT_PACKET_VERSION_OPT << 24U) | (MOT_PACKET_VERSION_MAJOR << 16U) | (MOT_PACKET_VERSION_MINOR << 8U) | (MOT_PACKET_VERSION_FIX))

#define MOT_PACKET_START_BYTE           (0xA5U)

#define MOT_PACKET_LENGTH_MAX           (40U) /*  */
#define MOT_PACKET_LENGTH_MIN           (2U) /* sizeof(MotPacket_Sync_T) */
#define MOT_PACKET_HEADER_LENGTH        (8U)
#define MOT_PACKET_PAYLOAD_LENGTH_MAX   (32U)

#define MOT_PACKET_ID_INDEX             (1U)
#define MOT_PACKET_LENGTH_INDEX         (4U)

#define MOT_PACKET_PACKED __attribute__((packed))

#if (__STDC_VERSION__ >= 202311L)
#define ENUM8_T (: uint8_t)
#else
#define ENUM8_T
#endif

typedef uint16_t checksum_t;

/*
    Packet and correspondence type. Per unique packet structure/parsing/processing pattern
    May process directly as cmd, or lead extended cmd id
    Essentially the packets "OpCode"
*/
// typedef enum MotProtocol_Id
typedef enum MOT_PACKET_PACKED MotPacket_Id ENUM8_T
{
    /*
        2-Byte Id Packets
        Response Packet must use different ID - Length compare determined by Id
    */
    MOT_PACKET_PING = 0xA0U,            /* */
    // MOT_PACKET_PING_RESP = 0xA1U,    /* */
    MOT_PACKET_SYNC_ACK = 0xA2U,
    MOT_PACKET_SYNC_NACK = 0xA3U,
    MOT_PACKET_SYNC_ABORT = 0xA4U,
    MOT_PACKET_SYNC_RESV = MOT_PACKET_START_BYTE,
    // MOT_PACKET_FLOW_ = ,
    MOT_PACKET_PING_ALT = 0xAAU,
    MOT_PACKET_PING_BOOT = 0xABU,
    // MOT_PACKET_PING_WATCHDOG = 0xAAU,

    /* Fixed Length */
    MOT_PACKET_STOP_ALL = 0x00U,
    MOT_PACKET_VERSION = 0x01U,
    // MOT_PACKET_REF = 0x02U,

    MOT_PACKET_CALL = 0xC0U,
    // MOT_PACKET_CALL_ADDRESS = 0xCAU,

    // MOT_PACKET_FIXED_VAR_READ = 0xB1U,     /* Read Single Var32 */
    // MOT_PACKET_FIXED_VAR_WRITE = 0xB2U,    /* Write Single Var32 */

    /* Variable Length */
    /*
        Read/Write by VarId:
        Real-Time Variable, NvMemory Config, Call functions passing 1 argument
    */
    MOT_PACKET_VAR_READ = 0xB3U,         /* Up to 16 uint16_t values */ /* Up to 8 uint32_t values */
    MOT_PACKET_VAR_WRITE = 0xB4U,        /* Up to 8 uint16_t values */ /* Up to 4 uint32_t values */
    // MOT_PACKET_VAR32_READ = 0xB5U,
    // MOT_PACKET_VAR32_WRITE = 0xB6U,

    /* Read/Write by Address */
    MOT_PACKET_MEM_READ = 0xD1U,            /* Read Memory Address */
    MOT_PACKET_MEM_WRITE = 0xD2U,           /* Write Memory Address */

    /* Stateful Read/Write */
    MOT_PACKET_DATA_MODE_READ = 0xDAU,      /* Stateful NvMemory Read using Address */
    MOT_PACKET_DATA_MODE_WRITE = 0xDBU,     /* Stateful NvMemory Write using Address */
    MOT_PACKET_DATA_MODE_DATA = 0xDDU,      /* Data Mode Data */
    MOT_PACKET_DATA_MODE_ABORT = MOT_PACKET_SYNC_ABORT,

    _MOT_PACKET_ID_END_255 = 0xFFU,
    /// Datagram todo
}
MotPacket_Id_T;


/* 2-Byte Sync Packet */
// typedef struct MOT_PACKET_PACKED MotPacket_ControlChar
// typedef struct MOT_PACKET_PACKED MotPacket_Min
// typedef struct MOT_PACKET_PACKED MotPacket_HeaderId
typedef struct MOT_PACKET_PACKED MotPacket_Sync
{
    uint8_t Start;  /* MOT_PACKET_START_BYTE */
    uint8_t SyncId; /* MotPacket_Id_T */
}
MotPacket_Sync_T;

// generic via union
typedef struct MOT_PACKET_PACKED MotPacket_Header
{
    uint8_t Start;  /* MOT_PACKET_START_BYTE */
    uint8_t Id;     /* MotPacket_Id_T */
    uint16_t Checksum;
    union
    {
        struct { uint8_t Length; uint8_t Sequence; uint8_t Flags0; uint8_t Flags1; };
        struct { uint16_t SubId; uint16_t Flags; };  /*  */
        struct { uint8_t SourceId; uint16_t Status; };
        struct { uint16_t Lower16; uint16_t Upper16; };
        uint32_t FlexValue;
    };
}
MotPacket_Header_T;

// 4-byte, checksum at end
// typedef struct MOT_PACKET_PACKED MotPacket_Header
// {
//     uint8_t Start;  /* MOT_PACKET_START_BYTE */
//     uint8_t Id;     /* MotPacket_Id_T */
//     union
//     {
//         struct { uint8_t Length; uint8_t Sequence; };
//         uint16_t Flags;
//         uint16_t SubId;
//         uint16_t Status;
//     };
// }
// MotPacket_Header_T;

typedef union MOT_PACKET_PACKED MotPacket
{
    struct
    {
        MotPacket_Header_T Header;
        // struct { uint16_t Source; uint16_t Dest; } Ext;
        uint8_t Payload[MOT_PACKET_LENGTH_MAX - sizeof(MotPacket_Header_T)];
    };
    uint8_t Bytes[MOT_PACKET_LENGTH_MAX];
}
MotPacket_T;


// todo remove Packet wrap. keep payload
// pass payload separately eliminates need for combined packet + payload struct
// extern uint8_t MotPacket_VersionResp1_Build(MotPacket_VersionResp_T * p_payload, MotPacket_T * p_header, uint32_t library, uint32_t firmware, uint32_t board);

/******************************************************************************/
/*!
    Fixed Length
*/
/******************************************************************************/
/******************************************************************************/
/*
    For simplicity Packet Payload should not contain meta/format information
*/
/******************************************************************************/
/******************************************************************************/
/*! Common Resp */
/******************************************************************************/
// typedef struct  MotPacket_StatusResp { MotPacket_HeaderFixed_T Header; uint16_t StatusResp; } MotPacket_StatusResp_T;

/******************************************************************************/
/*! Ping */
/******************************************************************************/
typedef MotPacket_Sync_T MotPacket_PingReq_T;
typedef MotPacket_Sync_T MotPacket_PingResp_T;

/******************************************************************************/
/*! Version - Static Response */
/******************************************************************************/
typedef struct MotPacket_VersionReq_Payload {}                                                                              MotPacket_VersionReq_Payload_T;
typedef struct MotPacket_VersionResp_Payload { uint32_t Protocol; uint32_t Library; uint32_t Firmware; }                    MotPacket_VersionResp_Payload_T;
typedef MotPacket_Header_T                                                                                                  MotPacket_VersionReq_T;
typedef struct MotPacket_VersionResp { MotPacket_Header_T Header; MotPacket_VersionResp_Payload_T VersionResp; }            MotPacket_VersionResp_T;

/* fixed 8 */
typedef struct MotPacket_VersionFlexResp_Payload { uint32_t Versions[MOT_PACKET_PAYLOAD_LENGTH_MAX / sizeof(uint32_t)]; }   MotPacket_VersionFlexResp_Payload_T;
typedef struct MotPacket_VersionFlexResp { MotPacket_Header_T Header; MotPacket_VersionFlexResp_Payload_T VersionResp; }    MotPacket_VersionFlexResp_T;
// typedef struct MotPacket_SoftwareVersionResp_Payload { uint32_t Protocol; uint32_t Library; uint32_t Firmware; }                    MotPacket_VersionResp_Payload_T;

/******************************************************************************/
/*! Stop - Stop All */
/******************************************************************************/
typedef struct MotPacket_StopReq_Payload {}                                                                     MotPacket_StopReq_Payload_T;
typedef struct MotPacket_StopResp_Payload { uint16_t Status; }                                                  MotPacket_StopResp_Payload_T;
typedef MotPacket_Header_T                                                                                      MotPacket_StopReq_T;
typedef struct MotPacket_StopResp { MotPacket_Header_T Header; MotPacket_StopResp_Payload_T StopResp; }         MotPacket_StopResp_T;

/******************************************************************************/
/*! Call - Arbitrary Functions */
/******************************************************************************/
/* Id/Address */
typedef struct MotPacket_CallReq_Payload { uint32_t Id; uint32_t Arg; }                                     MotPacket_CallReq_Payload_T;
typedef struct MotPacket_CallResp_Payload { uint32_t Id; uint16_t Status; }                                 MotPacket_CallResp_Payload_T;

typedef struct MotPacket_CallReq { MotPacket_Header_T Header; MotPacket_CallReq_Payload_T CallReq; }        MotPacket_CallReq_T;
typedef struct MotPacket_CallResp { MotPacket_Header_T Header; MotPacket_CallResp_Payload_T CallResp; }     MotPacket_CallResp_T;

// multiple parameters
// typedef struct MotPacket_CallArgVReq_Payload { uint32_t Id; uint16_t Flags; uint16_t ArgC; uint32_t ArgV[MOT_PACKET_PAYLOAD_LENGTH_MAX - 8U]; }   MotPacket_CallReq_Payload_T;
// typedef struct MotPacket_CallArgVResp_Payload { uint32_t Id; uint16_t Flags; uint16_t Status; }                                                   MotPacket_CallResp_Payload_T;

/******************************************************************************/
/*!
    Flex Length - Use interface functions for length
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Read/Write Operations by Var Id, or a function with 1-arg
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Read Vars - flex length extensible
    Req     [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Flags16],   [MotVarIds][16]
    Resp    [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Status16],  [Value16][16]

    Alt 32-bit
    Req     [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Flags16_Alt],   [MotVarIds][8]
    Resp    [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Status16],      [Value32][8]
*/
/******************************************************************************/
typedef struct MotPacket_VarReadReq_Payload { uint16_t MotVarIds[16U]; }                                            MotPacket_VarReadReq_Payload_T;
typedef struct MotPacket_VarReadResp_Payload { uint16_t Value16[16U]; }                                             MotPacket_VarReadResp_Payload_T;
typedef struct MotPacket_VarReadReq { MotPacket_Header_T Header; MotPacket_VarReadReq_Payload_T VarReadReq; }       MotPacket_VarReadReq_T;
typedef struct MotPacket_VarReadResp { MotPacket_Header_T Header; MotPacket_VarReadResp_Payload_T VarReadResp; }    MotPacket_VarReadResp_T;

/******************************************************************************/
/*!
    Write Vars - extensible flex Up to 8 uint16_t, 4 uint32_t
    Req     [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Flags16],    [MotVarIds, Value16][8]
    Resp    [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Status16],   [VarStatus8][8]

    Alt 32-bit
    Req     [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Flags16],   [MotVarId16, Value32][5]
    Resp    [Start, Id, Checksum[2]], [Length, IdFlags/Meta, Status16],  [Status8][4]
*/
/******************************************************************************/
typedef struct MotPacket_VarWriteReq_Payload { struct { uint16_t MotVarId; uint16_t Value16; } Pairs[8U]; }             MotPacket_VarWriteReq_Payload_T;
typedef struct MotPacket_VarWriteResp_Payload { uint8_t VarStatus[8U]; }                                                MotPacket_VarWriteResp_Payload_T;
typedef struct MotPacket_VarWriteReq { MotPacket_Header_T Header; MotPacket_VarWriteReq_Payload_T VarWriteReq; }        MotPacket_VarWriteReq_T;
typedef struct MotPacket_VarWriteResp { MotPacket_Header_T Header; MotPacket_VarWriteResp_Payload_T VarWriteResp; }     MotPacket_VarWriteResp_T;


/******************************************************************************/
/*! Mem Address Read */
/******************************************************************************/
typedef struct MotPacket_MemReadReq_Payload { uint32_t Address; uint8_t Size; uint8_t Resv; uint16_t Config; }      MotPacket_MemReadReq_Payload_T;
typedef struct MotPacket_MemReadResp_Payload { uint8_t ByteData[MOT_PACKET_PAYLOAD_LENGTH_MAX]; }                   MotPacket_MemReadResp_Payload_T;
typedef struct MotPacket_MemReadReq { MotPacket_Header_T Header; MotPacket_MemReadReq_Payload_T MemReadReq; }       MotPacket_MemReadReq_T;
typedef struct MotPacket_MemReadResp { MotPacket_Header_T Header; MotPacket_MemReadResp_Payload_T MemReadResp; }    MotPacket_MemReadResp_T;

/******************************************************************************/
/*! Mem Address Write */
/* Alternatively move Size and Flags to header */
/******************************************************************************/
#define MOT_PACKET_MEM_WRITE_SIZE_MAX 16U /* 24 available */
typedef struct MotPacket_MemWriteReq_Payload { uint32_t Address; uint8_t Size; uint8_t Resv; uint16_t Config; uint8_t ByteData[MOT_PACKET_MEM_WRITE_SIZE_MAX]; } MotPacket_MemWriteReq_Payload_T;
typedef struct MotPacket_MemWriteResp_Payload { uint16_t Status; }                                                                          MotPacket_MemWriteResp_Payload_T;
typedef struct MotPacket_MemWriteReq { MotPacket_Header_T Header; MotPacket_MemWriteReq_Payload_T MemWriteReq; }                            MotPacket_MemWriteReq_T;
typedef struct MotPacket_MemWriteResp { MotPacket_Header_T Header; MotPacket_MemWriteResp_Payload_T MemWriteResp; }                         MotPacket_MemWriteResp_T;


/******************************************************************************/
/*!
    Stateful Sequence
*/
/******************************************************************************/
/******************************************************************************/
/*! Data Mode Read/Write Control/Framing Common */
/******************************************************************************/
typedef struct MotPacket_DataModeReq_Payload { uint32_t AddressStart; uint32_t SizeBytes; uint32_t Config; }            MotPacket_DataModeReq_Payload_T;
typedef struct MotPacket_DataModeResp_Payload { uint16_t Status; }                                                      MotPacket_DataModeResp_Payload_T;
typedef struct MotPacket_DataModeReq { MotPacket_Header_T Header; MotPacket_DataModeReq_Payload_T DataModeReq; }        MotPacket_DataModeReq_T;
typedef struct MotPacket_DataModeResp { MotPacket_Header_T Header; MotPacket_DataModeResp_Payload_T DataModeResp; }     MotPacket_DataModeResp_T;

/******************************************************************************/
/*! Data Mode Raw Data Packet */
/******************************************************************************/
typedef struct MotPacket_DataMode_Payload { uint8_t ByteData[MOT_PACKET_PAYLOAD_LENGTH_MAX]; }              MotPacket_DataMode_Payload_T;
typedef struct MotPacket_DataMode { MotPacket_Header_T Header; MotPacket_DataMode_Payload_T DataMode; }     MotPacket_DataMode_T;

/******************************************************************************/
/*! Ctrlr side */
/******************************************************************************/
/******************************************************************************/
/*! Common */
/******************************************************************************/
static inline uint8_t MotPacket_ParsePayloadLength(const MotPacket_T * p_packet)  { return p_packet->Header.Length - sizeof(MotPacket_Header_T); }
static inline uint8_t MotPacket_ParseTotalLength(const MotPacket_T * p_packet)    { return p_packet->Header.Length; }
static inline void MotPacket_BuildPayloadLength(MotPacket_T * p_packet, uint8_t payloadLength)    { p_packet->Header.Length = payloadLength + sizeof(MotPacket_Header_T); }
static inline void MotPacket_BuildTotalLength(MotPacket_T * p_packet, uint8_t totalLength)        { p_packet->Header.Length = totalLength; }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern bool MotPacket_ProcChecksum(const MotPacket_T * p_packet, size_t totalSize);
extern uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId);
extern uint8_t MotPacket_BuildHeader(MotPacket_T * p_packet, MotPacket_Id_T headerId, uint8_t payloadLength);

extern uint8_t MotPacket_PingResp_Build(MotPacket_PingResp_T * p_respPacket, MotPacket_Id_T syncId);
extern uint8_t MotPacket_VersionResp_Build(MotPacket_VersionResp_T * p_respPacket, uint32_t firmware);
extern uint8_t MotPacket_StopResp_Build(MotPacket_StopResp_T * p_respPacket, uint16_t status);
extern uint8_t MotPacket_CallResp_Build(MotPacket_CallResp_T * p_respPacket, uint32_t id, uint16_t status);

// todo flex length build cast payload pointer instead
extern uint8_t MotPacket_VarReadReq_ParseVarIdCount(const MotPacket_VarReadReq_T * p_reqPacket);
extern uint16_t MotPacket_VarReadReq_ParseVarId(const MotPacket_VarReadReq_T * p_reqPacket, uint8_t index);
extern void MotPacket_VarReadResp_BuildVarValue(MotPacket_VarReadResp_T * p_respPacket, uint8_t index, uint16_t value);
extern uint8_t MotPacket_VarReadResp_BuildHeader(MotPacket_VarReadResp_T * p_respPacket, uint8_t varsCount);
// extern void MotPacket_VarReadResp_BuildMeta(MotPacket_VarReadResp_T * p_respPacket, uint16_t idChecksum, uint16_t status16);

extern uint8_t MotPacket_VarWriteReq_ParseVarCount(const MotPacket_VarWriteReq_T * p_reqPacket);
extern uint16_t MotPacket_VarWriteReq_ParseVarId(const MotPacket_VarWriteReq_T * p_reqPacket, uint8_t index);
extern uint16_t MotPacket_VarWriteReq_ParseVarValue(const MotPacket_VarWriteReq_T * p_reqPacket, uint8_t index);
extern void MotPacket_VarWriteResp_BuildVarStatus(MotPacket_VarWriteResp_T * p_respPacket, uint8_t index, uint16_t status);
extern uint8_t MotPacket_VarWriteResp_BuildHeader(MotPacket_VarWriteResp_T * p_respPacket, uint8_t varsCount);
// extern void MotPacket_VarWriteResp_BuildMeta(MotPacket_VarWriteResp_T * p_respPacket, uint16_t idChecksum, uint16_t status16);

extern uint8_t MotPacket_MemWriteResp_Build(MotPacket_MemWriteResp_T * p_respPacket, uint16_t status);
extern uint8_t MotPacket_MemReadResp_BuildHeader(MotPacket_MemReadResp_T * p_respPacket, uint8_t size, uint16_t status);
extern uint8_t MotPacket_MemReadResp_Build(MotPacket_MemReadResp_T * p_respPacket, const uint8_t * p_data, uint8_t size, uint16_t status);

extern uint32_t MotPacket_DataModeReq_ParseAddress(const MotPacket_DataModeReq_T * p_reqPacket);
extern uint32_t MotPacket_DataModeReq_ParseSize(const MotPacket_DataModeReq_T * p_reqPacket);
extern uint8_t MotPacket_DataModeReadResp_Build(MotPacket_DataModeResp_T * p_respPacket, uint16_t status);
extern uint8_t MotPacket_DataModeWriteResp_Build(MotPacket_DataModeResp_T * p_respPacket, uint16_t status);

extern void MotPacket_DataRead_BuildStatus(MotPacket_DataMode_T * p_dataPacket, uint16_t sequence, uint16_t checksum);
extern uint8_t MotPacket_ByteData_Build(MotPacket_DataMode_T * p_dataPacket, const uint8_t * p_data, uint8_t size);

extern const uint8_t * MotPacket_ByteData_ParsePtrData(const MotPacket_DataMode_T * p_dataPacket);
extern uint8_t MotPacket_ByteData_ParseSize(const MotPacket_DataMode_T * p_dataPacket);

/******************************************************************************/
/*!
    Read Var by Id
    Req     [Start, Id, Checksum[2]],
    Resp    [Start, Id, Checksum[2]],

        Read Var n
            Req  [IdChecksum], [MotVarIds][16]
            Resp [IdChecksum, Status16], [Value16][16]
        Read Var x1
            [MotVarId]
            [MotVarId, Status16], [Value16]
*/
/******************************************************************************/
// typedef struct MotPacket_FixedVarReadReq_Payload { uint16_t MotVarId; }                                                         MotPacket_FixedVarReadReq_Payload_T;
// typedef struct MotPacket_FixedVarReadResp_Payload { uint16_t MotVarId; uint16_t Status; uint32_t Value32; }                     MotPacket_FixedVarReadResp_Payload_T;
// typedef struct MotPacket_FixedVarReadReq { MotPacket_HeaderFixed_T Header; MotPacket_FixedVarReadReq_Payload_T Payload; }       MotPacket_FixedVarReadReq_T;
// typedef struct MotPacket_FixedVarReadResp { MotPacket_HeaderFixed_T Header; MotPacket_FixedVarReadResp_Payload_T Payload; }     MotPacket_FixedVarReadResp_T;

/******************************************************************************/
/*!
    Write Var by Id
    Req     [Start, Id, Checksum[2]],
    Resp    [Start, Id, Checksum[2]],

    Write Var n
        [IdChecksum, ValueChecksum/Resv], [MotVarIds, Value16][8]
        [IdChecksum, Status16], [VarStatus8][8]
    Write Var x1
        [MotVarId][Value]
        [MotVarId][Status16],
*/
/******************************************************************************/
// typedef struct MotPacket_WriteVarReq_Payload { uint16_t MotVarId; uint16_t Flags; uint32_t Value32; }                   MotPacket_WriteVarReq_Payload_T;
// typedef struct MotPacket_WriteVarResp_Payload { uint16_t MotVarId; uint16_t Status; }                                   MotPacket_WriteVarResp_Payload_T;
// typedef struct MotPacket_WriteVarReq { MotPacket_HeaderFixed_T Header; MotPacket_WriteVarReq_Payload_T Payload; }       MotPacket_WriteVarReq_T;
// typedef struct MotPacket_WriteVarResp { MotPacket_HeaderFixed_T Header; MotPacket_WriteVarResp_Payload_T Payload; }     MotPacket_WriteVarResp_T;

// extern uint16_t MotPacket_FixedVarReadReq_ParseVarId(const MotPacket_FixedVarReadReq_T * p_reqPacket);
// extern uint8_t MotPacket_FixedVarReadResp_Build(MotPacket_FixedVarReadResp_T * p_respPacket, uint32_t value);
// extern uint16_t MotPacket_FixedWriteVarReq_ParseVarId(const MotPacket_WriteVarReq_T * p_reqPacket);
// extern uint32_t MotPacket_FixedWriteVarReq_ParseVarValue(const MotPacket_WriteVarReq_T * p_reqPacket);
// extern uint8_t MotPacket_FixedWriteVarResp_Build(MotPacket_WriteVarResp_T * p_respPacket, uint16_t status);


/******************************************************************************/
/*! Cmdr side */
/******************************************************************************/
// extern uint8_t MotPacket_GetRespLength(MotPacket_Id_T headerId);
// extern uint8_t MotPacket_PingReq_GetRespLength(void);
// extern uint8_t MotPacket_PingReq_Build(MotPacket_PingReq_T * p_reqPacket);
// extern MotPacket_Id_T MotPacket_PingResp_Parse(const MotPacket_PingResp_T * p_respPacket);

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


#endif

