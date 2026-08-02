#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>

#define MOT_PACKET_VERSION_OPT          (255U)
#define MOT_PACKET_VERSION_MAJOR        (26U)
#define MOT_PACKET_VERSION_MINOR        (7U)
#define MOT_PACKET_VERSION_FIX          (13U)
#define MOT_PACKET_VERSION_WORD32       ((MOT_PACKET_VERSION_OPT << 24U) | (MOT_PACKET_VERSION_MAJOR << 16U) | (MOT_PACKET_VERSION_MINOR << 8U) | (MOT_PACKET_VERSION_FIX))

#define MOT_PACKET_START_BYTE           (0xA5U) /* 165 */

#define MOT_PACKET_LENGTH_MAX           (40U) /*  */
#define MOT_PACKET_LENGTH_MIN           (4U)  /* sizeof(MotPacket_Sync_T) */
#define MOT_PACKET_HEADER_LENGTH        (8U)
#define MOT_PACKET_PAYLOAD_LENGTH_MAX   (32U)

#define MOT_PACKET_ID_INDEX             (1U)
#define MOT_PACKET_LENGTH_INDEX         (2U)

#define MOT_PACKET_PACKED __attribute__((packed))

// #if (__STDC_VERSION__ >= 202311L)
// #define ENUM8_T (: uint8_t)
// #else
#define ENUM8_T
// #endif

typedef uint16_t checksum_t;

/*
    Packet and correspondence type.
    Per unique packet structure/parsing/processing pattern
    May process directly as cmd, or lead extended cmd id
    Effectively the packets "OpCode"
*/
typedef enum MotPacket_Id ENUM8_T
{
    /*
        4-Byte Id Packets
        Response Packet must use different ID - Length compare determined by Id
    */
    MOT_PACKET_PING = 0xA0U,            /* */
    MOT_PACKET_PING_RESP = 0xA1U,       /* */
    MOT_PACKET_SYNC_ACK = 0xA2U,
    MOT_PACKET_SYNC_NACK = 0xA3U,
    MOT_PACKET_SYNC_ABORT = 0xA4U,
    MOT_PACKET_SYNC_RESV = MOT_PACKET_START_BYTE,
    // MOT_PACKET_FLOW_CONTROL = ,
    // alternatively as subId
    MOT_PACKET_PING_ALT = 0xAAU,
    MOT_PACKET_PING_BOOT = 0xABU,
    // MOT_PACKET_PING_WATCHDOG = 0xAAU,

    /* Fixed Length */
    MOT_PACKET_STOP_ALL = 0x00U,
    MOT_PACKET_VERSION = 0x01U,

    MOT_PACKET_CALL = 0xC0U,

    MOT_PACKET_FIXED_VAR_READ = 0xB1U,     /* Read Single Var32 */
    MOT_PACKET_FIXED_VAR_WRITE = 0xB2U,    /* Write Single Var32 */

    /* Variable Length */
    /*
        Read/Write by VarId - Field-like Access:
        Real-Time Variable, NvMemory Config, Call functions passing 1 argument
    */
    MOT_PACKET_VAR_READ = 0xB3U,        /* Up to 16 uint16_t values */
    MOT_PACKET_VAR_WRITE = 0xB4U,       /* Up to 8 uint16_t values */
    MOT_PACKET_VAR32_READ = 0xB5U,      /* Up to 8 uint32_t values */
    MOT_PACKET_VAR32_WRITE = 0xB6U,     /* Up to 4 uint32_t values */

    /* Read/Write by Address */
    MOT_PACKET_MEM_READ = 0xD1U,            /* Read Memory Address */
    MOT_PACKET_MEM_WRITE = 0xD2U,           /* Write Memory Address */

    /* Stateful Read/Write */
    MOT_PACKET_DATA_MODE_READ = 0xDAU,      /* Stateful NvMemory Read using Address */
    MOT_PACKET_DATA_MODE_WRITE = 0xDBU,     /* Stateful NvMemory Write using Address */
    MOT_PACKET_DATA_MODE_ERASE = 0xDCU,     /* Stateful NvMemory Erase using Address */
    MOT_PACKET_DATA_MODE_DATA = 0xDDU,      /* Data Mode Data */
    MOT_PACKET_DATA_MODE_ABORT = MOT_PACKET_SYNC_ABORT,

    _MOT_PACKET_ID_END_255 = 0xFFU,
}
MotPacket_Id_T;

/*
    4-Byte Sync Control
*/
typedef struct MOT_PACKET_PACKED MotPacket_Sync
{
    uint8_t Start;      /* MOT_PACKET_START_BYTE */
    uint8_t SyncId;     /* MotPacket_Id_T */
    uint8_t Flex;       /* Optional SubId */
    uint8_t Checksum;
}
MotPacket_Sync_T;

/*
    General 8-byte Header for all variable length packets
*/
typedef struct MOT_PACKET_PACKED MotPacket_Header
{
    uint8_t Start;      /* MOT_PACKET_START_BYTE */
    uint8_t Id;         /* MotPacket_Id_T */
    uint8_t Length;
    uint8_t Sequence;
    uint16_t Checksum;
    uint16_t Flags;     /* Source/Dest */
}
MotPacket_Header_T;

/* Generic bases */
// typedef struct MOT_PACKET_PACKED MotPacket_HeaderFixed
// {
//     uint8_t Start;      /* MOT_PACKET_START_BYTE */
//     uint8_t Id;         /* MotPacket_Id_T */
//     uint8_t Imm[2U];     /* Imm or checksum */
// }
// MotPacket_HeaderFixed_T;

// typedef struct MOT_PACKET_PACKED MotPacket_HeaderFlex
// {
//     uint8_t Start;       /* MOT_PACKET_START_BYTE */
//     uint8_t Id;          /* MotPacket_Id_T */
//     uint8_t Length;
//     uint8_t Ext;         /* Id or Length Ext */
//     uint16_t Flags;      /* Flex/Source/Dest/Sequence */
//     uint16_t Checksum;
// }
// MotPacket_HeaderFlex_T;

typedef union MOT_PACKET_PACKED MotPacket
{
    struct
    {
        MotPacket_Header_T Header;
        uint8_t Payload[MOT_PACKET_LENGTH_MAX - sizeof(MotPacket_Header_T)];
    };
    uint8_t Bytes[MOT_PACKET_LENGTH_MAX];
}
MotPacket_T;

// typedef union MOT_PACKET_PACKED MotPacket
// {
//     struct
//     {
//         MotPacket_HeaderFixed_T Header;
//         uint32_t PayloadImm; /* optional immediate data */
//         uint8_t Payload[MOT_PACKET_LENGTH_MAX - 8];
//     };
//     uint8_t Bytes[MOT_PACKET_LENGTH_MAX];
// }
// MotPacket_T;

/******************************************************************************/
/*! Common */
/******************************************************************************/
static inline uint8_t MotPacket_ParsePayloadLength(const MotPacket_T * p_packet) { return p_packet->Header.Length - sizeof(MotPacket_Header_T); }
static inline uint8_t MotPacket_ParseTotalLength(const MotPacket_T * p_packet) { return p_packet->Header.Length; }
static inline void MotPacket_BuildPayloadLength(MotPacket_T * p_packet, uint8_t payloadLength) { p_packet->Header.Length = payloadLength + sizeof(MotPacket_Header_T); }
static inline void MotPacket_BuildTotalLength(MotPacket_T * p_packet, uint8_t totalLength) { p_packet->Header.Length = totalLength; }


/******************************************************************************/
/*
    Struct as transparent data transfer object.
    Meta format handled by header
*/
/******************************************************************************/

/******************************************************************************/
/*! Header Only */
/******************************************************************************/
/******************************************************************************/
/*! Ping */
/******************************************************************************/
typedef MotPacket_Sync_T MotPacket_PingReq_T;
typedef MotPacket_Sync_T MotPacket_PingResp_T;

/******************************************************************************/
/*! Common Generic Status */
/******************************************************************************/
// typedef struct MotPacket_StatusResp { uint8_t Start; uint8_t Id; uint16_t Status; } MotPacket_StatusResp_T;

/******************************************************************************/
/*!
    Fixed Length
    Optionally handle with 4-Byte Header
*/
/******************************************************************************/
/******************************************************************************/
/*! Version - Static Response */
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_VersionReq {} MotPacket_VersionReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_VersionResp { uint32_t Protocol; uint32_t Library; uint32_t Firmware; } MotPacket_VersionResp_T;

typedef struct MOT_PACKET_PACKED MotPacket_VersionFlexResp { uint32_t Versions[MOT_PACKET_PAYLOAD_LENGTH_MAX / sizeof(uint32_t)]; } MotPacket_VersionFlexResp_T;

/******************************************************************************/
/*! Stop - Stop All */
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_StopReq {} MotPacket_StopReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_StopResp { uint16_t Status; } MotPacket_StopResp_T;

/******************************************************************************/
/*! Call - Arbitrary Functions */
/******************************************************************************/
/* 32-bit Id / Address */
typedef struct MOT_PACKET_PACKED MotPacket_CallReq { uint32_t Id; uint32_t Arg; }      MotPacket_CallReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_CallResp { uint32_t Id; uint16_t Status; }  MotPacket_CallResp_T;

/* multiple parameters */
// typedef struct MOT_PACKET_PACKED MotPacket_CallArgVReq { uint32_t Id; uint16_t ArgC; uint16_t Flags; uint32_t ArgV[MOT_PACKET_PAYLOAD_LENGTH_MAX - 8U]; } MotPacket_CallArgVReq_T;
// typedef struct MOT_PACKET_PACKED MotPacket_CallArgVResp { uint32_t Id; uint16_t Status; }                                                                 MotPacket_CallArgVResp_T;

/******************************************************************************/
/*! Read/Write Var by Id */
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_VarReadFixedReq { uint16_t MotVarId; uint16_t Flags; }   MotPacket_VarReadFixedReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_VarReadFixedResp { uint32_t Value; }                     MotPacket_VarReadFixedResp_T;

typedef struct MOT_PACKET_PACKED MotPacket_VarWriteFixedReq { uint16_t MotVarId; uint16_t Flags; uint32_t Value; }    MotPacket_VarWriteFixedReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_VarWriteFixedResp { uint8_t Status; }                                      MotPacket_VarWriteFixedResp_T;
// alternatively return 16-bit status if using shared pool

/******************************************************************************/
/*!
    Flex Length - Use interface functions for length
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Vars 16
*/
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_VarReadReq { uint16_t MotVarIds[16U]; } MotPacket_VarReadReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_VarReadResp { uint16_t Value16[16U]; } MotPacket_VarReadResp_T;

typedef struct MOT_PACKET_PACKED MotPacket_VarWriteReq { struct { uint16_t MotVarId; uint16_t Value16; } Pairs[8U]; }  MotPacket_VarWriteReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_VarWriteResp { uint8_t VarStatus[8U]; }                                     MotPacket_VarWriteResp_T;

/******************************************************************************/
/*!
    Vars 32
*/
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_Var32ReadReq { MotPacket_VarReadFixedReq_T Read[8U]; } MotPacket_Var32ReadReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_Var32ReadResp { uint32_t Values[8U]; } MotPacket_Var32ReadResp_T;

/* Request reponse common */
static inline uint8_t MotPacket_Var32Read_ParseCount(const MotPacket_T * p_packet) { return MotPacket_ParsePayloadLength(p_packet) / sizeof(MotPacket_VarReadFixedReq_T); }

typedef struct MOT_PACKET_PACKED MotPacket_Var32WriteReq { MotPacket_VarWriteFixedReq_T Write[4U]; }    MotPacket_Var32WriteReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_Var32WriteResp { uint8_t VarStatus[4U]; }                    MotPacket_Var32WriteResp_T;

static inline uint8_t MotPacket_Var32WriteReq_ParseCount(const MotPacket_T * p_packet) { return MotPacket_ParsePayloadLength(p_packet) / sizeof(MotPacket_VarWriteFixedReq_T); }


/******************************************************************************/
/*! Mem Address Read */
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_MemReadReq { uint32_t Address; uint8_t Size; uint8_t Resv; uint16_t Config; }  MotPacket_MemReadReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_MemReadResp { uint8_t ByteData[MOT_PACKET_PAYLOAD_LENGTH_MAX]; }               MotPacket_MemReadResp_T;

/******************************************************************************/
/*! Mem Address Write */
/******************************************************************************/
#define MOT_PACKET_MEM_WRITE_SIZE_MAX 16U /* 24 available */
typedef struct MOT_PACKET_PACKED MotPacket_MemWriteReq { uint32_t Address; uint8_t Size; uint8_t Resv; uint16_t Config; uint8_t ByteData[MOT_PACKET_MEM_WRITE_SIZE_MAX]; } MotPacket_MemWriteReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_MemWriteResp { uint16_t Status; } MotPacket_MemWriteResp_T;

/******************************************************************************/
/*!
    Stateful Sequence
*/
/******************************************************************************/
/******************************************************************************/
/*! Data Mode Read/Write Control Common */
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_DataModeReq { uint32_t AddressStart; uint32_t SizeBytes; uint32_t Config; }    MotPacket_DataModeReq_T;
typedef struct MOT_PACKET_PACKED MotPacket_DataModeResp { uint16_t Status; }                                              MotPacket_DataModeResp_T;

/******************************************************************************/
/*! Data Mode Raw Data Packet */
/******************************************************************************/
typedef struct MOT_PACKET_PACKED MotPacket_DataMode { uint8_t ByteData[MOT_PACKET_PAYLOAD_LENGTH_MAX]; } MotPacket_DataMode_T;


/******************************************************************************/

/******************************************************************************/
// typedef union MotPacket_MotorStateFlags
// {
//     struct
//     {
//         uint8_t Direction    : 2U;
//         uint8_t FeedbackMode : 4U;
//         uint8_t VOutState    : 2U;
//     };
//     uint8_t Value;
// }
// MotPacket_MotorStateFlags_T;


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern uint16_t MotPacket_Checksum(const MotPacket_T * p_packet, size_t totalSize);
extern uint8_t MotPacket_Sync_Build(MotPacket_Sync_T * p_txPacket, MotPacket_Id_T syncId);
extern uint8_t MotPacket_BuildHeader(MotPacket_T * p_packet, MotPacket_Id_T headerId, uint8_t payloadLength);
