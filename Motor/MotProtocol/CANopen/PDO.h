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
    @file   PDO.h
    @author FireSourcery
    @brief  CANopen Process Data Object — channels, mapping codec, and parameter objects
*/
/******************************************************************************/
#include "CANopen.h"
#include "OD.h"

#include <string.h> /* memcpy — RPDO field extraction */


/******************************************************************************/
/*
    PDO — CiA 301 process data objects, configurable over SDO

    One channel type serves both directions: a channel is its communication and
    mapping records. The parameter index names everything else — direction,
    record, and channel — so nothing stores a direction. The mapping, not code,
    says what a frame's bytes are: RPDO frames decode into Set calls, TPDO frames
    encode from Get calls.

    A master reconfigures a channel with SDO downloads, in CiA 301 order:
        comm:COB_ID    = COB-ID | 0x80000000    disable
        mapping:COUNT  = 0                      open the mapping
        mapping:ENTRY+k = entry                 k = 0..count-1
        mapping:COUNT  = count                  validate and close
        comm:COB_ID    = COB-ID                 enable
*/
/******************************************************************************/
#define PDO_MAP_MAX              (8U)        /* entries per PDO */
#define PDO_BITS_MAX             (64U)       /* classic CAN payload */

/*
    PDO Frame — up to 8 bytes of pre-mapped process data.
    Layout is set by the PDO mapping objects (0x1A00.., 0x1600..) — see [PDO_Channel_T] —
    or a fixed layout overlays Bytes.
*/
typedef union PDO
{
    uint8_t Bytes[8];
}
PDO_T;

/*
    Object dictionary entries for the communication area (0x1000 - 0x1FFF): the PDO parameter records.
    Each is a base: + n addresses PDO n + 1, up to 512 per record.
*/
typedef enum PDO_OdRecord
{
    PDO_OD_RPDO_COMM_PARAM           = (0x1400U), /* RECORD  RPDO communication parameter */
    PDO_OD_RPDO_MAPPING_PARAM        = (0x1600U), /* RECORD  RPDO mapping parameter */
    PDO_OD_TPDO_COMM_PARAM           = (0x1800U), /* RECORD  TPDO communication parameter */
    PDO_OD_TPDO_MAPPING_PARAM        = (0x1A00U), /* RECORD  TPDO mapping parameter */
}
PDO_OdRecord_T;

#define PDO_OD_CHANNEL_MASK      (0x01FFU)   /* + n */

/* An index's record, channel bits cleared — the value the PDO_OdRecord_T constants name — and its channel. */
static inline PDO_OdRecord_T PDO_OdRecord(uint16_t index) { return (PDO_OdRecord_T)(index & (uint16_t)~PDO_OD_CHANNEL_MASK); }
static inline uint16_t PDO_OdChannel(uint16_t index) { return (uint16_t)(index & PDO_OD_CHANNEL_MASK); }

static inline bool _PDO_OdRecord_IsTx(PDO_OdRecord_T record)      { return (record == PDO_OD_TPDO_COMM_PARAM) || (record == PDO_OD_TPDO_MAPPING_PARAM); }
static inline bool _PDO_OdRecord_IsMapping(PDO_OdRecord_T record) { return (record == PDO_OD_RPDO_MAPPING_PARAM) || (record == PDO_OD_TPDO_MAPPING_PARAM); }

/* Sub-indices of a PDO communication record — one layout, RPDO and TPDO */
typedef enum PDO_OdCommSub
{
    PDO_OD_COMM_HIGHEST          = (0U), /* RO  U8   highest sub-index supported */
    PDO_OD_COMM_COB_ID           = (1U), /* RW  U32  PDO_CobId_T */
    PDO_OD_COMM_TRANSMISSION     = (2U), /* RW  U8   PDO_Transmission_T */
    PDO_OD_COMM_EVENT_TIMER      = (5U), /* RW  U16  TPDO period, ms. 3 inhibit time, 4 reserved, 6 SYNC start: not implemented */
}
PDO_OdCommSub_T;

/* Sub-indices of a PDO mapping record */
typedef enum PDO_OdMappingSub
{
    PDO_OD_MAPPING_COUNT         = (0U), /* RW  U8   number of mapped objects, 0 while edited */
    PDO_OD_MAPPING_ENTRY         = (1U), /* RW  U32  first of PDO_MAP_MAX entries, PDO_MapEntry_T */
}
PDO_OdMappingSub_T;

/* Event-driven only — SYNC is not implemented. RPDO: apply on reception. TPDO: send on the event timer. */
typedef enum PDO_Transmission
{
    PDO_TRANSMISSION_EVENT_MANUFACTURER  = 254U,
    PDO_TRANSMISSION_EVENT_PROFILE       = 255U,
}
PDO_Transmission_T;

static inline bool PDO_Transmission_IsSupported(uint8_t type) { return (type == PDO_TRANSMISSION_EVENT_MANUFACTURER) || (type == PDO_TRANSMISSION_EVENT_PROFILE); }

/* comm : COB_ID */
typedef union PDO_CobId
{
    struct
    {
        uint32_t CanId      : 11;   /* [10:0]  node bits 0 = this node's predefined ID */
        uint32_t Resv       : 18;   /* [28:11] */
        uint32_t Frame      : 1;    /* [29]    29-bit ID — not supported */
        uint32_t Rtr        : 1;    /* [30]    TPDO only — RTR is not served */
        uint32_t Invalid    : 1;    /* [31]    PDO does not exist */
    };
    uint32_t Value;
}
PDO_CobId_T;

/* 11-bit data frames only. */
static inline bool PDO_CobId_IsSupported(PDO_CobId_T cobId) { return (cobId.Resv == 0U) && (cobId.Frame == 0U); }

/* mapping : ENTRY.. — one mapped object: its OD address and its width */
typedef union PDO_MapEntry
{
    struct
    {
        uint32_t BitLength  : 8;    /* [7:0]   */
        uint32_t SubIndex   : 8;    /* [15:8]  */
        uint32_t Index      : 16;   /* [31:16] */
    };
    uint32_t Value;
}
PDO_MapEntry_T;

/* One entry, its width taken from the C type that carries the object. */
#define PDO_MAP_ENTRY(index, subindex, type) { .Index = (index), .SubIndex = (subindex), .BitLength = (uint8_t)(8U * sizeof(type)) }

/* A whole 8, 16 or 32-bit object — the widths [OD_Data_T] carries. */
static inline bool PDO_MapEntry_IsSupported(PDO_MapEntry_T entry) { return (entry.BitLength == 8U) || (entry.BitLength == 16U) || (entry.BitLength == 32U); }

/* One PDO's configuration, either direction — RAM, because the master rewrites it. */
typedef struct PDO_Channel
{
    PDO_CobId_T     CobId;              /* comm    : COB_ID */
    uint8_t         Transmission;       /* comm    : TRANSMISSION */
    uint16_t        EventTimer;         /* comm    : EVENT_TIMER — TPDO period, ms; 0 = no periodic transmission */
    uint8_t         MapCount;           /* mapping : COUNT */
    PDO_MapEntry_T  Map[PDO_MAP_MAX];   /* mapping : ENTRY.. */
}
PDO_Channel_T;

/* A node's channels in one direction. */
typedef const struct PDO_Table
{
    PDO_Channel_T * P_CHANNELS;
    uint8_t COUNT;
}
PDO_Table_T;

/* A node's PDOs, both directions — what its PDO parameter indices address. */
typedef const struct PDO_Tables
{
    PDO_Table_T RX;   /* PDO_OD_RPDO_COMM_PARAM, PDO_OD_RPDO_MAPPING_PARAM + n */
    PDO_Table_T TX;   /* PDO_OD_TPDO_COMM_PARAM, PDO_OD_TPDO_MAPPING_PARAM + n */
}
PDO_Tables_T;

/* Byte offset of entry i — the widths of the entries before it. */
static inline uint8_t _PDO_Channel_Offset(const PDO_Channel_T * p_channel, uint8_t i)
{
    uint16_t bits = 0U;
    for (uint8_t k = 0U; k < i; k++) { bits += p_channel->Map[k].BitLength; }
    return (uint8_t)(bits / 8U);
}

/* Bytes a frame carries. */
static inline uint8_t PDO_Channel_Length(const PDO_Channel_T * p_channel) { return _PDO_Channel_Offset(p_channel, p_channel->MapCount); }

/*
    A state the parameter objects could have left: a supported COB-ID and transmission type, enabled only with something
    mapped, at most PDO_MAP_MAX supported entries within one CAN payload. What a channel loaded from erased or
    stale NVM fails.
*/
static inline bool PDO_Channel_IsWellFormed(const PDO_Channel_T * p_channel)
{
    bool isWellFormed = PDO_CobId_IsSupported(p_channel->CobId) && PDO_Transmission_IsSupported(p_channel->Transmission)
        && ((p_channel->CobId.Invalid == 1U) || (p_channel->MapCount != 0U)) && (p_channel->MapCount <= PDO_MAP_MAX);
    for (uint8_t i = 0U; (i < p_channel->MapCount) && isWellFormed; i++) { isWellFormed = PDO_MapEntry_IsSupported(p_channel->Map[i]); }
    return isWellFormed && (PDO_Channel_Length(p_channel) <= (PDO_BITS_MAX / 8U));
}

/* The exact COB-ID, or with node bits 0, this node's predefined ID in that function class. The bus filter admits only this node. */
static inline bool PDO_Channel_IsFor(const PDO_Channel_T * p_channel, uint16_t cob_id)
{
    uint16_t canId = (uint16_t)p_channel->CobId.CanId;
    return (p_channel->CobId.Invalid == 0U) && ((canId == cob_id) || ((COB_NODE(canId) == 0U) && (COB_FUNCTION(canId) == COB_FUNCTION(cob_id))));
}

/******************************************************************************/
/*
    Reception
*/
/******************************************************************************/
/* Entry i: decode its bytes from the frame and Set the object. */
static inline void _PDO_Channel_WriteEntry(const OD_Interface_T * p_od, void * p_context, const PDO_Channel_T * p_channel, uint8_t i, const uint8_t * p_data)
{
    PDO_MapEntry_T entry = p_channel->Map[i];
    OD_Data_T value = { .U32 = 0U };
    memcpy(value.Bytes, &p_data[_PDO_Channel_Offset(p_channel, i)], entry.BitLength / 8U);
    (void)p_od->Set(p_context, entry.Index, entry.SubIndex, OD_Data_Decode(p_od->GetInfo(p_context, entry.Index, entry.SubIndex).Type, value));
}

/*
    The one handler every RxPDO route shares. Finds the channel configured for cob_id, requires the frame to carry
    exactly the mapped length, and writes each mapped object through p_od, in map order — the path an SDO download takes.
*/
static inline void PDO_HandleRx(const OD_Interface_T * p_od, void * p_context, PDO_Table_T * p_table, uint16_t cob_id, const PDO_T * p_pdo, uint8_t dlc)
{
    for (uint8_t n = 0U; n < p_table->COUNT; n++)
    {
        const PDO_Channel_T * p_channel = &p_table->P_CHANNELS[n];
        if (PDO_Channel_IsFor(p_channel, cob_id) == true)
        {
            /* Exact length only. CANopenNode also reports a mismatch as EMCY 0x8210 short / 0x8220 long. */
            if (dlc == PDO_Channel_Length(p_channel))
            {
                for (uint8_t i = 0U; i < p_channel->MapCount; i++) { _PDO_Channel_WriteEntry(p_od, p_context, p_channel, i, p_pdo->Bytes); }
            }
            break;
        }
    }
}

/******************************************************************************/
/*
    Transmission
*/
/******************************************************************************/
/* A valid TPDO is due each time its event timer elapses; 0 = no periodic transmission. Wrap-safe. */
static inline bool PDO_Channel_IsTxDue(const PDO_Channel_T * p_channel, uint32_t last_ms, uint32_t now_ms)
{
    return (p_channel->CobId.Invalid == 0U) && (p_channel->EventTimer != 0U) && ((now_ms - last_ms) >= p_channel->EventTimer);
}

/* Entry i: Get the object and encode its bytes into the frame. */
static inline void _PDO_Channel_ReadEntry(const OD_Interface_T * p_od, void * p_context, const PDO_Channel_T * p_channel, uint8_t i, uint8_t * p_data)
{
    PDO_MapEntry_T entry = p_channel->Map[i];
    int32_t value = 0;
    (void)p_od->Get(p_context, entry.Index, entry.SubIndex, &value);
    OD_Data_T data = OD_Data_Encode(p_od->GetInfo(p_context, entry.Index, entry.SubIndex).Type, value);
    memcpy(&p_data[_PDO_Channel_Offset(p_channel, i)], data.Bytes, entry.BitLength / 8U);
}

/* Mirror of reception: reads each mapped object through p_od and packs it little-endian. Returns the frame length. */
static inline uint8_t PDO_BuildTx(const OD_Interface_T * p_od, void * p_context, const PDO_Channel_T * p_channel, PDO_T * p_pdo)
{
    for (uint8_t i = 0U; i < p_channel->MapCount; i++) { _PDO_Channel_ReadEntry(p_od, p_context, p_channel, i, p_pdo->Bytes); }
    return PDO_Channel_Length(p_channel);
}

/******************************************************************************/
/*
    Parameter objects — the PDO records of the communication area, as a dictionary. Context: a node's [PDO_Tables_T].
*/
/******************************************************************************/
extern const OD_Interface_T PDO_PARAM_OD;
