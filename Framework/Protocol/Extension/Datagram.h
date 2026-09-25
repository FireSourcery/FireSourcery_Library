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
    @file   Datagram.h
    @author FireSourcery
    @brief  Datagram mode - a host-configured set of vars, streamed continuously.
*/
/******************************************************************************/
#include "../Protocol_Request.h"
#include "../Packet.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/*
    Var streaming mode over the request engine.
*/
/******************************************************************************/
#ifndef DATAGRAM_MAP_MAX
#define DATAGRAM_MAP_MAX (8U)       /* entries per datagram */
#endif

typedef uint16_t datagram_id_t;     /* the application's var key - [MotVarId_T] for MotProtocol */

/******************************************************************************/
/*!
    Mapping - one mapped var: its key and the bytes it occupies in the frame
*/
/******************************************************************************/
typedef union Datagram_Entry
{
    struct
    {
        uint32_t VarId  : 16;   /* [15:0]  */
        uint32_t Size   : 8;    /* [23:16] bytes: 1, 2 or 4 */
        uint32_t Resv   : 8;    /* [31:24] */
    };
    uint32_t Value;
}
Datagram_Entry_T;

/*! One entry, its width taken from the C type the value is carried as. */
#define DATAGRAM_ENTRY(varId, type) { .VarId = (varId), .Size = (uint8_t)sizeof(type) }

/*! The widths [Datagram_Data_T] packs. */
static inline bool Datagram_Entry_IsSupported(Datagram_Entry_T entry) { return (entry.Size == 1U) || (entry.Size == 2U) || (entry.Size == 4U); }

/*! A value on its way to the wire. Little-endian target, so Size bytes are the low bytes. */
typedef union Datagram_Data { uint8_t Bytes[4]; int32_t Value; } Datagram_Data_T;

/******************************************************************************/
/*!
    Application binding
*/
/******************************************************************************/
/*!
    One mapped value, by key. The mapping is the host's, so a key the application does not
    know answers 0 - a run is not the place to discover that, and the host already learned it
    from the read that built the mapping.
*/
typedef int32_t (*Datagram_GetFn_T)(void * p_context, datagram_id_t varId);

typedef int32_t (*Datagram_VarFn_T)(void * p_context, datagram_id_t varId);
typedef void (*Datagram_StreamFn_T)(void * p_context, uint8_t id, uint8_t * p_data, uint8_t length);

typedef const struct Datagram_Interface
{
    Datagram_GetFn_T GET;
    void * P_CONTEXT;           /* Passed back to GET */
    packet_id_t CONFIG_ID;      /* Opens a run, and labels its status replies */
    packet_id_t DATA_ID;        /* Labels each datagram */
    packet_size_t LENGTH_MAX;   /* Bounded by the format's payload capacity */
}
Datagram_Interface_T;

/******************************************************************************/
/*!
    Wire payloads
*/
/******************************************************************************/
/*! The entry count is the frame's, not a field - the entries run to the end of the payload. */
typedef struct PACKET_PACKED Datagram_ConfigReq
{
    uint16_t Flags;                             /* Reserved */
    uint16_t Cycles;                            /* Datagrams to send, 0 = until aborted */
    Datagram_Entry_T Map[DATAGRAM_MAP_MAX];
}
Datagram_ConfigReq_T;

typedef struct PACKET_PACKED Datagram_ConfigResp
{
    uint16_t Status;
    uint8_t MapCount;   /* Entries accepted, 0 when refused */
    uint8_t Length;     /* Bytes each datagram carries */
}
Datagram_ConfigResp_T;

#define DATAGRAM_CONFIG_REQ_MIN (offsetof(Datagram_ConfigReq_T, Map))

#define DATAGRAM_STATUS_OK              (0U)
#define DATAGRAM_STATUS_MALFORMED       (0xD001U)   /* Opening request shorter than its fixed fields */
#define DATAGRAM_STATUS_MAP_COUNT       (0xD002U)   /* More entries than DATAGRAM_MAP_MAX */
#define DATAGRAM_STATUS_NOT_MAPPABLE    (0xD003U)   /* An entry's width is not 1, 2 or 4 */
#define DATAGRAM_STATUS_LENGTH          (0xD004U)   /* The mapped set exceeds one payload */

/*! PRECONDITION: payloadLength >= DATAGRAM_CONFIG_REQ_MIN - the classifier refuses anything shorter. */
static inline uint8_t Datagram_MapCountOf(packet_size_t payloadLength) { return (uint8_t)((payloadLength - DATAGRAM_CONFIG_REQ_MIN) / sizeof(Datagram_Entry_T)); }

/******************************************************************************/
/*!
    State - the mapping and the run, held in the socket's P_SUB_STATE buffer
*/
/******************************************************************************/
typedef enum Datagram_StateId
{
    DATAGRAM_STATE_IDLE,    /* Nothing mapped, nothing owed */
    DATAGRAM_STATE_OPEN,    /* Opening request - install the mapping, reply with status */
    DATAGRAM_STATE_DATA,    /* Stage one datagram, the run continues */
    DATAGRAM_STATE_CLOSE,   /* Cycle budget spent - reply and close */
    DATAGRAM_STATE_ERROR,   /* Opening request too short to be one - report and close */
}
Datagram_StateId_T;

typedef struct Datagram_State
{
    /*! Derived per pass from the arriving id and the run. Inspection only - never an input. */
    union
    {
        Datagram_StateId_T StateId;
        uint32_t StateIndex;
    };

    uint8_t MapCount;
    Datagram_Entry_T Map[DATAGRAM_MAP_MAX];

    uint16_t Cycles;        /* Declared by the opening request, 0 = unbounded */
    uint16_t Count;         /* Datagrams sent so far */

    packet_id_t ReqId;      /* Echoed on the status replies. Taken from the opening request. */
    uint16_t Status;        /* Last result, for inspection */
}
Datagram_State_T;

/*! Byte offset of entry i - the widths of the entries before it. */
static inline packet_size_t _Datagram_Offset(const Datagram_State_T * p_state, uint8_t i)
{
    packet_size_t offset = 0U;
    for (uint8_t k = 0U; k < i; k++) { offset += p_state->Map[k].Size; }
    return offset;
}

/*! Bytes a datagram carries. */
static inline packet_size_t Datagram_Length(const Datagram_State_T * p_state) { return _Datagram_Offset(p_state, p_state->MapCount); }

/*! An unbounded run is never spent. */
static inline bool Datagram_IsSpent(const Datagram_State_T * p_state) { return (p_state->Cycles != 0U) && (p_state->Count >= p_state->Cycles); }

/******************************************************************************/
/*!
    Proc

    [Datagram_Proc] takes the interface, not the application context, so a row registers
    through a handler that names its own context and builds the interface - the shape
    [Protocol_DataMode_Read] is registered in. The socket's P_SUB_STATE must hold a
    [Datagram_State_T], which the mapping makes the largest sub-state a socket carrying this
    row will need.
*/
/******************************************************************************/
extern void Datagram_Begin(Datagram_State_T * p_state, const Packet_Meta_T * p_rxMeta, const Datagram_ConfigReq_T * p_req, packet_size_t lengthMax);
extern packet_size_t Datagram_Build(Datagram_Interface_T * p_app, const Datagram_State_T * p_state, void * p_payload);
extern Protocol_ReqCode_T Datagram_Proc(Datagram_Interface_T * p_app, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload);
