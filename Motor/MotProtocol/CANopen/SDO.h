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
    @file   SDO.h
    @author FireSourcery
    @brief  CANopen Service Data Object — frames and the expedited server
*/
/******************************************************************************/
#include "OD.h"


/******************************************************************************/
/*
    SDO (Service Data Object) — expedited transfers

    Frame layout (8-byte CAN payload):
      ┌────────┬─────────────┬──────────┬──────────────────────────┐
      │ Byte 0 │ Byte 1..2   │ Byte 3   │ Byte 4..7                │
      │ Cmd    │ Index (LE)  │ SubIdx   │ Data (LE, up to 4 bytes) │
      └────────┴─────────────┴──────────┴──────────────────────────┘
*/
/******************************************************************************/
/*
    SDO Command Specifier — byte 0 of the SDO payload.
    GCC packs first-declared bitfield in LSB; layout below matches
    [bit7..5: ccs][bit4: rsv][bit3..2: n][bit1: e][bit0: s].
*/
typedef union SDO_Cmd
{
    struct __attribute__((packed))
    {
        uint8_t Size      : 1; /* [0]  s    1 = data size indicated by N */
        uint8_t Expedited : 1; /* [1]  e    1 = data fits in bytes 4..7 */
        uint8_t N         : 2; /* [2:3] n   number of unused bytes in data field (0..3) */
        uint8_t Reserved  : 1; /* [4]       always 0 */
        uint8_t Ccs       : 3; /* [5:7] ccs command code (SDO_Ccs_T) */
    };
    uint8_t Byte;
}
SDO_Cmd_T;

/*
    Client/Server Command Specifier (CCS) — top 3 bits of byte 0.
    Distinguishes request kind (download = write, upload = read, etc).
*/
typedef enum SDO_Ccs
{
    SDO_CCS_DOWNLOAD_SEG_REQ     = 0U, /* segmented download request  (client → server) */
    SDO_CCS_DOWNLOAD_INIT_REQ    = 1U, /* download initiate          (client → server) */
    SDO_CCS_UPLOAD_INIT_REQ      = 2U, /* upload initiate            (client → server) */
    SDO_CCS_UPLOAD_SEG_REQ       = 3U, /* segmented upload request   (client → server) */
    SDO_CCS_ABORT                = 4U, /* abort transfer             (either direction) */
    SDO_CCS_BLOCK_UPLOAD         = 5U, /* block upload               (either direction) */
    SDO_CCS_BLOCK_DOWNLOAD       = 6U, /* block download             (either direction) */
    /* SCS (server response codes) reuse the same field — context distinguishes */
    SDO_SCS_UPLOAD_INIT_RSP      = 2U, /* upload initiate response   (server → client) */
    SDO_SCS_DOWNLOAD_INIT_RSP    = 3U, /* download initiate response (server → client) */
}
SDO_Ccs_T;

/*
    SDO Frame — 8-byte CAN payload for SDO request and response.
    Fields are little-endian on the wire; the packed layout matches
    standard CiA 301 byte ordering.

    Use Data.<type> to read/write the value of the indexed object directly.
    For abort frames, AbortCode holds the U32 abort reason in the same bytes.
*/
typedef union SDO
{
    struct __attribute__((packed))
    {
        SDO_Cmd_T   Cmd;        /* byte 0     */
        uint16_t    Index;      /* bytes 1..2 little-endian */
        uint8_t     SubIndex;   /* byte 3     */
        union
        {
            OD_Data_T   Data;       /* bytes 4..7 little-endian — the object's value */
            uint32_t    AbortCode;  /* bytes 4..7 little-endian — abort frames, OD_Status_T */
        };
    };
    uint8_t Bytes[8];
}
SDO_T;

/* less than 2 registers */
static inline SDO_T SDO_EncodeAbort(uint16_t index, uint8_t subindex, OD_Status_T abortCode)
{
    return (SDO_T) { .Cmd = { .Ccs = SDO_CCS_ABORT, }, .Index = index, .SubIndex = subindex, .AbortCode = (uint32_t)abortCode, };
}

static inline SDO_T SDO_EncodeDownloadAck(uint16_t index, uint8_t subindex)
{
    return (SDO_T) { .Cmd = { .Ccs = SDO_SCS_DOWNLOAD_INIT_RSP }, .Index = index, .SubIndex = subindex };
}

static inline SDO_T SDO_EncodeUploadResponse(uint16_t index, uint8_t subindex, OD_Info_T info, int32_t value)
{
    return (SDO_T)
    {
        .Cmd      = { .Ccs = SDO_SCS_UPLOAD_INIT_RSP, .Expedited = 1U, .Size = 1U, .N = (uint8_t)(4U - info.Size) },
        .Index    = index,
        .SubIndex = subindex,
        .Data     = OD_Data_Encode(info.Type, value),
    };
}


/******************************************************************************/
/*
    SDO server entry point
*/
/******************************************************************************/
static inline uint8_t SDO_HandleRequest(const OD_Interface_T * p_od, void * p_context, const SDO_T * p_req, SDO_T * p_rsp)
{
    OD_Info_T info = (p_od->GetInfo != NULL) ? p_od->GetInfo(p_context, p_req->Index, p_req->SubIndex) : (OD_Info_T) { 0 };

    switch ((SDO_Ccs_T)p_req->Cmd.Ccs)
    {
        case SDO_CCS_DOWNLOAD_INIT_REQ: /* master writes object */
            {
                if (info.Type == OD_TYPE_NONE)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, (p_req->SubIndex != 0U) ? OD_ERR_SUBINDEX : OD_ERR_NO_OBJECT);
                    break;
                }
                if (info.Access == OD_ACCESS_RO)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, OD_ERR_READ_ONLY);
                    break;
                }
                int32_t value = OD_Data_Decode(info.Type, p_req->Data);
                OD_Status_T r = (p_od->Set != NULL) ? p_od->Set(p_context, p_req->Index, p_req->SubIndex, value) : OD_ERR_GENERAL;
                *p_rsp = (r == OD_OK) ? SDO_EncodeDownloadAck(p_req->Index, p_req->SubIndex) : SDO_EncodeAbort(p_req->Index, p_req->SubIndex, r);
                break;
            }

        case SDO_CCS_UPLOAD_INIT_REQ: /* master reads object */
            {
                if (info.Type == OD_TYPE_NONE)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, (p_req->SubIndex != 0U) ? OD_ERR_SUBINDEX : OD_ERR_NO_OBJECT);
                    break;
                }
                if (info.Access == OD_ACCESS_WO)
                {
                    *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, OD_ERR_WRITE_ONLY);
                    break;
                }

                int32_t value = 0;
                OD_Status_T r = (p_od->Get != NULL) ? p_od->Get(p_context, p_req->Index, p_req->SubIndex, &value) : OD_ERR_GENERAL;
                *p_rsp = (r == OD_OK) ? SDO_EncodeUploadResponse(p_req->Index, p_req->SubIndex, info, value) : SDO_EncodeAbort(p_req->Index, p_req->SubIndex, r);
                break;
            }

        case SDO_CCS_ABORT:
            /* Master aborted — no response per CiA 301 */
            return 0U;

        default:
            /* Segmented and block transfers not supported by this minimal server */
            *p_rsp = SDO_EncodeAbort(p_req->Index, p_req->SubIndex, OD_ERR_GENERAL);
            break;
    }

    return 8U;
}

