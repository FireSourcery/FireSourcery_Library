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
    @file   OD.h
    @author FireSourcery
    @brief  CANopen Object Dictionary — index areas, object metadata, abort codes, value codec, and the dictionary interface
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/******************************************************************************/
/*
    Object Dictionary Indices
*/
/******************************************************************************/
/*
    Index interpretation — the area view
*/
typedef union OD_Index
{
    struct __attribute__((packed)) { uint16_t Offset : 12; uint16_t Area : 4; };    /* area view — CiA 301 areas */
    uint16_t Index;
}
OD_Index_T;

static inline OD_Index_T OD_Index(uint16_t index) { return (OD_Index_T) { .Index = index }; }

/*
    CiA 301 areas. Each value is the index the area starts at; the switch
    below carries how far it runs.
*/
typedef enum OD_Area
{
    OD_AREA_DATA_TYPES     = 0x0000U, /* Page   0-1   */
    OD_AREA_COMM_PROFILE   = 0x1000U, /* Page   2-3   */
    OD_AREA_MANUFACTURER   = 0x2000U, /* Page   4-11  */
    OD_AREA_DEVICE_PROFILE = 0x6000U, /* Page  12-19  */
    OD_AREA_RESERVED       = 0xA000U, /* Page  20-31  */
}
OD_Area_T;

#define OD_AREA_BITS         (12U)
#define OD_AREA_ID(index)    ((index) >> OD_AREA_BITS)

static inline OD_Area_T OD_Area_Of(uint16_t index)
{
    switch (OD_Index(index).Area)
    {
        case OD_AREA_ID(OD_AREA_DATA_TYPES):      return OD_AREA_DATA_TYPES;
        case OD_AREA_ID(OD_AREA_COMM_PROFILE):    return OD_AREA_COMM_PROFILE;
        case OD_AREA_ID(OD_AREA_MANUFACTURER):
        case OD_AREA_ID(OD_AREA_MANUFACTURER) + 1U:
        case OD_AREA_ID(OD_AREA_MANUFACTURER) + 2U:
        case OD_AREA_ID(OD_AREA_MANUFACTURER) + 3U: return OD_AREA_MANUFACTURER;
        case OD_AREA_ID(OD_AREA_DEVICE_PROFILE):
        case OD_AREA_ID(OD_AREA_DEVICE_PROFILE) + 1U:
        case OD_AREA_ID(OD_AREA_DEVICE_PROFILE) + 2U:
        case OD_AREA_ID(OD_AREA_DEVICE_PROFILE) + 3U: return OD_AREA_DEVICE_PROFILE;
        default:    return OD_AREA_RESERVED;
    }
}


/******************************************************************************/
/*
    Object Dictionary metadata
*/
/******************************************************************************/
typedef enum OD_Type
{
    OD_TYPE_NONE,
    OD_TYPE_I8,
    OD_TYPE_U8,
    OD_TYPE_I16,
    OD_TYPE_U16,
    OD_TYPE_I32,
    OD_TYPE_U32,
}
OD_Type_T;

static inline uint8_t OD_Type_Size(OD_Type_T type)
{
    switch (type)
    {
        case OD_TYPE_I8:  return 1U;
        case OD_TYPE_U8:  return 1U;
        case OD_TYPE_I16: return 2U;
        case OD_TYPE_U16: return 2U;
        case OD_TYPE_I32: return 4U;
        case OD_TYPE_U32: return 4U;
        default: return 0U;
    }
}

typedef enum OD_Access
{
    OD_ACCESS_NONE = 0U,
    OD_ACCESS_RO   = 1U,
    OD_ACCESS_WO   = 2U,
    OD_ACCESS_RW   = 3U,
}
OD_Access_T;

typedef struct OD_Info
{
    OD_Type_T   Type;
    OD_Access_T Access;
    uint8_t     Size; /* in bytes */
}
OD_Info_T;

/* SDO abort codes per CiA 301 */
typedef enum OD_Status
{
    OD_OK                    = 0,
    OD_ERR_TOGGLE_BIT        = (int)0x05030000, /* Toggle bit not alternated */
    OD_ERR_TIMEOUT           = (int)0x05040000, /* SDO protocol timed out */
    OD_ERR_INVALID_CCS       = (int)0x05040001, /* Client/server cmd specifier invalid */
    OD_ERR_NO_OBJECT         = (int)0x06020000, /* Object does not exist */
    OD_ERR_NOT_MAPPABLE      = (int)0x06040041, /* Object cannot be mapped to PDO */
    OD_ERR_PDO_LENGTH        = (int)0x06040042, /* Mapped PDO length exceeds */
    OD_ERR_GENERAL_PARAM     = (int)0x06040043, /* General parameter incompatibility */
    OD_ERR_GENERAL_INTERNAL  = (int)0x06040047, /* General internal incompatibility */
    OD_ERR_HARDWARE          = (int)0x06060000, /* Access failed due to hardware error */
    OD_ERR_LENGTH_MISMATCH   = (int)0x06070010, /* Data type / length mismatch */
    OD_ERR_LENGTH_HIGH       = (int)0x06070012, /* Length too high */
    OD_ERR_LENGTH_LOW        = (int)0x06070013, /* Length too low */
    OD_ERR_SUBINDEX          = (int)0x06090011, /* Subindex does not exist */
    OD_ERR_UNSUPPORTED_ACCESS= (int)0x06010000, /* Unsupported access to an object */
    OD_ERR_READ_ONLY         = (int)0x06010002, /* Write to RO object */
    OD_ERR_WRITE_ONLY        = (int)0x06010001, /* Read of WO object */
    OD_ERR_VALUE_RANGE       = (int)0x06090030, /* Value out of range */
    OD_ERR_VALUE_HIGH        = (int)0x06090031, /* Value too high */
    OD_ERR_VALUE_LOW         = (int)0x06090032, /* Value too low */
    OD_ERR_GENERAL           = (int)0x08000000, /* General error */
    OD_ERR_DEVICE_STATE      = (int)0x08000022, /* Refused due to present device state */
}
OD_Status_T;

/*
    OD Data — one object's value, viewed by type. Carried by SDO expedited data
    (bytes 4..7) and by each mapped field of a PDO:
        value = p_req->Data.I16;
*/
typedef union OD_Data
{
    uint8_t  Bytes[4];
    int8_t   I8;
    uint8_t  U8;
    int16_t  I16;
    uint16_t U16;
    int32_t  I32;
    uint32_t U32;
}
OD_Data_T;

/*
    Handles sign extension
*/
static int32_t OD_Data_Decode(OD_Type_T type, OD_Data_T data)
{
    switch (type)
    {
        case OD_TYPE_I8:  return (int32_t)data.I8;
        case OD_TYPE_U8:  return (int32_t)data.U8;
        case OD_TYPE_I16: return (int32_t)data.I16;
        case OD_TYPE_U16: return data.U16;
        case OD_TYPE_I32: return data.I32;
        case OD_TYPE_U32: return data.U32;
        default:          return 0;
    }
}

static OD_Data_T OD_Data_Encode(OD_Type_T type, int32_t value)
{
    switch (type)
    {
        case OD_TYPE_I8:  return (OD_Data_T) { .I8 = (int8_t)value };
        case OD_TYPE_U8:  return (OD_Data_T) { .U8 = (uint8_t)value };
        case OD_TYPE_I16: return (OD_Data_T) { .I16 = (int16_t)value };
        case OD_TYPE_U16: return (OD_Data_T) { .U16 = (uint16_t)value };
        case OD_TYPE_I32: return (OD_Data_T) { .I32 = value };
        case OD_TYPE_U32: return (OD_Data_T) { .U32 = (uint32_t)value };
        default:          return (OD_Data_T) { .U32 = 0 };
    }
}


/******************************************************************************/
/*
    Object Dictionary callback interface

    The SDO server and the PDO codec are generic — they reach objects only
    through these callbacks. The integration supplies the function pointers,
    and the context each call carries.
*/
/******************************************************************************/
typedef OD_Info_T(*OD_GetInfoFn_T)(void * p_context, uint16_t index, uint8_t subindex);
typedef OD_Status_T(*OD_GetFn_T)  (void * p_context, uint16_t index, uint8_t subindex, int32_t * p_value);
typedef OD_Status_T(*OD_SetFn_T)  (void * p_context, uint16_t index, uint8_t subindex, int32_t value);

typedef const struct OD
{
    // void * p_Context;
    OD_GetInfoFn_T GetInfo;
    OD_GetFn_T Get;
    OD_SetFn_T Set;
}
OD_Interface_T;

