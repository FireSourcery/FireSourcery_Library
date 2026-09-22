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
    @file   Cia402.c
    @author FireSourcery
    @brief  CiA 402 protocol-layer parser/dispatcher.

            SDO server: parses inbound SDO frames as SDO_T overlays,
                        dispatches via OD_Interface_T, builds the
                        response into the caller's buffer.

            PDO router: dispatches inbound PDO frames by COB-ID using typed
                        Cia402_RxPdo_*_T overlays for default mappings.
*/
/******************************************************************************/
#include "Cia402.h"

#include <stddef.h>

/******************************************************************************/
/*
    OD metadata — spec-fixed compile-time table.

    Rows are sorted by (Index, SubIndex) for readability and to permit
    binary search if the table grows past ~50 entries. Linear scan is
    fine at this size.

    To add an object: append a row. To remove: delete the row.
    No code change required in the SDO server.
*/
/******************************************************************************/
const Cia402_OdMeta_T CIA402_OD_META[] =
{
    { CIA402_OD_CONTROLWORD,             0U, OD_TYPE_U16, OD_ACCESS_RW, /* sizeof(uint16_t) */ },
    { CIA402_OD_STATUSWORD,              0U, OD_TYPE_U16, OD_ACCESS_RO, /* sizeof(uint16_t) */ },
    { CIA402_OD_QUICK_STOP_OPTION_CODE,  0U, OD_TYPE_I16, OD_ACCESS_RW, /* sizeof(int16_t)  */ },
    { CIA402_OD_SHUTDOWN_OPTION_CODE,    0U, OD_TYPE_I16, OD_ACCESS_RW, /* sizeof(int16_t)  */ },
    { CIA402_OD_DISABLE_OP_OPTION_CODE,  0U, OD_TYPE_I16, OD_ACCESS_RW, /* sizeof(int16_t)  */ },
    { CIA402_OD_HALT_OPTION_CODE,        0U, OD_TYPE_I16, OD_ACCESS_RW, /* sizeof(int16_t)  */ },
    { CIA402_OD_FAULT_REACTION_CODE,     0U, OD_TYPE_I16, OD_ACCESS_RW, /* sizeof(int16_t)  */ },
    { CIA402_OD_MODES_OF_OPERATION,      0U, OD_TYPE_I8,  OD_ACCESS_RW, /* sizeof(int8_t)   */ },
    { CIA402_OD_MODES_OF_OPERATION_DISP, 0U, OD_TYPE_I8,  OD_ACCESS_RO, /* sizeof(int8_t)   */ },
    { CIA402_OD_POSITION_ACTUAL,         0U, OD_TYPE_I32, OD_ACCESS_RO, /* sizeof(int32_t)  */ },
    { CIA402_OD_VELOCITY_ACTUAL,         0U, OD_TYPE_I32, OD_ACCESS_RO, /* sizeof(int32_t)  */ },
    { CIA402_OD_TARGET_TORQUE,           0U, OD_TYPE_I16, OD_ACCESS_RW, /* sizeof(int16_t)  */ },
    { CIA402_OD_TORQUE_ACTUAL,           0U, OD_TYPE_I16, OD_ACCESS_RO, /* sizeof(int16_t)  */ },
    { CIA402_OD_CURRENT_ACTUAL,          0U, OD_TYPE_I16, OD_ACCESS_RO, /* sizeof(int16_t)  */ },
    { CIA402_OD_DC_LINK_VOLTAGE,         0U, OD_TYPE_U32, OD_ACCESS_RO, /* sizeof(uint32_t) */ },
    { CIA402_OD_QUICK_STOP_DECELERATION, 0U, OD_TYPE_U32, OD_ACCESS_RW, /* sizeof(uint32_t) */ },
    { CIA402_OD_TARGET_VELOCITY,         0U, OD_TYPE_I32, OD_ACCESS_RW, /* sizeof(int32_t)  */ },
    { CIA402_OD_SUPPORTED_DRIVE_MODES,   0U, OD_TYPE_U32, OD_ACCESS_RO, /* sizeof(uint32_t) */ },
// { CIA402_OD_QUICK_STOP_OPTION_CODE,  0U, OD_TYPE_I16, CIA402_OD_ACCESS_R/* W, offsetof(Cia4 */02_Adapter_T, Config.QuickStopOption) },
};

static const OD_Info_T OD_ABSENT = { OD_TYPE_NONE, OD_ACCESS_NONE, 0U };

/* switch mapped. check if degenerates */
const Cia402_OdMeta_T * _TableIndex(uint16_t index, uint8_t subindex)
{
    switch (index)
    {
        case CIA402_OD_CONTROLWORD:             return &CIA402_OD_META[0U];
        case CIA402_OD_STATUSWORD:              return &CIA402_OD_META[1U];
        case CIA402_OD_QUICK_STOP_OPTION_CODE:  return &CIA402_OD_META[2U];
        case CIA402_OD_SHUTDOWN_OPTION_CODE:    return &CIA402_OD_META[3U];
        case CIA402_OD_DISABLE_OP_OPTION_CODE:  return &CIA402_OD_META[4U];
        case CIA402_OD_HALT_OPTION_CODE:        return &CIA402_OD_META[5U];
        case CIA402_OD_FAULT_REACTION_CODE:     return &CIA402_OD_META[6U];
        case CIA402_OD_MODES_OF_OPERATION:      return &CIA402_OD_META[7U];
        case CIA402_OD_MODES_OF_OPERATION_DISP: return &CIA402_OD_META[8U];
        case CIA402_OD_POSITION_ACTUAL:         return &CIA402_OD_META[9U];
        case CIA402_OD_VELOCITY_ACTUAL:         return &CIA402_OD_META[10U];
        case CIA402_OD_TARGET_TORQUE:           return &CIA402_OD_META[11U];
        case CIA402_OD_TORQUE_ACTUAL:           return &CIA402_OD_META[12U];
        case CIA402_OD_CURRENT_ACTUAL:          return &CIA402_OD_META[13U];
        case CIA402_OD_DC_LINK_VOLTAGE:         return &CIA402_OD_META[14U];
        case CIA402_OD_TARGET_POSITION:         return &CIA402_OD_META[15U];
        case CIA402_OD_TARGET_VELOCITY:         return &CIA402_OD_META[16U];
        case CIA402_OD_QUICK_STOP_DECELERATION: return &CIA402_OD_META[17U];
        case CIA402_OD_SUPPORTED_DRIVE_MODES:   return &CIA402_OD_META[18U];
        default:                                return NULL;
    }
}

OD_Info_T Cia402_Od_GetInfo(uint16_t index, uint8_t subindex)
{
    if (subindex != 0U) { return OD_ABSENT; }

    const Cia402_OdMeta_T * p_meta = _TableIndex(index, subindex);
    if (p_meta == NULL) { return OD_ABSENT; }
    return (OD_Info_T) { .Type = p_meta->Type, .Access = p_meta->Access, .Size = OD_Type_Size(p_meta->Type) };
}



