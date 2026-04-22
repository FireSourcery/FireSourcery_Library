#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   MotVarId.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    Type: includes the access type
    Prefix: Namespace, zero offset. includes the source module
*/
typedef union MotVarId
{
    struct
    {
        uint16_t Base           : 4U; /* Name - corresponds with enum index value. Struct member */
        uint16_t Type           : 4U; /* enum type literal / struct type. some cases n:1, as TypeObject */
        uint16_t Prefix         : 4U; /*  */
        uint16_t Instance       : 2U; /* Instance. instance > 4 can use Prefix or Resv */
        uint16_t Resv           : 2U;
    };
    uint16_t Value;
}
MotVarId_T;

#define MOT_VAR_ID_TYPE_ID(Prefix, Type) ((uint16_t)(((Prefix) << 4U) | (Type)))

/* Prefixs Type to start as 0 index */
typedef enum MotVarId_Prefix
{
    MOT_VAR_ID_PREFIX_MOTOR,
    MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE,
    MOT_VAR_ID_PREFIX_MOTOR_SENSOR,
    MOT_VAR_ID_PREFIX_MOTOR_RESV,
    MOT_VAR_ID_PREFIX_GENERAL,
    MOT_VAR_ID_PREFIX_V_MONITOR,
    MOT_VAR_ID_PREFIX_HEAT_MONITOR,
    // MOT_VAR_ID_PREFIX_V_MONITOR_SOURCE,
    // MOT_VAR_ID_PREFIX_V_MONITOR_AUX,
    // MOT_VAR_ID_PREFIX_HEAT_MONITOR_PCB,
    // MOT_VAR_ID_PREFIX_HEAT_MONITOR_MOSFETS,
    MOT_VAR_ID_PREFIX_COMMUNICATION,
    MOT_VAR_ID_PREFIX_SYSTEM_COMMAND,
    MOT_VAR_ID_PREFIX_APP_USER,
    _MOT_VAR_ID_PREFIX_END,
}
MotVarId_Prefix_T;


/*
    Status Response for Read/Write
*/
typedef enum MotVarId_Status
{
    MOT_VAR_STATUS_OK,
    MOT_VAR_STATUS_ERROR,
    MOT_VAR_STATUS_ERROR_INVALID_ID,
    MOT_VAR_STATUS_ERROR_READ_ONLY,
    MOT_VAR_STATUS_ERROR_WRITE_ONLY,
    MOT_VAR_STATUS_ERROR_ACCESS_DISABLED,
    MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE,
    MOT_VAR_STATUS_ERROR_NOT_RUNNING_STATE,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVarId_Status_T;

