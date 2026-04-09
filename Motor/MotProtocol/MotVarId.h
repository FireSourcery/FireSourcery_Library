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
typedef union MotVarId
{
    struct
    {
        uint16_t Base           : 4U; /* Name - corresponds with enum index value. Field Id */
        uint16_t InnerType      : 6U; /* Accessor. Corresponds with Base enum type, may be n:1 handlers to enum type literal. Struct Type */
        uint16_t OuterType      : 1U; /* 0-Motor, 1-MotorController */
        uint16_t Instance       : 2U; /* Instance */
        uint16_t Resv           : 3U;
    };
    uint16_t Value;
}
MotVarId_T;

// typedef union MotVarId
// {
//     struct
//     {
//         uint16_t Base           : 4U; /* Name - corresponds with enum index value. Struct member */
//         uint16_t Type           : 4U; /* enum type literal / struct type. Corresponds with Base enum type, may be n:1, as TypeObject */
                                            /* includes the access type */
                                            // Acess:2
                                            //extended:2
//         uint16_t Prefix      : 4U; /* Namespace, zero offset */ /* includes the source module */
//         uint16_t Instance       : 2U; /* Instance. instance > 4 can use ParitionId or Resv */
//         uint16_t Resv           : 2U;
//     };
//     uint16_t Value;
// }
// MotVarId_T;

#define MOT_VAR_ID_TYPE_ID(Partition, Type) ((uint16_t)(((Partition) << 4U) | (Type)))
/* Partitions Type to start as 0 index */
// typedef enum MotVarId_Partition
// {
//     MOT_VAR_ID_PARITION_MOTOR,
//     MOT_VAR_ID_PARITION_MOTOR_SUB_MODULE,
//     MOT_VAR_ID_PARITION_MOTOR_SENSOR,
//     MOT_VAR_ID_PARITION_GENERAL,
//     MOT_VAR_ID_PARITION_V_MONITOR,
//     MOT_VAR_ID_PARITION_HEAT_MONITOR,

//     MOT_VAR_ID_PARITION_V_MONITOR_SOURCE,,
//     MOT_VAR_ID_PARITION_V_MONITOR_AUX,
//     MOT_VAR_ID_PARITION_HEAT_MONITOR_PCB,
//     MOT_VAR_ID_PARITION_HEAT_MONITOR_MOSFETS,
//     MOT_VAR_ID_PARITION_COMMUNICATION,
//     MOT_VAR_ID_PARITION_SYSTEM_COMMAND,
//     MOT_VAR_ID_PARITION_APP,
//     _MOT_VAR_ID_PARITION_END,
// }
// MotVarId_Partition_T;

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

