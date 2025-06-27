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

// typedef union { uint32_t Unsigned; int32_t Signed; } var32_t;
// typedef union { uint16_t Unsigned; int16_t Signed; } var16_t;
// typedef union { uint8_t Unsigned; int8_t Signed; } var8_t;

// typedef void (*set_var_t)(void * p_context, size_t key, int value);
// typedef int(*get_var_t)(const void * p_context, size_t key);

/******************************************************************************/
/*
    [MotVarId]
    Type index
*/
/******************************************************************************/
/*
    Type of NameBase e.g.
    directly corresponds to enum type containing index ids
        may be n:1 type
*/

/******************************************************************************/
/*
    [MotVarId_HandlerType]
    MotVarId_Handler / Source FIle Module

    Ideally mutually exclusive attribute groups when possible
        determine handler logic by id
*/
/******************************************************************************/
typedef enum MotVarId_HandlerType
{
    /* Instance count of Motor */
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR,
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG,
    /* Sensor Table Entries */
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR_STATE,
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR_CONFIG,

    /* */
    MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE,
    MOT_VAR_ID_HANDLER_TYPE_MONITOR,

    // MOT_VAR_ID_HANDLER_TYPE_USER_INPUT,
    // MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND,

    _MOT_VAR_ID_HANDLER_TYPE_END = 8U,
}
MotVarId_HandlerType_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef union MotVarId
{
    struct
    {
        uint16_t NameBase       : 4U; /* Name - corresponds 1:1 with enum value */
        uint16_t NameType       : 4U; /* Name's Type - corresponds with enum type. may be a subtype, instance type n:1, in case of discontinous mapping */
        uint16_t Instance       : 3U; /* TypeInstance - Upto 8 Instances Per Type */
        uint16_t HandlerType    : 3U; /* NameType's Type */
        uint16_t Resv           : 2U; /* Resv/Alternative unit/format */
    };

    struct
    {
        uint16_t NamePart       : 8U;
        uint16_t InstancePart   : 3U;
        uint16_t HandlerPart    : 3U;
        uint16_t ResvPart       : 2U;
    };
    uint16_t Value;
}
MotVarId_T;


/******************************************************************************/
/*!
    @brief Access control functions based on NameType patterns
*/
/******************************************************************************/
// static inline bool MotVarId_IsConfig(MotVarId_T varId)
// {
//     switch(varId.HandlerType)
//     {
//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:
//             return true; // Config types are always config

//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:
//             // switch (varId.NameType)
//             // {
//             //     case MOTOR_VAR_TYPE_HALL_CONFIG:
//             //     case MOTOR_VAR_TYPE_ENCODER_CONFIG: return true; // Sensor config types are config
//             //     default: return false; // Other sensor types are not config
//             // }

//         case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:
//             switch (varId.NameType)
//             {
//                 case MOT_VAR_TYPE_GENERAL_CONFIG:
//                 case MOT_VAR_TYPE_ANALOG_USER_CONFIG:
//                 case MOT_VAR_TYPE_PROTOCOL_CONFIG:
//                 case MOT_VAR_TYPE_CAN_BUS_CONFIG: return true; // System service config types are config
//                 default: return false; // Other system service types are not config
//             }

//         default: return false; // Default to not config for other handlers
//     }
// }

// // Access pattern detection by NameType suffix/pattern
// static inline bool MotVarId_IsReadOnly(MotVarId_T varId)
// {
//     switch(varId.HandlerType)
//     {
//         // case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:
//         //     switch (varId.NameType)
//         //     {
//         //         case MOTOR_VAR_TYPE_USER_OUT:
//         //         case MOTOR_VAR_TYPE_FOC_OUT:
//         //         case MOTOR_VAR_TYPE_SPEED_ANGLE_OUT: return true;  // *_OUT types are read-only
//         //         case MOTOR_VAR_TYPE_PID_TUNING_IO:
//         //         case MOTOR_VAR_TYPE_USER_IO: return false; // *_IO types are read-write
//         //         case MOTOR_VAR_TYPE_CMD_IN: return false; // Commands are execute-only (writable)
//         //         default: return true;  // Default to read-only for safety
//         //     }

//         // case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:
//         //     return false; // Config types are read-write

//         // case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:
//         //     switch (varId.NameType)
//         //     {
//         //         case MOTOR_VAR_TYPE_HALL_STATE:
//         //         case MOTOR_VAR_TYPE_ENCODER_STATE: return true;  // State types are read-only
//         //         case MOTOR_VAR_TYPE_HALL_CONFIG:
//         //         case MOTOR_VAR_TYPE_ENCODER_CONFIG: return false; // Config types are read-write
//         //         default: return true;  // Default to read-only for safety
//         //     }

//         case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:
//             switch (varId.NameType)
//             {
//                 case MOT_VAR_TYPE_GENERAL_VAR_OUT:
//                 case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT: return true;  // *_OUT types are read-only
//                 case MOT_VAR_TYPE_GENERAL_CONFIG:
//                 case MOT_VAR_TYPE_ANALOG_USER_CONFIG: return false; // Config types are read-write
//                 case MOT_VAR_TYPE_PROTOCOL_CONFIG:
//                 case MOT_VAR_TYPE_CAN_BUS_CONFIG: return false; // Protocol and CAN bus configs are read-write
//                 case MOT_VAR_TYPE_INSTANCES_REF:
//                 case MOT_VAR_TYPE_DEBUG: return true;  // Read-only
//                 default: return true;  // Default to read-only for safety
//             }

//         case MOT_VAR_ID_HANDLER_TYPE_MONITOR:
//             switch (varId.NameType)
//             {
//                 case MOT_VAR_TYPE_V_MONITOR_INSTANCE_STATE: return true;
//                 case MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE: return true;
//                 case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE: return true;  // State types are read-only
//                 case MOT_VAR_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF: return true;  // VDivider reference is read-only
//                 case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF: return true;  // Thermistor reference is read-only

//                 case MOT_VAR_TYPE_V_MONITOR_INSTANCE_CONFIG:
//                 case MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG:
//                 case MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG: return false; // Config types are read-write
//                 default: return true;  // Default to read-only for safety
//             }

//         case MOT_VAR_ID_HANDLER_TYPE_USER_INPUT: return false; // Commands are execute-only (writable)

//         default: return true;  // Default to read-only for safety
//     }
// }


// // Main access control functions
// static inline bool MotVarId_IsReadable(MotVarId_T var_id)
// {
//     uint8_t handler_type = (var_id.Value >> 8) & 0x7;
//     uint8_t name_type = (var_id.Value >> 4) & 0xF;

//     return MotVarId_IsReadOnly_ByNameType(name_type, handler_type) ||
//         MotVarId_IsReadWrite_ByNameType(name_type, handler_type);
// }

// static inline bool MotVarId_IsWritable(MotVarId_T var_id)
// {
//     uint8_t handler_type = (var_id.Value >> 8) & 0x7;
//     uint8_t name_type = (var_id.Value >> 4) & 0xF;

//     return MotVarId_IsReadWrite_ByNameType(name_type, handler_type) ||
//         MotVarId_IsCommand_ByNameType(name_type, handler_type);
// }

// static inline bool MotVarId_IsCommand(MotVarId_T var_id)
// {
//     uint8_t handler_type = (var_id.Value >> 8) & 0x7;
//     uint8_t name_type = (var_id.Value >> 4) & 0xF;

//     return MotVarId_IsCommand_ByNameType(name_type, handler_type);
// }


/*
    Status Response for Read/Write
*/
typedef enum MotVarId_Status
{
    MOT_VAR_STATUS_OK,
    MOT_VAR_STATUS_ERROR,
    MOT_VAR_STATUS_ERROR_INVALID_ID,
    MOT_VAR_STATUS_ERROR_READ_ONLY,
    MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED,
    MOT_VAR_STATUS_ERROR_RUNNING, // Not in config state
    // MOT_VAR_STATUS_ASYNC,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVarId_Status_T;

