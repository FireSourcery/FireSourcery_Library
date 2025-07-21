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
    @file   MotorController_Var.h
    @author FireSourcery
    @brief  Var - Field-like Property Interface Getter/Setter via Id Key
                Effectively serialize selected set of struct members
                Store in host cache and periodically poll
*/
/******************************************************************************/
#include "MotorController_User.h"
#include "MotorController.h"
#include "../Motor/Sensor/Motor_Sensor.h"
#include "../Motor/Motor_Var.h"
#include "../MotProtocol/MotVarId.h"
#include "System/SysTime/SysTime.h"

/******************************************************************************/
/*
    Var Id Base
*/
/******************************************************************************/
// MOTOR_CONTROLLER_VAR ,
typedef enum MotorController_VarOutput
{
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_MC_STATE,
    MOT_VAR_MC_STATUS_FLAGS,
    MOT_VAR_MC_FAULT_FLAGS,
    // MOT_DRIVE_DIRECTION,
}
MotorController_VarOutput_T;

typedef enum MotorController_VarOutput_Debug
{
    MOT_VAR_DEBUG0,
    MOT_VAR_DEBUG1,
    MOT_VAR_DEBUG2,
    MOT_VAR_DEBUG3,
    MOT_VAR_DEBUG4,
    MOT_VAR_DEBUG5,
    MOT_VAR_DEBUG6,
    MOT_VAR_DEBUG7,
}
MotorController_VarOutput_Debug_T;

/******************************************************************************/
/*
*/
/******************************************************************************/
/*
    Collective set motors for convenience
    Write only, or io
*/
typedef enum MotorController_VarInput
{
    MOT_VAR_USER_MOTOR_SET_POINT,          // [-32768:32767]
    MOT_VAR_USER_MOTOR_DIRECTION,          //
    MOT_VAR_USER_MOTOR_FEEDBACK_MODE,      //
    MOT_VAR_USER_MOTOR_PHASE_OUTPUT,       // includes release/hold/float

    MOT_VAR_USER_OPT_SPEED_LIMIT_ON_OFF,         // 1:Enable, 0:Disable
    MOT_VAR_USER_OPT_I_LIMIT_ON_OFF,             // 1:Enable, 0:Disable

    MOT_VAR_USER_RELAY_TOGGLE,
    MOT_VAR_USER_METER_TOGGLE,
}
MotorController_VarInput_T;


/******************************************************************************/
/*
    Config - Nvm variables
*/
/******************************************************************************/
typedef enum MotorController_ConfigId
{
    MOT_VAR_V_SUPPLY_VOLTS,
    MOT_VAR_I_LIMIT_LOW_V,
    // MOT_VAR_I_LIMIT_DC,

    MOT_VAR_USER_INIT_MODE,                 // MotorController_MainMode_T
    MOT_VAR_USER_INPUT_MODE,                // MotorController_InputMode_T
    MOT_VAR_BUZZER_FLAGS_ENABLE,            // MotorController_BuzzerFlags_T

    /* OptDin */
    MOT_VAR_OPT_DIN_FUNCTION,               // MotorController_OptDinMode_T
    MOT_VAR_OPT_SPEED_LIMIT,                // Selectable Speed Limit
    MOT_VAR_OPT_I_LIMIT,
}
MotorController_ConfigId_T;

typedef enum MotorController_Config_BootRef
{
    MOT_VAR_BOOT_REF_FAST_BOOT,
    MOT_VAR_BOOT_REF_BEEP,
    MOT_VAR_BOOT_REF_BLINK,
    // MOT_VAR_BOOT_REF_WORD_VALUE,
    // MOT_VAR_BOOT_REF_PROTOCOL_INDEX,
}
MotorController_Config_BootRef_T;

/******************************************************************************/
/*!
    Read-only Reference
*/
/******************************************************************************/
typedef enum MotorController_GeneralRef
{
    MOT_VAR_REF_MOTOR_COUNT,
    MOT_VAR_REF_V_MONITOR_COUNT, /*  */
    MOT_VAR_REF_THERMISTOR_MOSFETS_COUNT,
    MOT_VAR_REF_PROTOCOL_SOCKET_COUNT,
    MOT_VAR_REF_CAN_SOCKET_COUNT,
    // MOT_VAR_REF_MAIN_SOFTWARE_VERSION, /* Read-only */
}
MotorController_GeneralRef_T;

/******************************************************************************/
/*
    [MotVarId] Interface
*/
/******************************************************************************/
/******************************************************************************/
/*
    Types
    Type of Base
        directly corresponds to enum type containing index ids
        may be n:1
*/
/******************************************************************************/
/*
    [MotorController_VarType_General]
    General Service Types
    Single Instance
*/
typedef enum MotorController_VarType_General
{
    /* General */
    MOT_VAR_TYPE_GENERAL_VAR_OUT, // GENERAL_STATE */
    MOT_VAR_TYPE_GENERAL_CONFIG,
    MOT_VAR_TYPE_BOOT_REF_CONFIG,

    /* Polling inputs */
    MOT_VAR_TYPE_USER_INPUT,
    MOT_VAR_TYPE_MOT_DRIVE_CONTROL, /* MotDrive Submodule */
    MOT_VAR_TYPE_MOT_DRIVE_CONFIG,

    /*  */
    // MOT_VAR_TYPE_OPT_DIN_CONFIG,
    // MOT_VAR_TYPE_BUZZER_CONFIG,
    // MOT_VAR_TYPE_RELAY_CONFIG,

    MOT_VAR_TYPE_ANALOG_USER_VAR_OUT, // peripheral status
    MOT_VAR_TYPE_ANALOG_USER_CONFIG,

    MOT_VAR_TYPE_GENERAL_REF, /* Read-only */
    MOT_VAR_TYPE_DEBUG,

    // MOT_VAR_TYPE_V_MONITOR_SUPPLY_STATE,
    // MOT_VAR_TYPE_V_MONITOR_SUPPLY_CONFIG,
}
MotorController_VarType_General_T;

// alternatively vMonitors as named, staticc order
// typedef enum MotorController_VarType_VMonitor
// {
//     MOT_VAR_TYPE_V_MONITOR_INSTANCE_STATE,
//     MOT_VAR_TYPE_V_MONITOR_INSTANCE_CONFIG, /* Instanced */
//     MOT_VAR_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF,
// }
// MotorController_VarType_VMonitor_T;

// altnerively share  instancce id type when not applicable
typedef enum MotorController_VarType_Monitor
{
    MOT_VAR_TYPE_V_MONITOR_INSTANCE_STATE,
    MOT_VAR_TYPE_V_MONITOR_INSTANCE_CONFIG, /* Instanced */
    MOT_VAR_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF,

    /* Heat monitors sub types motor, pcb, mosfet */
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF, /* read-only coeffcients */
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE, /* 0-3 */
    // MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_CONFIG, /* reserved */
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF, /* 0-3 */
}
MotorController_VarType_Monitor_T;

typedef enum MotorController_VarType_Communication
{
    MOT_VAR_TYPE_PROTOCOL_CONFIG, /* Instance by Protocol Count */
    MOT_VAR_TYPE_CAN_BUS_CONFIG,
}
MotorController_VarType_Communication_T;

/******************************************************************************/
/*
    [MotorController_VarHandlerType]
    Handler by Source File Module
    determine handler logic by id
    Ideally mutually exclusive attribute groups when possible
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
    MOT_VAR_ID_HANDLER_TYPE_GENERAL,
    // MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND,
    MOT_VAR_ID_HANDLER_TYPE_MONITOR,
    MOT_VAR_ID_HANDLER_TYPE_COMMUNICATION,

    _MOT_VAR_ID_HANDLER_TYPE_END,

    // normalized to instance type, host side handler alone determines instance type
    // MOT_VAR_ID_HANDLER_TYPE_V_MONITOR,
    // MOT_VAR_ID_HANDLER_TYPE_PROTOCOL,
    // MOT_VAR_ID_HANDLER_TYPE_PCB_MONITOR, /*  */
    // MOT_VAR_ID_HANDLER_TYPE_MOSFETS_MONITOR, /*  */
}
MotVarId_HandlerType_T;

/******************************************************************************/
/*!
    Instance Select
*/
/******************************************************************************/
// optionally separate as only type named
typedef enum MotVarId_Instance_VMonitor
{
    MOT_VAR_ID_V_MONITOR_SOURCE,
    MOT_VAR_ID_V_MONITOR_ACCS,
    MOT_VAR_ID_V_MONITOR_ANALOG,
    // MOT_VAR_ID_V_MONITOR_AUX,
}
MotVarId_Instance_VMonitor_T;


/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline uint8_t MotorController_Var_GetMotorCount(const MotorController_T * p_context) { return p_context->MOTORS.LENGTH; }
static inline uint8_t MotorController_Var_GetHeatMosfetCount(const MotorController_T * p_context) { return HeatMonitor_Group_GetInstanceCount(&p_context->HEAT_MOSFETS); }
static inline uint8_t MotorController_Var_GetVMonitorCount(const MotorController_T * p_context)
{
    return (p_context->V_SOURCE.P_STATE != NULL) + (p_context->V_ACCESSORIES.P_STATE != NULL) + (p_context->V_ANALOG.P_STATE != NULL);
}
static inline uint8_t MotorController_Var_GetProtocolCount(const MotorController_T * p_context) { return p_context->PROTOCOL_COUNT; }


/******************************************************************************/
/*
    [MotVarId]
    Type index
*/
/******************************************************************************/
extern int MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId);
extern MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_context, MotVarId_T varId, int varValue);

/*
    Index corresponds to external user interface
*/
// typedef enum MotVarId_Instance_HeatMonitorMosfets
// {
//     MOT_VAR_ID_THERMISTOR_MOSFETS_0,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_1,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_2,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_3,
// }
// MotVarId_Instance_HeatMonitorMosfets_T;

// typedef enum MotVarId_Instance_Motor
// {
//     MOT_VAR_ID_MOTOR_0,
//     MOT_VAR_ID_MOTOR_1,
//     MOT_VAR_ID_MOTOR_2,
//     MOT_VAR_ID_MOTOR_3,
// }
// MotVarId_Instance_Motor_T;

// typedef enum MotVarId_Instance_ProtocolSocket
// {
//     MOT_VAR_ID_PROTOCOL_SOCKET_0,
//     MOT_VAR_ID_PROTOCOL_SOCKET_1,
//     MOT_VAR_ID_PROTOCOL_SOCKET_2,
//     MOT_VAR_ID_PROTOCOL_SOCKET_3,
// }
// MotVarId_Instance_ProtocolSocket_T;

/******************************************************************************/
/*!
    @brief Access control functions based on InnerType patterns
*/
/******************************************************************************/
// static inline bool MotVarId_IsConfig(MotVarId_T varId)
// {
//     switch(varId.OuterType)
//     {
//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:
//             return true; // Config types are always config

//         case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:
//             // switch (varId.InnerType)
//             // {
//             //     case MOTOR_VAR_TYPE_HALL_CONFIG:
//             //     case MOTOR_VAR_TYPE_ENCODER_CONFIG: return true; // Sensor config types are config
//             //     default: return false; // Other sensor types are not config
//             // }

//         case MOT_VAR_ID_HANDLER_TYPE_GENERAL:
//             switch (varId.InnerType)
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

// // Access pattern detection by InnerType suffix/pattern
// static inline bool MotVarId_IsReadOnly(MotVarId_T varId)
// {
//     switch(varId.OuterType)
//     {
//         // case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:
//         //     switch (varId.InnerType)
//         //     {
//         //         case MOTOR_VAR_TYPE_USER_OUT:
//         //         case MOTOR_VAR_TYPE_FOC_OUT:
//         //         case MOTOR_VAR_TYPE_ROTOR_OUT: return true;  // *_OUT types are read-only
//         //         case MOTOR_VAR_TYPE_PID_TUNING_IO:
//         //         case MOTOR_VAR_TYPE_USER_IO: return false; // *_IO types are read-write
//         //         case MOTOR_VAR_TYPE_CMD_IN: return false; // Commands are execute-only (writable)
//         //         default: return true;  // Default to read-only for safety
//         //     }

//         // case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:
//         //     return false; // Config types are read-write

//         // case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:
//         //     switch (varId.InnerType)
//         //     {
//         //         case MOTOR_VAR_TYPE_HALL_STATE:
//         //         case MOTOR_VAR_TYPE_ENCODER_STATE: return true;  // State types are read-only
//         //         case MOTOR_VAR_TYPE_HALL_CONFIG:
//         //         case MOTOR_VAR_TYPE_ENCODER_CONFIG: return false; // Config types are read-write
//         //         default: return true;  // Default to read-only for safety
//         //     }

//         case MOT_VAR_ID_HANDLER_TYPE_GENERAL:
//             switch (varId.InnerType)
//             {
//                 case MOT_VAR_TYPE_GENERAL_VAR_OUT:
//                 case MOT_VAR_TYPE_ANALOG_USER_VAR_OUT: return true;  // *_OUT types are read-only
//                 case MOT_VAR_TYPE_GENERAL_CONFIG:
//                 case MOT_VAR_TYPE_ANALOG_USER_CONFIG: return false; // Config types are read-write
//                 case MOT_VAR_TYPE_PROTOCOL_CONFIG:
//                 case MOT_VAR_TYPE_CAN_BUS_CONFIG: return false; // Protocol and CAN bus configs are read-write
//                 case MOT_VAR_TYPE_GENERAL_REF:
//                 case MOT_VAR_TYPE_DEBUG: return true;  // Read-only
//                 default: return true;  // Default to read-only for safety
//             }

//         case MOT_VAR_ID_HANDLER_TYPE_MONITOR:
//             switch (varId.InnerType)
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
