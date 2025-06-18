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
    Per Motor Instance
*/
/******************************************************************************/
typedef enum MotVarId_Type_MotorVar
{
    MOT_VAR_ID_TYPE_MOTOR_VAR_USER_OUT, // MOT_VAR_ID_TYPE_MOTOR_STATE_USER,
    MOT_VAR_ID_TYPE_MOTOR_VAR_FOC_OUT,
    MOT_VAR_ID_TYPE_MOTOR_VAR_SENSOR_OUT, /* Speed Angle */
    MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING_IO, /* Non polling *//* PID tunning with non-Config state permissions  */
    MOT_VAR_ID_TYPE_MOTOR_VAR_USER_IO, /* Setpoint/StateMachine IO */
    MOT_VAR_ID_TYPE_MOTOR_VAR_CMD_IN, /* Non polling */
    // MOT_VAR_ID_TYPE_MOTOR_VAR_OPEN_LOOP_CMD,
    MOT_VAR_ID_TYPE_MOTOR_VAR_HEAT_MONITOR_OUT,
    // _MOT_VAR_ID_TYPE_MOTOR_VAR_END = 15U,
}
MotVarId_Type_MotorVar_T;

typedef enum MotVarId_Type_MotorConfig
{
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION_ALIAS,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_ACTUATION,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_PID,

    MOT_VAR_ID_TYPE_MOTOR_CONFIG_HEAT_MONITOR,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_THERMISTOR,

    MOT_VAR_ID_TYPE_MOTOR_CONFIG_CMD,       /* Calibration Cmds */
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_BOARD_REF, /* Not instanced */
}
MotVarId_Type_MotorConfig_T;

/* using compile time conditional */
typedef enum MotVarId_Type_MotorSensor
{
    MOT_VAR_ID_TYPE_MOTOR_HALL_STATE,
    MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG,
    MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE,
    MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG,
}
MotVarId_Type_MotorSensor_T;

/******************************************************************************/
/*
    System Collective
*/
/******************************************************************************/
typedef enum MotVarId_Type_SystemService
{
    /* General */
    MOT_VAR_ID_TYPE_GENERAL_VAR_OUT, // GENERAL State /* STATE_VAR/VAR_OUT */
    MOT_VAR_ID_TYPE_GENERAL_CONFIG,
    // MOT_VAR_ID_TYPE_OPT_DIN_CONFIG,
    // MOT_VAR_ID_TYPE_BUZZER_CONFIG,
    // MOT_VAR_ID_TYPE_RELAY_CONFIG,
    MOT_VAR_ID_TYPE_BOOT_REF_CONFIG,
    // MOT_VAR_ID_TYPE_MOT_DRIVE_CONFIG,

    /* Communication */
    MOT_VAR_ID_TYPE_ANALOG_USER_VAR_OUT, // peripheral status
    MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG,

    MOT_VAR_ID_TYPE_PROTOCOL_CONFIG, /* Instance by Protocol Count */
    MOT_VAR_ID_TYPE_CAN_BUS_CONFIG,

    // MOT_VAR_ID_TYPE_BOARD_REF, /* Read-only */
    MOT_VAR_ID_TYPE_INSTANCES_REF, /* Read-only */
    MOT_VAR_ID_TYPE_DEBUG,
}
MotVarId_Type_SystemService_T;

typedef enum MotVarId_Type_Monitor
{
    // MOT_VAR_ID_TYPE_V_MONITOR_STATE, /* all V Monitors */
    MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_STATE,
    MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_CONFIG, /* Instanced */
    MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF, /* alternatively include in board ref */

    /* Heat monitors split sub type motor, pcb, mosfet */
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_STATE, /* optionally shared state */
    MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_STATE,
    MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG,
    MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF, /* read-only coeffcients */
    MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_STATE,
    MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG,
    MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE, /* 0-3 */
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_CONFIG, /* reserved */
    MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF, /* 0-3 */

    // MOT_VAR_ID_TYPE_V_MONITOR_STATE, /* all V Monitors */
    // MOT_VAR_ID_TYPE_V_MONITOR_SOURCE_CONFIG,
    // MOT_VAR_ID_TYPE_V_MONITOR_AUX_CONFIG,
    // MOT_VAR_ID_TYPE_V_MONITOR_ACCS_CONFIG,
    // MOT_VAR_ID_TYPE_V_MONITOR_ANALOG_CONFIG,
    // MOT_VAR_ID_TYPE_V_MONITOR_VDIVIDER_REF, /* alternatively include in board ref */

    /* instance all + mostfet collective */
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_STATE, /* Instanced */
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_CONFIG, /* Instanced, Mosfets null */
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_THERMISTOR_REF, /* Instanced */
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_GROUP_STATE,
    // MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_GROUP_CONFIG,
}
MotVarId_Type_Monitor_T;

/*
    Motor Collective commands
    Periodic mostly
*/
typedef enum  MotVarId_Type_Input
{
    MOT_VAR_ID_TYPE_INPUT_USER, /* change to io? */
    MOT_VAR_ID_TYPE_INPUT_MOT_DRIVE,  /* MotDrive StateMachine */
}
 MotVarId_Type_Input_T;

/*
    Non Polling
    write only/subroutine, no retaining var value state, optionally return a status
*/
/* Alternatively, handle with Call */
typedef enum MotVarId_Type_SystemCommand
{
    // MOT_VAR_ID_TYPE_COMMAND_SYSTEM,
    // MOT_VAR_ID_TYPE_COMMAND_BLOCKING, // Nvm + Calibration
    MOT_VAR_ID_TYPE_COMMAND_NVM,
    MOT_VAR_ID_TYPE_COMMAND_CALIBRATION,
    MOT_VAR_ID_TYPE_COMMAND_BUZZER,
    MOT_VAR_ID_TYPE_COMMAND_CONFIG,
}
MotVarId_Type_SystemCommand_T;


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
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR, // if types become excess

    /* */
    MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE,
    MOT_VAR_ID_HANDLER_TYPE_MONITOR,

    MOT_VAR_ID_HANDLER_TYPE_COMMAND,
    MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND,

    _MOT_VAR_ID_HANDLER_TYPE_END = 8U,
}
MotVarId_HandlerType_T;

typedef union MotVarId
{
    struct
    {
        uint16_t NameBase       : 4U; /* Name - corresponds 1:1 with enum value */
        uint16_t NameType       : 4U; /* Name's Type - corresponds with enum type + instance type n:1 */
        // uint16_t AccessType  : 2U;
        uint16_t Instance       : 3U; /* TypeInstance - Upto 8 Instances Per Type */
        uint16_t HandlerType    : 3U; /* NameType's Type */
        uint16_t Resv           : 2U; /* Resv/Alternative unit/format */
    };
    /* Correspond to host side */
    struct
    {
        uint16_t NamePart       : 11U; /* name can be determined by nameId + nameId_Type if prefix maps to nameId_Type 1:1 */
        uint16_t InstancePart   : 3U;
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
static inline bool MotVarId_IsConfig(MotVarId_T varId)
{
    switch(varId.HandlerType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:
            return true; // Config types are always config

        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:
            switch (varId.NameType)
            {
                case MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG:
                case MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG: return true; // Sensor config types are config
                default: return false; // Other sensor types are not config
            }

        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:
            switch (varId.NameType)
            {
                case MOT_VAR_ID_TYPE_GENERAL_CONFIG:
                case MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG:
                case MOT_VAR_ID_TYPE_PROTOCOL_CONFIG:
                case MOT_VAR_ID_TYPE_CAN_BUS_CONFIG: return true; // System service config types are config
                default: return false; // Other system service types are not config
            }

        default: return false; // Default to not config for other handlers
    }
}

// Access pattern detection by NameType suffix/pattern
static inline bool MotVarId_IsReadOnly(MotVarId_T varId)
{
    switch(varId.HandlerType)
    {
        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_VAR:
            switch (varId.NameType)
            {
                case MOT_VAR_ID_TYPE_MOTOR_VAR_USER_OUT:
                case MOT_VAR_ID_TYPE_MOTOR_VAR_FOC_OUT:
                case MOT_VAR_ID_TYPE_MOTOR_VAR_SENSOR_OUT: return true;  // *_OUT types are read-only
                case MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING_IO:
                case MOT_VAR_ID_TYPE_MOTOR_VAR_USER_IO: return false; // *_IO types are read-write
                case MOT_VAR_ID_TYPE_MOTOR_VAR_CMD_IN: return false; // Commands are execute-only (writable)
                default: return true;  // Default to read-only for safety
            }

        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG:
            return false; // Config types are read-write

        case MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR:
            switch (varId.NameType)
            {
                case MOT_VAR_ID_TYPE_MOTOR_HALL_STATE:
                case MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE: return true;  // State types are read-only
                case MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG:
                case MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG: return false; // Config types are read-write
                default: return true;  // Default to read-only for safety
            }

        case MOT_VAR_ID_HANDLER_TYPE_SYSTEM_SERVICE:
            switch (varId.NameType)
            {
                case MOT_VAR_ID_TYPE_GENERAL_VAR_OUT:
                case MOT_VAR_ID_TYPE_ANALOG_USER_VAR_OUT: return true;  // *_OUT types are read-only
                case MOT_VAR_ID_TYPE_GENERAL_CONFIG:
                case MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG: return false; // Config types are read-write
                case MOT_VAR_ID_TYPE_PROTOCOL_CONFIG:
                case MOT_VAR_ID_TYPE_CAN_BUS_CONFIG: return false; // Protocol and CAN bus configs are read-write
                case MOT_VAR_ID_TYPE_INSTANCES_REF:
                case MOT_VAR_ID_TYPE_DEBUG: return true;  // Read-only
                default: return true;  // Default to read-only for safety
            }

        case MOT_VAR_ID_HANDLER_TYPE_MONITOR:
            switch (varId.NameType)
            {
                case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_STATE:
                case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_STATE:
                case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE: return true;  // State types are read-only
                case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_VDIVIDER_REF: return true;  // VDivider reference is read-only
                case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF: return true;  // Thermistor reference is read-only

                case MOT_VAR_ID_TYPE_V_MONITOR_INSTANCE_CONFIG:
                case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG:
                case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG: return false; // Config types are read-write
                default: return true;  // Default to read-only for safety
            }

        case MOT_VAR_ID_HANDLER_TYPE_COMMAND: return false; // Commands are execute-only (writable)

        default: return true;  // Default to read-only for safety
    }

}


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

