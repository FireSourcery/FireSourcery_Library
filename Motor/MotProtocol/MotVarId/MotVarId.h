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
    corresponds to enum type containing index ids
*/
typedef enum MotVarId_Type_Service
{
    MOT_VAR_ID_TYPE_GENERAL_VAR_OUT, // GENERAL State
    MOT_VAR_ID_TYPE_GENERAL_CONFIG,
    // MOT_VAR_ID_TYPE_OPT_DIN_CONFIG,

    MOT_VAR_ID_TYPE_ANALOG_USER_VAR_OUT, // peripheral status
    MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG,

    MOT_VAR_ID_TYPE_MONITOR_VAR_OUT, /* all monitors */
    MOT_VAR_ID_TYPE_V_MONITOR_SOURCE_CONFIG,
    MOT_VAR_ID_TYPE_V_MONITOR_AUX_CONFIG,           /* Instanced */
    MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG,    /* Instanced */
    MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG,
    // MOT_VAR_ID_TYPE_THERMISTOR_MOSFETS, /* alternatively seperate descriptors */
    // MOT_VAR_ID_TYPE_THERMISTOR_PCB, /* alternatively seperate descriptors */

    MOT_VAR_ID_TYPE_PROTOCOL_CONFIG, /* Instance by Protocol Count */
    // MOT_VAR_ID_TYPE_CAN_BUS_CONFIG,
    MOT_VAR_ID_TYPE_BOOT_REF_CONFIG,

    MOT_VAR_ID_TYPE_BOARD_REF, /* Read-only */
    MOT_VAR_ID_TYPE_DEBUG,
}
MotVarId_Type_Service_T;

/* Instance count of Motor */
typedef enum MotVarId_Type_MotorVar
{
    MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_METRICS,
    MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_FOC,
    MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_SENSOR, /* Generic */
    MOT_VAR_ID_TYPE_MOTOR_VAR_IO,
    MOT_VAR_ID_TYPE_MOTOR_VAR_CMD,
    // MOT_VAR_ID_TYPE_MOTOR_VAR_OPEN_LOOP_CMD,
    MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING, /* PID Config with non-Config state permissions  */
    // _MOT_VAR_ID_TYPE_MOTOR_VAR_END = 15U,
}
MotVarId_Type_MotorVar_T;

typedef enum MotVarId_Type_MotorConfig
{
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_ACTUATION,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_PID,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION_ALIAS,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_THERMISTOR,

    MOT_VAR_ID_TYPE_MOTOR_CONFIG_ROUTINE, /* Calibration Cmds */
    MOT_VAR_ID_TYPE_MOTOR_CONFIG_BOARD_REF, /* Not instanced */
}
MotVarId_Type_MotorConfig_T;

typedef enum MotVarId_Type_MotorSensor
{
    MOT_VAR_ID_TYPE_MOTOR_SENSOR_STATE, // common, generic
    MOT_VAR_ID_TYPE_MOTOR_SENSOR_CONFIG,
    MOT_VAR_ID_TYPE_MOTOR_HALL_STATE,
    MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG,
    MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE,
    MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG,
}
MotVarId_Type_MotorSensor_T;

typedef enum MotVarId_Type_Command
{
    MOT_VAR_ID_TYPE_COMMAND_MOTOR_CONTEXT, /* passthrough */
    // MOT_VAR_ID_TYPE_COMMAND_MOT_DRIVE, /* MotDrive StateMachine */
    // MOT_VAR_ID_TYPE_COMMAND_SYSTEM, /* Alternatively, handle with Call */
    MOT_VAR_ID_TYPE_COMMAND_BLOCKING, // Nvm + Calibration
    // MOT_VAR_ID_TYPE_COMMAND_NVM,
    // MOT_VAR_ID_TYPE_COMMAND_CONFIG,
}
MotVarId_Type_Command_T;


/******************************************************************************/
/*
    [MotVarId_TypeType]
    Ideally mutually exclusive attribute groups when possible
        determine handler logic by id
    MotVarId_Handler/SubModule
*/
/******************************************************************************/
typedef enum MotVarId_TypeType
{
    MOT_VAR_ID_TYPE_SERVICE,
    // corresponding write only/subroutine, no retaining var, optionally return a status
    MOT_VAR_ID_TYPE_COMMAND,

    /* Instance count of Motor */
    MOT_VAR_ID_TYPE_MOTOR_VAR,
    // MOT_VAR_ID_TYPE_MOTOR_STREAM,
    MOT_VAR_ID_TYPE_MOTOR_CONFIG,
    MOT_VAR_ID_TYPE_MOTOR_SENSOR, // if types become excess

    /* Misc submodules */
    // MOT_VAR_ID_TYPE_SUBMODULE,

    MOT_VAR_ID_TYPE_TYPE_END = 8U,
}
MotVarId_TypeType_T;

typedef union MotVarId
{
    struct
    {
        uint16_t NameBase       : 4U; /* Name - corresponds 1:1 with enum value */
        uint16_t NameType       : 4U; /* Name's Type - corresponds 1:1 with enum type */
        uint16_t NameTypeType   : 3U; /* Type's Type */
        uint16_t Instance       : 3U; /* TypeInstance1 - Upto 8 Instances Per Type */
        uint16_t Alt            : 2U; /* Alternative unit/format */
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

/*
    Status Response for Read/Write
*/
typedef enum MotVarId_Status
{
    MOT_VAR_STATUS_OK,
    MOT_VAR_STATUS_ERROR,
    // MOT_VAR_STATUS_ERROR_INVALID_ID,
    MOT_VAR_STATUS_ERROR_READ_ONLY,
    MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED,
    MOT_VAR_STATUS_ERROR_RUNNING, // Not in config state
    // MOT_VAR_STATUS_ERROR_REJECT_BY_STATE_MACHINE,
    // MOT_VAR_STATUS_ASYNC,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVarId_Status_T;

/*
    Type of NameBase e.g.
    enum type Id
*/
// typedef enum MotVarId_Type_RealTime /* : uint16_t */
// {
//     MOT_VAR_ID_TYPE_MONITOR_MOTOR,
//     MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC,
//     MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR,
//     MOT_VAR_ID_TYPE_CONTROL_MOTOR,
//     MOT_VAR_ID_TYPE_CMD_MOTOR,

//     MOT_VAR_ID_TYPE_MONITOR_GENERAL,
//     MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER,
//     MOT_VAR_ID_TYPE_CMD_GENERAL,
//     MOT_VAR_ID_TYPE_CONTROL_GENERAL,
//     MOT_VAR_ID_TYPE_MOT_DRIVE,

//     // MOT_VAR_ID_TYPE_VAR_PROTOCOL,
//     // MOT_VAR_ID_TYPE_CMD_SYSTEM,
//     MOT_VAR_ID_TYPE_DEBUG,
//     MOT_VAR_ID_TYPE_REAL_TIME_END = 15U,
// }
// MotVarId_Type_RealTime_T;

// /* Mot_VarTypeConfig */
// typedef enum MotVarId_Type_Config
// {
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY,
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY,
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_CALIBRATION_ALIAS,

//     // MOT_VAR_ID_TYPE_CONFIG_MOTOR_SENSOR, generic
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL,
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER,

//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR,
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID,
//     MOT_VAR_ID_TYPE_CONFIG_MOTOR_ROUTINE,

//     MOT_VAR_ID_TYPE_CONFIG_GENERAL,
//     MOT_VAR_ID_TYPE_CONFIG_ANALOG_USER,
//     MOT_VAR_ID_TYPE_CONFIG_VMONITOR,
//     MOT_VAR_ID_TYPE_CONFIG_BOARD_THERMISTOR,
//     MOT_VAR_ID_TYPE_CONFIG_PROTOCOL,
//     MOT_VAR_ID_TYPE_CONFIG_BOOT_REF,
//     MOT_VAR_ID_TYPE_CONFIG_END = 15U,
//     // MOT_VAR_ID_TYPE_CONFIG_THERMISTOR,
// }
// MotVarId_Type_Config_T;