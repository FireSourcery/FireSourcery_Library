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
    @brief  [MotVarId_T] - wire id. Namespace/Type enums shared by all MotProtocol transports.
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
        uint16_t Type           : 4U; /* enum type literal / struct type / handler / table. some cases n:1 TypeObject */
        uint16_t Prefix         : 4U; /* Namespace */
        uint16_t Instance       : 2U; /* Instance. instance > 4 can use Prefix or Resv */
        uint16_t Resv           : 2U;
    };
    uint16_t Value;
}
MotVarId_T;

/* MotVarId_Prefix_T, *VarType */
#define MOT_VAR_ID_TYPE_ID(Prefix, Type) ((uint16_t)(((Prefix) << 4U) | (Type)))

/*
    Prefixs
    Namespaces for struct type to restart at 0 index

    [Motor_T] holds more groups than one Type nibble addresses, so it spans
    several Prefixs - all resolved to one instance by [Motor_MotVar].
*/
typedef enum MotVarId_Prefix
{
    MOT_VAR_ID_PREFIX_MOTOR,
    MOT_VAR_ID_PREFIX_MOTOR_SUB_MODULE,
    MOT_VAR_ID_PREFIX_MOTOR_SENSOR,
    MOT_VAR_ID_PREFIX_MOTOR_RESV,
    MOT_VAR_ID_PREFIX_GENERAL,
    MOT_VAR_ID_PREFIX_V_MONITOR,
    MOT_VAR_ID_PREFIX_HEAT_MONITOR,
    MOT_VAR_ID_PREFIX_COMMUNICATION,
    MOT_VAR_ID_PREFIX_SYSTEM_COMMAND,
    MOT_VAR_ID_PREFIX_APP_USER,
    _MOT_VAR_ID_PREFIX_END,
}
MotVarId_Prefix_T;


/******************************************************************************/
/*
    [MotorController_VarType]

    Types
    Type of Base
        each id directly corresponds to var base literal enum type
        Corresponds to the "object type". accounts for type literal and specialized properties.
    partition by Prefix. Each sub-enum indexes within its Prefix group.
*/
/******************************************************************************/
typedef enum MotorController_VarType_General
{
    MOT_VAR_TYPE_GENERAL_USER_OUT,
    MOT_VAR_TYPE_GENERAL_USER_IN,
    // MOT_VAR_TYPE_GENERAL_USER_CONTROL,
    MOT_VAR_TYPE_GENERAL_CONFIG,
    MOT_VAR_TYPE_GENERAL_DEBUG,
    MOT_VAR_TYPE_GENERAL_BOARD_CONST,
    MOT_VAR_TYPE_ANALOG_USER_VAR_OUT, // peripheral status
    MOT_VAR_TYPE_ANALOG_USER_CONFIG,

    MOT_VAR_TYPE_OPT_DIN_CONFIG, // common values
    MOT_VAR_TYPE_USER_DIN_STATE,
    MOT_VAR_TYPE_USER_DIN_CONFIG, // instanced

    MOT_VAR_TYPE_USER_AIN_STATE,
    MOT_VAR_TYPE_USER_AIN_CONFIG,
    // MOT_VAR_TYPE_BUZZER_CONTROL,
    // MOT_VAR_TYPE_BUZZER_CONFIG,
    // MOT_VAR_TYPE_RELAY_CONFIG,
    _MOT_VAR_TYPE_GENERAL_END,
}
MotorController_VarType_General_T;

typedef enum MotorController_VarType_VMonitor
{
    /*
        Specialized instances. effectively access as object classes. simplify static value bounds.
    */
    MOT_VAR_TYPE_VBUS_OUT,
    MOT_VAR_TYPE_VBUS_CONFIG, /* VNominal, Derate scaling */
    MOT_VAR_TYPE_V_MONITOR_VBUS_STATE,
    MOT_VAR_TYPE_V_MONITOR_VBUS_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_VBUS_VDIVIDER,

    MOT_VAR_TYPE_V_MONITOR_ACCS_STATE,
    MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER,

    MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE,
    MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER,
    _MOT_VAR_TYPE_V_MONITOR_END,
}
MotorController_VarType_VMonitor_T;

typedef enum MotorController_VarType_HeatMonitor
{
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR, /* read-only coefficients */

    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE, /* 0-3 */
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR, /* 0-3 */
    // MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_CONFIG, /* reserved */
    _MOT_VAR_TYPE_HEAT_MONITOR_END,
}
MotorController_VarType_HeatMonitor_T;

typedef enum MotorController_VarType_Communication
{
    /* Communication */
    MOT_VAR_TYPE_SOCKET_STATE,
    MOT_VAR_TYPE_SOCKET_CONFIG, /* Instance by Protocol Count */
    MOT_VAR_TYPE_CAN_STATE,
    MOT_VAR_TYPE_CAN_CONFIG,
    MOT_VAR_TYPE_CIA_402_STATE,
    MOT_VAR_TYPE_CIA_402_CONFIG, /* Instanced */
    _MOT_VAR_TYPE_COMMUNICATION_END,
}
MotorController_VarType_Communication_T;

/*
    Application_User SubModules
    app table handle compile time define
*/
typedef enum MotorController_VarType_AppUser
{
    MOT_VAR_TYPE_TRACTION_CONTROL,
    MOT_VAR_TYPE_TRACTION_CONFIG,
    _MOT_VAR_TYPE_APP_USER_END,
}
MotorController_VarType_AppUser_T;

/******************************************************************************/
/*
    [Motor_VarType]
    Identifies the struct, object, or interface segment.
    VarType directly corresponds to base enum type literal
    partition by Prefix. Each sub-enum indexes within its Prefix group.
*/
/******************************************************************************/
typedef enum Motor_VarType_Base
{
    MOTOR_VAR_TYPE_USER_OUT,
    MOTOR_VAR_TYPE_USER_CONTROL, /* Polling IO. Setpoint/StateMachine. */
    MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
    MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
    MOTOR_VAR_TYPE_OPEN_LOOP_CMD,
    MOTOR_VAR_TYPE_CALIBRATION_CMD,
    MOTOR_VAR_TYPE_CMD_RESV,
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
    MOTOR_VAR_TYPE_CONFIG_ACTUATION,
    MOTOR_VAR_TYPE_CONFIG_PID,
    MOTOR_VAR_TYPE_CONFIG_DEBUG,
    MOTOR_VAR_TYPE_CONFIG_RESV,
    _MOTOR_VAR_TYPE_BASE_END,
}
Motor_VarType_Base_T;


typedef enum Motor_VarType_SubModule
{
    MOTOR_VAR_TYPE_BOARD_CONST,    /* Not instanced */
    MOTOR_VAR_TYPE_ROTOR_OUT, /* Common generic interface */
    MOTOR_VAR_TYPE_PHASE,
    MOTOR_VAR_TYPE_PHASE_INPUT,
    MOTOR_VAR_TYPE_HEAT_MONITOR_OUT,    /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG, /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_THERMISTOR_CONFIG,
    MOTOR_VAR_TYPE_PID_TUNING_IO,       /* Non polling. PID tunning with non-Config state access permissions */
    MOTOR_VAR_TYPE_FOC_OUT,
    MOTOR_VAR_TYPE_FOC_CONFIG,
    // opt move to sensor module
    MOTOR_VAR_TYPE_FOC_SENSORLESS,
    MOTOR_VAR_TYPE_FOC_SENSORLESS_CONFIG,
    _MOTOR_VAR_TYPE_SUB_MODULE_END,
}
Motor_VarType_SubModule_T;

/*
    Instead of using SensorTable Ids, This way it takes only one field to associate properties.
*/
typedef enum Motor_VarType_Sensor
{
    MOTOR_VAR_TYPE_HALL_STATE,
    MOTOR_VAR_TYPE_HALL_CONFIG,
    MOTOR_VAR_TYPE_HALL_CMD,
    MOTOR_VAR_TYPE_ENCODER_STATE,
    MOTOR_VAR_TYPE_ENCODER_CONFIG,
    MOTOR_VAR_TYPE_ENCODER_CMD,
    _MOTOR_VAR_TYPE_SENSOR_END,
}
Motor_VarType_Sensor_T;



/******************************************************************************/
/*
    [MotVarId_T] field widths are wire format - enum values are frozen by the
    transport mapping. [MotCan] derives its CANopen index from (Prefix, Type)
    and its subindex from Base, so an overflowing enum silently aliases ids.
*/
/******************************************************************************/
static_assert(sizeof(MotVarId_T) == sizeof(uint16_t), "MotVarId_T must serialize as uint16_t");

#define MOT_VAR_ID_NIBBLE_MAX (16U)

static_assert(_MOT_VAR_ID_PREFIX_END          <= MOT_VAR_ID_NIBBLE_MAX, "Prefix exceeds MotVarId.Prefix");
static_assert(_MOT_VAR_TYPE_GENERAL_END       <= MOT_VAR_ID_NIBBLE_MAX, "General Type exceeds MotVarId.Type");
static_assert(_MOT_VAR_TYPE_V_MONITOR_END     <= MOT_VAR_ID_NIBBLE_MAX, "VMonitor Type exceeds MotVarId.Type");
static_assert(_MOT_VAR_TYPE_HEAT_MONITOR_END  <= MOT_VAR_ID_NIBBLE_MAX, "HeatMonitor Type exceeds MotVarId.Type");
static_assert(_MOT_VAR_TYPE_COMMUNICATION_END <= MOT_VAR_ID_NIBBLE_MAX, "Communication Type exceeds MotVarId.Type");
static_assert(_MOT_VAR_TYPE_APP_USER_END      <= MOT_VAR_ID_NIBBLE_MAX, "AppUser Type exceeds MotVarId.Type");
static_assert(_MOTOR_VAR_TYPE_BASE_END        <= MOT_VAR_ID_NIBBLE_MAX, "Motor Base Type exceeds MotVarId.Type");
static_assert(_MOTOR_VAR_TYPE_SUB_MODULE_END  <= MOT_VAR_ID_NIBBLE_MAX, "Motor SubModule Type exceeds MotVarId.Type");
static_assert(_MOTOR_VAR_TYPE_SENSOR_END      <= MOT_VAR_ID_NIBBLE_MAX, "Motor Sensor Type exceeds MotVarId.Type");

/******************************************************************************/
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

