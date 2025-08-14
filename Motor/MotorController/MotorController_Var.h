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
typedef enum MotorController_Var_Output
{
    // MOTOR_CONTROLLER_VAR ,
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_MC_STATE,
    MOT_VAR_MC_STATUS_FLAGS,
    MOT_VAR_MC_FAULT_FLAGS,
    MOT_VAR_CONTROL_LOOP_PROFILE,
    MOT_VAR_CONTROL_LOOP_MOTOR_PROFILE,
    // MOT_DRIVE_DIRECTION,
}
MotorController_Var_Output_T;

typedef enum MotorController_Var_OutputDebug
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
MotorController_Var_OutputDebug_T;

/******************************************************************************/
/*
*/
/******************************************************************************/
/*
    Collective set motors for convenience
    Write only, or io
*/
typedef enum MotorController_Var_Input
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
MotorController_Var_Input_T;

/******************************************************************************/
/*
    Config - Nvm variables
*/
/******************************************************************************/
typedef enum MotorController_Var_Config
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
MotorController_Var_Config_T;

typedef enum MotorController_Var_ConfigBootRef
{
    MOT_VAR_BOOT_REF_FAST_BOOT,
    MOT_VAR_BOOT_REF_BEEP,
    MOT_VAR_BOOT_REF_BLINK,
    // MOT_VAR_BOOT_REF_WORD_VALUE,
    // MOT_VAR_BOOT_REF_PROTOCOL_INDEX,
}
MotorController_Var_ConfigBootRef_T;

/******************************************************************************/
/*!
    Read-only Reference
*/
/******************************************************************************/
typedef enum MotorController_Var_StaticRef
{
    MOT_VAR_REF_MOTOR_COUNT,
    MOT_VAR_REF_V_MONITOR_COUNT, /*  */
    MOT_VAR_REF_THERMISTOR_MOSFETS_COUNT,
    MOT_VAR_REF_PROTOCOL_SOCKET_COUNT,
    MOT_VAR_REF_CAN_SOCKET_COUNT,
    // MOT_VAR_REF_MAIN_SOFTWARE_VERSION, /* Read-only */
}
MotorController_Var_StaticRef_T;

/******************************************************************************/
/*
    Types
    Type of Base
        directly corresponds to enum type containing index ids
        Corresponds to the "object type". accounts for type literal and specialized properties.
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
    MOT_VAR_TYPE_GENERAL_USER_OUT,
    MOT_VAR_TYPE_GENERAL_USER_IN, /* Polling inputs */
    MOT_VAR_TYPE_GENERAL_CONFIG,
    MOT_VAR_TYPE_BOOT_REF_CONFIG,
    MOT_VAR_TYPE_GENERAL_DEBUG,
    MOT_VAR_TYPE_GENERAL_REF, /* Read-only */

    /* MotDrive Submodule */
    MOT_VAR_TYPE_MOT_DRIVE_CONTROL,
    MOT_VAR_TYPE_MOT_DRIVE_CONFIG,

    MOT_VAR_TYPE_ANALOG_USER_VAR_OUT, // peripheral status
    MOT_VAR_TYPE_ANALOG_USER_CONFIG,

    // MOT_VAR_TYPE_BUZZER_CONTROL,
    // MOT_VAR_TYPE_BUZZER_CONFIG,

    /*  */
    // MOT_VAR_TYPE_OPT_DIN_CONFIG,
    // MOT_VAR_TYPE_RELAY_CONFIG,
}
MotorController_VarType_General_T;

typedef enum MotorController_VarType_VMonitor
{
    MOT_VAR_TYPE_V_MONITOR_SOURCE_STATE,
    MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_SOURCE_VDIVIDER_REF,

    MOT_VAR_TYPE_V_MONITOR_ACCS_STATE,
    MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER_REF,

    MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE,
    MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER_REF,
}
MotorController_VarType_VMonitor_T;

typedef enum MotorController_VarType_HeatMonitor
{
    /* Heat monitors sub types motor, pcb, mosfet */
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF, /* read-only coeffcients */

    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE, /* 0-3 */
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF, /* 0-3 */
    // MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_CONFIG, /* reserved */
}
MotorController_VarType_HeatMonitor_T;

typedef enum MotorController_VarType_Communication
{
    MOT_VAR_TYPE_PROTOCOL_STATE,
    MOT_VAR_TYPE_PROTOCOL_CONFIG, /* Instance by Protocol Count */

    MOT_VAR_TYPE_CAN_BUS_STATE,
    MOT_VAR_TYPE_CAN_BUS_CONFIG,
}
MotorController_VarType_Communication_T;

/******************************************************************************/
/*
    [MotorController_VarHandlerType]
    Handler by Source File Module
    Effectively name space for types
    determine handler logic by id
    Mutually exclusive attributes when possible
    partition for contigously expandable ids
*/
/******************************************************************************/
typedef enum MotVarId_HandlerType
{
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONTROL,
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG,
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR, /* Generic */
    MOT_VAR_ID_HANDLER_TYPE_MOTOR_SUB_MODULE,
    MOT_VAR_ID_HANDLER_TYPE_GENERAL,
    MOT_VAR_ID_HANDLER_TYPE_V_MONITOR,
    MOT_VAR_ID_HANDLER_TYPE_HEAT_MONITOR,
    MOT_VAR_ID_HANDLER_TYPE_COMMUNICATION,
    // MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND,
    _MOT_VAR_ID_HANDLER_TYPE_END,
}
MotVarId_HandlerType_T;

/******************************************************************************/
/*
    [MotVarId]
    Type index
*/
/******************************************************************************/
extern int MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId);
extern MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_context, MotVarId_T varId, int varValue);



/* Map to sub module if supported */
// typedef enum MotVarId_AccessType
// {
//     MOT_VAR_ACCESS_VAR_IO,
//     MOT_VAR_ACCESS_VAR_CMD,
//     MOT_VAR_ACCESS_CONFIG,
//     MOT_VAR_ACCESS_CONFIG_CMD,
//     MOT_VAR_ACCESS_CONST_INFO,
// } MotVarId_AccessType_T;


/******************************************************************************/
/*!
    Instance Select
*/
/******************************************************************************/
// typedef enum MotVarId_Instance_VMonitor
// {
//     MOT_VAR_ID_V_MONITOR_SOURCE,
//     MOT_VAR_ID_V_MONITOR_ACCS,
//     MOT_VAR_ID_V_MONITOR_ANALOG,
//     // MOT_VAR_ID_V_MONITOR_AUX,
// }
// MotVarId_Instance_VMonitor_T;

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
