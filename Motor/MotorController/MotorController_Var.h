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
    @brief  Var - Field-like Property Interface, Getter/Setter via Id Key
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
    Effectively serialize selected set of struct members
    Store in host cache, periodically poll
    Alternatively as Field Id
*/
/******************************************************************************/

/******************************************************************************/
/*
    Var Id Base
*/
/******************************************************************************/
// MOTOR_CONTROLLER_VAR,
/* GeneralState */
typedef enum MotorController_Var_Output
{
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_MC_STATE,
    MOT_VAR_MC_SUB_STATE,
    MOT_VAR_MC_FAULT_FLAGS,
    MOT_VAR_MC_STATUS_FLAGS, /* Resv */
    MOT_VAR_MC_DIRECTION,
}
MotorController_Var_Output_T;

typedef enum MotorController_Var_OutputDebug
{
    MOT_VAR_CONTROL_LOOP_PROFILE,
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

/*
    User Control
    Disabled on Analog Mode
    Collective set motors for convenience
    Write only, or io
*/
typedef enum MotorController_Var_Input
{
    MOT_VAR_USER_GENERAL_SET_POINT,             // [-32768:32767]
    MOT_VAR_USER_GENERAL_DIRECTION,             // Collective Direction 1:Forward, -1:Reverse, 0:Neutral, different from Motor Stop
    MOT_VAR_USER_GENERAL_FEEDBACK_MODE,         //
    MOT_VAR_USER_GENERAL_PHASE_OUTPUT,          // Output mode Float/Hold/VPwm. [Phase_Output_T]

    // MOT_VAR_USER_OPT_SPEED_LIMIT_ON_OFF,        // 1:Enable, 0:Disable
    // MOT_VAR_USER_OPT_I_LIMIT_ON_OFF,            // 1:Enable, 0:Disable
    // MOT_VAR_USER_RELAY_TOGGLE,
    // MOT_VAR_USER_METER_TOGGLE,
    /* MotorController_Var_User */
    // MOT_VAR_USER_ENTER_PARK,
    // MOT_VAR_USER_STATE_CMD,
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

    MOT_VAR_MAIN_MODE,                  // [MotorController_MainMode_T]
    MOT_VAR_INPUT_MODE,                 // [MotorController_InputMode_T]
    MOT_VAR_BUZZER_FLAGS_ENABLE,        // [MotorController_BuzzerFlags_T]

    /* OptDin */
    MOT_VAR_OPT_DIN_FUNCTION,           // [MotorController_OptDinMode_T]
    MOT_VAR_OPT_SPEED_LIMIT,            //
    MOT_VAR_OPT_I_LIMIT,

    MOT_VAR_BOOT_REF_FAST_BOOT,
    MOT_VAR_BOOT_REF_BEEP,
    MOT_VAR_BOOT_REF_BLINK,
}
MotorController_Var_Config_T;


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
        each id directly corresponds to var base literal enum type
        Corresponds to the "object type". accounts for type literal and specialized properties.
        By source module

    Control and UserIn
    Config Types need to check StateMachine is in Config State
*/
/******************************************************************************/
/*
    [MotorController_VarType]
    General Service Types
    Single Instance
*/
typedef enum MotorController_VarType
{
    MOT_VAR_TYPE_GENERAL_USER_OUT,
    MOT_VAR_TYPE_GENERAL_USER_IN, /* alternatively as IO */
    // MOT_VAR_TYPE_GENERAL_USER_CONTROL,
    MOT_VAR_TYPE_GENERAL_CONFIG,
    MOT_VAR_TYPE_GENERAL_DEBUG,
    MOT_VAR_TYPE_GENERAL_STATIC_REF, /* Read-only */
    MOT_VAR_TYPE_ANALOG_USER_VAR_OUT, // peripheral status
    MOT_VAR_TYPE_ANALOG_USER_CONFIG,
    // MOT_VAR_TYPE_BUZZER_CONTROL,
    // MOT_VAR_TYPE_BUZZER_CONFIG,
    // MOT_VAR_TYPE_OPT_DIN_CONFIG,
    // MOT_VAR_TYPE_RELAY_CONFIG,

    /*
        Specialized instances. effectively access as object classes. simplify static value bounds.
    */
    MOT_VAR_TYPE_V_MONITOR_SOURCE_STATE,
    MOT_VAR_TYPE_V_MONITOR_SOURCE_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_SOURCE_VDIVIDER_REF,

    MOT_VAR_TYPE_V_MONITOR_ACCS_STATE,
    MOT_VAR_TYPE_V_MONITOR_ACCS_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_ACCS_VDIVIDER_REF,

    MOT_VAR_TYPE_V_MONITOR_ANALOG_STATE,
    MOT_VAR_TYPE_V_MONITOR_ANALOG_CONFIG,
    MOT_VAR_TYPE_V_MONITOR_ANALOG_VDIVIDER_REF,

    MOT_VAR_TYPE_HEAT_MONITOR_PCB_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_PCB_THERMISTOR_REF, /* read-only coefficients */

    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_STATE,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_CONFIG,
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_STATE, /* 0-3 */
    MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_THERMISTOR_REF, /* 0-3 */
    // MOT_VAR_TYPE_HEAT_MONITOR_MOSFETS_INSTANCE_CONFIG, /* reserved */
    /* Communication */
    MOT_VAR_TYPE_SOCKET_STATE,
    MOT_VAR_TYPE_SOCKET_CONFIG, /* Instance by Protocol Count */
    MOT_VAR_TYPE_CAN_BUS_STATE,
    MOT_VAR_TYPE_CAN_BUS_CONFIG,
    // MotorController_SystemCmd_T in call for now
    /*
        Application_User SubModules
        app table handle compile time define
    */
    /* Vehicle Submodule */
    MOT_VAR_TYPE_VEHICLE_CONTROL,
    MOT_VAR_TYPE_VEHICLE_CONFIG,
}
MotorController_VarType_T;

/******************************************************************************/
/*
    [MotorController_VarHandlerType]
    Handler by Source File Module
    Effectively name space for types
    Mutually exclusive attributes when possible
    partition for contiguously expandable ids
*/
/******************************************************************************/
typedef enum MotVarId_HandlerType
{
    MOT_VAR_ID_HANDLER_TYPE_MOTOR,
    MOT_VAR_ID_HANDLER_TYPE_SYSTEM,
    // MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONTROL,
    // MOT_VAR_ID_HANDLER_TYPE_MOTOR_CONFIG,
    // MOT_VAR_ID_HANDLER_TYPE_MOTOR_SENSOR,
    // MOT_VAR_ID_HANDLER_TYPE_MOTOR_SUB_MODULE,
    // MOT_VAR_ID_HANDLER_TYPE_GENERAL,
    // MOT_VAR_ID_HANDLER_TYPE_V_MONITOR,
    // MOT_VAR_ID_HANDLER_TYPE_HEAT_MONITOR,
    // MOT_VAR_ID_HANDLER_TYPE_COMMUNICATION,
    // MOT_VAR_ID_HANDLER_TYPE_SYSTEM_COMMAND,
    // MOT_VAR_ID_HANDLER_TYPE_APPLICATION_COMMAND,
    // _MOT_VAR_ID_HANDLER_TYPE_END,
}
MotVarId_HandlerType_T;

/******************************************************************************/
/*
    [MotVarId]
*/
/******************************************************************************/
extern int MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId);
extern MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_context, MotVarId_T varId, int varValue);


/*
    check state - restrict state by interface - optionally map on state machine
    map to int accessor, or keyed accessor
    call accessor handle value error checking only
*/
// typedef enum
// {
//     ACCESS_POLICY_NONE              = 0,
//     ACCESS_POLICY_NOT_ANALOG_MODE   = (1U << 0),  // Requires non-analog input mode
//     ACCESS_POLICY_CONFIG_STATE      = (1U << 1),  // Requires config state
//     ACCESS_POLICY_PROTOCOL_CONTROL  = (1U << 2),  // Requires protocol control enabled
//     ACCESS_POLICY_MOTOR_CMD_STATE   = (1U << 3),  // Requires motor command state
//     ACCESS_POLICY_WRITE_DISABLED    = (1U << 4),  // Read-only
// } AccessPolicy_T;

// typedef struct
// {
//     MotVarId_HandlerType_T HandlerType;
//     MotorController_VarType_T InnerType;  // Use as union for different handler types
//     AccessPolicy_T ReadPolicy;
//     AccessPolicy_T WritePolicy;
// } VarAccessRule_T;