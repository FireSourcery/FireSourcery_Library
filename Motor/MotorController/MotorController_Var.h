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
#include "../Motor/Motor_Var.h"
#include "../MotProtocol/MotVarId/MotVarId.h"
#include "System/SysTime/SysTime.h"

// typedef union { uint32_t Unsigned; int32_t Signed; } var32_t;
// typedef union { uint16_t Unsigned; int16_t Signed; } var16_t;
// typedef union { uint8_t Unsigned; int8_t Signed; } var8_t;
// typedef uint32_t mot_var_status_t; // generic status, type depending on input

/******************************************************************************/
/*
    Var Id Base
*/
/******************************************************************************/
typedef enum MotorController_VarOutput
{
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_MC_STATE,
    MOT_VAR_MC_STATUS_FLAGS,
    MOT_VAR_MC_FAULT_FLAGS,

    MOT_DRIVE_DIRECTION,
}
MotorController_VarOutput_T;

/*
    Collective Modules
*/
// typedef enum MotVMonitor_VarOutput
// {
//     MOT_MONITOR_V_SOURCE,
//     MOT_MONITOR_V_SOURCE_STATUS,
//     MOT_MONITOR_V_ACCS,
//     MOT_MONITOR_V_ACCS_STATUS,
//     MOT_MONITOR_V_ANALOG,
//     MOT_MONITOR_V_ANALOG_STATUS,
// }
// MotVMonitor_VarOutput_T;

// typedef enum MotVMonitor_VarRef
// {
//     MOT_MONITOR_V_SOURCE_R1,
//     MOT_MONITOR_V_SOURCE_R2,
//     MOT_MONITOR_V_ACCS_R1,
//     MOT_MONITOR_V_ACCS_R2,
//     MOT_MONITOR_V_ANALOG_R1,
//     MOT_MONITOR_V_ANALOG_R2,
// }
// MotVMonitor_VarRef_T;

// typedef enum MotMonitor_VarOutput
// {
//     MOT_MONITOR_V_SOURCE,
//     MOT_MONITOR_V_ACCS,
//     MOT_MONITOR_V_ANALOG,
//     MOT_MONITOR_HEAT_PCB,
//     // MOT_MONITOR_HEAT_MOSFETS_INSTANCED, alternatively
//     MOT_MONITOR_HEAT_MOSFETS,
//     MOT_MONITOR_HEAT_MOSFETS_1,
//     MOT_MONITOR_HEAT_MOSFETS_2,
//     MOT_MONITOR_HEAT_MOSFETS_3,
// }
// MotMonitor_VarOutput_T;



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
    per motor use use motor instance functions
    Some overlap with call
*/
typedef enum MotorController_Command_MotorContext
{
    MOTOR_CONTROLLER_USER_CMD,                           // [-32768:32767]
    MOTOR_CONTROLLER_USER_FEEDBACK_MODE,                 //

    /* handled by call */
    // MOT_USER_FORCE_DISABLE_CONTROL,       // Force Disable control Non StateMachine checked, also handled via Call
    // MOT_USER_SYSTEM_CLEAR_FAULT,         // fault flags
    // MOT_USER_SYSTEM_BEEP,
    /*  */
    // MOT_VAR_TRY_HOLD,                       // bypass FOC, MOT_VAR_USER_CMD = 0, VoltageMode
    // MOT_VAR_TRY_RELEASE,                    // same as either neutral or driveZero

    /*
        MotDrive Submodule
    */
    MOTOR_CONTROLLER_MOT_DRIVE_VAR_THROTTLE,                       // [0:65535]
    MOTOR_CONTROLLER_MOT_DRIVE_VAR_BRAKE,                          // [0:65535]
    MOTOR_CONTROLLER_MOT_DRIVE_CMD_DIRECTION, /* or IO */

    /*
        Stateless apply to all motors, move to control to share with analog input
    */
    MOTOR_CONTROLLER_OPT_SPEED_LIMIT_ON_OFF,         // 1:Enable, 0:Disable
    MOTOR_CONTROLLER_OPT_I_LIMIT_ON_OFF,             // 1:Enable, 0:Disable

    MOTOR_CONTROLLER_RELAY_TOGGLE,
    MOTOR_CONTROLLER_METER_TOGGLE,
}
MotorController_Command_MotorContext_T;

typedef enum MotorController_Command_Blocking
{
    // MOTOR_CONTROLLER_CMD_CALIBRATE, /* begin each motor sequence, use per motor cmd instead */
    MOTOR_CONTROLLER_CMD_CALIBRATE_ADC,
    MOTOR_CONTROLLER_CMD_NVM_SAVE_CONFIG,
    MOTOR_CONTROLLER_CMD_NVM_RESTORE_CONFIG,
}
MotorController_Command_Blocking_T;


/******************************************************************************/
/*
    Config - Nvm variables
*/
/******************************************************************************/
typedef enum MotorController_ConfigId
{
    MOT_VAR_V_SOURCE_REF_VOLTS,
    MOT_VAR_I_LIMIT_LOW_V,

    // MOT_VAR_DEFAULT_FEEDBACK_MODE,       // Motor_FeedbackMode_T
    MOT_VAR_USER_INIT_MODE,                 // MotorController_MainMode_T
    MOT_VAR_USER_INPUT_MODE,                // MotorController_InputMode_T
    MOT_VAR_BUZZER_FLAGS_ENABLE,            // MotorController_BuzzerFlags_T

    MOT_VAR_OPT_DIN_FUNCTION,               // MotorController_OptDinMode_T
    MOT_VAR_OPT_SPEED_LIMIT,                // Selectable Speed Limit
    MOT_VAR_OPT_I_LIMIT,

    /* Drive */
    MOTOR_CONTROLLER_MOT_DRIVE_CONFIG_THROTTLE_MODE,          // MotDrive_ThrottleMode_T
    MOTOR_CONTROLLER_MOT_DRIVE_CONFIG_BRAKE_MODE,             // MotDrive_BrakeMode_T
    MOTOR_CONTROLLER_MOT_DRIVE_CONFIG_ZERO_MODE,              // MotDrive_ZeroMode_T

    // MOT_VAR_BOOT_REF_FAST_BOOT,
    // MOT_VAR_BOOT_REF_BEEP,
    // MOT_VAR_BOOT_REF_BLINK,

    // MOT_VAR_CAN_SERVICES_ID,
    // MOT_VAR_CAN_IS_ENABLE,
    // MOT_VAR_BOOT_REF,
}
MotorController_ConfigId_T;


typedef enum MotorController_Config_BootRef
{
    MOT_VAR_BOOT_REF_FAST_BOOT,
    MOT_VAR_BOOT_REF_BEEP,
    MOT_VAR_BOOT_REF_BLINK,
    // MOT_VAR_BOOT_REF_PROTOCOL_INDEX,
}
MotorController_Config_BootRef_T;

typedef enum MotorController_VarRef
{
    MOT_VAR_REF_MOTOR_COUNT,
    MOT_VAR_REF_THERMISTOR_MOSFETS_COUNT,
    MOT_VAR_REF_PROTOCOL_SOCKET_COUNT,
    // MOT_VAR_REF_V_AUX_COUNT,
}
MotorController_VarRef_T;


static inline uint8_t MotorController_Var_GetMotorCount(const MotorController_T * p_context) { return p_context->MOTORS.LENGTH; }
static inline uint8_t MotorController_Var_GetHeatMosfetCount(const MotorController_T * p_context) { return HeatMonitor_Group_GetInstanceCount(&p_context->HEAT_MOSFETS); }



/******************************************************************************/
/*!
    Instance Select
    @return may return null
*/
/******************************************************************************/
// typedef enum MotVarId_Instance_BoardThermistor
// {
//     // MOT_INSTANCE_THERMISTOR_PCB,
//     // MOT_VAR_ID_THERMISTOR_PCB,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_0,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_1,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_2,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_3,
// }
// MotVarId_Instance_BoardHeatMonitor_T;

// typedef enum MotVarId_Instance_VMonitor
// {
//     MOT_VAR_ID_V_MONITOR_SOURCE,
//     MOT_VAR_ID_V_MONITOR_ANALOG,
//     MOT_VAR_ID_V_MONITOR_ACCS,
// }
// MotVarId_Instance_VMonitor_T;

// typedef enum MotVarId_Instance_Motor
// {
//     MOT_VAR_ID_MOTOR_0,
//     MOT_VAR_ID_MOTOR_1,
//     MOT_VAR_ID_MOTOR_2,
//     MOT_VAR_ID_MOTOR_3,
// }
// MotVarId_Instance_Motor_State_T;

/*
    Index corresponds to external user interface
*/
// static inline Motor_State_T * MotorController_User_GetPtrMotor(const MotorController_T * p_context, uint8_t motorIndex)
// {
//     return (motorIndex < p_context->MOTORS.LENGTH) ? MotorController_MotorAt(p_context, motorIndex) : NULL;
// }

// static inline HeatMonitor_T * MotorController_User_GetPtrThermistor(const MotorController_T * p_context, uint8_t index)
// {
//     MotorController_State_T * p_mc = p_context->P_ACTIVE;

//     const HeatMonitor_T * p_thermistor;
//     switch (index)
//     {
//         case 0U: p_thermistor = &p_mc->ThermistorPcb; break;
//         default:
//             p_thermistor = ((index - 1U) < p_context->MOT_HEAT_MONITOR_CONVERSIONS.HEAT_MOSFETS_COUNT) ? &p_mc->MosfetsThermistors[index - 1U] : NULL;
//             break;

//     }
//     return (HeatMonitor_T *)p_thermistor;
// }

// static inline VMonitor_T * MotorController_User_GetPtrVMonitor(const MotorController_T * p_context, uint8_t index)
// {
//     MotorController_State_T * p_mc = p_context->P_ACTIVE;

//     const VMonitor_T * p_vMonitor;
//     switch (index)
//     {
//         case 0U: p_vMonitor = &p_mc->V_SOURCE; break;
//         case 1U: p_vMonitor = &p_mc->VMonitorSense;  break;
//         case 2U: p_vMonitor = &p_mc->VMonitorAccs;   break;
//         default: p_vMonitor = NULL; break;
//     }
//     return (VMonitor_T *)p_vMonitor;
// }

// static inline Protocol_T * MotorController_User_GetPtrProtocol(const MotorController_T * p_context, uint8_t protocolIndex)
// {
//     return (protocolIndex < p_context->PROTOCOL_COUNT) ? &p_context->P_PROTOCOLS[protocolIndex] : NULL;
// }


/******************************************************************************/
/*!
*/
/******************************************************************************/

/******************************************************************************/
/*
    [MotVarId]
    Type index
*/
/******************************************************************************/
extern int32_t MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId);
extern MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_context, MotVarId_T varId, int32_t varValue);


