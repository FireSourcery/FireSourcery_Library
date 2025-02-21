/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @version V0

    @brief  Var - Field-like Property Interface Getter/Setter via Id Key
                Effectively serialize selected set of struct members
                Store in host cache and periodically poll
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_VAR_H
#define MOTOR_CONTROLLER_VAR_H

#include "MotorController_User.h"
#include "MotorController.h"
#include "../Motor/Motor_Var.h"
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
    MOT_VAR_V_SOURCE,
    MOT_VAR_V_SENSOR,
    MOT_VAR_V_ACCS,
    MOT_VAR_HEAT_PCB,
    MOT_VAR_HEAT_MOSFETS,
    MOT_VAR_HEAT_MOSFETS_1,
    MOT_VAR_HEAT_MOSFETS_2,
    MOT_VAR_HEAT_MOSFETS_3,
    // MOT_VAR_HEAT_MOSFETS_TOP,
    // MOT_VAR_HEAT_MOSFETS_BOT,
    // MOT_VAR_HEAT_MOSFETS_2,
    // MOT_VAR_HEAT_MOSFETS_3,
}
MotorController_VarOutput_T;

typedef enum MotorController_VarOutput_AnalogUser
{
    MOT_VAR_ANALOG_THROTTLE,
    MOT_VAR_ANALOG_THROTTLE_DIN,
    MOT_VAR_ANALOG_BRAKE,
    MOT_VAR_ANALOG_BRAKE_DIN,
    MOT_VAR_ANALOG_BISTATE_BRAKE_DIN,
    MOT_VAR_ANALOG_FORWARD_DIN,
    MOT_VAR_ANALOG_NEUTRAL_DIN,
    MOT_VAR_ANALOG_REVERSE_DIN,
    MOT_VAR_ANALOG_OPT_DIN,
}
MotorController_VarOutput_AnalogUser_T;

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

// typedef enum MotorController_VarOutput_Pin
// {
// }
// MotorController_VarOutput_Pin_T;

/*
    Collective set motors for convenience
    get use individual motor instanace

    Direction or DriveState use feedback. Other
*/
typedef enum MotorController_VarInput
{
    /*
        for all motors, or primary Motor
        per motor use use motor instance functions
    */
    MOT_VAR_USER_CMD,                           // [-32768:32767]
    MOT_VAR_USER_FEEDBACK_MODE,                 //
    // MOT_VAR_FORCE_DISABLE_CONTROL,          // Force Disable control Non StateMachine checked, also handled via Call
    // MOT_VAR_TRY_HOLD,                       // bypass FOC, MOT_VAR_USER_CMD = 0, VoltageMode
    // MOT_VAR_TRY_RELEASE,                    // same as either neutral or driveZero

    /*
        Brake and Throttle invoke SubStates
        If both are set, one must be 0 and is ignored
        if both are 0, invoke DriveZeroMode
    */
    MOT_VAR_THROTTLE,                       // [0:65535]
    MOT_VAR_BRAKE,                          // [0:65535]
    /*
        Stateless apply to all motors, move to control to share with analog input
    */
    MOT_VAR_OPT_SPEED_LIMIT_ON_OFF,         // 1:Enable, 0:Disable
    MOT_VAR_OPT_I_LIMIT_ON_OFF,             // 1:Enable, 0:Disable
    MOT_VAR_RELAY_TOGGLE,
    MOT_VAR_METER_TOGGLE,
}
MotorController_VarInput_T;

// MotorController_VarIO_T
typedef enum MotorController_VarIO
{
    MOT_VAR_DIRECTION,                      // MotorController_Direction_T,
}
MotorController_VarIO_T;


/******************************************************************************/
/*
    Config - Nvm variables
*/
/******************************************************************************/
typedef enum MotorController_ConfigId
{
    MOT_VAR_V_SOURCE_REF_VOLTS,
    MOT_VAR_DEFAULT_FEEDBACK_MODE,          // Motor_FeedbackMode_T
    MOT_VAR_USER_INIT_MODE,                 // MotorController_MainMode_T
    MOT_VAR_USER_INPUT_MODE,                // MotorController_InputMode_T
    MOT_VAR_THROTTLE_MODE,                  // MotorController_ThrottleMode_T
    MOT_VAR_BRAKE_MODE,                     // MotorController_BrakeMode_T
    MOT_VAR_DRIVE_ZERO_MODE,                // MotorController_DriveZeroMode_T
    MOT_VAR_I_LIMIT_LOW_V,
    MOT_VAR_BUZZER_FLAGS_ENABLE,            // MotorController_BuzzerFlags_T
    MOT_VAR_OPT_DIN_FUNCTION,               // MotorController_OptDinMode_T
    MOT_VAR_OPT_SPEED_LIMIT,                // Selectable Speed Limit
    MOT_VAR_OPT_I_LIMIT,
    // MOT_VAR_CAN_SERVICES_ID,
    // MOT_VAR_CAN_IS_ENABLE,
    // MOT_VAR_BATTERY_ZERO_ADCU,
    // MOT_VAR_BATTERY_FULL_ADCU,
    // MOT_VAR_BOOT_REF,
}
MotorController_ConfigId_T;

typedef enum MotorController_Config_AnalogUser
{
    MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU,
    MOT_VAR_ANALOG_THROTTLE_MAX_ADCU,
    MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE,
    MOT_VAR_ANALOG_BRAKE_ZERO_ADCU,
    MOT_VAR_ANALOG_BRAKE_MAX_ADCU,
    MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE,
    MOT_VAR_ANALOG_DIN_BRAKE_VALUE,
    MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE,
    MOT_VAR_ANALOG_DIRECTION_PINS,
}
MotorController_Config_AnalogUser_T;

typedef enum MotorController_Config_BootRef
{
    MOT_VAR_BOOT_REF_FAST_BOOT,
    MOT_VAR_BOOT_REF_BEEP,
    MOT_VAR_BOOT_REF_BLINK,
    // MOT_VAR_BOOT_REF_PROTOCOL_INDEX,
}
MotorController_Config_BootRef_T;


/* or handle with call */
// typedef enum Motor_VarConfig_Cmd
// {
//     MOTOR_VAR_CMD_CALIBRATE_SENSOR,
//     MOTOR_VAR_CMD_CALIBRATE_ADC,
//     MOTOR_VAR_CMD_NVM_SAVE_CONFIG,
//     MOTOR_VAR_CMD_NVM_RESTORE_CONFIG,
//     MOTOR_VAR_CMD_REBOOT,
//     MOTOR_VAR_CMD_NVM_SAVE_BOOT,
//     MOTOR_VAR_CMD_NVM_WRITE_ONCE,
//     MOTOR_VAR_CMD_NVM_READ_ONCE,
// }
// Motor_VarConfig_Cmd_T;

// todo
// typedef Thermistor_ConfigId_T MotVarId_Config_BoardThermistor_T;
// typedef Thermistor_ConfigId_T MotVarId_Config_MotorThermistor_T;

int32_t MotorController_VarOutput_Get(const MotorController_T * p_mc, MotorController_VarOutput_T id);
int32_t MotorController_VarOutput_Debug_Get(const MotorController_T * p_mc, MotorController_VarOutput_Debug_T id);
int32_t MotorController_VarOutput_AnalogUser_Get(const MotorController_T * p_mc, MotorController_VarOutput_AnalogUser_T id);

uint32_t MotorController_VarInput_Set(MotorController_T * p_mc, MotorController_VarInput_T id, int32_t value);

int32_t MotorController_VarIO_Get(const MotorController_T * p_mc, MotorController_VarIO_T id);
uint32_t MotorController_VarIO_Set(MotorController_T * p_mc, MotorController_VarIO_T id, int32_t value);

int32_t MotorController_Config_Get(const MotorController_T * p_mc, MotorController_ConfigId_T id);
uint32_t MotorController_Config_Set(MotorController_T * p_mc, MotorController_ConfigId_T id, int32_t value);
int32_t MotorController_Config_BootRef_Get(const MotorController_T * p_mc, MotorController_Config_BootRef_T id);
uint32_t MotorController_Config_BootRef_Set(MotorController_T * p_mc, MotorController_Config_BootRef_T id, int32_t value);
int32_t MotorController_Config_AnalogUser_Get(const MotorController_T * p_mc, MotorController_Config_AnalogUser_T id);
uint32_t MotorController_Config_AnalogUser_Set(MotorController_T * p_mc, MotorController_Config_AnalogUser_T id, int32_t value);

/******************************************************************************/
/*
    Meta
    todo reanme
*/
/******************************************************************************/
/*
    Type of NameBase e.g. MotorController_VarOutput_T
    struct id
*/
/* Mot_VarTypeIO */
typedef enum MotVarId_Type_RealTime /* : uint16_t */
{
    /* Monitor - Read-Only */
    /* Control - Read/Write */
    /* Cmd - Write-Only */
    MOT_VAR_ID_TYPE_MONITOR_GENERAL,
    MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER,
    MOT_VAR_ID_TYPE_MONITOR_MOTOR,
    MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC,
    MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR,
    MOT_VAR_ID_TYPE_CONTROL_GENERAL,
    MOT_VAR_ID_TYPE_CONTROL_MOTOR,
    // MOT_VAR_ID_TYPE_VAR_PROTOCOL,
    MOT_VAR_ID_TYPE_CMD_GENERAL,
    MOT_VAR_ID_TYPE_CMD_MOTOR,
    // MOT_VAR_ID_TYPE_CMD_SYSTEM,
    MOT_VAR_ID_TYPE_DEBUG,
    MOT_VAR_ID_TYPE_REAL_TIME_END = 15U,
}
MotVarId_Type_RealTime_T;

/* Mot_VarTypeConfig */
typedef enum MotVarId_Type_Config
{
    // MOT_VAR_TYPE_CONFIG_MOTOR_PRIMARY,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_CMD,
    MOT_VAR_ID_TYPE_CONFIG_GENERAL,
    MOT_VAR_ID_TYPE_CONFIG_ANALOG_USER,
    MOT_VAR_ID_TYPE_CONFIG_VMONITOR,
    MOT_VAR_ID_TYPE_CONFIG_BOARD_THERMISTOR,
    MOT_VAR_ID_TYPE_CONFIG_PROTOCOL,
    MOT_VAR_ID_TYPE_CONFIG_BOOT_REF,
    MOT_VAR_ID_TYPE_CONFIG_END = 15U,
    // MOT_VAR_ID_TYPE_CONFIG_THERMISTOR,
}
MotVarId_Type_Config_T;

/* Type of MotVarId_Type_Config_T, MotVarId_Type_RealTime_T */
/* Mot_VarTypeType */
typedef enum MotVarId_TypeType
{
    MOT_VAR_ID_TYPE_REAL_TIME,
    MOT_VAR_ID_TYPE_CONFIG,
}
MotVarId_TypeType_T;

// Mot_VarId_T
typedef union MotVarId
{
    struct
    {
        uint16_t NameBase           : 4U;
        uint16_t NameType           : 4U; /* Name's Type - corresponds 1:1 with enum type */
        uint16_t NameTypeType       : 1U; /* Name Type's Type - Alternatively partition as monitor/cmd/call */
        uint16_t Instance           : 3U; /* TypeInstance1 - Upto 8 Instances Per Type */
        uint16_t Alt                : 4U; /* Alternative unit/format */
    };
    /* Correspond to host side */
    struct
    {
        uint16_t NamePart       : 9U; /* name can be determined by nameId + nameId_Type if prefix maps to nameId_Type 1:1 */
        uint16_t InstancePart   : 3U;
        uint16_t ResvPart       : 4U;
    };
    uint16_t Value;
}
MotVarId_T;

/*
    Status Response for Read/Write
*/
/* Mot_VarStatus */
typedef enum MotVarId_Status
{
    MOT_VAR_STATUS_OK,
    MOT_VAR_STATUS_ERROR,
    // MOT_VAR_STATUS_ERROR_INVALID_ID,
    // On Set
    MOT_VAR_STATUS_ERROR_READ_ONLY,
    MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED,
    MOT_VAR_STATUS_ERROR_RUNNING, // Not in config state
    // MOT_VAR_STATUS_ERROR_REJECT_BY_STATE_MACHINE,
    // MOT_VAR_STATUS_ASYNC,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVarId_Status_T;

extern int32_t MotorController_Var_Get(const MotorController_T * p_mc, MotVarId_T varId);
extern MotVarId_Status_T MotorController_Var_Set(MotorController_T * p_mc, MotVarId_T varId, int32_t varValue);



#endif