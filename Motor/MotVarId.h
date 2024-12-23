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
    @file   MotVarId.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#ifndef MOT_VAR_ID_H
#define MOT_VAR_ID_H

#include <stdint.h>

/* todo move base layer to individual modules */

/******************************************************************************/
/*!
    @note
    RealTime
        Monitor -> Read-Only
            Get/Output
        Cmd -> Write-Only, Get returns 0
            Set/Input
        Control -> Read/Write
            Access/IO
            May be paired getter/setter or a single variable
            Analogously a control loop in/out may differ
*/
/******************************************************************************/

/******************************************************************************/
/*
    Motor - Use instance index
*/
/******************************************************************************/
/*
    Speed/IPhase/VPhase/Power
        Units0 => UFract16, may over saturate
*/
typedef enum MotVarId_Monitor_Motor
{
    MOT_VAR_SPEED,
    MOT_VAR_I_PHASE,
    MOT_VAR_V_PHASE,
    MOT_VAR_ELECTRICAL_ANGLE,
    MOT_VAR_MECHANICAL_ANGLE,
    MOT_VAR_POWER,
    MOT_VAR_MOTOR_HEAT,
    MOT_VAR_MOTOR_STATE,
    MOT_VAR_MOTOR_STATE_FLAGS,
    MOT_VAR_MOTOR_FAULT_FLAGS,
    MOT_VAR_MOTOR_EFFECTIVE_FEEDBACK_MODE,
    MOT_VAR_MOTOR_EFFECTIVE_SPEED_LIMIT,
    MOT_VAR_MOTOR_EFFECTIVE_I_LIMIT,
    MOT_VAR_MOTOR_V_SPEED_DEBUG,
    MOT_VAR_MOTOR_V_SPEED_EFFECTIVE,
}
MotVarId_Monitor_Motor_T;

typedef enum MotVarId_Monitor_MotorFoc
{
    MOT_VAR_FOC_IA,
    MOT_VAR_FOC_IB,
    MOT_VAR_FOC_IC,
    MOT_VAR_FOC_IQ,
    MOT_VAR_FOC_ID,
    MOT_VAR_FOC_VQ,
    MOT_VAR_FOC_VD,
    MOT_VAR_FOC_REQ_Q,
    MOT_VAR_FOC_REQ_D,
    MOT_VAR_FOC_VA,
    MOT_VAR_FOC_VB,
    MOT_VAR_FOC_VC,
}
MotVarId_Monitor_MotorFoc_T;

typedef enum MotVarId_Monitor_MotorSensor
{
    MOT_VAR_ENCODER_FREQ,
}
MotVarId_Monitor_MotorSensor_T;

/*
    Value [-32768:32767]
*/
typedef enum MotVarId_Cmd_Motor
{
    // MOT_VAR_MOTOR_USER_CMD,      // RampIn Always reflects the input value
    MOT_VAR_MOTOR_CMD_SPEED,        // UserCmd as Speed
    MOT_VAR_MOTOR_CMD_CURRENT,
    MOT_VAR_MOTOR_CMD_VOLTAGE,
    MOT_VAR_MOTOR_CMD_ANGLE,
    MOT_VAR_MOTOR_CMD_OPEN_LOOP,
    MOT_VAR_MOTOR_FORCE_DISABLE_CONTROL,    // No value arg. Force Disable control Non StateMachine checked, also handled via Call
    MOT_VAR_MOTOR_TRY_RELEASE,              // No value arg. same as either neutral or driveZero
    MOT_VAR_MOTOR_TRY_HOLD,                 // No value arg. bypass FOC, MOT_VAR_USER_CMD = 0, VoltageMode
    MOT_VAR_MOTOR_CLEAR_FAULT,              // No value arg. Clear Faults
}
MotVarId_Cmd_Motor_T;

typedef enum MotVarId_Control_Motor
{
    MOT_VAR_MOTOR_DIRECTION,            // Motor_Direction_T - CW/CCW. Read state value, write interface value,
    MOT_VAR_MOTOR_USER_SET_POINT,       // RampIn(UserCmd)/RampOut(SetPoint), Generic mode select
    /* IO Vars, Read effective value, write interface value */
    MOT_VAR_MOTOR_USER_FEEDBACK_MODE,
    MOT_VAR_MOTOR_USER_SPEED_LIMIT,
    MOT_VAR_MOTOR_USER_I_LIMIT,
}
MotVarId_Control_Motor_T;

/******************************************************************************/
/*
    MotorController/General
*/
/******************************************************************************/
typedef enum MotVarId_Monitor_General
{
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_DEBUG,
    MOT_VAR_MC_STATE,
    MOT_VAR_MC_STATE_FLAGS,
    MOT_VAR_MC_FAULT_FLAGS,
    MOT_VAR_V_SOURCE,
    MOT_VAR_V_SENSOR,
    MOT_VAR_V_ACCS,
    MOT_VAR_BATTERY_CHARGE, // if CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MOT_VAR_HEAT_PCB,
    MOT_VAR_HEAT_MOSFETS,
    // MOT_VAR_HEAT_MOSFETS_TOP,
    // MOT_VAR_HEAT_MOSFETS_BOT,
    // MOT_VAR_HEAT_MOSFETS_2,
    // MOT_VAR_HEAT_MOSFETS_3,
}
MotVarId_Monitor_General_T;

typedef enum MotVarId_Monitor_AnalogUser
{
    MOT_VAR_ANALOG_THROTTLE,
    MOT_VAR_ANALOG_THROTTLE_DIN,
    MOT_VAR_ANALOG_BRAKE,
    MOT_VAR_ANALOG_BRAKE_DIN,
}
MotVarId_Monitor_AnalogUser_T;

/*
    Collective set motors for convenience
    get use individual motor instanace

    Direction or DriveState use feedback. Other
*/
typedef enum MotVarId_Cmd_General
{
    // MOT_VAR_BEEP,
    // MOT_VAR_CONTROLLER_MODE,                // Servo Or Drive
    // MOT_VAR_CLEAR_FAULT,

    /*
        for all motors, or primary Motor
        per motor use use motor instance functions
    */
    // MOT_VAR_FORCE_DISABLE_CONTROL,          // Force Disable control Non StateMachine checked, also handled via Call
    // MOT_VAR_TRY_HOLD,                       // bypass FOC, MOT_VAR_USER_CMD = 0, VoltageMode
    // MOT_VAR_TRY_RELEASE,                    // same as either neutral or driveZero
    MOT_VAR_USER_CMD,                           // [-32768:32767]
    MOT_VAR_USER_FEEDBACK_MODE,                 //

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
}
MotVarId_Cmd_General_T;

typedef enum MotVarId_Control_General
{
    MOT_VAR_DIRECTION,                      // MotorController_Direction_T,
}
MotVarId_Control_General_T;


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*
    Instance0 -> Motor[0]
    Enum values for =>
        Motor_CommutationMode_T,
        Motor_SensorMode_T,
        Motor_FeedbackModeId_T,
        Motor_DirectionCalibration_T
*/
typedef enum MotVarId_Config_MotorPrimary
{
    MOT_VAR_COMMUTATION_MODE, // if runtime supported
    MOT_VAR_SENSOR_MODE,
    MOT_VAR_DIRECTION_CALIBRATION,
    MOT_VAR_POLE_PAIRS,
    MOT_VAR_KV,
    MOT_VAR_V_SPEED_SCALAR,
    MOT_VAR_SPEED_V_REF_RPM,
    MOT_VAR_SPEED_V_MATCH_REF_RPM,
    MOT_VAR_IA_ZERO_REF_ADCU,
    MOT_VAR_IB_ZERO_REF_ADCU,
    MOT_VAR_IC_ZERO_REF_ADCU,
    MOT_VAR_I_PEAK_REF_ADCU,
}
MotVarId_Config_MotorPrimary_T;

/*
    Limits
        Units0 - > Percent16
*/
typedef enum MotVarId_Config_MotorSecondary
{
    MOT_VAR_BASE_SPEED_LIMIT_FORWARD,
    MOT_VAR_BASE_SPEED_LIMIT_REVERSE,
    MOT_VAR_BASE_I_LIMIT_MOTORING,
    MOT_VAR_BASE_I_LIMIT_GENERATING,
    MOT_VAR_RAMP_ACCEL_TIME,
    MOT_VAR_ALIGN_MODE,
    MOT_VAR_ALIGN_POWER,
    MOT_VAR_ALIGN_TIME,
    MOT_VAR_OPEN_LOOP_POWER,
    MOT_VAR_OPEN_LOOP_SPEED,
    MOT_VAR_OPEN_LOOP_ACCEL_TIME,
    MOT_VAR_PHASE_PWM_MODE,
}
MotVarId_Config_MotorSecondary_T;

// HostSide
// uint16_t SurfaceDiameter;
// uint16_t GearRatio_Factor;
// uint16_t GearRatio_Divisor;

typedef enum MotVarId_Config_MotorHall
{
    MOT_VAR_HALL_SENSOR_TABLE_1,
    MOT_VAR_HALL_SENSOR_TABLE_2,
    MOT_VAR_HALL_SENSOR_TABLE_3,
    MOT_VAR_HALL_SENSOR_TABLE_4,
    MOT_VAR_HALL_SENSOR_TABLE_5,
    MOT_VAR_HALL_SENSOR_TABLE_6,
}
MotVarId_Config_MotorHall_T;

typedef enum MotVarId_Config_MotorEncoder
{
    MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION,
    MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP,
    MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR,
    MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED,
    MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE,
}
MotVarId_Config_MotorEncoder_T;

/*
    Sine Cos Encoder
*/
// typedef enum MotVarId_Config_MotorSinCos
// {
//     MOT_VAR_SIN_COS_ZERO_ADCU,
//     MOT_VAR_SIN_COS_MAX_ADCU,
//     MOT_VAR_SIN_COS_MAX_MILLIV,
//     MOT_VAR_SIN_COS_ANGLE_OFFSET,
//     MOT_VAR_SIN_COS_IS_B_POSITIVE,
//     MOT_VAR_SIN_COS_ELECTRICAL_ROTATIONS_PER_CYCLE,
// }
// MotVarId_Config_MotorSinCos_T;

/*
    PID
    Fixed 16 Set with interface functions
*/
typedef enum MotVarId_Config_MotorPid
{
    MOT_VAR_PID_SPEED_KP_FIXED16,
    MOT_VAR_PID_SPEED_KI_FIXED16,
    MOT_VAR_PID_SPEED_KD_FIXED16,
    MOT_VAR_PID_SPEED_SAMPLE_FREQ,
    MOT_VAR_PID_FOC_IQ_KP_FIXED16,
    MOT_VAR_PID_FOC_IQ_KI_FIXED16,
    MOT_VAR_PID_FOC_IQ_KD_FIXED16,
    MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ,
    MOT_VAR_PID_FOC_ID_KP_FIXED16,
    MOT_VAR_PID_FOC_ID_KI_FIXED16,
    MOT_VAR_PID_FOC_ID_KD_FIXED16,
    MOT_VAR_PID_FOC_ID_SAMPLE_FREQ,
}
MotVarId_Config_MotorPid_T;

/*
    PID
    Host convert
*/
// Prop_Gain,
// Prop_Gain_Shift,
// Integral_Gain,
// Integral_Gain_Shift,
typedef enum MotVarId_Config_General
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
MotVarId_Config_General_T;

typedef enum MotVarId_Config_AnalogUser
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
MotVarId_Config_AnalogUser_T;

/*
    Use instance Id
*/
typedef enum MotVarId_Config_Protocol
{
    MOT_VAR_PROTOCOL_XCVR_ID,
    MOT_VAR_PROTOCOL_SPECS_ID,
    MOT_VAR_PROTOCOL_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL_BAUD_RATE, // On boot
 //  MOT_VAR_PROTOCOL_ACTIVE_BAUD_RATE,
    MOT_VAR_PROTOCOL_IS_ENABLED,
}
MotVarId_Config_Protocol_T;

/*
    Instance0 -> Pcb
    Instance1 -> Mosfets
    Instance2 -> Mosfets1
    Instance3 -> Motor[0]
*/
typedef enum MotVarId_Config_Thermistor
{
    MOT_VAR_THERMISTOR_R_SERIES, // All instance Read-Only
    MOT_VAR_THERMISTOR_R_PARALLEL, // All instance Read-Only
    MOT_VAR_THERMISTOR_TYPE,
    MOT_VAR_THERMISTOR_R0,
    MOT_VAR_THERMISTOR_T0,
    MOT_VAR_THERMISTOR_B,
    MOT_VAR_THERMISTOR_LINEAR_T0_ADCU,
    MOT_VAR_THERMISTOR_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_FAULT_TRIGGER_ADCU,
    MOT_VAR_THERMISTOR_FAULT_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_WARNING_TRIGGER_ADCU,
    MOT_VAR_THERMISTOR_WARNING_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_IS_MONITOR_ENABLE,
}
MotVarId_Config_Thermistor_T;

typedef MotVarId_Config_Thermistor_T MotVarId_Config_BoardThermistor_T;
typedef MotVarId_Config_Thermistor_T MotVarId_Config_MotorThermistor_T;

/*
    Instance0 -> VSource
    Instance1 -> VSensor
    Instance2 -> VAcc
*/
typedef enum MotVarId_Config_VMonitor
{
    MOT_VAR_VMONITOR_R1, // All instance Read-Only
    MOT_VAR_VMONITOR_R2, // All instance Read-Only
    MOT_VAR_VMONITOR_FAULT_UPPER_ADCU,
    MOT_VAR_VMONITOR_FAULT_LOWER_ADCU,
    MOT_VAR_VMONITOR_WARNING_UPPER_ADCU,
    MOT_VAR_VMONITOR_WARNING_LOWER_ADCU,
    MOT_VAR_VMONITOR_IS_ENABLE,
}
MotVarId_Config_VMonitor_T;

typedef enum MotVarId_Config_BootRef
{
    MOT_VAR_BOOT_REF_FAST_BOOT,
    MOT_VAR_BOOT_REF_BEEP,
    MOT_VAR_BOOT_REF_BLINK,
    // MOT_VAR_BOOT_REF_PROTOCOL_INDEX,
}
MotVarId_Config_BootRef_T;


/******************************************************************************/
/*
    Meta
*/
/******************************************************************************/
/*
    Type of NameBase e.g. MotVarId_Monitor_General_T
    struct id
*/
typedef enum MotVarId_Type_RealTime /* : uint16_t */
{
    /* Monitor - Read-Only */
    MOT_VAR_ID_TYPE_MONITOR_GENERAL,
    MOT_VAR_ID_TYPE_MONITOR_ANALOG_USER,
    MOT_VAR_ID_TYPE_MONITOR_MOTOR,
    MOT_VAR_ID_TYPE_MONITOR_MOTOR_FOC,
    MOT_VAR_ID_TYPE_MONITOR_MOTOR_SENSOR,
    /* Control - Read/Write */
    MOT_VAR_ID_TYPE_CONTROL_GENERAL,
    MOT_VAR_ID_TYPE_CONTROL_MOTOR,
    // MOT_VAR_ID_TYPE_VAR_PROTOCOL,
    /* Cmd - Write-Only */
    MOT_VAR_ID_TYPE_CMD_GENERAL,
    MOT_VAR_ID_TYPE_CMD_MOTOR,
    MOT_VAR_ID_TYPE_CMD_SYSTEM,
    MOT_VAR_ID_TYPE_REAL_TIME_END = 15U,
}
MotVarId_Type_RealTime_T;

typedef enum MotVarId_Type_Config
{
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_PRIMARY,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_SECONDARY,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_HALL,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_ENCODER,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_THERMISTOR,
    MOT_VAR_ID_TYPE_CONFIG_MOTOR_PID,
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
typedef enum MotVarId_TypeType
{
    MOT_VAR_ID_TYPE_REAL_TIME,
    MOT_VAR_ID_TYPE_CONFIG,
}
MotVarId_TypeType_T;

typedef enum MotVarId_Instance_BoardThermistor
{
    MOT_VAR_ID_THERMISTOR_PCB,
    MOT_VAR_ID_THERMISTOR_MOSFETS_0,
    MOT_VAR_ID_THERMISTOR_MOSFETS_1,
    MOT_VAR_ID_THERMISTOR_MOSFETS_2,
    MOT_VAR_ID_THERMISTOR_MOSFETS_3,
}
MotVarId_Instance_BoardThermistor_T;

typedef enum MotVarId_Instance_VMonitor
{
    MOT_VAR_ID_VMONITOR_SOURCE,
    MOT_VAR_ID_VMONITOR_SENSOR,
    MOT_VAR_ID_VMONITOR_ACCS,
}
MotVarId_Instance_VMonitor_T;

typedef enum MotVarId_Instance_Motor
{
    MOT_VAR_ID_MOTOR_0,
    MOT_VAR_ID_MOTOR_1,
    MOT_VAR_ID_MOTOR_2,
    MOT_VAR_ID_MOTOR_3,
}
MotVarId_Instance_Motor_T;

/* Alternatively map 1:1 with type */
// typedef enum MotVarId_Instance_Prefix
// {
//     MOT_VAR_ID_INSTANCE_PREFIX_BOARD,
//     MOT_VAR_ID_INSTANCE_PREFIX_MOTOR,
// }
// MotVarId_Instance_Prefix_T;

typedef union MotVarId
{
    struct
    {
        uint16_t NameBase           : 4U;
        uint16_t NameType           : 4U; /* Name's Type - corresponds 1:1 with enum type */
        uint16_t NameTypeType       : 1U; /* Name Type's Type */
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
    uint16_t Word16;
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
    // On Set
    MOT_VAR_STATUS_ERROR_READ_ONLY,
    MOT_VAR_STATUS_ERROR_PROTOCOL_CONTROL_DISABLED,
    MOT_VAR_STATUS_ERROR_RUNNING,
    // MOT_VAR_STATUS_ERROR_REFUSED_BY_STATE_MACHINE,
    // MOT_VAR_STATUS_ASYNC,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVarId_Status_T;

#endif

