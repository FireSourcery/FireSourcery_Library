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

/*

*/
typedef enum
{
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_DEBUG,

    MOT_VAR_SPEED,             // FracU16,
    MOT_VAR_I_PHASE,           // FracU16, may over saturate
    MOT_VAR_V_PHASE,           // FracU16, may over saturate
    MOT_VAR_POWER,             // FracU16, may over saturate
    MOT_VAR_RAMP_CMD,
    MOT_VAR_MOTOR_STATE,           // Value enum:
    MOT_VAR_MOTOR_STATUS_FLAGS,    // Includes Fault and Warning Flags
    MOT_VAR_MOTOR_FAULT_FLAGS,
    MOT_VAR_MOTOR_HEAT, // MOT_VAR_MOTOR_HEAT_DEG_C = 10U,

    // MOT_VAR_ELECTRICAL_ANGLE  = 7U, // Degrees 16
    // MOT_VAR_MECHANICAL_ANGLE  = 7U,
    // MOT_VAR_MOTOR_CALIBRATION_STATE = 260U,
    // MOT_VAR_MOTOR_CALIBRATION_STATE_INDEX = 260U,
    MOT_VAR_MOTOR_ACTIVE_SPEED_LIMIT , // Scalar16, RPM Option
    MOT_VAR_MOTOR_ACTIVE_I_LIMIT  ,

    MOT_VAR_FOC_IA,
    MOT_VAR_FOC_IB,
    MOT_VAR_FOC_IC,
    MOT_VAR_FOC_IQ,
    MOT_VAR_FOC_ID,
    MOT_VAR_FOC_VQ,
    MOT_VAR_FOC_VD,
    MOT_VAR_FOC_Q_REQ,
    MOT_VAR_FOC_D_REQ,

    MOT_VAR_ENCODER_FREQ,
    // MOT_VAR_ENCODER_   ,
    // MOT_VAR_PID_OUT  ,

    /*
        Motor Controller / Motor Global
        Group 1
    */
    MOT_VAR_MC_STATE,       // Value enum: 0:INIT, 1:STOP, 2:RUN, 3:FAULT
    MOT_VAR_MC_STATUS_FLAGS,
    MOT_VAR_MC_ERROR_FLAGS,
    MOT_VAR_V_SOURCE,       // ADCU, Volts * 10
    MOT_VAR_V_SENSOR,       // ADCU, mV
    MOT_VAR_V_ACC,          // ADCU, mV
    MOT_VAR_BATTERY_CHARGE,  // Scalar16, Fraction1000
    MOT_VAR_HEAT_PCB,       // ADCU, DegC
    MOT_VAR_HEAT_MOSFETS,
    // MOT_VAR_HEAT_PCB_DEG_C      ,
    // MOT_VAR_HEAT_MOSFETS_DEG_C  ,
    // MOT_VAR_ACTIVE_SPEED_LIMIT, //
    // MOT_VAR_ACTIVE_I_LIMIT,
    MOT_VAR_ANALOG_THROTTLE,         // Value Scalar16
    MOT_VAR_ANALOG_THROTTLE_DIN,     // Bool
    MOT_VAR_ANALOG_BRAKE,            // Value Scalar16
    MOT_VAR_ANALOG_BRAKE_DIN,        // Bool

    // MOT_VAR_TX_PACKET_COUNT = 254U,
    // MOT_VAR_RX_PACKET_COUNT = 255U,
}
MotVarId_RealTimeMonitor_T;

/*
    Group 2
*/
typedef enum
{
    /*
        Run time command functions
        Read Write
        Read Values may differ from write
    */
    /* Motor[0] */
    MOT_VAR_MOTOR_USER_CMD,                 // Read/Write buffered user value.
    MOT_VAR_MOTOR_DIRECTION,                // Motor_Direction_T // CW/CCW. Write buffered user value, read state value
    MOT_VAR_MOTOR_ACTIVE_FEEDBACK_MODE,     // Write buffered user value, read state value

    MOT_VAR_MOTOR_USER_SPEED_LIMIT, //
    MOT_VAR_MOTOR_USER_I_LIMIT,

    /* Motor Controller / Motor Global */
    MOT_VAR_USER_CMD,                   // Value [-32768:32767]. Selected Mode, Speed by default
    MOT_VAR_DIRECTION,                  // MotorController_Direction_T // Value enum: 0:Neutral, 1:Reverse, 2:Forward
    // MOT_VAR_ACTIVE_FEEDBACK_MODE,
    // MOT_VAR_USER_SPEED_LIMIT, //
    // MOT_VAR_USER_I_LIMIT,

    /*
        Run time command functions
        Write Only
        Write Via User StateMachine Function
    */
     /* Motor[0] */
    MOT_VAR_MOTOR_CMD_SPEED,           // Value [-32768:32767]
    MOT_VAR_MOTOR_CMD_CURRENT,         // Value [-32768:32767]
    MOT_VAR_MOTOR_CMD_VOLTAGE,         // Value [0:32767]
    MOT_VAR_MOTOR_CMD_ANGLE,           //
    // MOT_VAR_MOTOR_RELEASE_CONTROL,
    // MOT_VAR_MOTOR_DISABLE,
    // MOT_VAR_MOTOR_CLEAR_FAULT,
    // MOT_VAR_MOTOR_SET_FAULT,
    // MOT_VAR_MOTOR_CALIBRATE_SENSOR,
    // MOT_VAR_MOTOR_CALIBRATE_ADC,
    // MOT_VAR_MOTOR_CALIBRATE,     // Value enum: ADC, Sensor,
    // MOT_VAR_MOTOR_CALIBRATE_,

    /* Motor Controller / Motor Global */
    MOT_VAR_THROTTLE,     // Value Scalar16
    MOT_VAR_BRAKE,     // Value Scalar16
    MOT_VAR_RELEASE_CONTROL,
    MOT_VAR_DISABLE_CONTROL,
    MOT_VAR_CLEAR_FAULT,
    MOT_VAR_SET_FAULT,

    MOT_VAR_BEEP,     // Beep
}
MotVarId_RealTimeControl_T;

/*
    Group 3
    Parameter Vars - Read Write
    Motor[0] by default
*/
typedef enum
{
    MOT_VAR_COMMUTATION_MODE,               // Motor_CommutationMode_T
    MOT_VAR_SENSOR_MODE,                    // Motor_SensorMode_T
    MOT_VAR_DEFAULT_FEEDBACK_MODE,          // Motor_FeedbackModeId_T
    MOT_VAR_DIRECTION_CALIBRATION,          // Motor_DirectionCalibration_T
    MOT_VAR_POLE_PAIRS,
    MOT_VAR_KV,
    MOT_VAR_SPEED_FEEDBACK_REF_RPM,
    MOT_VAR_SPEED_V_REF_RPM,
    /* Calibration Set */
    MOT_VAR_I_REF_PEAK_ADCU,
    MOT_VAR_IA_REF_ZERO_ADCU,
    MOT_VAR_IB_REF_ZERO_ADCU,
    MOT_VAR_IC_REF_ZERO_ADCU,

    MOT_VAR_BASE_SPEED_LIMIT_FORWARD,      // ScalarU16
    MOT_VAR_BASE_SPEED_LIMIT_REVERSE,      // ScalarU16
    MOT_VAR_BASE_I_LIMIT_MOTORING,         // ScalarU16
    MOT_VAR_BASE_I_LIMIT_GENERATING,       // ScalarU16
    MOT_VAR_RAMP_ACCEL_TIME_CYCLES,
    MOT_VAR_ALIGN_MODE,
    MOT_VAR_ALIGN_POWER,                   // V or I, ScalarU16
    MOT_VAR_ALIGN_TIME_CYCLES,                      // Control Cycles
    MOT_VAR_OPEN_LOOP_POWER,               // U16
    MOT_VAR_OPEN_LOOP_SPEED,               // U16
    MOT_VAR_OPEN_LOOP_ACCEL_TIME_CYCLES,            // Cycles
    // uint16_t SurfaceDiameter;
    // uint16_t GearRatio_Factor;
    // uint16_t GearRatio_Divisor;
    MOT_VAR_PHASE_PWM_MODE,

    /* Hall */
    MOT_VAR_HALL_SENSOR_TABLE_1,
    MOT_VAR_HALL_SENSOR_TABLE_2,
    MOT_VAR_HALL_SENSOR_TABLE_3,
    MOT_VAR_HALL_SENSOR_TABLE_4,
    MOT_VAR_HALL_SENSOR_TABLE_5,
    MOT_VAR_HALL_SENSOR_TABLE_6,

    /*
        Hall/Encoder
    */
    MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION,
    MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP,
    MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR,
    MOT_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED,
    MOT_VAR_ENCODER_IS_A_LEAD_B_POSITIVE,
    //ScalarSpeedRef_Rpm;          Derive Frac16 Units.
    //SurfaceDiameter;             Derive Linear Units.
    //GearRatio_Factor;             Derive Linear Units. Surface:Encoder Ratio
    //GearRatio_Divisor;             Derive Linear Units.

    /*
        Sine Cos Encoder
    */
    // MOT_VAR_SIN_COS_ZERO_ADCU
    // MOT_VAR_SIN_COS_MAX_ADCU
    // MOT_VAR_SIN_COS_MAX_MILLIV
    // MOT_VAR_SIN_COS_ANGLE_OFFET
    // MOT_VAR_SIN_COS_IS_B_POSITIVE
    // MOT_VAR_SIN_COS_ELECTRICAL_ROTATIONS_PER_CYCLE

    /*
        PID
    */
    /* Set with interface functions */
    MOT_VAR_PID_SPEED_KP_FIXED16,
    MOT_VAR_PID_SPEED_KI_FIXED16,
    MOT_VAR_PID_SPEED_KD_FIXED16,
    MOT_VAR_PID_SPEED_SAMPLE_FREQ,
    // MOT_VAR_PID_SPEED_MODE,
    MOT_VAR_PID_FOC_IQ_KP_FIXED16,
    MOT_VAR_PID_FOC_IQ_KI_FIXED16,
    MOT_VAR_PID_FOC_IQ_KD_FIXED16,
    MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ,
    // MOT_VAR_PID_FOC_IQ_MODE,
    MOT_VAR_PID_FOC_ID_KP_FIXED16,
    MOT_VAR_PID_FOC_ID_KI_FIXED16,
    MOT_VAR_PID_FOC_ID_KD_FIXED16,
    MOT_VAR_PID_FOC_ID_SAMPLE_FREQ,
    // MOT_VAR_PID_FOC_ID_MODE

    // Prop_Gain,
    // Prop_Gain_Shift,
    // Integral_Gain,
    // Integral_Gain_Shift,

    MOT_VAR_THERMISTOR_MOTOR_TYPE,      // Thermistor_Type_T
    MOT_VAR_THERMISTOR_MOTOR_R,         // NTC Unit Conversion
    MOT_VAR_THERMISTOR_MOTOR_T,
    MOT_VAR_THERMISTOR_MOTOR_B,
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_ADCU,        // Linear Unit Conversion
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_MOTOR_FAULT_ADCU,            // Monitor Limits
    MOT_VAR_THERMISTOR_MOTOR_FAULT_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_WARNING_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_WARNING_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_IS_MONITOR_ENABLE,
}
MotVarId_ParamsMotor_T;

/*
    Group 4
*/
typedef enum
{
    MOT_VAR_V_SOURCE_VOLTS,             // Volts
    MOT_VAR_BATTERY_ZERO_ADCU,
    MOT_VAR_BATTERY_FULL_ADCU,
    MOT_VAR_USER_INPUT_MODE,            // MotorController_InputMode_T
    MOT_VAR_BRAKE_MODE,                 // MotorController_BrakeMode_T
    MOT_VAR_ZERO_CMD_MODE,              // MotorController_ZeroCmdMode_T
    MOT_VAR_BUZZER_FLAGS_ENABLE,        // MotorController_BuzzerFlags_T
    MOT_VAR_OPT_DIN_FUNCTION,           // MotorController_OptDinFunction_T
    MOT_VAR_OPT_DIN_SPEED_LIMIT,
    MOT_VAR_I_LIMIT_LOW_V,
    MOT_VAR_CAN_SERVICES_ID,
    MOT_VAR_CAN_IS_ENABLE,

    /* VMonitor_Params_T */
    MOT_VAR_VMONITOR_SOURCE_LIMIT_UPPER_ADCU,
    MOT_VAR_VMONITOR_SOURCE_LIMIT_LOWER_ADCU,
    MOT_VAR_VMONITOR_SOURCE_WARNING_UPPER_ADCU,
    MOT_VAR_VMONITOR_SOURCE_WARNING_LOWER_ADCU,
    MOT_VAR_VMONITOR_SOURCE_IS_ENABLE,

    MOT_VAR_VMONITOR_SENSE_LIMIT_UPPER_ADCU,
    MOT_VAR_VMONITOR_SENSE_LIMIT_LOWER_ADCU,
    MOT_VAR_VMONITOR_SENSE_WARNING_UPPER_ADCU,
    MOT_VAR_VMONITOR_SENSE_WARNING_LOWER_ADCU,
    MOT_VAR_VMONITOR_SENSE_IS_ENABLE,

    MOT_VAR_VMONITOR_ACC_LIMIT_UPPER_ADCU,
    MOT_VAR_VMONITOR_ACC_LIMIT_LOWER_ADCU,
    MOT_VAR_VMONITOR_ACC_WARNING_UPPER_ADCU,
    MOT_VAR_VMONITOR_ACC_WARNING_LOWER_ADCU,
    MOT_VAR_VMONITOR_ACC_IS_ENABLE,

    /* Thermistor_Params_T */
    MOT_VAR_THERMISTOR_PCB_TYPE,      // Thermistor_Type_T
    MOT_VAR_THERMISTOR_PCB_R,         // NTC Unit Conversion
    MOT_VAR_THERMISTOR_PCB_T,
    MOT_VAR_THERMISTOR_PCB_B,
    MOT_VAR_THERMISTOR_PCB_LINEAR_T0_ADCU,        // Linear Unit Conversion
    MOT_VAR_THERMISTOR_PCB_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_PCB_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_PCB_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_PCB_FAULT_ADCU,            // Monitor Limits
    MOT_VAR_THERMISTOR_PCB_FAULT_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_PCB_WARNING_ADCU,
    MOT_VAR_THERMISTOR_PCB_WARNING_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_PCB_IS_MONITOR_ENABLE,

    MOT_VAR_THERMISTOR_MOSFETS_TYPE,      // Thermistor_Type_T
    MOT_VAR_THERMISTOR_MOSFETS_R,         // NTC Unit Conversion
    MOT_VAR_THERMISTOR_MOSFETS_T,
    MOT_VAR_THERMISTOR_MOSFETS_B,
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_ADCU,        // Linear Unit Conversion
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_MOSFETS_FAULT_ADCU,            // Monitor Limits
    MOT_VAR_THERMISTOR_MOSFETS_FAULT_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_MOSFETS_WARNING_ADCU,
    MOT_VAR_THERMISTOR_MOSFETS_WARNING_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_MOSFETS_IS_MONITOR_ENABLE,

    /* MotAnalogUser_Params_T */
    MOT_VAR_ANALOG_THROTTLE_ZERO_ADCU,
    MOT_VAR_ANALOG_THROTTLE_MAX_ADCU,
    MOT_VAR_ANALOG_THROTTLE_EDGE_PIN_IS_ENABLE,
    MOT_VAR_ANALOG_BRAKE_ZERO_ADCU,
    MOT_VAR_ANALOG_BRAKE_MAX_ADCU,
    MOT_VAR_ANALOG_BRAKE_EDGE_PIN_IS_ENABLE,
    MOT_VAR_ANALOG_DIN_BRAKE_VALUE,
    MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE,
    MOT_VAR_ANALOG_DIRECTION_PINS, // MotAnalogUser_DirectionPins_T

    /* Protocol_Params_T */
    MOT_VAR_PROTOCOL0_XCVR_ID,
    MOT_VAR_PROTOCOL0_SPECS_ID,
    MOT_VAR_PROTOCOL0_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL0_BAUD_RATE,
    MOT_VAR_PROTOCOL0_IS_ENABLED,

    MOT_VAR_PROTOCOL1_XCVR_ID,
    MOT_VAR_PROTOCOL1_SPECS_ID,
    MOT_VAR_PROTOCOL1_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL1_BAUD_RATE,
    MOT_VAR_PROTOCOL1_IS_ENABLED,

    MOT_VAR_PROTOCOL2_XCVR_ID,
    MOT_VAR_PROTOCOL2_SPECS_ID,
    MOT_VAR_PROTOCOL2_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL2_BAUD_RATE,
    MOT_VAR_PROTOCOL2_IS_ENABLED,

    /*
        Const
    */
    // MOT_VAR_VERSION_KMC_0,
    // MOT_VAR_VERSION_KMC_1,
    // MOT_VAR_VERSION_KMC_2,
    // MOT_VAR_VERSION_KMC_3,
    // MOT_VAR_VERSION_KMC_REG32,
    MOT_VAR_I_MAX_AMP,
    MOT_VAR_V_MAX_VOLTS,

    /*
        Once Manufacturer
    */
    // MOT_VAR_NAME_0,
    // MOT_VAR_NAME_1,
    // MOT_VAR_NAME_2,
    // MOT_VAR_NAME_3,
    // MOT_VAR_NAME_4,
    // MOT_VAR_NAME_5,
    // MOT_VAR_NAME_6,
    // MOT_VAR_NAME_7,
    // MOT_VAR_NAME_REG32_0,
    // MOT_VAR_NAME_REG32_1,
    // MOT_VAR_SERIAL_NUMBER_0,
    // MOT_VAR_SERIAL_NUMBER_1,
    // MOT_VAR_SERIAL_NUMBER_2,
    // MOT_VAR_SERIAL_NUMBER_3,
    // MOT_VAR_SERIAL_NUMBER_REG32,
    // MOT_VAR_MANUFACTURE_NUMBER_0, // Day
    // MOT_VAR_MANUFACTURE_NUMBER_1, // Month
    // MOT_VAR_MANUFACTURE_NUMBER_2, // Year
    // MOT_VAR_MANUFACTURE_NUMBER_3, // Resv
    // MOT_VAR_MANUFACTURE_NUMBER_REG32,
    // MOT_VAR_HARDWARE_VERSION_0,
    // MOT_VAR_HARDWARE_VERSION_1,
    // MOT_VAR_HARDWARE_VERSION_2,
    // MOT_VAR_HARDWARE_VERSION_3,
    // MOT_VAR_HARDWARE_VERSION_REG32,
    // MOT_VAR_ID_EXT_0,
    // MOT_VAR_ID_EXT_1,
    // MOT_VAR_ID_EXT_2,
    // MOT_VAR_ID_EXT_3,
    // MOT_VAR_ID_EXT_REG32,
    // MOT_VAR_RESV_0,
    // MOT_VAR_RESV_1,
    // MOT_VAR_RESV_2,
    // MOT_VAR_RESV_3,
    // MOT_VAR_RESV_4,
    // MOT_VAR_RESV_5,
    // MOT_VAR_RESV_6,
    // MOT_VAR_RESV_7,
}
MotVarId_ParamsGlobal_T;

typedef enum MotVarId_Prefix_Tag
{
    MOT_VAR_ID_PREFIX_REAL_TIME_MONITOR = 0U,
    MOT_VAR_ID_PREFIX_REAL_TIME_CONTROL = 1U,
    MOT_VAR_ID_PREFIX_PARAMS_MOTOR = 2U,
    MOT_VAR_ID_PREFIX_PARAMS_GLOBAL = 3U,
    MOT_VAR_ID_PREFIX_END = 7U,
}
MotVarId_Prefix_T;

typedef union
{
    struct
    {
        uint8_t Prefix     : 2U;    /* 8 Sets/Pages, repeatable id  */
        uint8_t Alt        : 2U;    /* Data Option */
        uint8_t Motor      : 2U;    /* Allocate for 4 motors */
        uint8_t OptResv    : 2U;
    };
    uint8_t Byte;
}
MotVarId_Arg_T;

typedef union MotVarId_Tag
{
    struct
    {
        uint8_t Id8;     /* BaseId MotVarId_Monitor_T, MotVarId_Control_T,  MotVarId_ParamsMotor_T, MotVarId_ParamsGlobal_T */
        uint8_t Arg;    /*   */
    };
    struct
    {
        uint16_t Id10   : 10U;
        uint16_t Arg6   : 6U;
    };
    uint16_t Word16;
}
MotVarId_T;

#endif