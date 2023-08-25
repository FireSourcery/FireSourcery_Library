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

typedef enum MotVarId_Tag
{
    /*
        Partition0 - 0-255 0X0-0xFF
        Read Only
    */
    /* Motor[0] by default*/
    MOT_VAR_NULL = 0U,
    MOT_VAR_SPEED = 1U,             /* FracS16Abs,    */
    MOT_VAR_I_PHASE = 2U,           /* FracS16Abs, may over saturate */
    MOT_VAR_V_PHASE = 3U,           /* FracS16Abs, may over saturate */
    MOT_VAR_POWER = 4U,             /* FracS16Abs, may over saturate */
    MOT_VAR_RAMP_CMD = 5U,
    MOT_VAR_MOTOR_STATE = 6U,       /* Value enum:  */
    MOT_VAR_MOTOR_STATUS_FLAGS = 7U,
    // MOT_VAR_MOTOR_FAULT_FLAGS = 5U,

    MOT_VAR_FOC_IQ = 16U,
    MOT_VAR_FOC_ID = 17U,
    MOT_VAR_FOC_Q_REQ = 18U,
    MOT_VAR_FOC_D_REQ = 19U,
    MOT_VAR_FOC_IA = 20U,
    MOT_VAR_FOC_IB = 21U,
    MOT_VAR_FOC_IC = 22U,
    MOT_VAR_FOC_VQ = 23U,
    MOT_VAR_FOC_VD = 24U,

    /* Motor Controller / Motor Global */
    MOT_VAR_MC_STATE            = 32U,  /* Value enum: 0:INIT, 1:STOP, 2:RUN, 3:FAULT */
    MOT_VAR_MC_STATUS_FLAGS     = 33U,
    // MOT_VAR_MC_ERROR_FLAGS   = 34U,
    MOT_VAR_V_SOURCE            = 40U,  /* Base as ADCU */
    MOT_VAR_V_SENSOR            = 41U,
    MOT_VAR_V_ACC               = 42U,
    MOT_VAR_BATTERY_CHARGE      = 47U,
    MOT_VAR_HEAT_PCB            = 43U,
    MOT_VAR_HEAT_MOSFETS        = 44U,
    MOT_VAR_HEAT_PCB_DEG_C      = 45U,
    MOT_VAR_HEAT_MOSFETS_DEG_C  = 46U,

    MOT_VAR_DEBUG = 128U,
    MOT_VAR_MILLIS = 129U,

    MOT_VAR_ANALOG_THROTTLE = 130U,         /* Value U16 */
    MOT_VAR_ANALOG_BRAKE = 131U,            /* Value U16 */

    MOT_VAR_TX_PACKET_COUNT = 254U,
    MOT_VAR_RX_PACKET_COUNT = 255U,

    /*
        Partition1 256-511 0x100-0x1FF
        Read Write
    */
    /* Motor Controller / Motor Global */
    MOT_VAR_USER_CMD    = 256U,
    MOT_VAR_DIRECTION   = 259U,             /* Value enum: 0:Neutral, 1:Reverse, 2:Forward */
    // MOT_VAR_DIRECTION_FORWARD = 0x01U,   /* Main direction */
    // MOT_VAR_DIRECTION_REVERSE = 0x02U,   /* Main direction */
    // MOT_VAR_DIRECTION_NEUTRAL = 0x03U,   /* Main direction */
    MOT_VAR_ACTIVE_FEEDBACK_MODE = 260U,
    MOT_VAR_ACTIVE_SPEED_LIMIT_SCALAR16, /*   */
    MOT_VAR_ACTIVE_I_LIMIT_SCALAR16,

    /* Motor[0] */

    /*
        Partition2 256-512 0x200-0x2FF
        Write Only
    */
    MOT_VAR_THROTTLE            = 257U,     /* Value U16 */
    MOT_VAR_BRAKE               = 258U,     /* Value U16 */
    MOT_VAR_BEEP                = 512U,     /* Beep */
    MOT_VAR_CALIBRATE_SENSOR    = 514U,
    MOT_VAR_CALIBRATE_ADC       = 515U,
    MOT_VAR_CALIBRATE           = 516U,     /* Value enum: ADC, Sensor, */
    // Set Cmd and Feedback Mode
    // MOT_VAR_CMD_VOLTAGE = ,         /* Value [-32768:32767] */
    // MOT_VAR_CMD_CURRENT = ,         /* Value [-32768:32767] */
    // MOT_VAR_CMD_SPEED = ,           /* Value [-32768:32767] */
    // MOT_VAR_CMD_ANGLE = ,           /*   */

    /*
        Partition4 1024  0x400-0x4FF
        Parameter Vars Read Write
    */
    /*
        Motor[0] by default
    */
    MOT_VAR_COMMUTATION_MODE = 1024U,       /* Motor_CommutationMode_T */
    MOT_VAR_SENSOR_MODE,                    /* Motor_SensorMode_T */
    MOT_VAR_DEFAULT_FEEDBACK_MODE,          /* Motor_FeedbackModeId_T */
    MOT_VAR_DIRECTION_CALIBRATION,          /* Motor_DirectionCalibration_T */
    MOT_VAR_POLE_PAIRS,
    MOT_VAR_KV,
    MOT_VAR_SPEED_FEEDBACK_REF_RPM,
    MOT_VAR_SPEED_V_REF_RPM,
    /* Calibration Set */
    MOT_VAR_I_REF_PEAK_ADCU,
    MOT_VAR_IA_REF_ZERO_ADCU,
    MOT_VAR_IB_REF_ZERO_ADCU,
    MOT_VAR_IC_REF_ZERO_ADCU,

    MOT_VAR_BASE_SPEED_LIMIT_FORWARD_SCALAR16,      /* ScalarU16 */
    MOT_VAR_BASE_SPEED_LIMIT_REVERSE_SCALAR16,      /* ScalarU16 */
    MOT_VAR_BASE_I_LIMIT_MOTORING_SCALAR16,         /* ScalarU16 */
    MOT_VAR_BASE_I_LIMIT_GENERATING_SCALAR16,       /* ScalarU16 */
    MOT_VAR_RAMP_ACCEL_TIME_CYCLES,
    MOT_VAR_ALIGN_MODE,
    MOT_VAR_ALIGN_POWER_SCALAR16,                   /* V or I, ScalarU16 */
    MOT_VAR_ALIGN_TIME_CYCLES,                      /* Control Cycles */
    MOT_VAR_OPEN_LOOP_POWER_SCALAR16,               /* Scalar16 */
    MOT_VAR_OPEN_LOOP_SPEED_SCALAR16,         /* Scalar16 */
    MOT_VAR_OPEN_LOOP_ACCEL_TIME_CYCLES,            /* Cycles */
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
    MOT_VAR_ENCODER_COUNTS_PER_REVOLUTION, /* */

    /* Calibration Set */
    MOT_VAR_IS_QUADRATURE_CAPTURE_ENABLED,
    MOT_VAR_IS_A_LEAD_B_POSITIVE,
    /* Auto set */
    MOT_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP,
    MOT_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR,

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
    MOT_VAR_PID_SPEED_KP_FIXED_16,
    MOT_VAR_PID_SPEED_KI_FIXED_16,
    MOT_VAR_PID_SPEED_KD_FIXED_16,
    MOT_VAR_PID_SPEED_SAMPLE_FREQ, /* Read-Only for now */
    // MOT_VAR_PID_SPEED_MODE,
    MOT_VAR_PID_FOC_IQ_KP_FIXED_16,
    MOT_VAR_PID_FOC_IQ_KI_FIXED_16,
    MOT_VAR_PID_FOC_IQ_KD_FIXED_16,
    MOT_VAR_PID_FOC_IQ_SAMPLE_FREQ,
    // MOT_VAR_PID_FOC_IQ_MODE,
    MOT_VAR_PID_FOC_ID_KP_FIXED_16,
    MOT_VAR_PID_FOC_ID_KI_FIXED_16,
    MOT_VAR_PID_FOC_ID_KD_FIXED_16,
    MOT_VAR_PID_FOC_ID_SAMPLE_FREQ,
    // MOT_VAR_PID_FOC_ID_MODE
    // Prop_Gain,
    // Prop_Gain_Shift,
    // Integral_Gain,
    // Integral_Gain_Shift,

    MOT_VAR_THERMISTOR_MOTOR_TYPE,      /* Thermistor_Type_T */
    MOT_VAR_THERMISTOR_MOTOR_R,         /* NTC Unit Conversion */
    MOT_VAR_THERMISTOR_MOTOR_T,
    MOT_VAR_THERMISTOR_MOTOR_B,
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_ADCU,        /* Linear Unit Conversion */
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_MOTOR_FAULT_ADCU,            /* Monitor Limits */
    MOT_VAR_THERMISTOR_MOTOR_FAULT_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_WARNING_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_WARNING_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_MOTOR_IS_MONITOR_ENABLE,

    /*
        Motor Controller
    */
    MOT_VAR_V_SOURCE_VOLTS,             /* Volts */
    MOT_VAR_BATTERY_ZERO_ADCU,
    MOT_VAR_BATTERY_FULL_ADCU,
    MOT_VAR_USER_INPUT_MODE,            /* MotorController_InputMode_T */
    MOT_VAR_BRAKE_MODE,                 /* MotorController_BrakeMode_T */
    MOT_VAR_ZERO_CMD_MODE,              /* MotorController_ZeroCmdMode_T */
    MOT_VAR_BUZZER_FLAGS_ENABLE,        /* MotorController_BuzzerFlags_T */
    MOT_VAR_OPT_DIN_FUNCTION,           /* MotorController_OptDinFunction_T */
    MOT_VAR_OPT_DIN_SPEED_LIMIT_SCALAR16,
    MOT_VAR_I_LIMIT_LOW_V_SCALAR16,
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
    MOT_VAR_THERMISTOR_PCB_TYPE,      /* Thermistor_Type_T */
    MOT_VAR_THERMISTOR_PCB_R,         /* NTC Unit Conversion */
    MOT_VAR_THERMISTOR_PCB_T,
    MOT_VAR_THERMISTOR_PCB_B,
    MOT_VAR_THERMISTOR_PCB_LINEAR_T0_ADCU,        /* Linear Unit Conversion */
    MOT_VAR_THERMISTOR_PCB_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_PCB_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_PCB_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_PCB_FAULT_ADCU,            /* Monitor Limits */
    MOT_VAR_THERMISTOR_PCB_FAULT_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_PCB_WARNING_ADCU,
    MOT_VAR_THERMISTOR_PCB_WARNING_THRESHOLD_ADCU,
    MOT_VAR_THERMISTOR_PCB_IS_MONITOR_ENABLE,

    MOT_VAR_THERMISTOR_MOSFETS_TYPE,      /* Thermistor_Type_T */
    MOT_VAR_THERMISTOR_MOSFETS_R,         /* NTC Unit Conversion */
    MOT_VAR_THERMISTOR_MOSFETS_T,
    MOT_VAR_THERMISTOR_MOSFETS_B,
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_ADCU,        /* Linear Unit Conversion */
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T0_DEG_C,
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_ADCU,
    MOT_VAR_THERMISTOR_MOSFETS_LINEAR_T1_DEG_C,
    MOT_VAR_THERMISTOR_MOSFETS_FAULT_ADCU,            /* Monitor Limits */
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
    MOT_VAR_ANALOG_DIN_BRAKE_VALUE_FRAC16,
    MOT_VAR_ANALOG_DIN_BRAKE_IS_ENABLE,
    MOT_VAR_ANALOG_DIRECTION_PINS, /* MotAnalogUser_DirectionPins_T */

    /* Protocol_Params_T */
    MOT_VAR_PROTOCOL0_XCVR_ID,
    MOT_VAR_PROTOCOL0_SPECS_ID,
    MOT_VAR_PROTOCOL0_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL0_BAUD_RATE,
    MOT_VAR_PROTOCOL0_IS_ENABLED,   /* Enable on start up */

    MOT_VAR_PROTOCOL1_XCVR_ID,
    MOT_VAR_PROTOCOL1_SPECS_ID,
    MOT_VAR_PROTOCOL1_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL1_BAUD_RATE,
    MOT_VAR_PROTOCOL1_IS_ENABLED,   /* Enable on start up */

    MOT_VAR_PROTOCOL2_XCVR_ID,
    MOT_VAR_PROTOCOL2_SPECS_ID,
    MOT_VAR_PROTOCOL2_WATCHDOG_TIME,
    MOT_VAR_PROTOCOL2_BAUD_RATE,
    MOT_VAR_PROTOCOL2_IS_ENABLED,   /* Enable on start up */

    /*
        Partition5 2048  0x800-0x8FF
        Read Only, Once Manufacturer
    */
    MOT_VAR_VERSION_KMC_0 = 2048U,
    MOT_VAR_VERSION_KMC_1 = 2049U,
    MOT_VAR_VERSION_KMC_2 = 2050U,
    MOT_VAR_VERSION_KMC_3 = 2051U,
    MOT_VAR_VERSION_KMC_REG32 = 2052U,  /* Var32 */
    MOT_VAR_I_MAX_AMP       = 2053U,
    MOT_VAR_V_MAX_VOLTS     = 2054U,

    // /*
    //     Once Manufacturer
    // */
    // MOT_VAR_NAME_0 = 2304U,
    // MOT_VAR_NAME_1 = 2305U,
    // MOT_VAR_NAME_2 = 2306U,
    // MOT_VAR_NAME_3 = 2307U,
    // MOT_VAR_NAME_4 = 2308U,
    // MOT_VAR_NAME_5 = 2309U,
    // MOT_VAR_NAME_6 = 2310U,
    // MOT_VAR_NAME_7 = 2311U,
    // MOT_VAR_NAME_REG32_0 = 2311U,
    // MOT_VAR_NAME_REG32_1 = 2311U,
    // MOT_VAR_SERIAL_NUMBER_0 = 2312U,
    // MOT_VAR_SERIAL_NUMBER_1 = 2313U,
    // MOT_VAR_SERIAL_NUMBER_2 = 2314U,
    // MOT_VAR_SERIAL_NUMBER_3 = 2315U,
    // MOT_VAR_SERIAL_NUMBER_REG32 = 2315U,
    // MOT_VAR_MANUFACTURE_NUMBER_0 = 2316U, /* Day */
    // MOT_VAR_MANUFACTURE_NUMBER_1 = 2317U, /* Month */
    // MOT_VAR_MANUFACTURE_NUMBER_2 = 2318U, /* Year */
    // MOT_VAR_MANUFACTURE_NUMBER_3 = 2319U, /* Resv */
    // MOT_VAR_MANUFACTURE_NUMBER_REG32 = 2319U,
    // MOT_VAR_HARDWARE_VERSION_0 = 2320U,
    // MOT_VAR_HARDWARE_VERSION_1 = 2321U,
    // MOT_VAR_HARDWARE_VERSION_2 = 2322U,
    // MOT_VAR_HARDWARE_VERSION_3 = 2323U,
    // MOT_VAR_HARDWARE_VERSION_REG32 = 2323U,
    // MOT_VAR_ID_EXT_0 = 2324U,
    // MOT_VAR_ID_EXT_1 = 2325U,
    // MOT_VAR_ID_EXT_2 = 2326U,
    // MOT_VAR_ID_EXT_3 = 2327U,
    // MOT_VAR_ID_EXT_REG32 = 2327U,
    // MOT_VAR_RESV_0 = 2328U,
    // MOT_VAR_RESV_1 = 2329U,
    // MOT_VAR_RESV_2 = 2330U,
    // MOT_VAR_RESV_3 = 2331U,
    // MOT_VAR_RESV_4 = 2332U,
    // MOT_VAR_RESV_5 = 2333U,
    // MOT_VAR_RESV_6 = 2334U,
    // MOT_VAR_RESV_7 = 2335U,

    MOT_VAR_RESERVED_65535 = 0xFFFFU,
}
MotVarId_T;

// typedef enum MotVarSize_Tag
// {
//     MOT_VAR_SIZE_16 = 0U,
//     MOT_VAR_SIZE_32 = 1U,
// }
// MotVarSize_T;

// uint8_t MotVarId_SizeOf(MotVarId_T varId)
// {

// }

#endif