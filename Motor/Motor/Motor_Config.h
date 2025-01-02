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
    @file   Motor_Config.h
    @author FireSourcery
    @version V0

    @brief  Config Fields Interface. Part of User Interface.
*/
/******************************************************************************/
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "Motor.h"
#include "Motor_StateMachine.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Config Field Id
*/
/******************************************************************************/

// Motor_Config_Primary_T
// Motor_Config_Calibration_T
typedef enum MotVarId_Config_MotorPrimary
{
    MOT_VAR_COMMUTATION_MODE,       /* Motor_CommutationMode_T, if runtime supported */
    MOT_VAR_SENSOR_MODE,            /* Motor_SensorMode_T, */
    MOT_VAR_DIRECTION_CALIBRATION,  /* Motor_DirectionCalibration_T */
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
    Actuation Values
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
    // MOT_VAR_PID_CURRENT_KP_FIXED16,
    // MOT_VAR_PID_CURRENT_KI_FIXED16,
    // MOT_VAR_PID_CURRENT_KD_FIXED16,
    // MOT_VAR_PID_CURRENT_SAMPLE_FREQ,
}
MotVarId_Config_MotorPid_T;

/******************************************************************************/
/* Calibration */
/******************************************************************************/
static inline Motor_SensorMode_T Motor_Config_GetSensorMode(const Motor_T * p_motor)             { return p_motor->Config.SensorMode; }
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(const Motor_T * p_motor)    { return p_motor->Config.DirectionForward; }
static inline uint8_t Motor_Config_GetPolePairs(const Motor_T * p_motor)                         { return p_motor->Config.PolePairs; }
static inline uint16_t Motor_Config_GetKv(const Motor_T * p_motor)                               { return p_motor->Config.Kv; }
static inline uint16_t Motor_Config_GetVSpeedScalar_UFract16(const Motor_T * p_motor)            { return p_motor->Config.VSpeedScalar_UFract16; }

static inline uint16_t Motor_Config_GetSpeedVRef_Rpm(const Motor_T * p_motor)                    { return Motor_GetSpeedVRef_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetSpeedVMatchRef_Rpm(const Motor_T * p_motor)               { return (p_motor->Config.VSpeedScalar_UFract16 * Motor_GetSpeedVRef_Rpm(p_motor)) >> 15U; }

static inline uint16_t Motor_Config_GetIaZero_Adcu(const Motor_T * p_motor)                      { return p_motor->Config.IaZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(const Motor_T * p_motor)                      { return p_motor->Config.IbZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(const Motor_T * p_motor)                      { return p_motor->Config.IcZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIPeakRef_Adcu(const Motor_T * p_motor)                    { return Motor_GetIPeakRef_Adcu(p_motor); }

/******************************************************************************/
/* Persistent Base Limits */
/******************************************************************************/
static inline uint16_t Motor_Config_GetSpeedLimitForward_Fract16(const Motor_T * p_motor) { return p_motor->Config.SpeedLimitForward_Fract16; }
static inline uint16_t Motor_Config_GetSpeedLimitReverse_Fract16(const Motor_T * p_motor) { return p_motor->Config.SpeedLimitReverse_Fract16; }
static inline uint16_t Motor_Config_GetILimitMotoring_Fract16(const Motor_T * p_motor)    { return p_motor->Config.ILimitMotoring_Fract16; }
static inline uint16_t Motor_Config_GetILimitGenerating_Fract16(const Motor_T * p_motor)  { return p_motor->Config.ILimitGenerating_Fract16; }


/******************************************************************************/
/*
    Handle set using StateId
*/
/******************************************************************************/
static inline bool Motor_Config_IsConfigState(const Motor_T * p_motor)
{
    return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP);
    // || MSM_STATE_ID_FAULT
}

/******************************************************************************/
/* inline setters */
/******************************************************************************/
static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(const Motor_T * p_motor) { return p_motor->Config.CommutationMode; }
static inline void Motor_Config_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode) { p_motor->Config.CommutationMode = mode; }

/*  */
static inline uint32_t Motor_Config_GetRampAccel_Cycles(const Motor_T * p_motor) { return p_motor->Config.RampAccel_Cycles; }
static inline uint16_t Motor_Config_GetRampAccel_Millis(const Motor_T * p_motor) { return _Motor_MillisOf(p_motor->Config.RampAccel_Cycles); }
static inline void Motor_Config_SetRampAccel_Cycles(Motor_T * p_motor, uint32_t cycles) { p_motor->Config.RampAccel_Cycles = cycles; }
static inline void Motor_Config_SetRampAccel_Millis(Motor_T * p_motor, uint16_t millis) { p_motor->Config.RampAccel_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Config_GetAlignMode(const Motor_T * p_motor, Motor_AlignMode_T mode)    { return p_motor->Config.AlignMode; }
// static inline void Motor_Config_SetAlignMode( Motor_T * p_motor, Motor_AlignMode_T mode)         { p_motor->Config.AlignMode = mode; }
static inline uint16_t Motor_Config_GetAlignPower_Fract16(const Motor_T * p_motor) { return p_motor->Config.AlignPower_UFract16; }
static inline uint32_t Motor_Config_GetAlignTime_Cycles(const Motor_T * p_motor) { return p_motor->Config.AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(const Motor_T * p_motor) { return _Motor_MillisOf(p_motor->Config.AlignTime_Cycles); }
static inline void Motor_Config_SetAlignPower_Fract16(Motor_T * p_motor, uint16_t scalar_fract16) { p_motor->Config.AlignPower_UFract16 = (scalar_fract16 > MOTOR_STATIC.ALIGN_VPWM_MAX) ? MOTOR_STATIC.ALIGN_VPWM_MAX : scalar_fract16; }
static inline void Motor_Config_SetAlignTime_Cycles(Motor_T * p_motor, uint32_t cycles) { p_motor->Config.AlignTime_Cycles = cycles; }
static inline void Motor_Config_SetAlignTime_Millis(Motor_T * p_motor, uint16_t millis) { p_motor->Config.AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeed_Fract16(const Motor_T * p_motor) { return p_motor->Config.OpenLoopSpeed_UFract16; }
static inline uint16_t Motor_Config_GetOpenLoopPower_Fract16(const Motor_T * p_motor) { return p_motor->Config.OpenLoopPower_UFract16; }
static inline uint32_t Motor_Config_GetOpenLoopAccel_Cycles(const Motor_T * p_motor) { return p_motor->Config.OpenLoopAccel_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopAccel_Millis(const Motor_T * p_motor) { return _Motor_MillisOf(p_motor->Config.OpenLoopAccel_Cycles); }

static inline void Motor_Config_SetOpenLoopSpeed_Fract16(Motor_T * p_motor, uint16_t speed_fract16) { p_motor->Config.OpenLoopSpeed_UFract16 = speed_fract16; }
static inline void Motor_Config_SetOpenLoopPower_Fract16(Motor_T * p_motor, uint16_t scalar_fract16) { p_motor->Config.OpenLoopPower_UFract16 = scalar_fract16; }
static inline void Motor_Config_SetOpenLoopAccel_Cycles(Motor_T * p_motor, uint32_t cycles) { p_motor->Config.OpenLoopAccel_Cycles = cycles; }
static inline void Motor_Config_SetOpenLoopAccel_Millis(Motor_T * p_motor, uint16_t millis) { p_motor->Config.OpenLoopAccel_Cycles = _Motor_ControlCyclesOf(millis); }
#endif

/******************************************************************************/
/* Extern */
/******************************************************************************/
extern void Motor_Config_SetKv(Motor_T * p_motor, uint16_t kv);
extern void Motor_Config_SetVSpeedScalar_UFract16(Motor_T * p_motor, uint16_t scalar);
extern void Motor_Config_SetSpeedVRef_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_T * p_motor, uint16_t rpm);

extern void Motor_Config_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetDirectionCalibration(Motor_T * p_motor, Motor_Direction_T directionForward);
extern void Motor_Config_SetPolePairs(Motor_T * p_motor, uint8_t polePairs);
extern void Motor_Config_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode);

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_Config_SetIPeakRef_Adcu_Debug(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV);
#endif

extern void Motor_Config_SetSpeedLimitForward_Fract16(Motor_T * p_motor, uint16_t forward_Fract16);
extern void Motor_Config_SetSpeedLimitReverse_Fract16(Motor_T * p_motor, uint16_t reverse_Fract16);
extern void Motor_Config_SetILimitMotoring_Fract16(Motor_T * p_motor, uint16_t motoring_Fract16);
extern void Motor_Config_SetILimitGenerating_Fract16(Motor_T * p_motor, uint16_t generating_Fract16);
extern void Motor_Config_SetSpeedLimit_Fract16(Motor_T * p_motor, uint16_t forward_Fract16, uint16_t reverse_Fract16);
extern void Motor_Config_SetILimit_Fract16(Motor_T * p_motor, uint16_t motoring_Fract16, uint16_t generating_Fract16);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_Config_SetSpeedLimitForward_Rpm(Motor_T * p_motor, uint16_t forward_Rpm);
extern void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_T * p_motor, uint16_t reverse_Rpm);
extern void Motor_Config_SetSpeedLimit_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_Config_SetILimitMotoring_Amp(Motor_T * p_motor, uint16_t motoring_Amp);
extern void Motor_Config_SetILimitGenerating_Amp(Motor_T * p_motor, uint16_t generating_Amp);
extern void Motor_Config_SetILimit_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
#endif

#endif
