
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
    @file   Motor_Var.h
    @author FireSourcery
    @version V0

    @brief  Var - Field-like Property Interface Getter/Setter via Id Key
*/
/******************************************************************************/
#ifndef MOTOR_VAR_H
#define MOTOR_VAR_H

#include "Motor_User.h"
#include "Motor_Config.h"
#include "Motor_FOC.h"

#include <assert.h>


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Get/Output
    RealTime Read-Only
    Speed/IPhase/VPhase/Power -> UFract16, may over saturate
*/
// Motor_VarOutput_Primary_T
// Motor_VarOutput_Monitor_T
typedef enum Motor_VarOuput
{
    MOTOR_VAR_SPEED,
    MOTOR_VAR_I_PHASE,
    MOTOR_VAR_V_PHASE,
    MOTOR_VAR_ELECTRICAL_ANGLE,
    MOTOR_VAR_MECHANICAL_ANGLE,
    MOTOR_VAR_POWER,
    MOTOR_VAR_HEAT,
    MOTOR_VAR_STATE,
    MOTOR_VAR_STATUS_FLAGS,
    MOTOR_VAR_FAULT_FLAGS,
    MOTOR_VAR_EFFECTIVE_FEEDBACK_MODE,
    MOTOR_VAR_EFFECTIVE_SET_POINT,
    MOTOR_VAR_EFFECTIVE_SPEED_LIMIT,
    MOTOR_VAR_EFFECTIVE_I_LIMIT,
    MOTOR_VAR_V_SPEED_DEBUG,
    MOTOR_VAR_V_SPEED_EFFECTIVE,
}
Motor_VarOuput_T;

typedef enum Motor_VarOuput_Foc
{
    MOTOR_VAR_FOC_IA,
    MOTOR_VAR_FOC_IB,
    MOTOR_VAR_FOC_IC,
    MOTOR_VAR_FOC_IQ,
    MOTOR_VAR_FOC_ID,
    MOTOR_VAR_FOC_VQ,
    MOTOR_VAR_FOC_VD,
    MOTOR_VAR_FOC_REQ_Q,
    MOTOR_VAR_FOC_REQ_D,
    MOTOR_VAR_FOC_VA,
    MOTOR_VAR_FOC_VB,
    MOTOR_VAR_FOC_VC,
    MOTOR_VAR_FOC_INTEGRAL_Q,
    MOTOR_VAR_FOC_INTEGRAL_D,
}
Motor_VarOuput_Foc_T;

typedef enum Motor_VarOutput_PositionSensor
{
    MOTOR_VAR_ENCODER_FREQ,
    MOTOR_VAR_ENCODER_RPM,
}
Motor_VarOutput_PositionSensor_T;

/*
    Set/Input
    Write-Only, Get returns 0
    Value [-32768:32767]
*/
typedef enum Motor_VarInput
{
    MOTOR_VAR_CLEAR_FAULT,
    // MOTOR_VAR_USER_CMD,      // Active mode value
    MOTOR_VAR_CMD_SPEED,        // UserCmd as Speed
    MOTOR_VAR_CMD_CURRENT,
    MOTOR_VAR_CMD_VOLTAGE,
    MOTOR_VAR_CMD_ANGLE,
    MOTOR_VAR_CMD_OPEN_LOOP,
    MOTOR_VAR_FORCE_DISABLE_CONTROL,    // No value arg. Force Disable control Non StateMachine checked, also handled via Call

    MOTOR_VAR_PHASE_ALIGN,
    MOTOR_VAR_FEED_FORWARD_ANGLE,
}
Motor_VarInput_T;

/*
    IO/Access
    May be paired getter/setter or a single variable
    in/out may differ
*/
typedef enum Motor_VarIO
{
    MOTOR_VAR_DIRECTION,            // Motor_Direction_T - CW/CCW. Read state value, write interface value,
    /* IO Vars, Read effective value, write interface value */
    MOTOR_VAR_USER_SET_POINT,       // RampIn(UserCmd)/RampOut(SetPoint), Generic mode select
    MOTOR_VAR_USER_FEEDBACK_MODE,
    MOTOR_VAR_USER_PHASE_STATE,
    // Limits do not invoke state machine
    MOTOR_VAR_USER_SPEED_LIMIT,
    MOTOR_VAR_USER_I_LIMIT,
    MOTOR_VAR_RAMP_ON_OFF,      // 1:Enable, 0:Disable
}
Motor_VarIO_T;


/******************************************************************************/
/*
    Config Field Id
*/
/******************************************************************************/
// Motor_Config_Primary_T
typedef enum Motor_VarConfig_Calibration
{
    MOTOR_VAR_COMMUTATION_MODE,       /* Motor_CommutationMode_T, if runtime supported */
    MOTOR_VAR_SENSOR_MODE,            /* Motor_SensorMode_T, */
    MOTOR_VAR_DIRECTION_CALIBRATION,  /* Motor_DirectionCalibration_T */
    MOTOR_VAR_POLE_PAIRS,
    MOTOR_VAR_KV,
    MOTOR_VAR_V_SPEED_SCALAR,
    MOTOR_VAR_SPEED_V_REF_RPM,
    MOTOR_VAR_SPEED_V_MATCH_REF_RPM,
    MOTOR_VAR_IA_ZERO_REF_ADCU,
    MOTOR_VAR_IB_ZERO_REF_ADCU,
    MOTOR_VAR_IC_ZERO_REF_ADCU,
    MOTOR_VAR_I_PEAK_REF_ADCU,
}
Motor_VarConfig_Calibration_T;

typedef enum Motor_VarConfig_Actuation
{
    MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD,
    MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE,
    MOTOR_VAR_BASE_I_LIMIT_MOTORING,
    MOTOR_VAR_BASE_I_LIMIT_GENERATING,
    MOTOR_VAR_RAMP_ACCEL_TIME,
    MOTOR_VAR_ALIGN_MODE,
    MOTOR_VAR_ALIGN_POWER,
    MOTOR_VAR_ALIGN_TIME,
    MOTOR_VAR_OPEN_LOOP_POWER,
    MOTOR_VAR_OPEN_LOOP_SPEED,
    MOTOR_VAR_OPEN_LOOP_ACCEL_TIME,
    MOTOR_VAR_PHASE_PWM_MODE,
}
Motor_VarConfig_Actuation_T;

// LocalUnits
// uint16_t SurfaceDiameter;
// uint16_t GearRatio_Factor;
// uint16_t GearRatio_Divisor;

typedef enum Motor_VarConfig_Hall
{
    MOTOR_VAR_HALL_SENSOR_TABLE_1,
    MOTOR_VAR_HALL_SENSOR_TABLE_2,
    MOTOR_VAR_HALL_SENSOR_TABLE_3,
    MOTOR_VAR_HALL_SENSOR_TABLE_4,
    MOTOR_VAR_HALL_SENSOR_TABLE_5,
    MOTOR_VAR_HALL_SENSOR_TABLE_6,
}
Motor_VarConfig_Hall_T;

typedef enum Motor_VarConfig_Encoder
{
    MOTOR_VAR_ENCODER_COUNTS_PER_REVOLUTION,
    MOTOR_VAR_ENCODER_EXTENDED_TIMER_DELTA_T_STOP,
    MOTOR_VAR_ENCODER_INTERPOLATE_ANGLE_SCALAR,
    MOTOR_VAR_ENCODER_IS_QUADRATURE_CAPTURE_ENABLED,
    MOTOR_VAR_ENCODER_IS_A_LEAD_B_POSITIVE,
}
Motor_VarConfig_Encoder_T;

typedef enum Motor_VarConfig_SinCos
{
    MOTOR_VAR_SIN_COS_ZERO_ADCU,
    MOTOR_VAR_SIN_COS_MAX_ADCU,
    MOTOR_VAR_SIN_COS_MAX_MILLIV,
    MOTOR_VAR_SIN_COS_ANGLE_OFFSET,
    MOTOR_VAR_SIN_COS_IS_B_POSITIVE,
    MOTOR_VAR_SIN_COS_ELECTRICAL_ROTATIONS_PER_CYCLE,
}
Motor_VarConfig_SinCos_T;

/*
    PID
    Fixed 16 Set with interface functions
*/
typedef enum Motor_VarConfig_Pid
{
    MOTOR_VAR_PID_SPEED_KP_FIXED16,
    MOTOR_VAR_PID_SPEED_KI_FIXED16,
    MOTOR_VAR_PID_SPEED_KD_FIXED16,
    MOTOR_VAR_PID_SPEED_SAMPLE_FREQ,
    // MOTOR_VAR_PID_CURRENT_KP_FIXED16,
    // MOTOR_VAR_PID_CURRENT_KI_FIXED16,
    // MOTOR_VAR_PID_CURRENT_KD_FIXED16,
    // MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ,
    MOTOR_VAR_PID_FOC_IQ_KP_FIXED16,
    MOTOR_VAR_PID_FOC_IQ_KI_FIXED16,
    MOTOR_VAR_PID_FOC_IQ_KD_FIXED16,
    MOTOR_VAR_PID_FOC_IQ_SAMPLE_FREQ,
    // MOTOR_VAR_PID_FOC_ID_KP_FIXED16,
    // MOTOR_VAR_PID_FOC_ID_KI_FIXED16,
    // MOTOR_VAR_PID_FOC_ID_KD_FIXED16,
    // MOTOR_VAR_PID_FOC_ID_SAMPLE_FREQ,
}
Motor_VarConfig_Pid_T;

/******************************************************************************/
/*
   Extern
*/
/******************************************************************************/
extern int32_t Motor_VarOutput_Get(const Motor_T * p_motor, Motor_VarOuput_T varId);
extern int32_t Motor_VarOutput_Foc_Get(const Motor_T * p_motor, Motor_VarOuput_Foc_T varId);
extern int32_t Motor_VarOutput_PositionSensor_Get(const Motor_T * p_motor, Motor_VarOutput_PositionSensor_T varId);
extern int32_t Motor_VarIO_Get(const Motor_T * p_motor, Motor_VarIO_T varId);

extern void Motor_VarInput_Set(Motor_T * p_motor, Motor_VarInput_T varId, int32_t varValue);
extern void Motor_VarIO_Set(Motor_T * p_motor, Motor_VarIO_T varId, int32_t varValue);

extern int32_t Motor_VarConfig_Calibration_Get(const Motor_T * p_motor, Motor_VarConfig_Calibration_T varId);
extern int32_t Motor_VarConfig_Actuation_Get(const Motor_T * p_motor, Motor_VarConfig_Actuation_T varId);
extern int32_t Motor_VarConfig_Hall_Get(const Motor_T * p_motor, Motor_VarConfig_Hall_T varId);
extern int32_t Motor_VarConfig_Encoder_Get(const Motor_T * p_motor, Motor_VarConfig_Encoder_T varId);
extern int32_t Motor_VarConfig_SinCos_Get(const Motor_T * p_motor, Motor_VarConfig_SinCos_T varId);
extern int32_t Motor_VarConfig_Pid_Get(const Motor_T * p_motor, Motor_VarConfig_Pid_T varId);

extern void Motor_VarConfig_Calibration_Set(Motor_T * p_motor, Motor_VarConfig_Calibration_T varId, int32_t varValue);
extern void Motor_VarConfig_Actuation_Set(Motor_T * p_motor, Motor_VarConfig_Actuation_T varId, int32_t varValue);
extern void Motor_VarConfig_Hall_Set(Motor_T * p_motor, Motor_VarConfig_Hall_T varId, int32_t varValue);
extern void Motor_VarConfig_Encoder_Set(Motor_T * p_motor, Motor_VarConfig_Encoder_T varId, int32_t varValue);
extern void Motor_VarConfig_SinCos_Set(Motor_T * p_motor, Motor_VarConfig_SinCos_T varId, int32_t varValue);
extern void Motor_VarConfig_Pid_Set(Motor_T * p_motor, Motor_VarConfig_Pid_T varId, int32_t varValue);

#endif


