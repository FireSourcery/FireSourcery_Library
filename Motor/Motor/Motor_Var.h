
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

#include <assert.h>


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    RealTime Read-Only
    Get/Output
*/
/*
    Speed/IPhase/VPhase/Power
        Units0 => UFract16, may over saturate
*/
// Motor_Output_Primary_T
// Motor_Output_FeedbackValue_T // move state
// Motor_VarOutput_T
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
    MOT_VAR_MOTOR_EFFECTIVE_SET_POINT,
    MOT_VAR_MOTOR_EFFECTIVE_SPEED_LIMIT,
    MOT_VAR_MOTOR_EFFECTIVE_I_LIMIT,
    MOT_VAR_MOTOR_V_SPEED_DEBUG,
    MOT_VAR_MOTOR_V_SPEED_EFFECTIVE,
}
MotVarId_Monitor_Motor_T;

// Motor_Output_Foc_T
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
    Cmd -> Write-Only, Get returns 0
        Set/Input
    Value [-32768:32767]
*/
// Motor_Input_T
typedef enum MotVarId_Cmd_Motor
{
    // MOT_VAR_MOTOR_USER_CMD,      // Active mode value
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

/*
    IO/Access
    May be paired getter/setter or a single variable
    Analogously a control loop in/out may differ
*/
// Motor_IO_T
typedef enum MotVarId_Control_Motor
{
    MOT_VAR_MOTOR_DIRECTION,            // Motor_Direction_T - CW/CCW. Read state value, write interface value,
    /* IO Vars, Read effective value, write interface value */
    MOT_VAR_MOTOR_USER_SET_POINT,       // RampIn(UserCmd)/RampOut(SetPoint), Generic mode select
    MOT_VAR_MOTOR_USER_FEEDBACK_MODE,
    MOT_VAR_MOTOR_USER_SPEED_LIMIT,
    MOT_VAR_MOTOR_USER_I_LIMIT,
    MOT_VAR_MOTOR_SET_RAMP_ON_OFF,      // 1:Enable, 0:Disable
}
MotVarId_Control_Motor_T;


/******************************************************************************/
/*
    Config Field Id
*/
/******************************************************************************/

// Motor_Config_Primary_T
// Motor_Config_Calibration_T
// Motor_VarConfig_Calibration_T
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
/*
   Extern
*/
/******************************************************************************/
int32_t Motor_VarOutput_Get(const Motor_T * p_motor, MotVarId_Monitor_Motor_T varId);
int32_t Motor_VarOutput_Foc_Get(const Motor_T * p_motor, MotVarId_Monitor_MotorFoc_T varId);


#endif


