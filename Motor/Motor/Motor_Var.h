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
    @file   Motor_Var.h
    @author FireSourcery
    @brief  Var - Field-like Property Interface Getter/Setter via Id Key
*/
/******************************************************************************/
#include "Sensor/RotorSensor_Table.h"

#include "Utility/Var/VarAccess.h"
#include <assert.h>

/* Part of Motor */
/* Include by motor. for streamlined init */
typedef const struct Motor Motor_T;
typedef struct Motor_State Motor_State_T;

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    [Var_UserOut] Motor_User.h Implementation
    RealTime Read-Only
*/
typedef enum Motor_Var_UserOut
{
    MOTOR_VAR_SPEED,    /* Speed/IPhase/VPhase/Power -> UFract16, may over saturate */
    MOTOR_VAR_I_PHASE,
    MOTOR_VAR_V_PHASE,
    MOTOR_VAR_ELECTRICAL_ANGLE,
    MOTOR_VAR_MECHANICAL_ANGLE,
    MOTOR_VAR_STATE,
    MOTOR_VAR_SUB_STATE,
    MOTOR_VAR_FAULT_FLAGS,
    MOTOR_VAR_STATUS_FLAGS,
    MOTOR_VAR_HEAT,
    /* Derived Local */
    MOTOR_VAR_POWER,
    MOTOR_VAR_I_DC,
    MOTOR_VAR_V_SPEED_EFFECTIVE,
    MOTOR_VAR_ELECTRICAL_SPEED,
    // MOTOR_VAR_RAMP_SET_POINT,
}
Motor_Var_UserOut_T;

/*
    [Var_UserControl] Motor_User.h Implementation
    RealTime IO Access
    in/out may differ
    May be paired getter/setter or a single variable
    Read effective value, write interface value
    polymorphic handling depending on state
*/
typedef enum Motor_Var_UserControl
{
    MOTOR_VAR_USER_SET_POINT,           // RampIn(UserCmd), Generic mode select using active feedback mode
    // MOTOR_VAR_USER_SET_POINT,        // RampOut(SetPoint)

    /* StateMachine Inputs */
    MOTOR_VAR_USER_DIRECTION,           // 1:Forward, -1:Reverse, 0:Stop
    MOTOR_VAR_USER_ROTARY_DIRECTION,    // Motor_Direction_T, 1:Ccw, -1:Cw, 0:Stop, todo let Rotor Var handle this
    MOTOR_VAR_USER_FEEDBACK_MODE,
    MOTOR_VAR_USER_PHASE_OUTPUT,        /* Phase Output State: Float/Hold/VPwm. Direction must be set */

    // Limits do not invoke state machine.
    MOTOR_VAR_USER_SPEED_LIMIT,
    // MOTOR_VAR_USER_I_LIMIT,
    MOTOR_VAR_USER_I_LIMIT_MOTORING,
    MOTOR_VAR_USER_I_LIMIT_GENERATING,
    // MOTOR_VAR_RAMP_ON_OFF,           // 1:Enable, 0:Disable
}
Motor_Var_UserControl_T;

// polling, write only
// Value [-32768:32767]
// typedef enum Motor_Var_UserSetpoint
// {
//     MOTOR_VAR_USER_CMD,              // Active mode value
//     MOTOR_VAR_USER_CMD_SPEED,        // UserCmd as Speed
//     MOTOR_VAR_USER_CMD_CURRENT,
//     MOTOR_VAR_USER_CMD_VOLTAGE,
//     MOTOR_VAR_USER_CMD_ANGLE,
// }
// Motor_Var_UserSetpoint_T;

/*
    [Var_ControlState]
    Fixed struct member
    Motor_Var_UserControl out only
*/
// typedef enum Motor_Var_
// {
//     MOTOR_VAR_ROTARY_DIRECTION,          // Motor_Direction_T - CW/CCW.
//     MOTOR_VAR_EFFECTIVE_FEEDBACK_MODE,
//     MOTOR_VAR_EFFECTIVE_PHASE_STATE, // OutputState
//     MOTOR_VAR_SPEED_SET_POINT,  /* Speed Ramp always */
//     MOTOR_VAR_TORQUE_SET_POINT, /* Torque Ramp always */
//     MOTOR_VAR_EFFECTIVE_SPEED_LIMIT_FORWARD,
//     MOTOR_VAR_EFFECTIVE_SPEED_LIMIT_REVERSE,
//     MOTOR_VAR_EFFECTIVE_I_LIMIT_MOTORING,
//     MOTOR_VAR_EFFECTIVE_I_LIMIT_GENERATING,
//     MOTOR_VAR_EFFECTIVE_SET_POINT,
//     MOTOR_VAR_EFFECTIVE_SPEED_LIMIT,
//     MOTOR_VAR_EFFECTIVE_I_LIMIT,
//     MOTOR_VAR_EFFECTIVE_RAMP_ON_OFF,
// }
// Motor_Var_ControlState_T;

// typedef enum Motor_Var_Phase
// {
//     MOTOR_VAR_PHASE_OUTPUT,
//     MOTOR_VAR_PHASE_PIN_A,
//     MOTOR_VAR_PHASE_PWM_A,
// }
// Motor_Var_Phase_T;

/*
    Rotor Angle/Speed State + Feedback
    Read-Only, RealTime
*/
typedef enum Motor_Var_Rotor
{
    MOTOR_VAR_ROTOR_ELECTRICAL_ANGLE,
    MOTOR_VAR_ROTOR_ELECTRICAL_SPEED,
    MOTOR_VAR_ROTOR_MECHANICAL_ANGLE,
    MOTOR_VAR_ROTOR_SPEED_FRACT16,
    MOTOR_VAR_ROTOR_DIRECTION,
    MOTOR_VAR_ROTOR_SPEED_REQ, // effectively SpeedRamp
    // MOTOR_VAR_ROTOR_INTERGRAL, /* I or V Req */
    // MOTOR_VAR_ROTOR_ELECTRICAL_SPEED_RADS,
    // MOTOR_VAR_ROTOR_MECHANICAL_SPEED_RPM,
}
Motor_Var_Rotor_T;

/*
    FOC State
    Read-Only, RealTime
*/
typedef enum Motor_Var_Foc
{
    MOTOR_VAR_FOC_IA,
    MOTOR_VAR_FOC_IB,
    MOTOR_VAR_FOC_IC,
    MOTOR_VAR_FOC_IQ,
    MOTOR_VAR_FOC_ID,
    MOTOR_VAR_FOC_REQ_Q, // effectively TorqueRamp
    MOTOR_VAR_FOC_REQ_D,
    MOTOR_VAR_FOC_VQ,
    MOTOR_VAR_FOC_VD,
    MOTOR_VAR_FOC_VA,
    MOTOR_VAR_FOC_VB,
    MOTOR_VAR_FOC_VC,
    MOTOR_VAR_FOC_INTEGRAL_Q,
    MOTOR_VAR_FOC_INTEGRAL_D,
}
Motor_Var_Foc_T;


/*!
    [Var_StateCmd]
    Non polling, Write-Only, Get returns 0
*/
typedef enum Motor_Var_StateCmd
{
    MOTOR_VAR_CLEAR_FAULT,
    MOTOR_VAR_FORCE_DISABLE_CONTROL,    // No value arg. Force Disable control Non StateMachine checked, also handled via Call/Packet
    // MOTOR_VAR_USER_START,
    // MOTOR_VAR_USER_STOP,

    // MOTOR_VAR_CMD_OPEN_LOOP,
    MOTOR_VAR_OPEN_LOOP_ENTER,        /* Enter State. optional pass sub statecmd */
    MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT,
    MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN,
    MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN,
    MOTOR_VAR_OPEN_LOOP_JOG,
    MOTOR_VAR_OPEN_LOOP_RUN,
    // MOTOR_VAR_OPEN_LOOP_HOMING,

    // altneratively
    // MOTOR_VAR_CMD_EXIT_FAULT,
    // MOTOR_VAR_CMD_ENTER_CALIBRATION,
    // MOTOR_VAR_CMD_ENTER_OPEN_LOOP,
}
Motor_Var_StateCmd_T;


/******************************************************************************/
/*
    Config Field Id
*/
/******************************************************************************/
// typedef enum Motor_Var_ConfigCalibration
typedef enum Motor_VarConfig_Calibration
{
    MOTOR_VAR_COMMUTATION_MODE,       /* Motor_CommutationMode_T, if runtime supported */
    MOTOR_VAR_SENSOR_MODE,            /* RotorSensor_Id_T, */
    MOTOR_VAR_DIRECTION_CALIBRATION,  /* Motor_DirectionCalibration_T */
    MOTOR_VAR_POLE_PAIRS,
    MOTOR_VAR_KV,
    MOTOR_VAR_V_SPEED_SCALAR,
    MOTOR_VAR_SPEED_RATED_DEG,
    MOTOR_VAR_IA_ZERO_ADCU,
    MOTOR_VAR_IB_ZERO_ADCU,
    MOTOR_VAR_IC_ZERO_ADCU,
    // MOTOR_VAR_I_PEAK_REF_ADCU,
    // MOTOR_VAR_PHASE_POLAR_MODE,
}
Motor_VarConfig_Calibration_T;

/* Debug */
typedef enum Motor_VarConfig_CalibrationAlias
{
    MOTOR_VAR_SPEED_RATED_RPM,
    MOTOR_VAR_SPEED_V_REF_RPM,
    MOTOR_VAR_SPEED_V_MATCH_REF_RPM,
    MOTOR_VAR_SPEED_V_SVPWM_REF_RPM,

    // MOTOR_VAR_SPEED_RATED_ERPM,
    MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE,
    // MOTOR_VAR_SPEED_V_MATCH_REF_DEG_PER_CYCLE,
    MOTOR_VAR_SPEED_V_SVPWM_REF_DEG_PER_CYCLE,

    MOTOR_VAR_V_SPEED_RATED_FRACT16,
    // MOTOR_VAR_V_SPEED_REF_VOLTS,
}
Motor_VarConfig_CalibrationAlias_T;

/*
    Ramp / User Input
*/
typedef enum Motor_VarConfig_Actuation
{
    MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD,
    MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE,
    MOTOR_VAR_BASE_I_LIMIT_MOTORING,
    MOTOR_VAR_BASE_I_LIMIT_GENERATING,
    MOTOR_VAR_SPEED_RAMP_TIME,
    MOTOR_VAR_TORQUE_RAMP_TIME,
    MOTOR_VAR_OPEN_LOOP_POWER_LIMIT, // MOTOR_VAR_POWER_LIMIT_OPEN_LOOP,
    MOTOR_VAR_ALIGN_POWER,
    MOTOR_VAR_ALIGN_TIME,
    MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL,
    MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME,
    MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL,
    MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME,
}
Motor_VarConfig_Actuation_T;

/*
    PID
    Fixed 16 Set with interface functions
    Shared with Tuning
*/
typedef enum Motor_VarConfig_Pid
{
    MOTOR_VAR_PID_SPEED_KP_FIXED16,
    MOTOR_VAR_PID_SPEED_KI_FIXED16,
    MOTOR_VAR_PID_SPEED_KD_FIXED16,
    MOTOR_VAR_PID_SPEED_SAMPLE_FREQ,
    MOTOR_VAR_PID_CURRENT_KP_FIXED16,
    MOTOR_VAR_PID_CURRENT_KI_FIXED16,
    MOTOR_VAR_PID_CURRENT_KD_FIXED16,
    MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ,
}
Motor_VarConfig_Pid_T;

/*
    Calibration State Cmds
*/
typedef enum Motor_VarConfig_Cmd
{
    MOTOR_VAR_CONFIG_RUN_ADC_CALIBRATION,
    MOTOR_VAR_CONFIG_RUN_VIRTUAL_HOME,
    // MOTOR_VAR_CONFIG_ENTER_CALIBRATION,
    // MOTOR_VAR_CONFIG_RUN_SENSOR_CALIBRATION, /* Generic call for each type */
}
Motor_VarConfig_Cmd_T;


/******************************************************************************/
/*
    Read-Only Ref
    ConfigConst
*/
/******************************************************************************/
typedef enum Motor_Var_StaticRef
{
    MOTOR_VAR_REF_V_RATED,
    MOTOR_VAR_REF_I_RATED,
    MOTOR_VAR_REF_V_MAX,
    MOTOR_VAR_REF_I_MAX,
    MOTOR_VAR_REF_V_MAX_ADCU,
    MOTOR_VAR_REF_I_MAX_ADCU,
    MOTOR_VAR_REF_V_PHASE_R1,
    MOTOR_VAR_REF_V_PHASE_R2,
    MOTOR_VAR_REF_I_PHASE_R_BASE,
    MOTOR_VAR_REF_I_PHASE_R_MOSFETS,
    MOTOR_VAR_REF_I_PHASE_GAIN,
    MOTOR_VAR_REF_BOARD_V_RATED_VOLTS,
    MOTOR_VAR_REF_BOARD_I_RATED_AMPS,
}
Motor_Var_StaticRef_T;


/******************************************************************************/
/*
    Interface applying the command pattern
*/
/******************************************************************************/
/*
    Disable/Enable Set in VarAccess
*/
extern const VarAccess_VTable_T _MOTOR_VAR_ACCESS_PID_TUNING;
extern const VarAccess_VTable_T _MOTOR_VAR_ACCESS_USER_CONTROL;
extern const VarAccess_VTable_T _MOTOR_VAR_ACCESS_STATE_CMD;
/* check VarAccessControl user set state */
#define MOTOR_VAR_ACCESS_INIT_PID_TUNING(p_Motor)                   VAR_ACCESS_INIT(p_Motor, &_MOTOR_VAR_ACCESS_PID_TUNING, &((p_Motor)->VarAccessPidTunningState))
#define MOTOR_VAR_ACCESS_INIT_USER_CONTROL(p_MotorContext, p_Motor) VAR_ACCESS_INIT(p_MotorContext, &_MOTOR_VAR_ACCESS_USER_CONTROL, &((p_Motor)->VarAccessInputState))
#define MOTOR_VAR_ACCESS_INIT_STATE_CMD(p_MotorContext, p_Motor)    VAR_ACCESS_INIT(p_MotorContext, &_MOTOR_VAR_ACCESS_STATE_CMD, &((p_Motor)->VarAccessInputState))

typedef struct Motor_VarAccess
{
    const VarAccess_T ACCESS_PID_TUNING;
    const VarAccess_T ACCESS_USER_CONTROL;
    const VarAccess_T ACCESS_STATE_CMD;
}
Motor_VarAccess_T;

#define MOTOR_VAR_ACCESS_INIT(p_MotorContext, p_Motor) \
{ \
    .ACCESS_PID_TUNING      = MOTOR_VAR_ACCESS_INIT_PID_TUNING(p_Motor), \
    .ACCESS_USER_CONTROL    = MOTOR_VAR_ACCESS_INIT_USER_CONTROL(p_MotorContext, p_Motor), \
    .ACCESS_STATE_CMD       = MOTOR_VAR_ACCESS_INIT_STATE_CMD(p_MotorContext, p_Motor), \
}
/* staticc instance */
// extern VarAccess_State_T VarAccessInputState;

/******************************************************************************/
/*
    Optional
*/
/******************************************************************************/
extern const VarAccess_VTable_T MOTOR_VAR_OUT_USER;
extern const VarAccess_VTable_T MOTOR_VAR_OUT_ROTOR;
extern const VarAccess_VTable_T MOTOR_VAR_OUT_FOC;
#define MOTOR_VAR_ACCESS_INIT_OUTPUT_USER(p_Motor)              VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_USER, &((p_Motor)->VarAccessOuputState))
#define MOTOR_VAR_ACCESS_INIT_OUTPUT_ROTOR(p_Motor)             VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_ROTOR, &((p_Motor)->VarAccessOuputState))
#define MOTOR_VAR_ACCESS_INIT_OUTPUT_FOC(p_Motor)               VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_FOC, &((p_Motor)->VarAccessOuputState))

/*
    Shared Enable/Disable Set in VarAccess
    Set during StopState only
*/
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION_ALIAS;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_ACTUATION;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_PID;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_ROUTINE;
// extern const VarAccess_VTable_T MOTOR_VAR_REF;

/* check StateMachine Config State */
#define MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION(p_Motor)       VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CALIBRATION, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION_ALIAS(p_Motor) VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CALIBRATION_ALIAS, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_ACTUATION(p_Motor)         VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_ACTUATION, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_PID(p_Motor)               VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_PID, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_ROUTINE(p_Motor)           VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_ROUTINE, &((p_Motor)->VarAccessConfigState))




/******************************************************************************/
/*
   Base Id Access
*/
/******************************************************************************/
extern int _Motor_Var_UserOut_Get(const Motor_State_T * p_motor, Motor_Var_UserOut_T varId);
extern int _Motor_Var_Rotor_Get(const Motor_State_T * p_motor, Motor_Var_Rotor_T varId);
extern int _Motor_Var_Foc_Get(const Motor_State_T * p_motor, Motor_Var_Foc_T varId);

/*  */
extern void _Motor_Var_PidTunnng_Set(Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId, int varValue);
extern int _Motor_Var_PidTuning_Get(const Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId);

extern void _Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue);
extern int _Motor_Var_UserControl_Get(const Motor_T * p_motor, Motor_Var_UserControl_T varId);

extern void _Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue);

/*
*/
extern int _Motor_VarConfig_Calibration_Get(const Motor_State_T * p_motor, Motor_VarConfig_Calibration_T varId);
extern int _Motor_VarConfig_CalibrationAlias_Get(const Motor_State_T * p_motor, Motor_VarConfig_CalibrationAlias_T varId);
extern int _Motor_VarConfig_Actuation_Get(const Motor_State_T * p_motor, Motor_VarConfig_Actuation_T varId);
extern int _Motor_VarConfig_Pid_Get(const Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId);

extern void _Motor_VarConfig_Calibration_Set(Motor_State_T * p_motor, Motor_VarConfig_Calibration_T varId, int varValue);
extern void _Motor_VarConfig_Actuation_Set(Motor_State_T * p_motor, Motor_VarConfig_Actuation_T varId, int varValue);
extern void _Motor_VarConfig_Pid_Set(Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId, int varValue);

extern void _Motor_VarConfig_Cmd_Call(const Motor_T * p_motor, Motor_VarConfig_Cmd_T varId, int varValue);


/* With Access Control */
extern void Motor_Var_DisableInput(Motor_State_T * p_motor);
extern void Motor_Var_EnableInput(Motor_State_T * p_motor);

extern void Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue);
extern void Motor_Var_PidTuning_Set(const Motor_T * p_motor, Motor_VarConfig_Pid_T varId, int varValue);
extern void Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue);


/*  */
extern int Motor_Var_StaticRef_Get(Motor_Var_StaticRef_T varId);

/******************************************************************************/
/*
    Module handled type
*/
/******************************************************************************/
typedef enum Motor_VarType
{
    MOTOR_VAR_TYPE_USER_OUT, // MOTOR_VAR_TYPE_STATE_USER,
    MOTOR_VAR_TYPE_USER_CONTROL, /* Polling IO. Setpoint/StateMachine. */
    // MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
    MOTOR_VAR_TYPE_ROTOR_OUT, /* Speed Angle */
    MOTOR_VAR_TYPE_FOC_OUT,
    MOTOR_VAR_TYPE_HEAT_MONITOR_OUT, /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_PID_TUNING_IO, /* Non polling. PID tunning with non-Config state access permissions */
    MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
    // MOTOR_VAR_TYPE_OPEN_LOOP_CMD,
    // _MOTOR_VAR_TYPE_END = 15U,
}
Motor_VarType_T;

typedef enum Motor_VarType_Config
{
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION_ALIAS,
    MOTOR_VAR_TYPE_CONFIG_ACTUATION,
    MOTOR_VAR_TYPE_CONFIG_PID,

    MOTOR_VAR_TYPE_CONFIG_HEAT_MONITOR, /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_CONFIG_THERMISTOR,

    MOTOR_VAR_TYPE_CONFIG_CMD,          /* Calibration Cmds */
    MOTOR_VAR_TYPE_CONFIG_SENSOR_CMD,   /* Handle by Motor_Sensor.h/c. Invoke Sub StateMachine. Using Motor RotorSensor_Id_T as [varId] */
    MOTOR_VAR_TYPE_CONFIG_BOARD_REF,    /* Not instanced, altneratively */

    // MOTOR_VAR_TYPE_CONFIG_SENSOR_ENCODER,
    // MOTOR_VAR_TYPE_CONFIG_SENSOR_HALL,
}
Motor_VarType_Config_T;

extern int Motor_VarType_Get(const Motor_T * p_motor, Motor_VarType_T typeId, int varId);
extern void Motor_VarType_Set(const Motor_T * p_motor, Motor_VarType_T typeId, int varId, int varValue);

extern int Motor_VarType_Config_Get(const Motor_T * p_motor, Motor_VarType_Config_T typeId, int varId);
extern void Motor_VarType_Config_Set(const Motor_T * p_motor, Motor_VarType_Config_T typeId, int varId, int varValue);

/******************************************************************************/
/*
    Wrap on RotorSensor_Id_T RotorSensor_Table
    RotorSensor_Id_T implementation in RotorSensor.h/c
*/
/******************************************************************************/
extern int Motor_VarType_SensorState_Get(const Motor_T * p_motor, RotorSensor_Id_T typeId, int varId);

extern int Motor_VarType_SensorConfig_Get(const Motor_T * p_motor, RotorSensor_Id_T typeId, int varId);
extern void Motor_VarType_SensorConfig_Set(const Motor_T * p_motor, RotorSensor_Id_T typeId, int varId, int varValue);
