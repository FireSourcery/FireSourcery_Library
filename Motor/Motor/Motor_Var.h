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
#include "Motor_StateMachine.h"
#include "Sensor/RotorSensor_Table.h"
// #include "Type/Var/VarAccess.h"

#include <assert.h>

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    [Var_UserOut] Motor_User.h Implementation
    RealTime Read-Only
    Speed/IPhase/VPhase/Power -> UFract16, may over saturate
*/
typedef enum Motor_Var_UserOut
{
    MOTOR_VAR_SPEED,    /* User Direction */
    MOTOR_VAR_I_PHASE,
    MOTOR_VAR_V_PHASE,
    MOTOR_VAR_STATE,
    MOTOR_VAR_SUB_STATE,
    MOTOR_VAR_FAULT_FLAGS,
    MOTOR_VAR_STATUS_FLAGS,
    MOTOR_VAR_HEAT, /* included in HEAT_MONITOR_OUT */
    MOTOR_VAR_V_SPEED_EFFECTIVE,
    /* Derived Local */
    MOTOR_VAR_POWER,
    MOTOR_VAR_I_DC,
    // MOTOR_VAR_V_BUS,
    // MOTOR_VAR_SPEED_REQ,  /* with Sign */
    // MOTOR_VAR_TORQUE_REQ, /*   */
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
    /* State Control */
    MOTOR_VAR_USER_DIRECTION,           // 1:Forward, -1:Reverse, 0:Stop
    MOTOR_VAR_USER_ROTARY_DIRECTION,    // Motor_Direction_T, 1:Ccw, -1:Cw, 0:Stop, //alt let Rotor Var handle this
    MOTOR_VAR_USER_FEEDBACK_MODE,
    MOTOR_VAR_USER_PHASE_OUTPUT,        /* Phase Output State: Float/Hold/VPwm. Direction must be set */
    /* Limits do not invoke state machine. */
    MOTOR_VAR_USER_SPEED_LIMIT,
    // MOTOR_VAR_USER_I_LIMIT,
    MOTOR_VAR_USER_I_LIMIT_MOTORING,
    MOTOR_VAR_USER_I_LIMIT_GENERATING,
    // MOTOR_VAR_RAMP_ON_OFF,           // 1:Enable, 0:Disable
}
Motor_Var_UserControl_T;

/*
    [UserSetpoint]
    polling, write only
    Value [-32768:32767]
    with User Direction
    Optionally Abs version for Ccw/Cw
*/
typedef enum Motor_Var_UserSetpoint
{
    MOTOR_VAR_USER_SETPOINT,              // RampIn, Active/Generic mode select using active feedback mode
    MOTOR_VAR_USER_SETPOINT_SPEED,        // UserCmd as Speed
    MOTOR_VAR_USER_SETPOINT_CURRENT,
    MOTOR_VAR_USER_SETPOINT_VOLTAGE,
    MOTOR_VAR_USER_SETPOINT_ANGLE,
}
Motor_Var_UserSetpoint_T;

/*
    Rotor Angle/Speed State + Feedback
    Read-Only, RealTime
*/
typedef enum Motor_Var_Rotor
{
    MOTOR_VAR_ROTOR_ELECTRICAL_ANGLE,
    MOTOR_VAR_ROTOR_ELECTRICAL_SPEED, /* Absolute Ccw, CW */
    MOTOR_VAR_ROTOR_SPEED_FEEDBACK,
    MOTOR_VAR_ROTOR_MECHANICAL_ANGLE, /* if supported */
    MOTOR_VAR_ROTOR_DIRECTION, //  1:Ccw, -1:Cw, 0:Stop
    MOTOR_VAR_ROTOR_SPEED_REQ, // effectively SpeedRamp
    // MOTOR_VAR_ROTOR_SPEED_INTEGRAL, /* I or V Req */
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
    MOTOR_VAR_FOC_ID,
    MOTOR_VAR_FOC_IQ,
    MOTOR_VAR_FOC_VD,
    MOTOR_VAR_FOC_VQ,
    MOTOR_VAR_FOC_VA,
    MOTOR_VAR_FOC_VB,
    MOTOR_VAR_FOC_VC,
    MOTOR_VAR_FOC_REQ_D,
    MOTOR_VAR_FOC_REQ_Q, /* Iq or Vq Req */
    // MOTOR_VAR_FOC_ID_REQ, /* return I or 0 */
    // MOTOR_VAR_FOC_IQ_REQ,
    MOTOR_VAR_FOC_INTEGRAL_D,
    MOTOR_VAR_FOC_INTEGRAL_Q,
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

    MOTOR_VAR_OPEN_LOOP_ENTER,        /* Enter State. optional pass sub statecmd */
    MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT,
    MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN,
    MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN,
    MOTOR_VAR_OPEN_LOOP_JOG,
    MOTOR_VAR_OPEN_LOOP_RUN,
    // MOTOR_VAR_OPEN_LOOP_HOMING,

    // alternatively, main enter/exit only
    // MOTOR_VAR_CMD_EXIT_FAULT,
    // MOTOR_VAR_CMD_ENTER_CALIBRATION,
    // MOTOR_VAR_CMD_ENTER_OPEN_LOOP,

    // MOTOR_VAR_USER_START, // handle by direction for now
    // MOTOR_VAR_USER_STOP,
}
Motor_Var_StateCmd_T;

/******************************************************************************/
/*
    Config Field Id
    Preferably in dependency order. Simplify propagate write.
*/
/******************************************************************************/
typedef enum Motor_Var_ConfigCalibration
{
    MOTOR_VAR_COMMUTATION_MODE,       /* Motor_CommutationMode_T, if runtime supported */
    MOTOR_VAR_SENSOR_MODE,            /* RotorSensor_Id_T */
    MOTOR_VAR_DIRECTION_CALIBRATION,  /* Motor_DirectionCalibration_T */
    MOTOR_VAR_POLE_PAIRS,
    MOTOR_VAR_KV,
    MOTOR_VAR_SPEED_RATED,
    MOTOR_VAR_V_SPEED_SCALAR,
    MOTOR_VAR_IA_ZERO_ADCU,
    MOTOR_VAR_IB_ZERO_ADCU,
    MOTOR_VAR_IC_ZERO_ADCU,
    // MOTOR_VAR_I_PEAK_REF_ADCU,
    // MOTOR_VAR_PHASE_POLAR_MODE,
}
Motor_Var_ConfigCalibration_T;

/* Debug */
typedef enum Motor_Var_ConfigCalibrationAlias
{
    MOTOR_VAR_SPEED_RATED_RPM,
    MOTOR_VAR_SPEED_V_REF_RPM,
    MOTOR_VAR_SPEED_V_SVPWM_REF_RPM,
    MOTOR_VAR_SPEED_V_MATCH_REF_RPM,
    MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE,
    MOTOR_VAR_SPEED_V_SVPWM_REF_DEG_PER_CYCLE,
    MOTOR_VAR_V_SPEED_RATED_FRACT16,
    // MOTOR_VAR_V_SPEED_REF_VOLTS,
}
Motor_Var_ConfigCalibrationAlias_T;

/*
    Ramp / User Input
*/
typedef enum Motor_Var_ConfigActuation
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
Motor_Var_ConfigActuation_T;

/*
    PID
    Fixed 16 Set with interface functions
    Shared with Tuning
*/
typedef enum Motor_Var_ConfigPid
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
Motor_Var_ConfigPid_T;

/*
    Calibration State Cmds
*/
typedef enum Motor_Var_ConfigCmd
{
    MOTOR_VAR_CONFIG_CMD_ADC_CALIBRATION,
    MOTOR_VAR_CONFIG_CMD_VIRTUAL_HOME,
    MOTOR_VAR_CONFIG_ENTER_CALIBRATION, // enforce config in calibration state, rather than stop state
    // MOTOR_VAR_CONFIG_CMD_SENSOR_CALIBRATION, /* Generic call for active type */
}
Motor_Var_ConfigCmd_T;


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

typedef RotorSensor_Id_T Motor_Var_RotorSensorCmd_T; /* cmd as value, without additional parameter */

/*
    Submodule wrap
*/
// static inline int _Motor_Var_ConfigHeatMonitor_Get(const Motor_State_T * p_motor, Monitor_ConfigId_T varId) { return HeatMonitor_ConfigId_Get(&p_motor->HeatMonitorState, varId); }
// static inline void _Motor_Var_ConfigHeatMonitor_Set(Motor_State_T * p_motor, Monitor_ConfigId_T varId, int varValue) { HeatMonitor_ConfigId_Set(&p_motor->HeatMonitorState, varId, varValue); }
// static inline void _Motor_Var_SensorCmd_Call(const Motor_T * p_motor, Motor_Var_RotorSensorCmd_T varId, int varValue); // Motor_Sensor_CalibrationCmd_Call

/******************************************************************************/
/*
   Base Id Access
*/
/******************************************************************************/
// extern int _Motor_Var_UserOut_Get(const Motor_State_T * p_motor, Motor_Var_UserOut_T varId);
// extern int _Motor_Var_Rotor_Get(const Motor_State_T * p_motor, Motor_Var_Rotor_T varId);
// extern int _Motor_Var_Foc_Get(const Motor_State_T * p_motor, Motor_Var_Foc_T varId);

// extern void _Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue);
// extern int _Motor_Var_UserControl_Get(const Motor_T * p_motor, Motor_Var_UserControl_T varId);

// extern void _Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue);

// /* Config */
// extern int _Motor_Var_ConfigCalibration_Get(const Motor_State_T * p_motor, Motor_Var_ConfigCalibration_T varId);
// extern int _Motor_Var_ConfigCalibrationAlias_Get(const Motor_State_T * p_motor, Motor_Var_ConfigCalibrationAlias_T varId);
// extern int _Motor_Var_ConfigActuation_Get(const Motor_State_T * p_motor, Motor_Var_ConfigActuation_T varId);
// extern int _Motor_Var_ConfigPid_Get(const Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId);

// extern void _Motor_Var_ConfigCalibration_Set(Motor_State_T * p_motor, Motor_Var_ConfigCalibration_T varId, int varValue);
// extern void _Motor_Var_ConfigActuation_Set(Motor_State_T * p_motor, Motor_Var_ConfigActuation_T varId, int varValue);
// extern void _Motor_Var_ConfigPid_Set(Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);

// extern void _Motor_Var_ConfigCmd_Call(const Motor_T * p_motor, Motor_Var_ConfigCmd_T varId, int varValue);

/*  */
// extern void Motor_Var_PidTuning_Set(Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);
// extern int Motor_Var_PidTuning_Get(Motor_T * p_motor, Motor_Var_ConfigPid_T varId);

// extern int Motor_Var_UserOut_Get(Motor_T * p_motor, Motor_Var_UserOut_T varId);
// extern int Motor_Var_Rotor_Get(Motor_T * p_motor, Motor_Var_Rotor_T varId);
// extern int Motor_Var_Foc_Get(Motor_T * p_motor, Motor_Var_Foc_T varId);

// extern void Motor_Var_UserControl_Set(Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue);
// extern int Motor_Var_UserControl_Get(Motor_T * p_motor, Motor_Var_UserControl_T varId);

// extern void Motor_Var_StateCmd_Set(Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue);

// /* Config */
// extern int Motor_Var_ConfigCalibration_Get(Motor_T * p_motor, Motor_Var_ConfigCalibration_T varId);
// extern int Motor_Var_ConfigCalibrationAlias_Get(Motor_T * p_motor, Motor_Var_ConfigCalibrationAlias_T varId);
// extern int Motor_Var_ConfigActuation_Get(Motor_T * p_motor, Motor_Var_ConfigActuation_T varId);
// extern int Motor_Var_ConfigPid_Get(Motor_T * p_motor, Motor_Var_ConfigPid_T varId);

// extern void Motor_Var_ConfigCalibration_Set(Motor_T * p_motor, Motor_Var_ConfigCalibration_T varId, int varValue);
// extern void Motor_Var_ConfigActuation_Set(Motor_T * p_motor, Motor_Var_ConfigActuation_T varId, int varValue);
// extern void Motor_Var_ConfigPid_Set(Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);

// extern void Motor_Var_ConfigCmd_Call(Motor_T * p_motor, Motor_Var_ConfigCmd_T varId, int varValue);

// /*  */
// extern void Motor_Var_PidTuning_Set(Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);
// extern int Motor_Var_PidTuning_Get(Motor_T * p_motor, Motor_Var_ConfigPid_T varId);

/*  */
extern int Motor_Var_StaticRef_Get(Motor_Var_StaticRef_T varId);

/* Caller handle access control */
/* runtime inputs require on/off */
// extern void Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue);
// extern void Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue);
// extern void Motor_Var_PidTuning_Set(const Motor_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);


/******************************************************************************/
/*
    Module handled type
*/
/******************************************************************************/
typedef enum Motor_VarType_Control
{
    MOTOR_VAR_TYPE_USER_OUT, // MOTOR_VAR_TYPE_STATE_USER,
    MOTOR_VAR_TYPE_USER_CONTROL, /* Polling IO. Setpoint/StateMachine. */
    MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
    MOTOR_VAR_TYPE_ROTOR_OUT, /* Speed Angle */
    MOTOR_VAR_TYPE_FOC_OUT,
    MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
}
Motor_VarType_Control_T;

typedef enum Motor_VarType_Config
{
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION_ALIAS,
    MOTOR_VAR_TYPE_CONFIG_ACTUATION,
    MOTOR_VAR_TYPE_CONFIG_PID,
    MOTOR_VAR_TYPE_CONFIG_CMD,          /* Config State Cmds */
    MOTOR_VAR_TYPE_CONFIG_SENSOR_CMD,   /* Handle by Motor_Sensor.h/c. Calibration Sub StateMachine. Using Motor RotorSensor_Id_T as [varId] */
}
Motor_VarType_Config_T;

typedef enum Motor_VarType_SubModule
{
    MOTOR_VAR_TYPE_STATIC_BOARD_REF,    /* Not instanced */
    MOTOR_VAR_TYPE_HEAT_MONITOR_OUT,    /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG, /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_THERMISTOR_CONFIG,
    MOTOR_VAR_TYPE_PID_TUNING_IO,       /* Non polling. PID tunning with non-Config state access permissions */
}
Motor_VarType_SubModule_T;

/* This way it takes only one field to associate properties.
allows types to expand beyond 16 ids without reserving handlers */
typedef enum Motor_VarType_RotorSensor
{
    MOTOR_VAR_TYPE_HALL_STATE,
    MOTOR_VAR_TYPE_HALL_CONFIG,
    MOTOR_VAR_TYPE_ENCODER_STATE,
    MOTOR_VAR_TYPE_ENCODER_CONFIG,
}
Motor_VarType_RotorSensor_T;

extern int Motor_VarType_Control_Get(const Motor_T * p_motor, Motor_VarType_Control_T typeId, int varId);
extern void Motor_VarType_Control_Set(const Motor_T * p_motor, Motor_VarType_Control_T typeId, int varId, int varValue);

extern int Motor_VarType_Config_Get(const Motor_T * p_motor, Motor_VarType_Config_T typeId, int varId);
extern void Motor_VarType_Config_Set(const Motor_T * p_motor, Motor_VarType_Config_T typeId, int varId, int varValue);

extern int Motor_VarType_SubModule_Get(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId);
extern void Motor_VarType_SubModule_Set(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId, int varValue);

/*
    Wrap on RotorSensor_Table
*/
extern int Motor_VarType_Sensor_Get(const Motor_T * p_motor, Motor_VarType_RotorSensor_T typeId, int varId);
extern void Motor_VarType_Sensor_Set(const Motor_T * p_motor, Motor_VarType_RotorSensor_T typeId, int varId, int varValue);

extern int Motor_VarType_Sensor_Get(const Motor_T * p_motor, Motor_VarType_RotorSensor_T typeId, int varId);
extern void Motor_VarType_Sensor_Set(const Motor_T * p_motor, Motor_VarType_RotorSensor_T typeId, int varId, int varValue);



/******************************************************************************/
/*
    Interface applying the command pattern
*/
/******************************************************************************/

/* Part of Motor */
/* Include by motor. for streamlined init */
// typedef const struct Motor Motor_T;
// typedef struct Motor_State Motor_State_T;

/* With Access Control */
// extern void Motor_Var_DisableInput(Motor_State_T * p_motor);
// extern void Motor_Var_EnableInput(Motor_State_T * p_motor);
// /*
//     Disable/Enable Set in VarAccess
// */
// extern const VarAccess_VTable_T MOTOR_VAR_ACCESS_PID_TUNING;
// extern const VarAccess_VTable_T MOTOR_VAR_ACCESS_USER_CONTROL;
// extern const VarAccess_VTable_T MOTOR_VAR_ACCESS_STATE_CMD;
// /* check VarAccessControl user set state */
// #define MOTOR_VAR_ACCESS_INIT_PID_TUNING(p_Motor)                   VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_ACCESS_PID_TUNING, &((p_Motor)->VarAccessPidTunningState))
// #define MOTOR_VAR_ACCESS_INIT_USER_CONTROL(p_MotorContext, p_Motor) VAR_ACCESS_INIT(p_MotorContext, &MOTOR_VAR_ACCESS_USER_CONTROL, &((p_Motor)->VarAccessInputState))
// #define MOTOR_VAR_ACCESS_INIT_STATE_CMD(p_MotorContext, p_Motor)    VAR_ACCESS_INIT(p_MotorContext, &MOTOR_VAR_ACCESS_STATE_CMD, &((p_Motor)->VarAccessInputState))

// typedef struct Motor_VarAccess
// {
//     const VarAccess_T ACCESS_PID_TUNING;
//     const VarAccess_T ACCESS_USER_CONTROL;
//     const VarAccess_T ACCESS_STATE_CMD;
// }
// Motor_VarAccess_T;

// #define MOTOR_VAR_ACCESS_INIT(p_MotorContext, p_Motor) \
// { \
//     .ACCESS_PID_TUNING      = MOTOR_VAR_ACCESS_INIT_PID_TUNING(p_Motor), \
//     .ACCESS_USER_CONTROL    = MOTOR_VAR_ACCESS_INIT_USER_CONTROL(p_MotorContext, p_Motor), \
//     .ACCESS_STATE_CMD       = MOTOR_VAR_ACCESS_INIT_STATE_CMD(p_MotorContext, p_Motor), \
// }
// /* static  instance */
// // extern VarAccess_State_T VarAccessInputState;

// /******************************************************************************/
// /*
//     Optional
// */
// /******************************************************************************/
// extern const VarAccess_VTable_T MOTOR_VAR_OUT_USER;
// extern const VarAccess_VTable_T MOTOR_VAR_OUT_ROTOR;
// extern const VarAccess_VTable_T MOTOR_VAR_OUT_FOC;
// #define MOTOR_VAR_ACCESS_INIT_OUTPUT_USER(p_Motor)              VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_USER, &((p_Motor)->VarAccessOuputState))
// #define MOTOR_VAR_ACCESS_INIT_OUTPUT_ROTOR(p_Motor)             VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_ROTOR, &((p_Motor)->VarAccessOuputState))
// #define MOTOR_VAR_ACCESS_INIT_OUTPUT_FOC(p_Motor)               VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_FOC, &((p_Motor)->VarAccessOuputState))

// /*
//     Shared Enable/Disable Set in VarAccess
//     Set during StopState only
// */
// extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION;
// extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION_ALIAS;
// extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_ACTUATION;
// extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_PID;
// extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CMD;
// // extern const VarAccess_VTable_T MOTOR_VAR_REF;

// /* check StateMachine Config State */
// #define MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION(p_Motor)       VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CALIBRATION, &((p_Motor)->VarAccessConfigState))
// #define MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION_ALIAS(p_Motor) VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CALIBRATION_ALIAS, &((p_Motor)->VarAccessConfigState))
// #define MOTOR_VAR_ACCESS_INIT_CONFIG_ACTUATION(p_Motor)         VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_ACTUATION, &((p_Motor)->VarAccessConfigState))
// #define MOTOR_VAR_ACCESS_INIT_CONFIG_PID(p_Motor)               VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_PID, &((p_Motor)->VarAccessConfigState))
// #define MOTOR_VAR_ACCESS_INIT_CONFIG_ROUTINE(p_Motor)           VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CMD, &((p_Motor)->VarAccessConfigState))

