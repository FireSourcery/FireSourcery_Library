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
    /*   */
    MOTOR_VAR_SPEED_REQ,  /* Ramp Out with User Sign */
    MOTOR_VAR_TORQUE_REQ, /*   */
    /* Derived Local */
    MOTOR_VAR_V_SPEED_EFFECTIVE,
    MOTOR_VAR_POWER,
    MOTOR_VAR_I_DC,
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
    /* StateMachine Input with readable state */
    MOTOR_VAR_USER_DIRECTION,           // Calibrated direction. 1:Ccw, -1:Cw, 0:Stop
    MOTOR_VAR_USER_FEEDBACK_MODE,
    MOTOR_VAR_USER_PHASE_OUTPUT,        // Phase Output State: Float/Hold/VPwm. Direction must be set

    /* Limits do not invoke state machine. */
    MOTOR_VAR_USER_SPEED_LIMIT,
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
*/
typedef enum Motor_Var_UserSetpoint
{
    MOTOR_VAR_USER_SETPOINT_SCALAR,       // RampIn, Active/Generic mode select using active feedback mode, as scalar of full scale of active feedback mode. User Direction applies.
    // MOTOR_VAR_USER_SETPOINT_MIXED,     // UserCmd with mixed units, interpret based on state.
    MOTOR_VAR_USER_SETPOINT_SPEED,        // UserCmd as Speed
    MOTOR_VAR_USER_SETPOINT_TORQUE,
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
    MOTOR_VAR_ROTOR_ELECTRICAL_ANGLE,   /* in digital degrees */
    MOTOR_VAR_ROTOR_ELECTRICAL_DELTA,   /* Internal Ccw/Cw */
    MOTOR_VAR_ROTOR_SPEED_FEEDBACK,     /* Internal Ccw/Cw */
    MOTOR_VAR_ROTOR_MECHANICAL_ANGLE,   /* if supported */
    MOTOR_VAR_ROTOR_DIRECTION, // 1:Ccw, -1:Cw, 0:Stop
    // MOTOR_VAR_ROTOR_ELECTRICAL_SPEED_RADS,
    // MOTOR_VAR_ROTOR_MECHANICAL_SPEED_RPM,
    // optionally sensor + control loop state
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
    // optionally move
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
    Stop/Calibration/Fault State enforced same with Config
    Alternatively move type for separate policy handling
*/
// typedef enum Motor_Var_CalibrationCmd
typedef enum Motor_Var_ConfigCmd
{
    MOTOR_VAR_CONFIG_ENTER_CALIBRATION, /* Enter first before calling Substate */
    MOTOR_VAR_CONFIG_CMD_ADC_CALIBRATION,
    MOTOR_VAR_CONFIG_CMD_VIRTUAL_HOME,
    MOTOR_VAR_CALIBRATION_ENTER_TUNING,

    // MOTOR_VAR_CONFIG_CMD_SENSOR_CALIBRATION, /* Generic call for active type */
    // MOTOR_VAR_CALIBRATION_ADC,
    // MOTOR_VAR_CALIBRATION_SENSOR, /* Generic call for active type */
    // MOTOR_VAR_CALIBRATION_VIRTUAL_HOME,
}
Motor_Var_ConfigCmd_T;

/*  */
typedef RotorSensor_Id_T Motor_Var_RotorSensorCmd_T; /* sensorId as VarId, cmd as VarValue */

/******************************************************************************/
/*
    Read-Only Ref
    ConfigConst
*/
/******************************************************************************/
// typedef enum Motor_Var_Board
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


typedef enum Motor_Var_PhaseVBus
{
    MOTOR_VAR_PHASE_V_BUS,     // in frac16. additionally to vmonitor in adcu
    // MOTOR_VAR_PHASE_V_BUS_MONITOR_,
}
Motor_Var_PhaseVBus_T;

// typedef enum Motor_Var_PhaseVBusConfig
// {
//     MOTOR_VAR_PHASE_V_BUS_UPPER_FAULT,
//     MOTOR_VAR_PHASE_V_BUS_MONITOR,
// }
// Motor_Var_PhaseVBus_T;

/******************************************************************************/
/*
   Base Id Access
*/
/******************************************************************************/
int _Motor_Var_UserOut_Get(const Motor_State_T * p_motor, Motor_Var_UserOut_T varId);
int _Motor_Var_Rotor_Get(const Motor_State_T * p_motor, Motor_Var_Rotor_T varId);
int _Motor_Var_Foc_Get(const Motor_State_T * p_motor, Motor_Var_Foc_T varId);

/* Caller handle access control */
int _Motor_Var_UserControl_Get(const Motor_T * p_motor, Motor_Var_UserControl_T varId);
void _Motor_Var_UserControl_Set(const Motor_T * p_motor, Motor_Var_UserControl_T varId, int varValue);
void _Motor_Var_UserSetpoint_Set(const Motor_T * p_motor, Motor_Var_UserSetpoint_T varId, int varValue);
void _Motor_Var_StateCmd_Set(const Motor_T * p_motor, Motor_Var_StateCmd_T varId, int varValue);

int _Motor_Var_ConfigCalibration_Get(const Motor_State_T * p_motor, Motor_Var_ConfigCalibration_T varId);
void _Motor_Var_ConfigCalibration_Set(Motor_State_T * p_motor, Motor_Var_ConfigCalibration_T varId, int varValue);
int _Motor_Var_ConfigCalibrationAlias_Get(const Motor_State_T * p_motor, Motor_Var_ConfigCalibrationAlias_T varId);

int _Motor_Var_ConfigActuation_Get(const Motor_State_T * p_motor, Motor_Var_ConfigActuation_T varId);
void _Motor_Var_ConfigActuation_Set(Motor_State_T * p_motor, Motor_Var_ConfigActuation_T varId, int varValue);

int _Motor_Var_ConfigPid_Get(const Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId);
void _Motor_Var_ConfigPid_Set(Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);

void _Motor_Var_ConfigCmd_Call(const Motor_T * p_motor, Motor_Var_ConfigCmd_T varId, int varValue);

int _Motor_Var_PidTuning_Get(const Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId);
void _Motor_Var_PidTuning_Set(Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);

/*
    Submodule wrap
*/
// static inline int _Motor_Var_ConfigHeatMonitor_Get(const Motor_State_T * p_motor, Monitor_ConfigId_T varId) { return HeatMonitor_ConfigId_Get(&p_motor->HeatMonitorState, varId); }
// static inline void _Motor_Var_ConfigHeatMonitor_Set(Motor_State_T * p_motor, Monitor_ConfigId_T varId, int varValue) { HeatMonitor_ConfigId_Set(&p_motor->HeatMonitorState, varId, varValue); }
// static inline void _Motor_Var_SensorCmd_Call(const Motor_T * p_motor, Motor_Var_RotorSensorCmd_T varId, int varValue); // Motor_Sensor_CalibrationCmd_Call

/* static */
extern int Motor_Var_StaticRef_Get(Motor_Var_StaticRef_T varId);
extern int Motor_Var_PhaseVBus_Get(Motor_Var_PhaseVBus_T varId);



/******************************************************************************/
/*
    Module handled type
    VarType directly cooresponds to base enum type literlal
*/
/******************************************************************************/
/*
    Polling Except StateCmds
*/
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

/*
    Calibration or Stop State enforced Configs
*/
typedef enum Motor_VarType_Config
{
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION_ALIAS, //temp
    MOTOR_VAR_TYPE_CONFIG_ACTUATION,
    MOTOR_VAR_TYPE_CONFIG_PID,
    MOTOR_VAR_TYPE_CONFIG_CMD,          /* Config State Cmds */
    /* Alternatively move to submodule */
    MOTOR_VAR_TYPE_CONFIG_SENSOR_CMD,   /* (varId:sensorId, varValue:cmdId) Handle by Motor_Sensor.h/c. Calibration Sub StateMachine. Using Motor RotorSensor_Id_T as [varId] */
}
Motor_VarType_Config_T;

/*
    singletons,
    externally defined,
*/
typedef enum Motor_VarType_SubModule
{
    MOTOR_VAR_TYPE_STATIC_BOARD_REF,    /* Not instanced */
    MOTOR_VAR_TYPE_V_BUS,               /* Not instanced */
    MOTOR_VAR_TYPE_PHASE,
    MOTOR_VAR_TYPE_HEAT_MONITOR_OUT,    /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG, /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_THERMISTOR_CONFIG, // or handle within config
    MOTOR_VAR_TYPE_PID_TUNING_IO,       /* Non polling. PID tunning with non-Config state access permissions */
    // MOTOR_VAR_TYPE_ROTOR_SENSOR_CMD,
}
Motor_VarType_SubModule_T;

/*
    Subtype Data
    Handled by RotorSensor_Table
    Motor_Var_Sensor for Generic access

    Instead of using SensorTable Ids, This way it takes only one field to associate properties.
    allows types to expand beyond 16 ids without reserving handlers
    optionally move to rotor sensor to handle types listt
*/
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

