
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
    @brief  Var - Field-like Property Interface Getter/Setter via Id Key
*/
/******************************************************************************/
#ifndef MOTOR_VAR_H
#define MOTOR_VAR_H


#include "Utility/Var/VarAccess.h"
#include <assert.h>

/* Part of Motor */
typedef const struct Motor Motor_T;
typedef struct Motor_State Motor_State_T;

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    [Var_UserOut]
    RealTime Read-Only
    Speed/IPhase/VPhase/Power -> UFract16, may over saturate
    cross module selected values
*/
typedef enum Motor_Var_UserOut
{
    MOTOR_VAR_SPEED,
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
}
Motor_Var_UserOut_T;

typedef enum Motor_Var_FocOut
{
    MOTOR_VAR_FOC_IA,
    MOTOR_VAR_FOC_IB,
    MOTOR_VAR_FOC_IC,
    MOTOR_VAR_FOC_IQ,
    MOTOR_VAR_FOC_ID,
    MOTOR_VAR_FOC_REQ_Q,
    MOTOR_VAR_FOC_REQ_D,
    MOTOR_VAR_FOC_VQ,
    MOTOR_VAR_FOC_VD,
    MOTOR_VAR_FOC_VA,
    MOTOR_VAR_FOC_VB,
    MOTOR_VAR_FOC_VC,
    MOTOR_VAR_FOC_INTEGRAL_Q,
    MOTOR_VAR_FOC_INTEGRAL_D,
}
Motor_Var_FocOut_T;

/*
    Fixed struct member
    Var_State
*/
// typedef enum Motor_Var_State
// {
//     MOTOR_VAR_ROTARY_DIRECTION,            // Motor_Direction_T - CW/CCW.
//     MOTOR_VAR_EFFECTIVE_FEEDBACK_MODE,
//     MOTOR_VAR_EFFECTIVE_CONTROL_STATE, //OutputState
//     MOTOR_VAR_SPEED_SET_POINT,  /* Speed Ramp always */
//     MOTOR_VAR_TORQUE_SET_POINT, /* Torque Ramp always */
//     MOTOR_VAR_EFFECTIVE_SPEED_LIMIT_FORWARD,
//     MOTOR_VAR_EFFECTIVE_SPEED_LIMIT_REVERSE,
//     MOTOR_VAR_EFFECTIVE_I_LIMIT_MOTORING,
//     MOTOR_VAR_EFFECTIVE_I_LIMIT_GENERATING,
// }
// Motor_Var_State_T;

// typedef enum Motor_VarOuput_Effective
// {
//     MOTOR_VAR_EFFECTIVE_DIRECTION,
//     MOTOR_VAR_EFFECTIVE_FEEDBACK_MODE,
//     MOTOR_VAR_EFFECTIVE_CONTROL_STATE,
//     MOTOR_VAR_EFFECTIVE_SET_POINT,
//     MOTOR_VAR_EFFECTIVE_SPEED_LIMIT,
//     MOTOR_VAR_EFFECTIVE_I_LIMIT,
//     MOTOR_VAR_EFFECTIVE_RAMP_ON_OFF,
// }
// Motor_VarOuput_Effective_T;

/*
    [Var_Cmd]
    Set/Input Cmds
    Write-Only, Get returns 0
    Value [-32768:32767]
*/
typedef enum Motor_Var_Cmd
{
    MOTOR_VAR_CLEAR_FAULT,
    MOTOR_VAR_FORCE_DISABLE_CONTROL,    // No value arg. Force Disable control Non StateMachine checked, also handled via Call

    // MOTOR_VAR_USER_CMD,              // Active mode value
    // /* interface sets feedback mode first on shared ramps */
    // MOTOR_VAR_CMD_SPEED,             // UserCmd as Speed
    // MOTOR_VAR_CMD_CURRENT,
    // MOTOR_VAR_CMD_VOLTAGE,
    // MOTOR_VAR_CMD_ANGLE,
    // MOTOR_VAR_CMD_OPEN_LOOP,

    // non polling, could be io
    MOTOR_VAR_OPEN_LOOP_CONTROL, /* Enter State optional pas cmd / run */
    MOTOR_VAR_OPEN_LOOP_PHASE_OUTPUT,
    MOTOR_VAR_OPEN_LOOP_PHASE_ALIGN,
    MOTOR_VAR_OPEN_LOOP_ANGLE_ALIGN,
    MOTOR_VAR_OPEN_LOOP_JOG,
    MOTOR_VAR_OPEN_LOOP_RUN,

    // MOTOR_VAR_OPEN_LOOP_HOMING,
    // MOTOR_VAR_CALIBRATION_ENTER,
}
Motor_Var_Cmd_T;

/*
    [Var_UserIO]
    IO/Access
    May be paired getter/setter or a single variable
    in/out may differ

    All values
    polymorphic handling depending on state
*/
typedef enum Motor_Var_UserIO
{
    /* IO Vars, Read effective value, write interface value */
    MOTOR_VAR_USER_SET_POINT,       // RampIn(UserCmd)/RampOut(SetPoint), Generic mode select using active feedback mode
    MOTOR_VAR_USER_DIRECTION,       // Forwards/Reverse
    MOTOR_VAR_USER_FEEDBACK_MODE,
    MOTOR_VAR_USER_CONTROL_STATE,   /* Phase State */
    // Limits do not invoke state machine.
    MOTOR_VAR_USER_SPEED_LIMIT,
    // MOTOR_VAR_USER_I_LIMIT,
    MOTOR_VAR_USER_I_LIMIT_MOTORING,
    MOTOR_VAR_USER_I_LIMIT_GENERATING,
    // MOTOR_VAR_RAMP_ON_OFF,       // 1:Enable, 0:Disable
}
Motor_Var_UserIO_T;


/******************************************************************************/
/*
    Config Field Id
*/
/******************************************************************************/
typedef enum Motor_VarConfig_Calibration
{
    // MOTOR_CONFIG ,
    MOTOR_VAR_COMMUTATION_MODE,       /* Motor_CommutationMode_T, if runtime supported */
    MOTOR_VAR_SENSOR_MODE,            /* MotorSensor_Id_T, */
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
    Config State Cmds
*/
typedef enum Motor_VarConfig_Routine
{
    MOTOR_VAR_CONFIG_RUN_VIRTUAL_HOME,
    MOTOR_VAR_CONFIG_RUN_ADC_CALIBRATION,
    MOTOR_VAR_CONFIG_RUN_SENSOR_CALIBRATION, /* Generic call for each type */
}
Motor_VarConfig_Routine_T;

typedef enum Motor_VarRef
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
Motor_VarRef_T;


/******************************************************************************/
/*
    Interface applying the command pattern
*/
/******************************************************************************/
extern const VarAccess_VTable_T MOTOR_VAR_OUT_USER;
extern const VarAccess_VTable_T MOTOR_VAR_OUT_FOC;
extern const VarAccess_VTable_T MOTOR_VAR_OUT_SENSOR;

/*
    Disable/Enable Set in VarAccess
*/
extern const VarAccess_VTable_T MOTOR_VAR_IO_PID_TUNNING;

extern const VarAccess_VTable_T MOTOR_VAR_IN; /* VAR_CMD */
extern const VarAccess_VTable_T MOTOR_VAR_IO; /* VAR_USER_IO */

/*
    Shared Enable/Disable Set in VarAccess
    Set during StopState only
*/
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_CALIBRATION_ALIAS;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_ACTUATION;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_PID;
extern const VarAccess_VTable_T MOTOR_VAR_CONFIG_ROUTINE;

extern const VarAccess_VTable_T MOTOR_VAR_REF;

/* staticc instance */
// extern VarAccess_State_T VarAccessInputState;

#define MOTOR_VAR_ACCESS_INIT_OUTPUT_METRIC(p_Motor)            VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_USER, &((p_Motor)->VarAccessOuputState))
#define MOTOR_VAR_ACCESS_INIT_OUTPUT_FOC(p_Motor)               VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_FOC, &((p_Motor)->VarAccessOuputState))
#define MOTOR_VAR_ACCESS_INIT_OUTPUT_SENSOR(p_Motor)            VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_OUT_SENSOR, &((p_Motor)->VarAccessOuputState))

/* only this VarAccessControl is used for now */
#define MOTOR_VAR_ACCESS_INIT_PID_TUNNING(p_Motor)              VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_IO_PID_TUNNING, &((p_Motor)->VarAccessInputState))
#define MOTOR_VAR_ACCESS_INIT_INPUT(p_MotorContext, p_Motor)    VAR_ACCESS_INIT(p_MotorContext, &MOTOR_VAR_IN, &((p_Motor)->VarAccessInputState))
#define MOTOR_VAR_ACCESS_INIT_IO(p_MotorContext, p_Motor)       VAR_ACCESS_INIT(p_MotorContext, &MOTOR_VAR_IO, &((p_Motor)->VarAccessInputState))

#define MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION(p_Motor)       VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CALIBRATION, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION_ALIAS(p_Motor) VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_CALIBRATION_ALIAS, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_ACTUATION(p_Motor)         VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_ACTUATION, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_PID(p_Motor)               VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_PID, &((p_Motor)->VarAccessConfigState))
#define MOTOR_VAR_ACCESS_INIT_CONFIG_ROUTINE(p_Motor)           VAR_ACCESS_INIT(p_Motor, &MOTOR_VAR_CONFIG_ROUTINE, &((p_Motor)->VarAccessConfigState))

typedef struct Motor_VarAccess
{
    const VarAccess_T OUT_METRIC;
    const VarAccess_T OUT_FOC;
    const VarAccess_T OUT_POSITION_SENSOR;
    const VarAccess_T IO_PID_TUNNING; /* PID Tunning */
    const VarAccess_T IN;
    const VarAccess_T IO;
    const VarAccess_T CONFIG_CALIBRATION;
    const VarAccess_T CONFIG_CALIBRATION_ALIAS;
    const VarAccess_T CONFIG_ACTUATION;
    const VarAccess_T CONFIG_PID;
    const VarAccess_T CONFIG_ROUTINE;
}
Motor_VarAccess_T;


// some will need to change to the const struct
#define MOTOR_VAR_ACCESS_INIT(p_MotorContext, p_Motor) \
{ \
    .OUT_METRIC                  = MOTOR_VAR_ACCESS_INIT_OUTPUT_METRIC(p_Motor), \
    .OUT_FOC                     = MOTOR_VAR_ACCESS_INIT_OUTPUT_FOC(p_Motor), \
    .OUT_POSITION_SENSOR         = MOTOR_VAR_ACCESS_INIT_OUTPUT_SENSOR(p_Motor), \
    .IN                          = MOTOR_VAR_ACCESS_INIT_INPUT(p_MotorContext, p_Motor), \
    .IO                          = MOTOR_VAR_ACCESS_INIT_IO(p_MotorContext, p_Motor), \
    .CONFIG_CALIBRATION          = MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION(p_Motor), \
    .CONFIG_CALIBRATION_ALIAS    = MOTOR_VAR_ACCESS_INIT_CONFIG_CALIBRATION_ALIAS(p_Motor), \
    .CONFIG_ACTUATION            = MOTOR_VAR_ACCESS_INIT_CONFIG_ACTUATION(p_Motor), \
    .CONFIG_PID                  = MOTOR_VAR_ACCESS_INIT_CONFIG_PID(p_Motor), \
    .CONFIG_ROUTINE              = MOTOR_VAR_ACCESS_INIT_CONFIG_ROUTINE(p_Motor) \
}

// static inline void Motor_Var_DisableInput(Motor_State_T * p_motor) { _VarAccess_DisableSet(&p_motor->VarAccessInputState); }
// static inline void Motor_Var_EnableInput(Motor_State_T * p_motor) { _VarAccess_EnableSet(&p_motor->VarAccessInputState); }


/******************************************************************************/
/*
   Extern
*/
/******************************************************************************/
// extern int32_t _Motor_VarOutput_PositionSensor_Get(const Motor_State_T * p_motor, Motor_VarOutput_PositionSensor_T varId);
/* commands on handler struct */


extern int32_t _Motor_Var_UserOut_Get(const Motor_State_T * p_motor, Motor_Var_UserOut_T varId);
extern int32_t _Motor_Var_Foc_Get(const Motor_State_T * p_motor, Motor_Var_FocOut_T varId);

extern void _Motor_Var_Cmd_Set(const Motor_T * p_motor, Motor_Var_Cmd_T varId, int32_t varValue);

extern void _Motor_Var_PidTunning_Set(Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId, int32_t varValue);
extern int32_t _Motor_Var_PidTunning_Get(const Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId);

extern void _Motor_Var_UserIO_Set(const Motor_T * p_motor, Motor_Var_UserIO_T varId, int32_t varValue);
extern int32_t _Motor_Var_UserIO_Get(const Motor_T * p_motor, Motor_Var_UserIO_T varId);

extern int32_t _Motor_VarConfig_Calibration_Get(const Motor_State_T * p_motor, Motor_VarConfig_Calibration_T varId);
extern int32_t _Motor_VarConfig_CalibrationAlias_Get(const Motor_State_T * p_motor, Motor_VarConfig_CalibrationAlias_T varId);
extern int32_t _Motor_VarConfig_Actuation_Get(const Motor_State_T * p_motor, Motor_VarConfig_Actuation_T varId);
extern int32_t _Motor_VarConfig_Pid_Get(const Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId);

extern void _Motor_VarConfig_Calibration_Set(Motor_State_T * p_motor, Motor_VarConfig_Calibration_T varId, int32_t varValue);
extern void _Motor_VarConfig_Actuation_Set(Motor_State_T * p_motor, Motor_VarConfig_Actuation_T varId, int32_t varValue);
extern void _Motor_VarConfig_Pid_Set(Motor_State_T * p_motor, Motor_VarConfig_Pid_T varId, int32_t varValue);

extern void _Motor_VarConfig_Routine_Call(const Motor_T * p_motor, Motor_VarConfig_Routine_T varId, int32_t varValue);

extern int32_t Motor_VarRef_Get(Motor_VarRef_T varId);

// extern void Motor_Var_UserIO_Set(const Motor_T * p_motor, Motor_Var_UserIO_T varId, int32_t varValue);


/*
    Generic Wrapper
*/
// extern int Motor_VarOutput_Get(const Motor_State_T * p_motor, int varId);
// extern int Motor_VarOutput_Foc_Get(const Motor_State_T * p_motor, int varId);
// // extern int Motor_VarOutput_PositionSensor_Get(const Motor_State_T * p_motor, int varId);

// extern void Motor_Var_Cmd_Set(const Motor_T * p_motor, int varId, int varValue);

// extern int Motor_Var_UserIO_Get(const Motor_T * p_motor, int varId);
// extern void Motor_Var_UserIO_Set(const Motor_T * p_motor, int varId, int varValue);

// extern int Motor_VarConfig_Calibration_Get(const Motor_State_T * p_motor, int varId);
// extern int Motor_VarConfig_CalibrationAlias_Get(const Motor_State_T * p_motor, int varId);
// extern int Motor_VarConfig_Actuation_Get(const Motor_State_T * p_motor, int varId);
// extern int Motor_VarConfig_Pid_Get(const Motor_State_T * p_motor, int varId);

// extern void Motor_VarConfig_Calibration_Set(Motor_State_T * p_motor, int varId, int varValue);
// extern void Motor_VarConfig_Actuation_Set(Motor_State_T * p_motor, int varId, int varValue);
// extern void Motor_VarConfig_Pid_Set(Motor_State_T * p_motor, int varId, int varValue);

// extern void Motor_VarConfig_Routine_Call(const Motor_T * p_motor, int varId, int varValue);
// extern void Motor_VarConfig_Routine_Call(const Motor_T * p_motor, int varId, int varValue);



#endif


