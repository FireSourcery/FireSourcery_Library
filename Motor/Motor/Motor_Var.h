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
#include "Motor.h"
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
    MOTOR_VAR_TORQUE_I_REQ, /*   */
    MOTOR_VAR_TORQUE_V_REQ, /*   */
    /* Derived Local */
    MOTOR_VAR_V_SPEED_EFFECTIVE,
    MOTOR_VAR_POWER,
    MOTOR_VAR_I_BUS,

}
Motor_Var_UserOut_T;

// typedef enum Motor_Var_SubstateId
// {
// MOTOR_VAR_OPEN_LOOP_SUBSTATE,
// MOTOR_VAR_CALIBRATION_SUBSTATE,
// }
// Motor_Var_SubstateId_T;

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
    MOTOR_VAR_USER_SETPOINT_SCALAR,       // RampIn, I/Speed by default. Active/Generic mode select using active feedback mode, as scalar of full scale of active feedback mode. User Direction applies.
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
    // optionally control loop state
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

    // MOTOR_VAR_USER_START, // handle by direction for now
    // MOTOR_VAR_USER_STOP,

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
}
Motor_Var_StateCmd_T;

/*
    Calibration State Base Cmds
    Stop/Calibration/Fault State enforced same with Config
    Alternatively move type for separate policy handling
*/
typedef enum Motor_Var_CalibrationCmd
{
    MOTOR_VAR_CALIBRATION_ENTER, /* Enter first before calling Substate */
    MOTOR_VAR_CALIBRATION_CMD_ADC,
    MOTOR_VAR_CALIBRATION_CMD_VIRTUAL_HOME,
    MOTOR_VAR_CALIBRATION_ENTER_TUNING,
    // MOTOR_VAR_CALIBRATION_RESET_TUNING,
    // MOTOR_VAR_CALIBRATION_SENSOR, /* Generic call for active type */
}
Motor_Var_CalibrationCmd_T;



/******************************************************************************/
/*
    Read-Only Ref
    static const
*/
/******************************************************************************/
typedef enum Motor_Var_Board
{
    MOTOR_VAR_BOARD_V_RATED,
    MOTOR_VAR_BOARD_I_RATED,
    MOTOR_VAR_BOARD_V_MAX,
    MOTOR_VAR_BOARD_I_MAX,
    MOTOR_VAR_BOARD_V_MAX_ADCU,
    MOTOR_VAR_BOARD_I_MAX_ADCU,
    MOTOR_VAR_BOARD_V_PHASE_R1,
    MOTOR_VAR_BOARD_V_PHASE_R2,
    MOTOR_VAR_BOARD_I_PHASE_R_BASE,
    MOTOR_VAR_BOARD_I_PHASE_R_MOSFETS,
    MOTOR_VAR_BOARD_I_PHASE_GAIN,
    MOTOR_VAR_BOARD_BOARD_V_RATED_VOLTS,
    MOTOR_VAR_BOARD_BOARD_I_RATED_AMPS,
}
Motor_Var_Board_T;


typedef enum Motor_Var_PhaseVBus
{
    MOTOR_VAR_PHASE_V_BUS,     // in frac16. additionally to vmonitor in adcu
    // MOTOR_VAR_PHASE_V_BUS_MONITOR_,
}
Motor_Var_PhaseVBus_T;



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
void _Motor_Var_CalibrationCmd_Call(const Motor_T * p_motor, Motor_Var_CalibrationCmd_T varId, int varValue);

#include "Motor_Config.h"
int _Motor_Var_PidTuning_Get(const Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId);
void _Motor_Var_PidTuning_Set(Motor_State_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);


/* static */
extern int Motor_Var_Board_Get(Motor_Var_Board_T varId);
extern int Motor_Var_PhaseVBus_Get(Motor_Var_PhaseVBus_T varId);



/******************************************************************************/
/*
    [Motor_VarType]
    VarType directly corresponds to base enum type literal
    partition by Prefix. Each sub-enum indexes within its Prefix group.
*/
/******************************************************************************/
typedef enum Motor_VarType_Base
{
    MOTOR_VAR_TYPE_USER_OUT,
    MOTOR_VAR_TYPE_USER_CONTROL, /* Polling IO. Setpoint/StateMachine. */
    MOTOR_VAR_TYPE_USER_SETPOINT, /* Setpoint Input only */
    MOTOR_VAR_TYPE_ROTOR_OUT, /* Speed Angle */
    MOTOR_VAR_TYPE_FOC_OUT,
    MOTOR_VAR_TYPE_STATE_CMD, /* Non polling Cmds */
    // MOTOR_VAR_TYPE_CMD_RESV,
    MOTOR_VAR_TYPE_CONFIG_CALIBRATION,
    MOTOR_VAR_TYPE_CONFIG_ACTUATION,
    MOTOR_VAR_TYPE_CONFIG_PID,
    MOTOR_VAR_TYPE_CALIBRATION_CMD,
    MOTOR_VAR_TYPE_CONFIG_DEBUG,
    MOTOR_VAR_TYPE_CONFIG_RESV,
}
Motor_VarType_Base_T;

typedef enum Motor_VarType_SubModule
{
    MOTOR_VAR_TYPE_BOARD_CONST,    /* Not instanced */
    MOTOR_VAR_TYPE_V_BUS, /* Resv */               /* Not instanced */
    MOTOR_VAR_TYPE_PHASE,
    MOTOR_VAR_TYPE_HEAT_MONITOR_OUT,    /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_HEAT_MONITOR_CONFIG, /* Handle by HeatMonitor.c/h */
    MOTOR_VAR_TYPE_THERMISTOR_CONFIG,
    MOTOR_VAR_TYPE_PID_TUNING_IO,       /* Non polling. PID tunning with non-Config state access permissions */
}
Motor_VarType_SubModule_T;

extern int Motor_VarType_Base_Get(const Motor_T * p_motor, Motor_VarType_Base_T typeId, int varId);
extern void Motor_VarType_Base_Set(const Motor_T * p_motor, Motor_VarType_Base_T typeId, int varId, int varValue);
extern bool Motor_VarType_Base_CheckSet(const Motor_T * p_motor, Motor_VarType_Base_T typeId);

extern int Motor_VarType_SubModule_Get(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId);
extern void Motor_VarType_SubModule_Set(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId, int varId, int varValue);
extern bool Motor_VarType_SubModule_CheckSet(const Motor_T * p_motor, Motor_VarType_SubModule_T typeId);

