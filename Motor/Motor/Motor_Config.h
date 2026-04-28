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


    @brief  Config Fields Interface. Part of User Interface.
            Modifying Config values. Derived calls keep in Motor.h/c
*/
/******************************************************************************/
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H


#include "Motor.h"

#include <stdint.h>
#include <stdbool.h>



/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
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
    // MOTOR_VAR_SPEED_RAMP_RATE,
    // MOTOR_VAR_TORQUE_RAMP_RATE,
    MOTOR_VAR_OPEN_LOOP_POWER_LIMIT,
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

/* Debug */
typedef enum Motor_Var_ConfigDebug
{
    MOTOR_VAR_SPEED_RATED_RPM,
    MOTOR_VAR_SPEED_V_REF_RPM,
    MOTOR_VAR_SPEED_V_SVPWM_REF_RPM,
    MOTOR_VAR_SPEED_V_MATCH_REF_RPM,
    MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE,
    MOTOR_VAR_SPEED_V_SVPWM_REF_DEG_PER_CYCLE,
    MOTOR_VAR_V_SPEED_RATED_FRACT16,
    MOTOR_VAR_SPEED_RAMP_RATE,
    MOTOR_VAR_TORQUE_RAMP_RATE,
    // MOTOR_VAR_V_SPEED_REF_VOLTS,
}
Motor_Var_ConfigDebug_T;



/******************************************************************************/
/*

*/
/******************************************************************************/
int _Motor_Var_ConfigCalibration_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigCalibration_T varId);
void _Motor_Var_ConfigCalibration_Set(Motor_Config_T * p_motor, Motor_Var_ConfigCalibration_T varId, int varValue);

int _Motor_Var_ConfigActuation_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigActuation_T varId);
void _Motor_Var_ConfigActuation_Set(Motor_Config_T * p_motor, Motor_Var_ConfigActuation_T varId, int varValue);

int _Motor_Var_ConfigPid_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId);
void _Motor_Var_ConfigPid_Set(Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue);

int _Motor_Var_ConfigDebug_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigDebug_T varId);



#endif
