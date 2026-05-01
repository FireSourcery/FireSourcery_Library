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
    @file   Motor_Config.c
    @author FireSourcery
    @brief @see Motor_Config.h
*/
/******************************************************************************/
#include "Motor_Config.h"
// #include "_Motor_Config.h"
#include "Motor.h"

/******************************************************************************/
/*
    Nvm Config, check Stop State
    combine with VarConfig or UserConfig
*/
/******************************************************************************/

/******************************************************************************/
/*
    Nvm Reference/Calibration
    Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/
/******************************************************************************/
/*
    Speed Position Calibration
*/
/******************************************************************************/

/* Reboot unless deinit is implemented in HAL */
void Motor_Config_SetSensorMode(Motor_Config_T * p_config, RotorSensor_Id_T mode)
{
    p_config->SensorMode = mode;
}

void Motor_Config_SetPolePairs(Motor_Config_T * p_config, uint8_t polePairs)
{
    p_config->PolePairs = polePairs;
}

/* Setting Kv overwrites SpeedRefs. SpeedRefs can be set independently from Kv or lock */
void Motor_Config_SetKv(Motor_Config_T * p_config, uint16_t kv)
{
    p_config->Kv = kv;
    p_config->SpeedRated_Rpm = Motor_GetSpeedVNominalRef_Rpm(p_config);
}

/* allow independent set */
void Motor_Config_SetSpeedRated(Motor_Config_T * p_config, uint16_t rpm)
{
    p_config->SpeedRated_Rpm = math_min(rpm, Motor_GetSpeedVNominalRef_Rpm(p_config) * 2);
}

void Motor_Config_ResolveSpeedRated(Motor_Config_T * p_config, uint16_t vNominal) { p_config->SpeedRated_Rpm = Motor_SpeedVRef_Rpm(p_config, vNominal); }

/*
    V of Speed Ref
    SpeedVRef =< SpeedFeedbackRef to ensure not match to higher speed
    [0:32767] unitless Fract16
*/
void Motor_Config_SetVSpeedScalar_UFract16(Motor_Config_T * p_config, uint16_t scalar) { p_config->VSpeedScalar_Fract16 = math_min(scalar, INT16_MAX); }

void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_Config_T * p_motor, uint16_t rpm) { Motor_Config_SetVSpeedScalar_UFract16(p_motor, fract16_div(rpm, Motor_GetSpeedVNominalRef_Rpm(p_motor))); }
static inline uint16_t Motor_Config_GetSpeedVMatchRef_Rpm(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->VSpeedScalar_Fract16, Motor_GetSpeedVNominalRef_Rpm(p_motor)); }

// void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_Config_T * p_motor, uint16_t rpm) { Motor_Config_SetVSpeedScalar_UFract16(p_motor, fract16_div(rpm, Motor_GetSpeedRated_Rpm(p_motor))); }
// static inline uint16_t Motor_Config_GetSpeedVMatchRef_Rpm(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->VSpeedScalar_Fract16, Motor_GetSpeedRated_Rpm(p_motor)); }

static inline uint16_t _Motor_GetSpeedRated_Rpm(const Motor_Config_T * p_config) { return Motor_GetSpeedVNominalRef_Rpm(p_config); }
static inline uint16_t _Motor_GetVSpeedRated_Fract16(const Motor_Config_T * p_config) { return Motor_VFract16OfKv(p_config, Motor_GetSpeedRated_Rpm(p_config)); }



/******************************************************************************/
/*
    I Sensor Ref
*/
/******************************************************************************/
void Motor_Config_SetIaZero_Adcu(Motor_Config_T * p_config, uint16_t adcu) { p_config->IabcZeroRef_Adcu.A = adcu; }
void Motor_Config_SetIbZero_Adcu(Motor_Config_T * p_config, uint16_t adcu) { p_config->IabcZeroRef_Adcu.B = adcu; }
void Motor_Config_SetIcZero_Adcu(Motor_Config_T * p_config, uint16_t adcu) { p_config->IabcZeroRef_Adcu.C = adcu; }


/******************************************************************************/
/*
    Persistent Base Limits / Ramps
*/
/******************************************************************************/
/*
    Persistent Base SpeedLimit
*/
void Motor_Config_SetSpeedLimitForward_Fract16(Motor_Config_T * p_config, uint16_t forward_Fract16) { p_config->SpeedLimitForward_Fract16 = Motor_SpeedRatedLimitOf(p_config, forward_Fract16); }
void Motor_Config_SetSpeedLimitReverse_Fract16(Motor_Config_T * p_config, uint16_t reverse_Fract16) { p_config->SpeedLimitReverse_Fract16 = Motor_SpeedRatedLimitOf(p_config, reverse_Fract16); }


/*
    Persistent Base ILimit
*/
void Motor_Config_SetILimitMotoring_Fract16(Motor_Config_T * p_config, uint16_t motoring_Fract16) { p_config->ILimitMotoring_Fract16 = Motor_IRatedLimitOf(motoring_Fract16); }
void Motor_Config_SetILimitGenerating_Fract16(Motor_Config_T * p_config, uint16_t generating_Fract16) { p_config->ILimitGenerating_Fract16 = Motor_IRatedLimitOf(generating_Fract16); }

/*
    Ramps
*/
void Motor_Config_SetSpeedRampTime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->SpeedRampTime_Cycles = cycles; }
void Motor_Config_SetSpeedRampTime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetSpeedRampTime_Cycles(p_config, MOTOR_SPEED_CYCLES(millis)); }
void Motor_Config_SetTorqueRampTime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->TorqueRampTime_Cycles = cycles; }
void Motor_Config_SetTorqueRampTime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetTorqueRampTime_Cycles(p_config, MOTOR_TORQUE_CYCLES(millis)); }


/******************************************************************************/
/*

*/
/******************************************************************************/
/*  */
void Motor_Config_SetOpenLoopScalarLimit(Motor_Config_T * p_config, uint16_t scalar16)
{
    p_config->OpenLoopLimitScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_CEILING);
    p_config->AlignScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_CEILING);
    p_config->OpenLoopRampIFinal_Fract16 = _Motor_OpenLoopIRatedLimitOf(p_config, p_config->OpenLoopRampIFinal_Fract16);
}


/* */
void Motor_Config_SetAlignPowerScalar(Motor_Config_T * p_config, uint16_t scalar16) { p_config->AlignScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_CEILING); }

void Motor_Config_SetAlignTime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->AlignTime_Cycles = cycles; }
void Motor_Config_SetAlignTime_Millis(Motor_Config_T * p_config, uint16_t millis) { p_config->AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }

/*
*/
void Motor_Config_SetOpenLoopRampIFinal_Fract16(Motor_Config_T * p_config, uint16_t i_fract16) { p_config->OpenLoopRampIFinal_Fract16 = _Motor_OpenLoopIRatedLimitOf(p_config, i_fract16); }

void Motor_Config_SetOpenLoopRampITime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->OpenLoopRampITime_Cycles = cycles; }
void Motor_Config_SetOpenLoopRampITime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetOpenLoopRampITime_Cycles(p_config, _Motor_ControlCyclesOf(millis)); }

/*
    OpenLoop Ramp tick on control timer
*/
void Motor_Config_SetOpenLoopRampSpeedFinal_Fract16(Motor_Config_T * p_config, uint16_t speed_fract16) { p_config->OpenLoopRampSpeedFinal_Fract16 = speed_fract16; }

void Motor_Config_SetOpenLoopRampSpeedTime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->OpenLoopRampSpeedTime_Cycles = cycles; }
void Motor_Config_SetOpenLoopRampSpeedTime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetOpenLoopRampSpeedTime_Cycles(p_config, _Motor_ControlCyclesOf(millis)); }


/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    Keep interface for potential VField map and in case base unit changes
*/
/******************************************************************************/
/*
    Calibration
*/
/******************************************************************************/
/* inline set */
static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(const Motor_Config_T * p_config) { return p_config->CommutationMode; }
static inline void Motor_Config_SetCommutationMode(Motor_Config_T * p_config, Motor_CommutationMode_T mode) { p_config->CommutationMode = mode; }

/* The user direction that is the positive direction */
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(const Motor_Config_T * p_config) { return p_config->DirectionForward; }
static inline void Motor_Config_SetDirectionCalibration(Motor_Config_T * p_config, Motor_Direction_T forward) { if (forward != MOTOR_DIRECTION_NULL) { p_config->DirectionForward = forward; } }

/*
    Set with Propagate
*/
static inline RotorSensor_Id_T Motor_Config_GetSensorMode(const Motor_Config_T * p_config) { return p_config->SensorMode; }
static inline uint8_t Motor_Config_GetPolePairs(const Motor_Config_T * p_config) { return p_config->PolePairs; }
static inline uint16_t Motor_Config_GetKv(const Motor_Config_T * p_config) { return p_config->Kv; }
static inline uint16_t Motor_Config_GetSpeedRated(const Motor_Config_T * p_config) { return p_config->SpeedRated_Rpm; }
static inline uint16_t Motor_Config_GetVSpeedScalar_UFract16(const Motor_Config_T * p_config) { return p_config->VSpeedScalar_Fract16; }

static inline uint16_t Motor_Config_GetIaZero_Adcu(const Motor_Config_T * p_config) { return p_config->IabcZeroRef_Adcu.A; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(const Motor_Config_T * p_config) { return p_config->IabcZeroRef_Adcu.B; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(const Motor_Config_T * p_config) { return p_config->IabcZeroRef_Adcu.C; }
// static inline uint16_t Motor_Config_GetIPeakRef_Adcu(const Motor_State_T * p_motor)                   { return Phase_Calibration_GetIRatedPeak_Adcu(); }

// static inline Motor_AlignMode_T Motor_Config_GetAlignMode(const Motor_State_T * p_motor, Motor_AlignMode_T mode)    { return p_motor->Config.AlignMode; }
// static inline void Motor_Config_SetAlignMode( Motor_State_T * p_motor, Motor_AlignMode_T mode)     { p_motor->Config.AlignMode = mode; }

/******************************************************************************/
/*
    Actuation values
*/
/******************************************************************************/
/******************************************************************************/
/* Persistent Base Limits */
/******************************************************************************/
static inline uint16_t Motor_Config_GetSpeedLimitForward_Fract16(const Motor_Config_T * p_config) { return p_config->SpeedLimitForward_Fract16; }
static inline uint16_t Motor_Config_GetSpeedLimitReverse_Fract16(const Motor_Config_T * p_config) { return p_config->SpeedLimitReverse_Fract16; }
static inline uint16_t Motor_Config_GetILimitMotoring_Fract16(const Motor_Config_T * p_config) { return p_config->ILimitMotoring_Fract16; }
static inline uint16_t Motor_Config_GetILimitGenerating_Fract16(const Motor_Config_T * p_config) { return p_config->ILimitGenerating_Fract16; }

/******************************************************************************/
/* Ramps */
/******************************************************************************/
static inline uint32_t Motor_Config_GetSpeedRampTime_Cycles(const Motor_Config_T * p_config) { return p_config->SpeedRampTime_Cycles; }
static inline uint32_t Motor_Config_GetTorqueRampTime_Cycles(const Motor_Config_T * p_config) { return p_config->TorqueRampTime_Cycles; }
static inline uint16_t Motor_Config_GetSpeedRampTime_Millis(const Motor_Config_T * p_config) { return p_config->SpeedRampTime_Cycles; } /* Base Cycle in Millis */
static inline uint16_t Motor_Config_GetTorqueRampTime_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->TorqueRampTime_Cycles); }

/******************************************************************************/
/* OpenLoop Ramp/Limit */
/******************************************************************************/
static inline uint16_t Motor_Config_GetOpenLoopScalarLimit(const Motor_Config_T * p_config) { return p_config->OpenLoopLimitScalar_Fract16; }

static inline uint16_t Motor_Config_GetAlignPowerScalar(const Motor_Config_T * p_config) { return p_config->AlignScalar_Fract16; }
static inline uint32_t Motor_Config_GetAlignTime_Cycles(const Motor_Config_T * p_config) { return p_config->AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->AlignTime_Cycles); }

// #if defined(MOTOR_OPEN_LOOP_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeedFinal_Fract16(const Motor_Config_T * p_config) { return p_config->OpenLoopRampSpeedFinal_Fract16; }
static inline uint32_t Motor_Config_GetOpenLoopSpeedRamp_Cycles(const Motor_Config_T * p_config) { return p_config->OpenLoopRampSpeedTime_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopSpeedRamp_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->OpenLoopRampSpeedTime_Cycles); } /*  */

static inline uint16_t Motor_Config_GetOpenLoopIFinal_Fract16(const Motor_Config_T * p_config) { return p_config->OpenLoopRampIFinal_Fract16; }
static inline uint32_t Motor_Config_GetOpenLoopIRamp_Cycles(const Motor_Config_T * p_config) { return p_config->OpenLoopRampITime_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopIRamp_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->OpenLoopRampITime_Cycles); }
// #endif


int _Motor_Var_ConfigCalibration_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigCalibration_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:        value = Motor_Config_GetCommutationMode(p_motor);           break;
        case MOTOR_VAR_SENSOR_MODE:             value = Motor_Config_GetSensorMode(p_motor);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:   value = Motor_Config_GetDirectionCalibration(p_motor);      break;
        case MOTOR_VAR_POLE_PAIRS:              value = Motor_Config_GetPolePairs(p_motor);                 break;
        case MOTOR_VAR_KV:                      value = Motor_Config_GetKv(p_motor);                        break;
        case MOTOR_VAR_SPEED_RATED:             value = Motor_Config_GetSpeedRated(p_motor);                break;
        case MOTOR_VAR_V_SPEED_SCALAR:          value = Motor_Config_GetVSpeedScalar_UFract16(p_motor);     break;
        case MOTOR_VAR_IA_ZERO_ADCU:            value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IB_ZERO_ADCU:            value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
        case MOTOR_VAR_IC_ZERO_ADCU:            value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
        // case MOTOR_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
    }
    return value;
}

void _Motor_Var_ConfigCalibration_Set(Motor_Config_T * p_motor, Motor_Var_ConfigCalibration_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:              Motor_Config_SetCommutationMode(p_motor, varValue);           break;
        case MOTOR_VAR_SENSOR_MODE:                   Motor_Config_SetSensorMode(p_motor, varValue);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:         Motor_Config_SetDirectionCalibration(p_motor, varValue);      break;
        case MOTOR_VAR_POLE_PAIRS:                    Motor_Config_SetPolePairs(p_motor, varValue);                 break;
        case MOTOR_VAR_KV:                            Motor_Config_SetKv(p_motor, varValue);                        break;
        case MOTOR_VAR_SPEED_RATED:                   Motor_Config_SetSpeedRated(p_motor, varValue);                break;
        case MOTOR_VAR_V_SPEED_SCALAR:                Motor_Config_SetVSpeedScalar_UFract16(p_motor, varValue);     break;
        case MOTOR_VAR_IA_ZERO_ADCU:                  Motor_Config_SetIaZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IB_ZERO_ADCU:                  Motor_Config_SetIbZero_Adcu(p_motor, varValue);               break;
        case MOTOR_VAR_IC_ZERO_ADCU:                  Motor_Config_SetIcZero_Adcu(p_motor, varValue);               break;
        // case MOTOR_VAR_I_PEAK_REF_ADCU:               Motor_Config_SetIPeakRef_Adcu(p_motor, varValue);             break;
    }
}

/* Rates */
int _Motor_Var_ConfigActuation_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigActuation_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    value = Motor_Config_GetSpeedLimitForward_Fract16(p_motor);     break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    value = Motor_Config_GetSpeedLimitReverse_Fract16(p_motor);     break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       value = Motor_Config_GetILimitMotoring_Fract16(p_motor);        break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     value = Motor_Config_GetILimitGenerating_Fract16(p_motor);      break;
        case MOTOR_VAR_SPEED_RAMP_TIME:             value = Motor_Config_GetSpeedRampTime_Millis(p_motor);          break;
        case MOTOR_VAR_TORQUE_RAMP_TIME:            value = Motor_Config_GetTorqueRampTime_Millis(p_motor);         break;
        case MOTOR_VAR_OPEN_LOOP_POWER_LIMIT:       value = Motor_Config_GetOpenLoopScalarLimit(p_motor);           break;
        case MOTOR_VAR_ALIGN_POWER:                 value = Motor_Config_GetAlignPowerScalar(p_motor);              break;
        case MOTOR_VAR_ALIGN_TIME:                  value = Motor_Config_GetAlignTime_Millis(p_motor);              break;
    // #if defined(MOTOR_OPEN_LOOP_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(MOTOR_DEBUG_ENABLE)
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      value = Motor_Config_GetOpenLoopIFinal_Fract16(p_motor);        break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       value = Motor_Config_GetOpenLoopIRamp_Millis(p_motor);          break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  value = Motor_Config_GetOpenLoopSpeedFinal_Fract16(p_motor);    break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME:   value = Motor_Config_GetOpenLoopSpeedRamp_Millis(p_motor);      break;
    // #endif
    // #if defined(MOTOR_SIX_STEP_ENABLE)
        // case MOTOR_VAR_PHASE_POLAR_MODE:         value = Motor_Config_GetPhasePolarMode(p_motor);                break;
    // #endif
    }
    return value;
}

void _Motor_Var_ConfigActuation_Set(Motor_Config_T * p_motor, Motor_Var_ConfigActuation_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    Motor_Config_SetSpeedLimitForward_Fract16(p_motor, varValue);       break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, varValue);       break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       Motor_Config_SetILimitMotoring_Fract16(p_motor, varValue);          break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     Motor_Config_SetILimitGenerating_Fract16(p_motor, varValue);        break;
        case MOTOR_VAR_SPEED_RAMP_TIME:             Motor_Config_SetSpeedRampTime_Millis (p_motor, varValue);           break;
        case MOTOR_VAR_TORQUE_RAMP_TIME:            Motor_Config_SetTorqueRampTime_Millis (p_motor, varValue);          break;
        case MOTOR_VAR_OPEN_LOOP_POWER_LIMIT:       Motor_Config_SetOpenLoopScalarLimit(p_motor, varValue);             break;
        case MOTOR_VAR_ALIGN_POWER:                 Motor_Config_SetAlignPowerScalar (p_motor, varValue);               break;
        case MOTOR_VAR_ALIGN_TIME:                  Motor_Config_SetAlignTime_Millis(p_motor, varValue);                break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  Motor_Config_SetOpenLoopRampSpeedFinal_Fract16(p_motor, varValue);  break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME:   Motor_Config_SetOpenLoopRampSpeedTime_Millis (p_motor, varValue);   break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      Motor_Config_SetOpenLoopRampIFinal_Fract16(p_motor, varValue);      break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       Motor_Config_SetOpenLoopRampITime_Millis(p_motor, varValue);        break;
        // case MOTOR_VAR_PHASE_POLAR_MODE:           Motor_Config_SetPhaseModeParam(p_motor,  varValue);  break;
    }
}


int _Motor_Var_ConfigPid_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:     value = _PID_GetSampleFreq(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:      value = _PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:      value = _PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
            // case MOTOR_VAR_PID_SPEED_KD_FIXED16:      value = PID_GetKd_Fixed16(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:    value = _PID_GetSampleFreq(&p_motor->PidI);     break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16:     value = _PID_GetKp_Fixed16(&p_motor->PidI);     break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16:     value = _PID_GetKi_Fixed16(&p_motor->PidI);     break;
            // case MOTOR_VAR_PID_CURRENT_KD_FIXED16:     value = PID_GetKd_Fixed16(&p_motor->PidI);     break;
    }
    return value;
}

/*  */
/* Coefficients in 9.7 */
void _Motor_Var_ConfigPid_Set(Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:   break;
        case MOTOR_VAR_PID_SPEED_KP_FIXED16:            _PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);            break;
        case MOTOR_VAR_PID_SPEED_KI_FIXED16:            _PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);            break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_CURRENT_KP_FIXED16:            _PID_SetKp_Fixed16(&p_motor->PidI, varValue);            break;
        case MOTOR_VAR_PID_CURRENT_KI_FIXED16:            _PID_SetKi_Fixed16(&p_motor->PidI, varValue);            break;
    }
}


int _Motor_Var_ConfigDebug_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigDebug_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_SPEED_RATED_RPM:                 value = _Motor_GetSpeedRated_Rpm(p_motor);             break;
        case MOTOR_VAR_SPEED_V_REF_RPM:                 value = Motor_GetSpeedVNominalRef_Rpm(p_motor);              break;
        case MOTOR_VAR_SPEED_V_REF_DEG_PER_CYCLE:       value = Motor_GetSpeedVNominalRef_Angle(p_motor);             break;
        case MOTOR_VAR_SPEED_V_MATCH_REF_RPM:           value = Motor_Config_GetSpeedVMatchRef_Rpm(p_motor);         break;
        case MOTOR_VAR_V_SPEED_RATED_FRACT16:           value = _Motor_GetVSpeedRated_Fract16(p_motor);               break;
    }
    return value;
}



/******************************************************************************/
/* Local Unit Conversion */
/******************************************************************************/
#ifdef MOTOR_UNIT_CONVERSION_LOCAL
void Motor_Config_SetSpeedLimitForward_Rpm(Motor_Config_T * p_motor, uint16_t forward_Rpm)
{
    Motor_Config_SetSpeedLimitForward_Fract16(p_motor, ConvertToSpeedLimitFract16(p_motor, forward_Rpm));
}

void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_Config_T * p_motor, uint16_t reverse_Rpm)
{
    Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, ConvertToSpeedLimitFract16(p_motor, reverse_Rpm));
}

uint16_t Motor_Config_GetSpeedLimitForward_Rpm(Motor_Config_T * p_motor)
{
    return _Motor_ConvertSpeed_Fract16ToRpm(p_motor, p_motor->Config.SpeedLimitForward_Fract16);
}

uint16_t Motor_Config_GetSpeedLimitReverse_Rpm(Motor_Config_T * p_motor)
{
    return _Motor_ConvertSpeed_Fract16ToRpm(p_motor, p_motor->Config.SpeedLimitReverse_Fract16);
}
void Motor_Config_SetILimit_Amp(Motor_Config_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
    Motor_Config_SetILimit_Fract16(p_motor, ConvertToILimitFract16(p_motor, motoring_Amp), ConvertToILimitFract16(p_motor, generating_Amp));
}
void Motor_Config_SetILimitMotoring_Amp(Motor_Config_T * p_motor, uint16_t motoring_Amp)
{
    Motor_Config_SetILimitMotoring_Fract16(p_motor, ConvertToILimitFract16(p_motor, motoring_Amp));
}

void Motor_Config_SetILimitGenerating_Amp(Motor_Config_T * p_motor, uint16_t generating_Amp)
{
    Motor_Config_SetILimitGenerating_Fract16(p_motor, ConvertToILimitFract16(p_motor, generating_Amp));
}
uint16_t Motor_Config_GetILimitMotoring_Amp(Motor_Config_T * p_motor) { return _Motor_ConvertI_Fract16ToAmp(p_motor->Config.ILimitMotoring_Fract16); }
uint16_t Motor_Config_GetILimitGenerating_Amp(Motor_Config_T * p_motor) { return _Motor_ConvertI_Fract16ToAmp(p_motor->Config.ILimitGenerating_Fract16); }
#endif