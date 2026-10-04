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

#include "Math/FOC_Ext.h"


/******************************************************************************/
/*
   Validate
*/
/******************************************************************************/
/*
    Rated Limit - applied on Config Bounds
    Align/OpenLoop take the full rated range. Runtime limits apply at the use site.
*/
/* optionally resolve as a config field, or get global */
/* approximate limit without coupling VBusNominal */
static inline uint16_t _Motor_SpeedRatedLimit(const Motor_Config_T * p_config) { (void)p_config; return Phase_VRated_Pu(); }
static inline uint16_t _Motor_SpeedFwLimit(const Motor_Config_T * p_config) { (void)p_config; return INT16_MAX; }
static inline uint16_t _Motor_IRatedLimit() { return Phase_IRatedPeak_Pu(); }
static inline uint16_t _Motor_VRatedLimit() { return Phase_VRated_Pu(); }

/* Unregulated V-mode drives Id = VAlign / Rs at standstill: bound to the V that drives rated I. Rs 0 is unset */
static inline ufract16_t _Motor_VAlignRsLimit(uint32_t rs) { return (rs != 0U) ? fract16_sat_positive(accum32_mul(_Motor_IRatedLimit(), rs)) : FRACT16_MAX; }

bool Motor_Config_IsValid(const Motor_Config_T * p_config)
{
    return
    (
        (p_config->DirectionForward != MOTOR_DIRECTION_NULL) &&
        (p_config->IabcZeroRef_Adcu.A != 0U) && (p_config->IabcZeroRef_Adcu.B != 0U) && (p_config->IabcZeroRef_Adcu.C != 0U) &&
        (p_config->KSpeed.PolePairs != 0U) && (p_config->KSpeed.AngleFreqPerVolt > 0) && (p_config->VSpeedAdjustment <= INT16_MAX) &&
        (p_config->ILimitMotoring_Pu <= _Motor_IRatedLimit()) &&
        (p_config->ILimitGenerating_Pu <= _Motor_IRatedLimit()) &&
        (p_config->IAlign_Pu <= p_config->ILimitMotoring_Pu) &&
        (p_config->VAlign_Pu <= _Motor_VRatedLimit()) &&
        (p_config->OpenLoopRampIFinal_Pu <= _Motor_IRatedLimit()) &&
        (p_config->OpenLoopRampSpeedFinal_Pu <= _Motor_SpeedRatedLimit(p_config) / 2)
    );
}

/*
    additional using caller context
*/
bool _Motor_Config_IsValidSpeed(const Motor_Config_T * p_config, uint16_t speedCeiling)
{
    return (p_config->SpeedLimitForward_Pu <= speedCeiling) && (p_config->SpeedLimitReverse_Pu <= speedCeiling);
}


/******************************************************************************/
/*

*/
/******************************************************************************/
void Motor_Config_ValidateOpenLoop(Motor_Config_T * p_config)
{
    p_config->IAlign_Pu                    = math_min(p_config->IAlign_Pu, _Motor_IRatedLimit());
    p_config->VAlign_Pu                    = math_min(p_config->VAlign_Pu, _Motor_VRatedLimit());
    p_config->OpenLoopRampIFinal_Pu        = math_min(p_config->OpenLoopRampIFinal_Pu, _Motor_IRatedLimit());
    p_config->OpenLoopRampSpeedFinal_Pu    = math_min(p_config->OpenLoopRampSpeedFinal_Pu, _Motor_SpeedRatedLimit(p_config) / 2);
}

void Motor_Config_Validate(Motor_Config_T * p_config)
{
    p_config->VSpeedAdjustment                  = math_min(p_config->VSpeedAdjustment, INT16_MAX);
    p_config->ILimitMotoring_Pu            = math_min(p_config->ILimitMotoring_Pu, _Motor_IRatedLimit());
    p_config->ILimitGenerating_Pu          = math_min(p_config->ILimitGenerating_Pu, _Motor_IRatedLimit());
    p_config->IAlign_Pu                    = math_min(p_config->IAlign_Pu, p_config->ILimitMotoring_Pu);
    p_config->VAlign_Pu                    = math_min(p_config->VAlign_Pu, _Motor_VRatedLimit());
    p_config->OpenLoopRampIFinal_Pu        = math_min(p_config->OpenLoopRampIFinal_Pu, _Motor_IRatedLimit());
    p_config->OpenLoopRampSpeedFinal_Pu    = math_min(p_config->OpenLoopRampSpeedFinal_Pu, _Motor_SpeedRatedLimit(p_config) / 2);
    //preliminary limit
    // p_config->SpeedLimitForward_Pu   = math_min(p_config->SpeedLimitForward_Pu, _Motor_SpeedRatedLimit(p_config));
    // p_config->SpeedLimitReverse_Pu   = math_min(p_config->SpeedLimitReverse_Pu, _Motor_SpeedRatedLimit(p_config));
}

void Motor_Config_ValidateSpeed(Motor_Config_T * p_config, uint16_t speedCeiling)
{
    p_config->SpeedLimitForward_Pu = math_min(speedCeiling, p_config->SpeedLimitForward_Pu);
    p_config->SpeedLimitReverse_Pu = math_min(speedCeiling, p_config->SpeedLimitReverse_Pu);
}

void Motor_Config_ValidateVAlign(Motor_Config_T * p_config)
{
    p_config->VAlign_Pu = math_min(_Motor_VAlignRsLimit(p_config->Electrical.Rs), p_config->VAlign_Pu);
}



/*
    Keep functions interface for potential descriptor map and in case base unit changes
*/
/******************************************************************************/
/*
    Speed Position Calibration
*/
/******************************************************************************/
/* Reboot unless deinit is implemented in HAL */
void Motor_Config_SetSensorMode(Motor_Config_T * p_config, RotorSensor_Id_T mode) { p_config->SensorMode = mode; }

/* Holds the electrical KSpeed. Kv entered after PolePairs, in id order, holds Kv */
void Motor_Config_SetPolePairs(Motor_Config_T * p_config, uint8_t polePairs) { p_config->KSpeed.PolePairs = polePairs; }

/* Setting Kv overwrites SpeedRefs. SpeedRefs can be set independently from Kv or lock */
void Motor_Config_SetKv(Motor_Config_T * p_config, uint16_t kv) { p_config->KSpeed.AngleFreqPerVolt = angle_freq_per_v_of_kv(kv, p_config->KSpeed.PolePairs); }

/*
    V of Speed Ref
    SpeedVRef =< SpeedFeedbackRef to ensure not match to higher speed
    [0:32767] <=> [0, 1] unitless Fract16
*/
void Motor_Config_SetVSpeedRatio(Motor_Config_T * p_config, uint16_t scalar) { p_config->VSpeedAdjustment = math_min(scalar, INT16_MAX); }

/* SpeedRated is derived (Kv·VRated); setting it back-solves Kv so Kv·VRated == rpm (recomputes Ke/Psi/SpeedTypeMax) */
void Motor_Config_SetSpeedRated(Motor_Config_T * p_config, uint16_t vbus, uint16_t rpm) { Motor_Config_SetKv(p_config, rpm / vbus); }

// void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_Config_T * p_motor, uint16_t rpm) { Motor_Config_SetVSpeedRatio(p_motor, fract16_div(rpm, _Motor_GetSpeedRated_Rpm(&p_motor->SpeedRating))); }
// static inline uint16_t Motor_Config_GetSpeedVMatchRef_Rpm(const Motor_Config_T * p_motor) { return fract16_mul(p_motor->SpeedRating.VSpeedAdjustment, _Motor_GetSpeedRated_Rpm(&p_motor->SpeedRating)); }


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
    Calibration
*/
/******************************************************************************/
/* inline set */
// static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(const Motor_Config_T * p_config) { return p_config->CommutationMode; }
// static inline void Motor_Config_SetCommutationMode(Motor_Config_T * p_config, Motor_CommutationMode_T mode) { p_config->CommutationMode = mode; }

/* The user direction that is the positive direction */
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(const Motor_Config_T * p_config) { return p_config->DirectionForward; }
static inline void Motor_Config_SetDirectionCalibration(Motor_Config_T * p_config, Motor_Direction_T forward) { if (forward != MOTOR_DIRECTION_NULL) { p_config->DirectionForward = forward; } }
// static inline void Motor_Config_SetCcwPositive(Motor_Config_T * p_motor, bool isCcwPositive) { p_motor->DirectionForward = (isCcwPositive) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW; }

static inline RotorSensor_Id_T Motor_Config_GetSensorMode(const Motor_Config_T * p_config) { return p_config->SensorMode; }
static inline uint8_t Motor_Config_GetPolePairs(const Motor_Config_T * p_config) { return p_config->KSpeed.PolePairs; }
static inline uint16_t Motor_Config_GetKv(const Motor_Config_T * p_config) { return kv_of_angle_freq_per_v(p_config->KSpeed.AngleFreqPerVolt, p_config->KSpeed.PolePairs); }
// static inline uint16_t Motor_Config_GetSpeedRated(const Motor_Config_T * p_config) { return _Motor_SpeedRated_Rpm(&p_config->SpeedRating); }
static inline uint16_t Motor_Config_GetVSpeedRatio(const Motor_Config_T * p_config) { return p_config->VSpeedAdjustment; }

static inline uint16_t Motor_Config_GetIaZero_Adcu(const Motor_Config_T * p_config) { return p_config->IabcZeroRef_Adcu.A; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(const Motor_Config_T * p_config) { return p_config->IabcZeroRef_Adcu.B; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(const Motor_Config_T * p_config) { return p_config->IabcZeroRef_Adcu.C; }
// static inline uint16_t Motor_Config_GetIPeakRef_Adcu(const Motor_Context_T * p_motor)                   { return Phase_IRatedPeak_Adcu(); }

// static inline Motor_AlignMode_T Motor_Config_GetAlignMode(const Motor_Context_T * p_motor, Motor_AlignMode_T mode)    { return p_motor->Config.AlignMode; }
// static inline void Motor_Config_SetAlignMode( Motor_Context_T * p_motor, Motor_AlignMode_T mode)     { p_motor->Config.AlignMode = mode; }

/******************************************************************************/
/*
    Actuation values
*/
/******************************************************************************/
/******************************************************************************/
/* Persistent Base Limits */
/******************************************************************************/
static inline uint16_t Motor_Config_GetSpeedLimitForward_Pu(const Motor_Config_T * p_config) { return p_config->SpeedLimitForward_Pu; }
static inline uint16_t Motor_Config_GetSpeedLimitReverse_Pu(const Motor_Config_T * p_config) { return p_config->SpeedLimitReverse_Pu; }
static inline uint16_t Motor_Config_GetILimitMotoring_Pu(const Motor_Config_T * p_config) { return p_config->ILimitMotoring_Pu; }
static inline uint16_t Motor_Config_GetILimitGenerating_Pu(const Motor_Config_T * p_config) { return p_config->ILimitGenerating_Pu; }

/*
    Persistent Base SpeedLimit
*/
void Motor_Config_SetSpeedLimitForward_Pu(Motor_Config_T * p_config, uint16_t forward_pu) { p_config->SpeedLimitForward_Pu = math_min(_Motor_SpeedRatedLimit(p_config), forward_pu); }
void Motor_Config_SetSpeedLimitReverse_Pu(Motor_Config_T * p_config, uint16_t reverse_pu) { p_config->SpeedLimitReverse_Pu = math_min(_Motor_SpeedRatedLimit(p_config), reverse_pu); }

/*
    Persistent Base ILimit
*/
void Motor_Config_SetILimitMotoring_Pu(Motor_Config_T * p_config, uint16_t motoring_pu) { p_config->ILimitMotoring_Pu = math_min(_Motor_IRatedLimit(), motoring_pu); }
void Motor_Config_SetILimitGenerating_Pu(Motor_Config_T * p_config, uint16_t generating_pu) { p_config->ILimitGenerating_Pu = math_min(_Motor_IRatedLimit(), generating_pu); }

/******************************************************************************/
/* Ramp Slope */
/******************************************************************************/
/*
    Ramp Slope access variations
    Interface in time to saturation
*/
// time to configured limit, both directions use forward as limit, optionally add opposite ramp coefficient later.
static inline uint32_t Motor_Config_GetSpeedRampTime_Ticks(const Motor_Config_T * p_config) { return (p_config->SpeedRampSlope_PuPerTick != 0U) ? RAMP_TICKS_OF_COEF(p_config->SpeedLimitForward_Pu, p_config->SpeedRampSlope_PuPerTick) : 0U; }
static inline uint32_t Motor_Config_GetTorqueRampTime_Ticks(const Motor_Config_T * p_config) { return (p_config->TorqueRampSlope_PuPerTick != 0U) ? RAMP_TICKS_OF_COEF(p_config->ILimitMotoring_Pu, p_config->TorqueRampSlope_PuPerTick) : 0U; }

/* Speed Base Ticks is Millis */
static inline uint16_t Motor_Config_GetSpeedRampTime_Millis(const Motor_Config_T * p_config) { return (Motor_Config_GetSpeedRampTime_Ticks(p_config)); }
static inline uint16_t Motor_Config_GetTorqueRampTime_Millis(const Motor_Config_T * p_config) { return MOTOR_TORQUE_TIME_MS(Motor_Config_GetTorqueRampTime_Ticks(p_config)); }

static inline uint32_t Motor_Config_GetSpeedRampSlope_PuPerTick(const Motor_Config_T * p_config) { return p_config->SpeedRampSlope_PuPerTick; }
static inline uint32_t Motor_Config_GetTorqueRampSlope_PuPerTick(const Motor_Config_T * p_config) { return p_config->TorqueRampSlope_PuPerTick; }
void Motor_Config_SetSpeedRampSlope_PuPerTick(Motor_Config_T * p_config, uint32_t slope) { p_config->SpeedRampSlope_PuPerTick = slope; }
void Motor_Config_SetTorqueRampSlope_PuPerTick(Motor_Config_T * p_config, uint32_t slope) { p_config->TorqueRampSlope_PuPerTick = slope; }

void Motor_Config_SetSpeedRampTime_Ticks(Motor_Config_T * p_config, uint32_t cycles) { p_config->SpeedRampSlope_PuPerTick = (cycles != 0U) ? RAMP_COEF_OF_TICKS(p_config->SpeedLimitForward_Pu, cycles) : 0; }
void Motor_Config_SetTorqueRampTime_Ticks(Motor_Config_T * p_config, uint32_t cycles) { p_config->TorqueRampSlope_PuPerTick = (cycles != 0U) ? RAMP_COEF_OF_TICKS(p_config->ILimitMotoring_Pu, cycles) : 0; }
void Motor_Config_SetSpeedRampTime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetSpeedRampTime_Ticks(p_config, MOTOR_SPEED_CYCLES(millis)); }
void Motor_Config_SetTorqueRampTime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetTorqueRampTime_Ticks(p_config, MOTOR_TORQUE_CYCLES(millis)); }

/*
    Interface in Physical Units (display/readout)
*/
static inline uint32_t Motor_Config_GetSpeedRampSlope_RpmPerS(const Motor_Config_T * p_config) { return Motor_Speed_RpmOfPu(&p_config->KSpeed, (int64_t)p_config->SpeedRampSlope_PuPerTick * MOTOR_SPEED_LOOP_FREQ / ACCUMULATOR_SCALE); }
static inline uint32_t Motor_Config_GetTorqueRampSlope_AmpPerS(const Motor_Config_T * p_config) { return Phase_I_AmpsOfPu((int64_t)p_config->TorqueRampSlope_PuPerTick * MOTOR_CONTROL_FREQ / ACCUMULATOR_SCALE); }

/*
    Ticks Time coversion first to prevent overflow
*/
void Motor_Config_SetSpeedRampSlope_RpmPerS(Motor_Config_T * p_config, uint32_t rpm) { p_config->SpeedRampSlope_PuPerTick = Motor_Speed_PuOfRpm(&p_config->KSpeed, rpm * ACCUMULATOR_SCALE / MOTOR_SPEED_LOOP_FREQ); }
void Motor_Config_SetTorqueRampSlope_AmpPerS(Motor_Config_T * p_config, uint32_t amps) { p_config->TorqueRampSlope_PuPerTick = Phase_I_PuOfAmps(amps * ACCUMULATOR_SCALE / MOTOR_CONTROL_FREQ); }

/*
    Interface in Fract16
*/
// static inline uint16_t Motor_Config_GetSpeedRampSlope_Fract16PerTick(const Motor_Config_T * p_config) { return p_config->SpeedRampSlope_PuPerTick / ACCUMULATOR_SCALE; }
// static inline uint16_t Motor_Config_GetTorqueRampSlope_Fract16PerTick(const Motor_Config_T * p_config) { return  p_config->TorqueRampSlope_PuPerTick / ACCUMULATOR_SCALE; }
// void Motor_Config_SetSpeedRampSlope_Fract16PerTick(Motor_Config_T * p_config, uint32_t slope) { p_config->SpeedRampSlope_PuPerTick = slope * ACCUMULATOR_SCALE; }
// void Motor_Config_SetTorqueRampSlope_Fract16PerTick(Motor_Config_T * p_config, uint32_t slope) { p_config->TorqueRampSlope_PuPerTick = slope * ACCUMULATOR_SCALE; }



/******************************************************************************/
/*
    Openloop
*/
/******************************************************************************/
/*  */
void Motor_Config_SetIAlign(Motor_Config_T * p_config, uint16_t scalar16) { p_config->IAlign_Pu = math_min(scalar16, _Motor_IRatedLimit()); }
void Motor_Config_SetVAlign(Motor_Config_T * p_config, uint16_t scalar16) { p_config->VAlign_Pu = math_min(scalar16, _Motor_VRatedLimit()); }


static inline uint32_t Motor_Config_GetAlignTime_Cycles(const Motor_Config_T * p_config) { return p_config->AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->AlignTime_Cycles); }

void Motor_Config_SetAlignTime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->AlignTime_Cycles = cycles; }
void Motor_Config_SetAlignTime_Millis(Motor_Config_T * p_config, uint16_t millis) { p_config->AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }

/******************************************************************************/
/*
    OpenLoop Run
*/
/******************************************************************************/
// #if defined(MOTOR_OPEN_LOOP_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeedFinal_Pu(const Motor_Config_T * p_config) { return p_config->OpenLoopRampSpeedFinal_Pu; }
static inline uint32_t Motor_Config_GetOpenLoopSpeedRamp_Cycles(const Motor_Config_T * p_config) { return p_config->OpenLoopRampSpeedTime_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopSpeedRamp_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->OpenLoopRampSpeedTime_Cycles); }

static inline uint16_t Motor_Config_GetOpenLoopIFinal_Pu(const Motor_Config_T * p_config) { return p_config->OpenLoopRampIFinal_Pu; }
static inline uint32_t Motor_Config_GetOpenLoopIRamp_Cycles(const Motor_Config_T * p_config) { return p_config->OpenLoopRampITime_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopIRamp_Millis(const Motor_Config_T * p_config) { return _Motor_MillisOf(p_config->OpenLoopRampITime_Cycles); }
// #endif
/*
    OpenLoop Ramp ticks on control timer, unlike speedRamp
*/
void Motor_Config_SetOpenLoopRampSpeedFinal_Pu(Motor_Config_T * p_config, uint16_t speed_pu) { p_config->OpenLoopRampSpeedFinal_Pu = math_min(speed_pu, _Motor_SpeedRatedLimit(p_config) / 2); }
void Motor_Config_SetOpenLoopRampSpeedTime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->OpenLoopRampSpeedTime_Cycles = cycles; }
void Motor_Config_SetOpenLoopRampSpeedTime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetOpenLoopRampSpeedTime_Cycles(p_config, _Motor_ControlCyclesOf(millis)); }

void Motor_Config_SetOpenLoopRampIFinal_Pu(Motor_Config_T * p_config, uint16_t i_pu) { p_config->OpenLoopRampIFinal_Pu = math_min(i_pu, _Motor_IRatedLimit()); }
void Motor_Config_SetOpenLoopRampITime_Cycles(Motor_Config_T * p_config, uint32_t cycles) { p_config->OpenLoopRampITime_Cycles = cycles; }
void Motor_Config_SetOpenLoopRampITime_Millis(Motor_Config_T * p_config, uint16_t millis) { Motor_Config_SetOpenLoopRampITime_Cycles(p_config, _Motor_ControlCyclesOf(millis)); }



/******************************************************************************/
/*
    Var Id Interface
*/
/******************************************************************************/
int _Motor_Var_ConfigCalibration_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigCalibration_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_COMMUTATION_MODE:            break;
        case MOTOR_VAR_SENSOR_MODE:             value = Motor_Config_GetSensorMode(p_motor);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:   value = Motor_Config_GetDirectionCalibration(p_motor);      break;
        case MOTOR_VAR_POLE_PAIRS:              value = Motor_Config_GetPolePairs(p_motor);                 break;
        case MOTOR_VAR_KV:                      value = Motor_Config_GetKv(p_motor);                        break;
        case MOTOR_VAR_SPEED_RATED:              break; /* getter needs (vbus, rpm), not exposed via single-value interface */
        case MOTOR_VAR_V_SPEED_TUNING:          value = Motor_Config_GetVSpeedRatio(p_motor);     break;
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
        case MOTOR_VAR_COMMUTATION_MODE:                break;
        case MOTOR_VAR_SENSOR_MODE:                   Motor_Config_SetSensorMode(p_motor, varValue);                break;
        case MOTOR_VAR_DIRECTION_CALIBRATION:         Motor_Config_SetDirectionCalibration(p_motor, varValue);      break;
        case MOTOR_VAR_POLE_PAIRS:                    Motor_Config_SetPolePairs(p_motor, varValue);                 break;
        case MOTOR_VAR_KV:                            Motor_Config_SetKv(p_motor, varValue);                        break;
        case MOTOR_VAR_SPEED_RATED:                    break; /* setter needs (vbus, rpm), not exposed via single-value interface */
        case MOTOR_VAR_V_SPEED_TUNING:                Motor_Config_SetVSpeedRatio(p_motor, varValue);     break;
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
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    value = Motor_Config_GetSpeedLimitForward_Pu(p_motor);     break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    value = Motor_Config_GetSpeedLimitReverse_Pu(p_motor);     break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       value = Motor_Config_GetILimitMotoring_Pu(p_motor);        break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     value = Motor_Config_GetILimitGenerating_Pu(p_motor);      break;
        // case MOTOR_VAR_SPEED_RAMP_RATE:             value = Motor_Config_GetSpeedRampTime_Millis(p_motor);          break;
        // case MOTOR_VAR_TORQUE_RAMP_RATE:            value = Motor_Config_GetTorqueRampTime_Millis(p_motor);         break;
        case MOTOR_VAR_SPEED_RAMP_RATE:             value = Motor_Config_GetSpeedRampSlope_PuPerTick(p_motor);          break;
        case MOTOR_VAR_TORQUE_RAMP_RATE:            value = Motor_Config_GetTorqueRampSlope_PuPerTick(p_motor);         break;
        case MOTOR_VAR_OPEN_LOOP_POWER_LIMIT:       break; /* retired, id kept for wire compatibility */
        case MOTOR_VAR_I_ALIGN:                     value = p_motor->IAlign_Pu;  break;
        case MOTOR_VAR_V_ALIGN:                     value = p_motor->VAlign_Pu;  break;
        case MOTOR_VAR_ALIGN_TIME:                  value = Motor_Config_GetAlignTime_Millis(p_motor);              break;
    // #if defined(MOTOR_OPEN_LOOP_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(MOTOR_DEBUG_ENABLE)
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      value = Motor_Config_GetOpenLoopIFinal_Pu(p_motor);        break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       value = Motor_Config_GetOpenLoopIRamp_Millis(p_motor);          break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  value = Motor_Config_GetOpenLoopSpeedFinal_Pu(p_motor);    break;
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
        case MOTOR_VAR_BASE_SPEED_LIMIT_FORWARD:    Motor_Config_SetSpeedLimitForward_Pu(p_motor, varValue);       break;
        case MOTOR_VAR_BASE_SPEED_LIMIT_REVERSE:    Motor_Config_SetSpeedLimitReverse_Pu(p_motor, varValue);       break;
        case MOTOR_VAR_BASE_I_LIMIT_MOTORING:       Motor_Config_SetILimitMotoring_Pu(p_motor, varValue);          break;
        case MOTOR_VAR_BASE_I_LIMIT_GENERATING:     Motor_Config_SetILimitGenerating_Pu(p_motor, varValue);        break;
        // case MOTOR_VAR_SPEED_RAMP_RATE:             Motor_Config_SetSpeedRampTime_Millis (p_motor, varValue);           break;
        // case MOTOR_VAR_TORQUE_RAMP_RATE:            Motor_Config_SetTorqueRampTime_Millis (p_motor, varValue);          break;
        case MOTOR_VAR_SPEED_RAMP_RATE:             Motor_Config_SetSpeedRampSlope_PuPerTick(p_motor, varValue);           break;
        case MOTOR_VAR_TORQUE_RAMP_RATE:            Motor_Config_SetTorqueRampSlope_PuPerTick(p_motor, varValue);          break;
        case MOTOR_VAR_OPEN_LOOP_POWER_LIMIT:       break; /* retired, id kept for wire compatibility */
        case MOTOR_VAR_I_ALIGN:                     Motor_Config_SetIAlign(p_motor, varValue);                                break;
        case MOTOR_VAR_V_ALIGN:                     Motor_Config_SetVAlign(p_motor, varValue);                                break;
        case MOTOR_VAR_ALIGN_TIME:                  Motor_Config_SetAlignTime_Millis(p_motor, varValue);                break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_FINAL:  Motor_Config_SetOpenLoopRampSpeedFinal_Pu(p_motor, varValue);  break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_SPEED_TIME:   Motor_Config_SetOpenLoopRampSpeedTime_Millis (p_motor, varValue);   break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_FINAL:      Motor_Config_SetOpenLoopRampIFinal_Pu(p_motor, varValue);      break;
        case MOTOR_VAR_OPEN_LOOP_RAMP_I_TIME:       Motor_Config_SetOpenLoopRampITime_Millis(p_motor, varValue);        break;
        // case MOTOR_VAR_PHASE_POLAR_MODE:           Motor_Config_SetPhaseModeParam(p_motor,  varValue);  break;
    }
}

/*
    32-bit return
*/
int _Motor_Var_ConfigPid_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:       value = p_motor->PidSpeed.SampleFreq;   break;
        case MOTOR_VAR_PID_SPEED_KP:                value = p_motor->PidSpeed.Kp_Fixed32;   break;
        case MOTOR_VAR_PID_SPEED_KI:                value = p_motor->PidSpeed.Ki_Fixed32;   break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:     value = p_motor->PidI.SampleFreq;     break;
        case MOTOR_VAR_PID_CURRENT_KP:              value = p_motor->PidI.Kp_Fixed32;     break;
        case MOTOR_VAR_PID_CURRENT_KI:              value = p_motor->PidI.Ki_Fixed32;     break;
        default: break;
    }
    return value;
}

void _Motor_Var_ConfigPid_Set(Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:       break;
        case MOTOR_VAR_PID_SPEED_KP:             p_motor->PidSpeed.Kp_Fixed32 = varValue;            break;
        case MOTOR_VAR_PID_SPEED_KI:             p_motor->PidSpeed.Ki_Fixed32 = varValue;            break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:     break;
        case MOTOR_VAR_PID_CURRENT_KP:           p_motor->PidI.Kp_Fixed32 = varValue;            break;
        case MOTOR_VAR_PID_CURRENT_KI:           p_motor->PidI.Ki_Fixed32 = varValue;            break;
        default: break;
    }
}


/*
    Coefficients in 9.7
*/
int _Motor_Var_ConfigPid16_Get(const Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId)
{
    int value = 0;
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:       value = _PID_GetSampleFreq(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KP:                value = _PID_GetKp_Fixed16(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_SPEED_KI:                value = _PID_GetKi_Fixed16(&p_motor->PidSpeed);   break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ:     value = _PID_GetSampleFreq(&p_motor->PidI);     break;
        case MOTOR_VAR_PID_CURRENT_KP:              value = _PID_GetKp_Fixed16(&p_motor->PidI);     break;
        case MOTOR_VAR_PID_CURRENT_KI:              value = _PID_GetKi_Fixed16(&p_motor->PidI);     break;
        default: break;
    }
    return value;
}

void _Motor_Var_ConfigPid16_Set(Motor_Config_T * p_motor, Motor_Var_ConfigPid_T varId, int varValue)
{
    switch (varId)
    {
        case MOTOR_VAR_PID_SPEED_SAMPLE_FREQ:   break;
        case MOTOR_VAR_PID_SPEED_KP:            _PID_SetKp_Fixed16(&p_motor->PidSpeed, varValue);            break;
        case MOTOR_VAR_PID_SPEED_KI:            _PID_SetKi_Fixed16(&p_motor->PidSpeed, varValue);            break;
        case MOTOR_VAR_PID_CURRENT_SAMPLE_FREQ: break;
        case MOTOR_VAR_PID_CURRENT_KP:            _PID_SetKp_Fixed16(&p_motor->PidI, varValue);            break;
        case MOTOR_VAR_PID_CURRENT_KI:            _PID_SetKi_Fixed16(&p_motor->PidI, varValue);            break;
        default: break;
    }
}

/******************************************************************************/
/* Local Unit Conversion */
/******************************************************************************/
#ifdef MOTOR_UNIT_CONVERSION_LOCAL
void Motor_Config_SetSpeedLimitForward_Rpm(Motor_Config_T * p_motor, uint16_t forward_Rpm)
{
    Motor_Config_SetSpeedLimitForward_Pu(p_motor, ConvertToSpeedLimitPu(p_motor, forward_Rpm));
}

void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_Config_T * p_motor, uint16_t reverse_Rpm)
{
    Motor_Config_SetSpeedLimitReverse_Pu(p_motor, ConvertToSpeedLimitPu(p_motor, reverse_Rpm));
}

uint16_t Motor_Config_GetSpeedLimitForward_Rpm(Motor_Config_T * p_motor)
{
    return _Motor_ConvertSpeed_PuToRpm(p_motor, p_motor->Config.SpeedLimitForward_Pu);
}

uint16_t Motor_Config_GetSpeedLimitReverse_Rpm(Motor_Config_T * p_motor)
{
    return _Motor_ConvertSpeed_PuToRpm(p_motor, p_motor->Config.SpeedLimitReverse_Pu);
}
void Motor_Config_SetILimit_Amp(Motor_Config_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
    Motor_Config_SetILimit_Pu(p_motor, ConvertToILimitPu(p_motor, motoring_Amp), ConvertToILimitPu(p_motor, generating_Amp));
}
void Motor_Config_SetILimitMotoring_Amp(Motor_Config_T * p_motor, uint16_t motoring_Amp)
{
    Motor_Config_SetILimitMotoring_Pu(p_motor, ConvertToILimitPu(p_motor, motoring_Amp));
}

void Motor_Config_SetILimitGenerating_Amp(Motor_Config_T * p_motor, uint16_t generating_Amp)
{
    Motor_Config_SetILimitGenerating_Pu(p_motor, ConvertToILimitPu(p_motor, generating_Amp));
}
uint16_t Motor_Config_GetILimitMotoring_Amp(Motor_Config_T * p_motor) { return _Motor_ConvertI_PuToAmp(p_motor->Config.ILimitMotoring_Pu); }
uint16_t Motor_Config_GetILimitGenerating_Amp(Motor_Config_T * p_motor) { return _Motor_ConvertI_PuToAmp(p_motor->Config.ILimitGenerating_Pu); }
#endif