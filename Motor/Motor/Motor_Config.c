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

/******************************************************************************/
/*
    Nvm Config, check Stop State
    combine with VarConfig or UserConfig
*/
/******************************************************************************/
static inline void PropagateSet(Motor_State_T * p_motor, Motor_Proc_T reset)
{
#ifdef MOTOR_CONFIG_PROPAGATE_SET_ENABLE
    reset(p_motor);
#else
    (void)p_motor;
    (void)reset;
#endif
}


/* local limit */
static inline uint16_t _Motor_Config_GetOpenLoopScalarLimit(const Motor_State_T * p_motor) { return math_min(p_motor->Config.OpenLoopLimitScalar_Fract16, MOTOR_OPEN_LOOP_MAX_SCALAR); }

static inline uint16_t _Motor_Config_GetOpenLoopILimit_Fract16(const Motor_State_T * p_motor) { return fract16_mul(_Motor_Config_GetOpenLoopScalarLimit(p_motor), Phase_Calibration_GetIRatedPeak_Fract16()); }
static inline uint16_t _Motor_Config_GetOpenLoopVLimit_Fract16(const Motor_State_T * p_motor) { return fract16_mul(_Motor_Config_GetOpenLoopScalarLimit(p_motor), Phase_Calibration_GetVRated_Fract16()); }

static inline uint16_t Motor_Config_OpenLoopILimitOf(const Motor_State_T * p_motor, uint16_t fract16) { return math_min(fract16, _Motor_Config_GetOpenLoopILimit_Fract16(p_motor)); }

/* Rated Limit - applied on set config */
static inline uint16_t Motor_IRatedLimitOf(uint16_t i_fract16) { return math_min(Phase_Calibration_GetIRatedPeak_Fract16(), i_fract16); }
static inline uint16_t Motor_VRatedLimitOf(uint16_t v_fract16) { return math_min(Phase_Calibration_GetVRated_Fract16(), v_fract16); }

/*
    Config Limit Bounds
    the setting limit by TypeMax, SpeedRated ~VBusRef
    When SpeedTypeMax is dynamically scaled, INT16_MAX is 2x RatedSpeed
*/
static inline uint16_t _Motor_Config_SpeedRatedLimit(const Motor_State_T * p_motor) { return INT16_MAX; }
static inline uint16_t _Motor_Config_SpeedRatedLimitOf(const Motor_State_T * p_motor, uint16_t speed_fract16) { return math_min(_Motor_Config_SpeedRatedLimit(p_motor), speed_fract16); }

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
/* Wrapper for Propagate Set */
void Motor_ReinitSensor(Motor_State_T * p_motor) { RotorSensor_Init(p_motor->p_ActiveSensor); }

/* Reboot unless deinit is implemented in HAL */
void Motor_Config_SetSensorMode(Motor_State_T * p_motor, RotorSensor_Id_T mode)
{
    p_motor->Config.SensorMode = mode;
    PropagateSet(p_motor, Motor_ReinitSensor);
}

void Motor_Config_SetPolePairs(Motor_State_T * p_motor, uint8_t polePairs)
{
    p_motor->Config.PolePairs = polePairs;
    PropagateSet(p_motor, Motor_ResetUnits);
}

/* Setting Kv overwrites SpeedRefs. SpeedRefs can be set independently from Kv or lock */
// p_motor->Config.SpeedFeedback_Rpm = kv * Phase_VBus_Volts();
void Motor_Config_SetKv(Motor_State_T * p_motor, uint16_t kv)
{
    p_motor->Config.Kv = kv;
    p_motor->Config.SpeedRated_Rpm = Motor_GetSpeedVBusRef_Rpm(p_motor);
    PropagateSet(p_motor, Motor_ResetUnits);
}

/* allow independent set */
// p_motor->Config.Kv = rpm / Phase_VBus_Volts();
void Motor_Config_SetSpeedRated(Motor_State_T * p_motor, uint16_t rpm)
{
    p_motor->Config.SpeedRated_Rpm = math_min(rpm, Motor_GetSpeedVBusRef_Rpm(p_motor) * 2);
    PropagateSet(p_motor, Motor_ResetUnits);
}

/*
    V of Speed Ref
    SpeedVRef =< SpeedFeedbackRef to ensure not match to higher speed
    [0:32767] unitless Fract16
*/
void Motor_Config_SetVSpeedScalar_UFract16(Motor_State_T * p_motor, uint16_t scalar) { p_motor->Config.VSpeedScalar_Fract16 = math_min(scalar, INT16_MAX); }
// void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_State_T * p_motor, uint16_t rpm)
// {
//     Motor_Config_SetVSpeedScalar_UFract16(p_motor, ((uint32_t)rpm << 15U) / Motor_GetSpeedVRef_Rpm(p_motor));
// }

#ifdef MOTOR_SIX_STEP_ENABLE
static inline void Motor_Config_SetPhaseMode(Motor_State_T * p_motor, Phase_Polar_Mode_T mode) { p_motor->Config.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->PHASE, mode); }
#endif


/******************************************************************************/
/*
    I Sensor Ref
*/
/******************************************************************************/
void Motor_Config_SetIaZero_Adcu(Motor_State_T * p_motor, uint16_t adcu) { p_motor->Config.IabcZeroRef_Adcu.A = adcu; /* PropagateSet(p_motor, Motor_ResetUnitsIa); */ }
void Motor_Config_SetIbZero_Adcu(Motor_State_T * p_motor, uint16_t adcu) { p_motor->Config.IabcZeroRef_Adcu.B = adcu; /* PropagateSet(p_motor, Motor_ResetUnitsIb); */ }
void Motor_Config_SetIcZero_Adcu(Motor_State_T * p_motor, uint16_t adcu) { p_motor->Config.IabcZeroRef_Adcu.C = adcu; /* PropagateSet(p_motor, Motor_ResetUnitsIc); */ }


/******************************************************************************/
/*
    Persistent Base Limits / Ramps
*/
/******************************************************************************/
/*
    Persistent Base SpeedLimit
*/
void Motor_Config_SetSpeedLimitForward_Fract16(Motor_State_T * p_motor, uint16_t forward_Fract16)
{
    p_motor->Config.SpeedLimitForward_Fract16 = _Motor_Config_SpeedRatedLimitOf(p_motor, forward_Fract16);
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Config_SetSpeedLimitReverse_Fract16(Motor_State_T * p_motor, uint16_t reverse_Fract16)
{
    p_motor->Config.SpeedLimitReverse_Fract16 = _Motor_Config_SpeedRatedLimitOf(p_motor, reverse_Fract16);
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}


/*
    Persistent Base ILimit
*/
void Motor_Config_SetILimitMotoring_Fract16(Motor_State_T * p_motor, uint16_t motoring_Fract16)
{
    p_motor->Config.ILimitMotoring_Fract16 = Motor_IRatedLimitOf(motoring_Fract16);
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

void Motor_Config_SetILimitGenerating_Fract16(Motor_State_T * p_motor, uint16_t generating_Fract16)
{
    p_motor->Config.ILimitGenerating_Fract16 = Motor_IRatedLimitOf(generating_Fract16);
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

/*
    Ramps
*/
void Motor_Config_SetSpeedRampTime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.SpeedRampTime_Cycles = cycles;
    PropagateSet(p_motor, Motor_ResetSpeedRamp);
}

void Motor_Config_SetSpeedRampTime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetSpeedRampTime_Cycles(p_motor, MOTOR_SPEED_CYCLES(millis));
}

void Motor_Config_SetTorqueRampTime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.TorqueRampTime_Cycles = cycles;
    PropagateSet(p_motor, Motor_ResetTorqueRamp);
}

void Motor_Config_SetTorqueRampTime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetTorqueRampTime_Cycles(p_motor, MOTOR_TORQUE_CYCLES(millis));
}


/******************************************************************************/
/*

*/
/******************************************************************************/
void Motor_ResetBaseOpenLoopILimit(Motor_State_T * p_motor)
{
    p_motor->Config.OpenLoopLimitScalar_Fract16 = math_min(p_motor->Config.OpenLoopLimitScalar_Fract16, MOTOR_OPEN_LOOP_MAX_SCALAR);
    p_motor->Config.AlignScalar_Fract16 = math_min(p_motor->Config.AlignScalar_Fract16, MOTOR_OPEN_LOOP_MAX_SCALAR);
    p_motor->Config.OpenLoopRampIFinal_Fract16 = Motor_OpenLoopILimitOf(p_motor, p_motor->Config.OpenLoopRampIFinal_Fract16);
}



/*  */
void Motor_Config_SetOpenLoopScalarLimit(Motor_State_T * p_motor, uint16_t scalar16)
{
    p_motor->Config.OpenLoopLimitScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);
    p_motor->Config.AlignScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);;
    p_motor->Config.OpenLoopRampIFinal_Fract16 = Motor_Config_OpenLoopILimitOf(p_motor, p_motor->Config.OpenLoopRampIFinal_Fract16);
}


/* */
void Motor_Config_SetAlignPowerScalar(Motor_State_T * p_motor, uint16_t scalar16) { p_motor->Config.AlignScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);; }

void Motor_Config_SetAlignTime_Cycles(Motor_State_T * p_motor, uint32_t cycles) { p_motor->Config.AlignTime_Cycles = cycles; }
void Motor_Config_SetAlignTime_Millis(Motor_State_T * p_motor, uint16_t millis) { p_motor->Config.AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }


/*
*/
void Motor_Config_SetOpenLoopRampIFinal_Fract16(Motor_State_T * p_motor, uint16_t i_fract16)
{
    p_motor->Config.OpenLoopRampIFinal_Fract16 = Motor_Config_OpenLoopILimitOf(p_motor, i_fract16);
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampITime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.OpenLoopRampITime_Cycles = cycles;
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampITime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetOpenLoopRampITime_Cycles(p_motor, _Motor_ControlCyclesOf(millis));
}

/*
    OpenLoop Ramp tick on control timer
*/
void Motor_Config_SetOpenLoopRampSpeedFinal_Fract16(Motor_State_T * p_motor, uint16_t speed_fract16)
{
    p_motor->Config.OpenLoopRampSpeedFinal_Fract16 = speed_fract16; /* optionally limit */
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampSpeedTime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.OpenLoopRampSpeedTime_Cycles = cycles;
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampSpeedTime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetOpenLoopRampSpeedTime_Cycles(p_motor, _Motor_ControlCyclesOf(millis));
}



/******************************************************************************/
/* Local Unit Conversion */
/******************************************************************************/
// #ifdef MOTOR_UNIT_CONVERSION_LOCAL
// static uint16_t ConvertToSpeedLimitPercent16(Motor_State_T * p_motor, uint16_t speed_rpm)
// {
//     int32_t speed_fract16 = _Motor_ConvertSpeed_RpmToPercent16(p_motor, speed_rpm);
//     return (speed_fract16 > UINT16_MAX) ? UINT16_MAX : speed_fract16;
// }

// void Motor_Config_SetSpeedLimit_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
// {
//     Motor_Config_SetSpeedLimit_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, forward_Rpm), ConvertToSpeedLimitPercent16(p_motor, reverse_Rpm));
// }

// void Motor_Config_SetSpeedLimitForward_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm)
// {
//     Motor_Config_SetSpeedLimitForward_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, forward_Rpm));
// }

// void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_State_T * p_motor, uint16_t reverse_Rpm)
// {
//     Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, reverse_Rpm));
// }

// uint16_t Motor_Config_GetSpeedLimitForward_Rpm(Motor_State_T * p_motor)
// {
//     return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, p_motor->Config.SpeedLimitForward_Fract16);
//     // uint16_t speedLimit = (p_motor->Config.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
//     //     p_motor->Config.SpeedLimitCcw_Fract16 : p_motor->Config.SpeedLimitCw_Fract16;
//     // return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, speedLimit);
// }

// uint16_t Motor_Config_GetSpeedLimitReverse_Rpm(Motor_State_T * p_motor)
// {
//     return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, p_motor->Config.SpeedLimitReverse_Fract16);
//     // uint16_t speedLimit = (p_motor->Config.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
//     //  p_motor->Config.SpeedLimitCw_Fract16 : p_motor->Config.SpeedLimitCcw_Fract16;
// }

// static uint16_t ConvertToILimitPercent16(Motor_State_T * p_motor, uint16_t i_amp)
// {
//     int32_t i_fract16 = _Motor_ConvertI_AmpToPercent16(i_amp);
//     return (i_fract16 > UINT16_MAX) ? UINT16_MAX : i_fract16;
// }

// void Motor_Config_SetILimit_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
// {
//     Motor_Config_SetILimit_Fract16(p_motor, ConvertToILimitPercent16(p_motor, motoring_Amp), ConvertToILimitPercent16(p_motor, generating_Amp));
// }

// void Motor_Config_SetILimitMotoring_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp)
// {
//     Motor_Config_SetILimitMotoring_Fract16(p_motor, ConvertToILimitPercent16(p_motor, motoring_Amp));
// }

// void Motor_Config_SetILimitGenerating_Amp(Motor_State_T * p_motor, uint16_t generating_Amp)
// {
//     Motor_Config_SetILimitGenerating_Fract16(p_motor, ConvertToILimitPercent16(p_motor, generating_Amp));
// }

// uint16_t Motor_Config_GetILimitMotoring_Amp(Motor_State_T * p_motor) { return _Motor_ConvertI_Percent16ToAmp(p_motor->Config.ILimitMotoring_Fract16); }
// uint16_t Motor_Config_GetILimitGenerating_Amp(Motor_State_T * p_motor) { return _Motor_ConvertI_Percent16ToAmp(p_motor->Config.ILimitGenerating_Fract16); }
// #endif