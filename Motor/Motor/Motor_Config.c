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
    @version V0

    @brief @see Motor_Config.h
*/
/******************************************************************************/
#include "Motor_Config.h"

/******************************************************************************/
/*
    Nvm Config, check StopState
*/
/******************************************************************************/
/*  */
typedef void(*Motor_PropagateSet_T)(Motor_T * p_motor);

static inline void PropagateSet(Motor_T * p_motor, Motor_PropagateSet_T reset)
{
#ifdef CONFIG_MOTOR_CONFIG_PROPAGATE_SET_ENABLE
    reset(p_motor);
#else
    (void)p_motor;
    (void)reset;
#endif
}

// functions as full width variable to preserve calling convention, alternatively caller handle check set state
// typedef void(*Motor_SetInt_T)(Motor_T * p_motor, int32_t value);
// typedef int32_t(*Motor_GetInt_T)(const Motor_T * p_motor);



/******************************************************************************/
/*
    Nvm Reference/Calibration
    Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/
/* Reboot unless deinit is implemented in HAL */
void Motor_Config_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode) { p_motor->Config.SensorMode = mode; PropagateSet(p_motor, Motor_InitSensor); }
void Motor_Config_SetPolePairs(Motor_T * p_motor, uint8_t polePairs) { p_motor->Config.PolePairs = polePairs; PropagateSet(p_motor, Motor_ResetUnitsSensor); }

/* Setting Kv overwrites SpeedRefs. SpeedRefs can be set independently from Kv or lock */
void Motor_Config_SetKv(Motor_T * p_motor, uint16_t kv)
{
    p_motor->Config.Kv = kv;
    // p_motor->Config.SpeedFeedback_Rpm = kv * Motor_Static_GetVSource_V();
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

/*
    V of Speed Ref
    SpeedVRef =< SetSpeedFeedbackRef to ensure not match to higher speed
    Base as UFract16, multiply with SpeedRef*2 without overflow.
*/
void Motor_Config_SetVSpeedScalar_UFract16(Motor_T * p_motor, uint16_t scalar)
{
    p_motor->Config.VSpeedScalar_UFract16 = math_min(scalar, INT16_MAX);
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

// void Motor_Config_CalibrateEncoderHomeOffset(Motor_T * p_motor)
// {
//     Encoder_CalibrateIndexZeroRef(&p_motor->Encoder);
//     // PropagateSet(p_motor, );
// }

/******************************************************************************/
/*  */
/******************************************************************************/
/* Kv * VSource */
/* SpeedFeedbackRef_Rpm */
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
void Motor_Config_SetSpeedVRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
    // p_motor->Config.SpeedFeedback_Rpm = rpm;
    p_motor->Config.Kv = rpm / Motor_Static_GetVSource_V();
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_T * p_motor, uint16_t rpm)
{
    Motor_Config_SetVSpeedScalar_UFract16(p_motor, ((uint32_t)rpm << 15U) / Motor_GetSpeedVRef_Rpm(p_motor));
}
#endif

/******************************************************************************/
/* ISensorRef */
/******************************************************************************/
void Motor_Config_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Config.IaZeroRef_Adcu = adcu; PropagateSet(p_motor, Motor_ResetUnitsIa); }
void Motor_Config_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Config.IbZeroRef_Adcu = adcu; PropagateSet(p_motor, Motor_ResetUnitsIb); }
void Motor_Config_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu) { p_motor->Config.IcZeroRef_Adcu = adcu; PropagateSet(p_motor, Motor_ResetUnitsIc); }

void Motor_Config_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    p_motor->Config.IPeakRef_Adcu = adcu;
    Motor_ResetUnitsIabc(p_motor);
#else
    p_motor->Config.IPeakRef_Adcu = (adcu > MOTOR_STATIC.I_PEAK_ADCU) ? MOTOR_STATIC.I_PEAK_ADCU : adcu;
    PropagateSet(p_motor, Motor_ResetUnitsIabc);
#endif
}

void Motor_Config_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
    uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * GLOBAL_ANALOG.ADC_MAX / 2U / GLOBAL_ANALOG.ADC_VREF_MILLIV;
    uint16_t adcuMax = (uint32_t)max_MilliV * GLOBAL_ANALOG.ADC_MAX / GLOBAL_ANALOG.ADC_VREF_MILLIV;
    Motor_Config_SetIPeakRef_Adcu(p_motor, adcuMax - adcuZero);
}

void Motor_Config_SetDirectionCalibration(Motor_T * p_motor, Motor_Direction_T directionForward)
{
    p_motor->Config.DirectionForward = directionForward;
    PropagateSet(p_motor, Motor_SetDirectionForward);
}

/******************************************************************************/
/*
    Persistent Base Limits
*/
/******************************************************************************/
/*
    Persistent Base SpeedLimit
*/
void Motor_Config_SetSpeedLimitForward_Fract16(Motor_T * p_motor, uint16_t forward_Fract16)
{
    p_motor->Config.SpeedLimitForward_Fract16 = forward_Fract16;
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Config_SetSpeedLimitReverse_Fract16(Motor_T * p_motor, uint16_t reverse_Fract16)
{
    p_motor->Config.SpeedLimitReverse_Fract16 = reverse_Fract16;
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Config_SetSpeedLimit_Fract16(Motor_T * p_motor, uint16_t forward_Fract16, uint16_t reverse_Fract16)
{
    p_motor->Config.SpeedLimitForward_Fract16 = forward_Fract16;
    p_motor->Config.SpeedLimitReverse_Fract16 = reverse_Fract16;
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

/*
    Persistent Base ILimit
*/
void Motor_Config_SetILimitMotoring_Fract16(Motor_T * p_motor, uint16_t motoring_Fract16)
{
    p_motor->Config.ILimitMotoring_Fract16 = motoring_Fract16;
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

void Motor_Config_SetILimitGenerating_Fract16(Motor_T * p_motor, uint16_t generating_Fract16)
{
    p_motor->Config.ILimitGenerating_Fract16 = generating_Fract16;
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

void Motor_Config_SetILimit_Fract16(Motor_T * p_motor, uint16_t motoring_Fract16, uint16_t generating_Fract16)
{
    p_motor->Config.ILimitMotoring_Fract16 = motoring_Fract16;
    p_motor->Config.ILimitGenerating_Fract16 = generating_Fract16;
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

#ifdef CONFIG_MOTOR_SIX_STEP_ENABLE
static inline void Motor_Config_SetPhaseMode(Motor_T * p_motor, Phase_Mode_T mode) { p_motor->Config.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }
#endif

/******************************************************************************/
/* Local Unit Conversion */
/******************************************************************************/
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToSpeedLimitPercent16(Motor_T * p_motor, uint16_t speed_rpm)
{
    int32_t speed_fract16 = _Motor_ConvertSpeed_RpmToPercent16(p_motor, speed_rpm);
    return (speed_fract16 > UINT16_MAX) ? UINT16_MAX : speed_fract16;
}

void Motor_Config_SetSpeedLimit_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
    Motor_Config_SetSpeedLimit_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, forward_Rpm), ConvertToSpeedLimitPercent16(p_motor, reverse_Rpm));
}

void Motor_Config_SetSpeedLimitForward_Rpm(Motor_T * p_motor, uint16_t forward_Rpm)
{
    Motor_Config_SetSpeedLimitForward_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, forward_Rpm));
}

void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_T * p_motor, uint16_t reverse_Rpm)
{
    Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, reverse_Rpm));
}

uint16_t Motor_Config_GetSpeedLimitForward_Rpm(Motor_T * p_motor)
{
    return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, p_motor->Config.SpeedLimitForward_Fract16);
    // uint16_t speedLimit = (p_motor->Config.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
    //     p_motor->Config.SpeedLimitCcw_Fract16 : p_motor->Config.SpeedLimitCw_Fract16;
    // return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, speedLimit);
}

uint16_t Motor_Config_GetSpeedLimitReverse_Rpm(Motor_T * p_motor)
{
    return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, p_motor->Config.SpeedLimitReverse_Fract16);
    // uint16_t speedLimit = (p_motor->Config.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
    //  p_motor->Config.SpeedLimitCw_Fract16 : p_motor->Config.SpeedLimitCcw_Fract16;
}

static uint16_t ConvertToILimitPercent16(Motor_T * p_motor, uint16_t i_amp)
{
    int32_t i_fract16 = _Motor_ConvertI_AmpToPercent16(i_amp);
    return (i_fract16 > UINT16_MAX) ? UINT16_MAX : i_fract16;
}

void Motor_Config_SetILimit_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
    Motor_Config_SetILimit_Fract16(p_motor, ConvertToILimitPercent16(p_motor, motoring_Amp), ConvertToILimitPercent16(p_motor, generating_Amp));
}

void Motor_Config_SetILimitMotoring_Amp(Motor_T * p_motor, uint16_t motoring_Amp)
{
    Motor_Config_SetILimitMotoring_Fract16(p_motor, ConvertToILimitPercent16(p_motor, motoring_Amp));
}

void Motor_Config_SetILimitGenerating_Amp(Motor_T * p_motor, uint16_t generating_Amp)
{
    Motor_Config_SetILimitGenerating_Fract16(p_motor, ConvertToILimitPercent16(p_motor, generating_Amp));
}

uint16_t Motor_Config_GetILimitMotoring_Amp(Motor_T * p_motor) { return _Motor_ConvertI_Percent16ToAmp(p_motor->Config.ILimitMotoring_Fract16); }
uint16_t Motor_Config_GetILimitGenerating_Amp(Motor_T * p_motor) { return _Motor_ConvertI_Percent16ToAmp(p_motor->Config.ILimitGenerating_Fract16); }
#endif