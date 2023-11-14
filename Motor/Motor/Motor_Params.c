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
    @file   Motor_Params.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Motor_Params.h"

/******************************************************************************/
/*
    Nvm Parameters, check StopState
*/
/******************************************************************************/
/*  */
typedef void(*Motor_PropagateSet_T)(MotorPtr_T p_motor);

static inline void PropagateSet(MotorPtr_T p_motor, Motor_PropagateSet_T reset)
{
#ifdef CONFIG_MOTOR_PARAMS_PROPAGATE_SET_ENABLE
    reset(p_motor);
#else
    (void)p_motor;
#endif
}

/******************************************************************************/
/*
    Nvm Param Persistent Limits
*/
/******************************************************************************/
/*
    Persistent SpeedLimit - effective Speed Feedback Mode only
*/
void Motor_Params_SetSpeedLimitForward_Scalar16(MotorPtr_T p_motor, uint16_t forward_Frac16)
{
    p_motor->Parameters.SpeedLimitForward_Scalar16 = forward_Frac16;
    // if(p_motor->Parameters.DirectionForward == MOTOR_FORWARD_IS_CCW)     { p_motor->Parameters.SpeedLimitCcw_FracS16 = forward_Frac16; }
    // else                                                                 { p_motor->Parameters.SpeedLimitCw_FracS16 = forward_Frac16; }
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Params_SetSpeedLimitReverse_Scalar16(MotorPtr_T p_motor, uint16_t reverse_Frac16)
{
    p_motor->Parameters.SpeedLimitReverse_Scalar16 = reverse_Frac16;
    // if(p_motor->Parameters.DirectionForward == MOTOR_FORWARD_IS_CCW)     { p_motor->Parameters.SpeedLimitCw_FracS16 = reverse_Frac16; }
    // else                                                                 { p_motor->Parameters.SpeedLimitCcw_FracS16 = reverse_Frac16; }
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Params_SetSpeedLimit_Scalar16(MotorPtr_T p_motor, uint16_t forwardScalar16, uint16_t reverseScalar16)
{
    p_motor->Parameters.SpeedLimitForward_Scalar16 = forwardScalar16;
    p_motor->Parameters.SpeedLimitReverse_Scalar16 = reverseScalar16;
    // if(p_motor->Parameters.DirectionForward == MOTOR_FORWARD_IS_CCW)
    // {
    //     p_motor->Parameters.SpeedLimitCcw_FracS16 = forward_Frac16;
    //     p_motor->Parameters.SpeedLimitCw_FracS16 = reverse_Frac16;
    // }
    // else
    // {
    //     p_motor->Parameters.SpeedLimitCcw_FracS16 = reverse_Frac16;
    //     p_motor->Parameters.SpeedLimitCw_FracS16 = forward_Frac16;
    // }
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

/*
    Persistent ILimit
*/
void Motor_Params_SetILimitMotoring_Scalar16(MotorPtr_T p_motor, uint16_t motoring_Frac16)
{
    p_motor->Parameters.ILimitMotoring_Scalar16 = motoring_Frac16;
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

void Motor_Params_SetILimitGenerating_Scalar16(MotorPtr_T p_motor, uint16_t generating_Frac16)
{
    p_motor->Parameters.ILimitGenerating_Scalar16 = generating_Frac16;
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

void Motor_Params_SetILimit_Scalar16(MotorPtr_T p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16)
{
    p_motor->Parameters.ILimitMotoring_Scalar16 = motoring_Frac16;
    p_motor->Parameters.ILimitGenerating_Scalar16 = generating_Frac16;
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

/******************************************************************************/
/* Local Unit Conversion */
/******************************************************************************/
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static uint16_t ConvertToSpeedLimitScalar16(MotorPtr_T p_motor, uint16_t speed_rpm)
{
    int32_t speed_frac16 = _Motor_ConvertSpeed_RpmToScalar16(p_motor, speed_rpm);
    return (speed_frac16 > UINT16_MAX) ? UINT16_MAX : speed_frac16;
}

void Motor_Params_SetSpeedLimit_Rpm(MotorPtr_T p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
    Motor_Params_SetSpeedLimit_Scalar16(p_motor, ConvertToSpeedLimitScalar16(p_motor, forward_Rpm), ConvertToSpeedLimitScalar16(p_motor, reverse_Rpm));
}

void Motor_Params_SetSpeedLimitForward_Rpm(MotorPtr_T p_motor, uint16_t forward_Rpm)
{
    Motor_Params_SetSpeedLimitForward_Scalar16(p_motor, ConvertToSpeedLimitScalar16(p_motor, forward_Rpm));
}

void Motor_Params_SetSpeedLimitReverse_Rpm(MotorPtr_T p_motor, uint16_t reverse_Rpm)
{
    Motor_Params_SetSpeedLimitReverse_Scalar16(p_motor, ConvertToSpeedLimitScalar16(p_motor, reverse_Rpm));
}

uint16_t Motor_Params_GetSpeedLimitForward_Rpm(MotorPtr_T p_motor)
{
    return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, p_motor->Parameters.SpeedLimitForward_Scalar16);
    // uint16_t speedLimit = (p_motor->Parameters.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
    //     p_motor->Parameters.SpeedLimitCcw_FracS16 : p_motor->Parameters.SpeedLimitCw_FracS16;
    // return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, speedLimit);
}

uint16_t Motor_Params_GetSpeedLimitReverse_Rpm(MotorPtr_T p_motor)
{
    return _Motor_ConvertSpeed_Scalar16ToRpm(p_motor, p_motor->Parameters.SpeedLimitReverse_Scalar16);
    // uint16_t speedLimit = (p_motor->Parameters.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
    //  p_motor->Parameters.SpeedLimitCw_FracS16 : p_motor->Parameters.SpeedLimitCcw_FracS16;
}

static uint16_t ConvertToILimitScalar16(MotorPtr_T p_motor, uint16_t i_amp)
{
    int32_t i_frac16 = _Motor_ConvertI_AmpToScalar16(i_amp);
    return (i_frac16 > UINT16_MAX) ? UINT16_MAX : i_frac16;
}

void Motor_Params_SetILimit_Amp(MotorPtr_T p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
    Motor_Params_SetILimit_Scalar16(p_motor, ConvertToILimitScalar16(p_motor, motoring_Amp), ConvertToILimitScalar16(p_motor, generating_Amp));
}

void Motor_Params_SetILimitMotoring_Amp(MotorPtr_T p_motor, uint16_t motoring_Amp)
{
    Motor_Params_SetILimitMotoring_Scalar16(p_motor, ConvertToILimitScalar16(p_motor, motoring_Amp));
}

void Motor_Params_SetILimitGenerating_Amp(MotorPtr_T p_motor, uint16_t generating_Amp)
{
    Motor_Params_SetILimitGenerating_Scalar16(p_motor, ConvertToILimitScalar16(p_motor, generating_Amp));
}

uint16_t Motor_Params_GetILimitMotoring_Amp(MotorPtr_T p_motor) { return _Motor_ConvertI_Scalar16ToAmp(p_motor->Parameters.ILimitMotoring_Scalar16); }
uint16_t Motor_Params_GetILimitGenerating_Amp(MotorPtr_T p_motor) { return _Motor_ConvertI_Scalar16ToAmp(p_motor->Parameters.ILimitGenerating_Scalar16); }
#endif

/******************************************************************************/
/*
    Nvm Reference/Calibration
    Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/
/* SpeedFeedbackRef_Rpm => 100% speed for PID feedback. */
void Motor_Params_SetSpeedFeedbackRef_Rpm(MotorPtr_T p_motor, uint16_t rpm)
{
    p_motor->Parameters.SpeedFeedbackRef_Rpm = rpm;
    // if(rpm > kv * Global_Motor_GetVSource_V())
    if(p_motor->Parameters.VSpeedRef_Rpm > rpm) { p_motor->Parameters.VSpeedRef_Rpm = rpm; }
    PropagateSet(p_motor, Motor_ResetUnitsVSpeed);
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

/* SpeedVRef =< SetSpeedFeedbackRef to ensure not match to higher speed */
void Motor_Params_SetVSpeedRef_Rpm(MotorPtr_T p_motor, uint16_t rpm)
{
    p_motor->Parameters.VSpeedRef_Rpm = (rpm > p_motor->Parameters.SpeedFeedbackRef_Rpm) ? p_motor->Parameters.SpeedFeedbackRef_Rpm : rpm;
    PropagateSet(p_motor, Motor_ResetUnitsVSpeed);
    PropagateSet(p_motor, Motor_ResetUnitsSensor);
}

static void SetSpeedFeedbackRef_Kv(MotorPtr_T p_motor, uint16_t kv)
{
    uint16_t rpm = kv * Global_Motor_GetVSource_V();
    if((p_motor->Parameters.SpeedFeedbackRef_Rpm == 0U) || rpm < p_motor->Parameters.SpeedFeedbackRef_Rpm)
    {
        Motor_Params_SetSpeedFeedbackRef_Rpm(p_motor, rpm);
    }
}

static void SetVSpeedRef_Kv(MotorPtr_T p_motor, uint16_t kv)
{
    Motor_Params_SetVSpeedRef_Rpm(p_motor, kv * Global_Motor_GetVSource_V());
}

/* Setting Kv overwrites higher SpeedFeedbackRef. SpeedFeedbackRef can be set independently from Kv */
void Motor_Params_SetKv(MotorPtr_T p_motor, uint16_t kv)
{
    p_motor->Parameters.Kv = kv;
    SetSpeedFeedbackRef_Kv(p_motor, kv);
    SetVSpeedRef_Kv(p_motor, kv);
}

void Motor_Params_SetIaZero_Adcu(MotorPtr_T p_motor, uint16_t adcu) { p_motor->Parameters.IaZeroRef_Adcu = adcu; PropagateSet(p_motor, Motor_ResetUnitsIa); }
void Motor_Params_SetIbZero_Adcu(MotorPtr_T p_motor, uint16_t adcu) { p_motor->Parameters.IbZeroRef_Adcu = adcu; PropagateSet(p_motor, Motor_ResetUnitsIb); }
void Motor_Params_SetIcZero_Adcu(MotorPtr_T p_motor, uint16_t adcu) { p_motor->Parameters.IcZeroRef_Adcu = adcu; PropagateSet(p_motor, Motor_ResetUnitsIc); }

void Motor_Params_SetDirectionCalibration(MotorPtr_T p_motor, Motor_Direction_T directionForward)
{
    p_motor->Parameters.DirectionForward = directionForward;
    PropagateSet(p_motor, Motor_SetDirectionForward);
}

void Motor_Params_SetPolePairs(MotorPtr_T p_motor, uint8_t polePairs) { p_motor->Parameters.PolePairs = polePairs; PropagateSet(p_motor, Motor_ResetUnitsSensor); }

/* Reboot unless deinit is implemented in HAL */
void Motor_Params_SetSensorMode(MotorPtr_T p_motor, Motor_SensorMode_T mode) { p_motor->Parameters.SensorMode = mode; PropagateSet(p_motor, Motor_InitSensor); }

/******************************************************************************/
/* IPeak  */
/******************************************************************************/
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
// void Motor_Params_SetIPeakRef_Adcu_Debug(MotorPtr_T p_motor, uint16_t adcu)
// {
//     p_motor->Parameters.IPeakRef_Adcu = adcu;
//     Motor_ResetUnitsIabc(p_motor);
// }

void Motor_Params_SetIPeakRef_Adcu(MotorPtr_T p_motor, uint16_t adcu)
{
    p_motor->Parameters.IPeakRef_Adcu = (adcu > GLOBAL_MOTOR.I_MAX_ADCU) ? GLOBAL_MOTOR.I_MAX_ADCU : adcu;
    PropagateSet(p_motor, Motor_ResetUnitsIabc);
}

void Motor_Params_SetIPeakRef_MilliV(MotorPtr_T p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
    uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * GLOBAL_ANALOG.ADC_MAX / 2U / GLOBAL_ANALOG.ADC_VREF_MILLIV;
    uint16_t adcuMax = (uint32_t)max_MilliV * GLOBAL_ANALOG.ADC_MAX / GLOBAL_ANALOG.ADC_VREF_MILLIV;
    Motor_Params_SetIPeakRef_Adcu(p_motor, adcuMax - adcuZero);
}
#endif
