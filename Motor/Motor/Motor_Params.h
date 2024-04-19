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
    @file   Motor_Params.h
    @author FireSourcery
    @brief  Params, part of User
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include "Motor.h"
#include "Motor_StateMachine.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Nvm Param function
    Selectively handle using StateId
*/
/******************************************************************************/
// typedef void (*Motor_Params_Set_T)(MotorPtr_T p_motor, uint16_t value);

// static inline uint16_t Motor_Params_Set(MotorPtr_T p_motor, Motor_Params_Set_T setFunction, uint16_t value)
// {
//     if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP) { setFunction(p_motor, value); }
// }

/* Persistent Base Limits */
static inline uint16_t Motor_Params_GetSpeedLimitForward_Scalar16(MotorPtr_T p_motor)   { return p_motor->Parameters.SpeedLimitForward_Scalar16; }
static inline uint16_t Motor_Params_GetSpeedLimitReverse_Scalar16(MotorPtr_T p_motor)   { return p_motor->Parameters.SpeedLimitReverse_Scalar16; }
static inline uint16_t Motor_Params_GetILimitMotoring_Scalar16(MotorPtr_T p_motor)      { return p_motor->Parameters.ILimitMotoring_Scalar16; }
static inline uint16_t Motor_Params_GetILimitGenerating_Scalar16(MotorPtr_T p_motor)    { return p_motor->Parameters.ILimitGenerating_Scalar16; }

/* Calibration */
static inline Motor_SensorMode_T Motor_Params_GetSensorMode(MotorPtr_T p_motor)             { return p_motor->Parameters.SensorMode; }
static inline Motor_Direction_T Motor_Params_GetDirectionCalibration(MotorPtr_T p_motor)    { return p_motor->Parameters.DirectionForward; }
static inline uint8_t Motor_Params_GetPolePairs(MotorPtr_T p_motor)                         { return p_motor->Parameters.PolePairs; }
static inline uint16_t Motor_Params_GetKv(MotorPtr_T p_motor)                               { return p_motor->Parameters.Kv; }
static inline uint16_t Motor_Params_GetSpeedFeedbackRef_Rpm(MotorPtr_T p_motor)             { return p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline uint16_t Motor_Params_GetVSpeedRef_Rpm(MotorPtr_T p_motor)                    { return p_motor->Parameters.VSpeedRef_Rpm; }
static inline uint16_t Motor_Params_GetIaZero_Adcu(MotorPtr_T p_motor)                      { return p_motor->Parameters.IaZeroRef_Adcu; }
static inline uint16_t Motor_Params_GetIbZero_Adcu(MotorPtr_T p_motor)                      { return p_motor->Parameters.IbZeroRef_Adcu; }
static inline uint16_t Motor_Params_GetIcZero_Adcu(MotorPtr_T p_motor)                      { return p_motor->Parameters.IcZeroRef_Adcu; }
static inline uint16_t Motor_Params_GetIPeakRef_Adcu(MotorPtr_T p_motor)                    { return Motor_GetIPeakRef_Adcu(p_motor); }

/*  */
static inline Motor_CommutationMode_T Motor_Params_GetCommutationMode(MotorPtr_T p_motor)                   { return p_motor->Parameters.CommutationMode; }
static inline void Motor_Params_SetCommutationMode(MotorPtr_T p_motor, Motor_CommutationMode_T mode)        { p_motor->Parameters.CommutationMode = mode; }

static inline Motor_FeedbackMode_T Motor_Params_GetDefaultFeedbackMode(MotorPtr_T p_motor)                  { return p_motor->Parameters.FeedbackModeDefault; }
static inline void Motor_Params_SetDefaultFeedbackMode(MotorPtr_T p_motor, Motor_FeedbackMode_T mode)       { p_motor->Parameters.FeedbackModeDefault = mode; }
static inline void Motor_Params_SetDefaultFeedbackMode_Cast(MotorPtr_T p_motor, uint16_t wordValue)         { p_motor->Parameters.FeedbackModeDefault.Word = wordValue; }

/*  */
static inline uint32_t Motor_Params_GetRampAccel_Cycles(MotorPtr_T p_motor)                     { return p_motor->Parameters.RampAccel_Cycles; }
static inline void Motor_Params_SetRampAccel_Cycles(MotorPtr_T p_motor, uint32_t cycles)        { p_motor->Parameters.RampAccel_Cycles = cycles; }
static inline uint16_t Motor_Params_GetRampAccel_Millis(MotorPtr_T p_motor)                     { return _Motor_MillisOf(p_motor->Parameters.RampAccel_Cycles); }
static inline void Motor_Params_SetRampAccel_Millis(MotorPtr_T p_motor, uint16_t millis)        { p_motor->Parameters.RampAccel_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Params_GetAlignMode(MotorPtr_T p_motor, Motor_AlignMode_T mode)         { return p_motor->Parameters.AlignMode; }
// static inline void Motor_Params_SetAlignMode(MotorPtr_T p_motor, Motor_AlignMode_T mode)         { p_motor->Parameters.AlignMode = mode; }
static inline uint16_t Motor_Params_GetAlignPower_Scalar16(MotorPtr_T p_motor)                      { return p_motor->Parameters.AlignPower_Scalar16; }
static inline uint32_t Motor_Params_GetAlignTime_Cycles(MotorPtr_T p_motor)                         { return p_motor->Parameters.AlignTime_Cycles; }
static inline void Motor_Params_SetAlignPower_Scalar16(MotorPtr_T p_motor, uint16_t v_scalar16)     { p_motor->Parameters.AlignPower_Scalar16 = (v_scalar16 > GLOBAL_MOTOR.ALIGN_VPWM_MAX) ? GLOBAL_MOTOR.ALIGN_VPWM_MAX : v_scalar16; }
static inline void Motor_Params_SetAlignTime_Cycles(MotorPtr_T p_motor, uint32_t cycles)            { p_motor->Parameters.AlignTime_Cycles = cycles; }
static inline uint16_t Motor_Params_GetAlignTime_Millis(MotorPtr_T p_motor)                         { return _Motor_MillisOf(p_motor->Parameters.AlignTime_Cycles); }
static inline void Motor_Params_SetAlignTime_Millis(MotorPtr_T p_motor, uint16_t millis)            { p_motor->Parameters.AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Params_SetVoltageBrakeScalar_Frac16(MotorPtr_T p_motor, uint16_t scalar_Frac16)     { p_motor->Parameters.VoltageBrakeScalar_InvFrac16 = 65535U - scalar_Frac16; }

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Params_GetOpenLoopSpeed_Scalar16(MotorPtr_T p_motor)                       { return p_motor->Parameters.OpenLoopSpeed_Scalar16; }
static inline uint16_t Motor_Params_GetOpenLoopPower_Scalar16(MotorPtr_T p_motor)                       { return p_motor->Parameters.OpenLoopPower_Scalar16; }
static inline uint32_t Motor_Params_GetOpenLoopAccel_Cycles(MotorPtr_T p_motor)                         { return p_motor->Parameters.OpenLoopAccel_Cycles; }

static inline void Motor_Params_SetOpenLoopSpeed_Scalar16(MotorPtr_T p_motor, uint16_t speed_scalar16)  { p_motor->Parameters.OpenLoopSpeed_Scalar16 = speed_scalar16; }
static inline void Motor_Params_SetOpenLoopPower_Scalar16(MotorPtr_T p_motor, uint16_t v_scalar16)      { p_motor->Parameters.OpenLoopPower_Scalar16 = v_scalar16; }
static inline void Motor_Params_SetOpenLoopAccel_Cycles(MotorPtr_T p_motor, uint32_t cycles)            { p_motor->Parameters.OpenLoopAccel_Cycles = cycles; }

static inline uint16_t Motor_Params_GetOpenLoopAccel_Millis(MotorPtr_T p_motor)                         { return _Motor_MillisOf(p_motor->Parameters.OpenLoopAccel_Cycles); }
static inline void Motor_Params_SetOpenLoopAccel_Millis(MotorPtr_T p_motor, uint16_t millis)            { p_motor->Parameters.OpenLoopAccel_Cycles = _Motor_ControlCyclesOf(millis); }
#endif

#ifdef CONFIG_MOTOR_SIX_STEP_ENABLE
static inline void Motor_Params_SetPhaseModeParam(MotorPtr_T p_motor, Phase_Mode_T mode)       { p_motor->Parameters.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }
#endif

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_Params_SetSpeedLimitForward_Scalar16(MotorPtr_T p_motor, uint16_t forward_Frac16);
extern void Motor_Params_SetSpeedLimitReverse_Scalar16(MotorPtr_T p_motor, uint16_t reverse_Frac16);
extern void Motor_Params_SetSpeedLimit_Scalar16(MotorPtr_T p_motor, uint16_t forward_Frac16, uint16_t reverse_Frac16);
extern void Motor_Params_SetILimitMotoring_Scalar16(MotorPtr_T p_motor, uint16_t motoring_Frac16);
extern void Motor_Params_SetILimitGenerating_Scalar16(MotorPtr_T p_motor, uint16_t generating_Frac16);
extern void Motor_Params_SetILimit_Scalar16(MotorPtr_T p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_Params_SetSpeedLimitForward_Rpm(MotorPtr_T p_motor, uint16_t forward_Rpm);
extern void Motor_Params_SetSpeedLimitReverse_Rpm(MotorPtr_T p_motor, uint16_t reverse_Rpm);
extern void Motor_Params_SetSpeedLimit_Rpm(MotorPtr_T p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_Params_SetILimitMotoring_Amp(MotorPtr_T p_motor, uint16_t motoring_Amp);
extern void Motor_Params_SetILimitGenerating_Amp(MotorPtr_T p_motor, uint16_t generating_Amp);
extern void Motor_Params_SetILimit_Amp(MotorPtr_T p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
#endif

extern void Motor_Params_SetSpeedFeedbackRef_Rpm(MotorPtr_T p_motor, uint16_t rpm);
extern void Motor_Params_SetSpeedFeedbackRef_Kv(MotorPtr_T p_motor, uint16_t kv);
extern void Motor_Params_SetVSpeedRef_Rpm(MotorPtr_T p_motor, uint16_t rpm);
extern void Motor_Params_SetVSpeedRef_Kv(MotorPtr_T p_motor, uint16_t kv);
extern void Motor_Params_SetKv(MotorPtr_T p_motor, uint16_t kv);

extern void Motor_Params_SetIaZero_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Params_SetIbZero_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Params_SetIcZero_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Params_SetDirectionCalibration(MotorPtr_T p_motor, Motor_Direction_T directionForward);
extern void Motor_Params_SetPolePairs(MotorPtr_T p_motor, uint8_t polePairs);
extern void Motor_Params_SetSensorMode(MotorPtr_T p_motor, Motor_SensorMode_T mode);

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_Params_SetIPeakRef_Adcu_Debug(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Params_SetIPeakRef_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Params_SetIPeakRef_MilliV(MotorPtr_T p_motor, uint16_t min_MilliV, uint16_t max_MilliV);
#endif

#endif
