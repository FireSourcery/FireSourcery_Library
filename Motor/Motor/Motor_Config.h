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
    @brief  Config, part of User
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

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
// typedef void (*Motor_Config_Set_T)(MotorPtr_T p_motor, uint16_t value);

// static inline uint16_t Motor_Config_Set(MotorPtr_T p_motor, Motor_Config_Set_T setFunction, uint16_t value)
// {
//     if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP) { setFunction(p_motor, value); }
// }

/* Persistent Base Limits */
static inline uint16_t Motor_Config_GetSpeedLimitForward_Scalar16(MotorPtr_T p_motor)   { return p_motor->Config.SpeedLimitForward_Scalar16; }
static inline uint16_t Motor_Config_GetSpeedLimitReverse_Scalar16(MotorPtr_T p_motor)   { return p_motor->Config.SpeedLimitReverse_Scalar16; }
static inline uint16_t Motor_Config_GetILimitMotoring_Scalar16(MotorPtr_T p_motor)      { return p_motor->Config.ILimitMotoring_Scalar16; }
static inline uint16_t Motor_Config_GetILimitGenerating_Scalar16(MotorPtr_T p_motor)    { return p_motor->Config.ILimitGenerating_Scalar16; }

/* Calibration */
static inline Motor_SensorMode_T Motor_Config_GetSensorMode(MotorPtr_T p_motor)             { return p_motor->Config.SensorMode; }
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(MotorPtr_T p_motor)    { return p_motor->Config.DirectionForward; }
static inline uint8_t Motor_Config_GetPolePairs(MotorPtr_T p_motor)                         { return p_motor->Config.PolePairs; }
static inline uint16_t Motor_Config_GetKv(MotorPtr_T p_motor)                               { return p_motor->Config.Kv; }
static inline uint16_t Motor_Config_GetSpeedFeedbackRef_Rpm(MotorPtr_T p_motor)             { return p_motor->Config.SpeedFeedbackRef_Rpm; }
static inline uint16_t Motor_Config_GetSpeedVRef_Rpm(MotorPtr_T p_motor)                    { return p_motor->Config.SpeedVRef_Rpm; }
static inline uint16_t Motor_Config_GetIaZero_Adcu(MotorPtr_T p_motor)                      { return p_motor->Config.IaZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(MotorPtr_T p_motor)                      { return p_motor->Config.IbZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(MotorPtr_T p_motor)                      { return p_motor->Config.IcZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIPeakRef_Adcu(MotorPtr_T p_motor)                    { return Motor_GetIPeakRef_Adcu(p_motor); }

/*  */
static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(MotorPtr_T p_motor)                   { return p_motor->Config.CommutationMode; }
static inline void Motor_Config_SetCommutationMode(MotorPtr_T p_motor, Motor_CommutationMode_T mode)        { p_motor->Config.CommutationMode = mode; }

// deprecate
// static inline Motor_FeedbackMode_T Motor_Config_GetDefaultFeedbackMode(MotorPtr_T p_motor)                  { return p_motor->Config.FeedbackModeDefault; }
// static inline void Motor_Config_SetDefaultFeedbackMode(MotorPtr_T p_motor, uint16_t wordValue)              { p_motor->Config.FeedbackModeDefault.Word = wordValue; }

/*  */
static inline uint32_t Motor_Config_GetRampAccel_Cycles(MotorPtr_T p_motor)                     { return p_motor->Config.RampAccel_Cycles; }
static inline uint16_t Motor_Config_GetRampAccel_Millis(MotorPtr_T p_motor)                     { return _Motor_MillisOf(p_motor->Config.RampAccel_Cycles); }
static inline void Motor_Config_SetRampAccel_Cycles(MotorPtr_T p_motor, uint32_t cycles)        { p_motor->Config.RampAccel_Cycles = cycles; }
static inline void Motor_Config_SetRampAccel_Millis(MotorPtr_T p_motor, uint16_t millis)        { p_motor->Config.RampAccel_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Config_GetAlignMode(MotorPtr_T p_motor, Motor_AlignMode_T mode)         { return p_motor->Config.AlignMode; }
// static inline void Motor_Config_SetAlignMode(MotorPtr_T p_motor, Motor_AlignMode_T mode)         { p_motor->Config.AlignMode = mode; }
static inline uint16_t Motor_Config_GetAlignPower_Scalar16(MotorPtr_T p_motor)                      { return p_motor->Config.AlignPower_Scalar16; }
static inline void Motor_Config_SetAlignPower_Scalar16(MotorPtr_T p_motor, uint16_t v_scalar16)     { p_motor->Config.AlignPower_Scalar16 = (v_scalar16 > MOTOR_STATIC.ALIGN_VPWM_MAX) ? MOTOR_STATIC.ALIGN_VPWM_MAX : v_scalar16; }

static inline uint32_t Motor_Config_GetAlignTime_Cycles(MotorPtr_T p_motor)                         { return p_motor->Config.AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(MotorPtr_T p_motor)                         { return _Motor_MillisOf(p_motor->Config.AlignTime_Cycles); }
static inline void Motor_Config_SetAlignTime_Cycles(MotorPtr_T p_motor, uint32_t cycles)            { p_motor->Config.AlignTime_Cycles = cycles; }
static inline void Motor_Config_SetAlignTime_Millis(MotorPtr_T p_motor, uint16_t millis)            { p_motor->Config.AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Config_SetVoltageBrakeScalar_Frac16(MotorPtr_T p_motor, uint16_t scalar_Frac16)     { p_motor->Config.VoltageBrakeScalar_InvFrac16 = 65535U - scalar_Frac16; }

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeed_Scalar16(MotorPtr_T p_motor)                       { return p_motor->Config.OpenLoopSpeed_Scalar16; }
static inline uint16_t Motor_Config_GetOpenLoopPower_Scalar16(MotorPtr_T p_motor)                       { return p_motor->Config.OpenLoopPower_Scalar16; }
static inline uint32_t Motor_Config_GetOpenLoopAccel_Cycles(MotorPtr_T p_motor)                         { return p_motor->Config.OpenLoopAccel_Cycles; }

static inline void Motor_Config_SetOpenLoopSpeed_Scalar16(MotorPtr_T p_motor, uint16_t speed_scalar16)  { p_motor->Config.OpenLoopSpeed_Scalar16 = speed_scalar16; }
static inline void Motor_Config_SetOpenLoopPower_Scalar16(MotorPtr_T p_motor, uint16_t v_scalar16)      { p_motor->Config.OpenLoopPower_Scalar16 = v_scalar16; }
static inline void Motor_Config_SetOpenLoopAccel_Cycles(MotorPtr_T p_motor, uint32_t cycles)            { p_motor->Config.OpenLoopAccel_Cycles = cycles; }

static inline uint16_t Motor_Config_GetOpenLoopAccel_Millis(MotorPtr_T p_motor)                         { return _Motor_MillisOf(p_motor->Config.OpenLoopAccel_Cycles); }
static inline void Motor_Config_SetOpenLoopAccel_Millis(MotorPtr_T p_motor, uint16_t millis)            { p_motor->Config.OpenLoopAccel_Cycles = _Motor_ControlCyclesOf(millis); }
#endif

#ifdef CONFIG_MOTOR_SIX_STEP_ENABLE
static inline void Motor_Config_SetPhaseModeParam(MotorPtr_T p_motor, Phase_Mode_T mode)       { p_motor->Config.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }
#endif

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_Config_SetSpeedLimitForward_Scalar16(MotorPtr_T p_motor, uint16_t forward_Frac16);
extern void Motor_Config_SetSpeedLimitReverse_Scalar16(MotorPtr_T p_motor, uint16_t reverse_Frac16);
extern void Motor_Config_SetSpeedLimit_Scalar16(MotorPtr_T p_motor, uint16_t forward_Frac16, uint16_t reverse_Frac16);
extern void Motor_Config_SetILimitMotoring_Scalar16(MotorPtr_T p_motor, uint16_t motoring_Frac16);
extern void Motor_Config_SetILimitGenerating_Scalar16(MotorPtr_T p_motor, uint16_t generating_Frac16);
extern void Motor_Config_SetILimit_Scalar16(MotorPtr_T p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_Config_SetSpeedLimitForward_Rpm(MotorPtr_T p_motor, uint16_t forward_Rpm);
extern void Motor_Config_SetSpeedLimitReverse_Rpm(MotorPtr_T p_motor, uint16_t reverse_Rpm);
extern void Motor_Config_SetSpeedLimit_Rpm(MotorPtr_T p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_Config_SetILimitMotoring_Amp(MotorPtr_T p_motor, uint16_t motoring_Amp);
extern void Motor_Config_SetILimitGenerating_Amp(MotorPtr_T p_motor, uint16_t generating_Amp);
extern void Motor_Config_SetILimit_Amp(MotorPtr_T p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
#endif

extern void Motor_Config_SetSpeedFeedbackRef_Rpm(MotorPtr_T p_motor, uint16_t rpm);
extern void Motor_Config_SetSpeedFeedbackRef_Kv(MotorPtr_T p_motor, uint16_t kv);
extern void Motor_Config_SetSpeedVRef_Rpm(MotorPtr_T p_motor, uint16_t rpm);
extern void Motor_Config_SetVSpeedRef_Kv(MotorPtr_T p_motor, uint16_t kv);
extern void Motor_Config_SetKv(MotorPtr_T p_motor, uint16_t kv);

extern void Motor_Config_SetIaZero_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Config_SetIbZero_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Config_SetIcZero_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Config_SetDirectionCalibration(MotorPtr_T p_motor, Motor_Direction_T directionForward);
extern void Motor_Config_SetPolePairs(MotorPtr_T p_motor, uint8_t polePairs);
extern void Motor_Config_SetSensorMode(MotorPtr_T p_motor, Motor_SensorMode_T mode);

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_Config_SetIPeakRef_Adcu_Debug(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Config_SetIPeakRef_Adcu(MotorPtr_T p_motor, uint16_t adcu);
extern void Motor_Config_SetIPeakRef_MilliV(MotorPtr_T p_motor, uint16_t min_MilliV, uint16_t max_MilliV);
#endif

#endif
