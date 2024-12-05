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
// typedef void (*Motor_Config_Set_T)(Motor_T * p_motor, uint16_t value);

// static inline uint16_t Motor_Config_Set(Motor_T * p_motor, Motor_Config_Set_T setFunction, uint16_t value)
// {
//     if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP) { setFunction(p_motor, value); }
// }


/* Calibration */
static inline Motor_SensorMode_T Motor_Config_GetSensorMode(Motor_T * p_motor)             { return p_motor->Config.SensorMode; }
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(Motor_T * p_motor)    { return p_motor->Config.DirectionForward; }
static inline uint8_t Motor_Config_GetPolePairs(Motor_T * p_motor)                         { return p_motor->Config.PolePairs; }
static inline uint16_t Motor_Config_GetKv(Motor_T * p_motor)                               { return p_motor->Config.Kv; }
static inline uint16_t Motor_Config_GetSpeedFeedbackRef_Rpm(Motor_T * p_motor)             { return p_motor->Config.SpeedFeedbackRef_Rpm; }
static inline uint16_t Motor_Config_GetSpeedVRef_Rpm(Motor_T * p_motor)                    { return p_motor->Config.SpeedMatchRef_Rpm; }
static inline uint16_t Motor_Config_GetIaZero_Adcu(Motor_T * p_motor)                      { return p_motor->Config.IaZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(Motor_T * p_motor)                      { return p_motor->Config.IbZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(Motor_T * p_motor)                      { return p_motor->Config.IcZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIPeakRef_Adcu(Motor_T * p_motor)                    { return Motor_GetIPeakRef_Adcu(p_motor); }
static inline uint16_t Motor_Config_GetKvRpm(Motor_T * p_motor)                            { return p_motor->Config.Kv * Motor_Static_GetVSource_V(); }

/*  */
static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(Motor_T * p_motor)                   { return p_motor->Config.CommutationMode; }
static inline void Motor_Config_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode)        { p_motor->Config.CommutationMode = mode; }

// deprecate
// static inline Motor_FeedbackMode_T Motor_Config_GetDefaultFeedbackMode(Motor_T * p_motor)                  { return p_motor->Config.FeedbackModeDefault; }
// static inline void Motor_Config_SetDefaultFeedbackMode(Motor_T * p_motor, uint16_t wordValue)              { p_motor->Config.FeedbackModeDefault.Word = wordValue; }

/*  */
static inline uint32_t Motor_Config_GetRampAccel_Cycles(Motor_T * p_motor)                     { return p_motor->Config.RampAccel_Cycles; }
static inline uint16_t Motor_Config_GetRampAccel_Millis(Motor_T * p_motor)                     { return _Motor_MillisOf(p_motor->Config.RampAccel_Cycles); }
static inline void Motor_Config_SetRampAccel_Cycles(Motor_T * p_motor, uint32_t cycles)        { p_motor->Config.RampAccel_Cycles = cycles; }
static inline void Motor_Config_SetRampAccel_Millis(Motor_T * p_motor, uint16_t millis)        { p_motor->Config.RampAccel_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Config_GetAlignMode(Motor_T * p_motor, Motor_AlignMode_T mode)         { return p_motor->Config.AlignMode; }
// static inline void Motor_Config_SetAlignMode(Motor_T * p_motor, Motor_AlignMode_T mode)         { p_motor->Config.AlignMode = mode; }
static inline uint16_t Motor_Config_GetAlignPower_Scalar16(Motor_T * p_motor)                      { return p_motor->Config.AlignPower_Scalar16; }
static inline void Motor_Config_SetAlignPower_Scalar16(Motor_T * p_motor, uint16_t v_scalar16)     { p_motor->Config.AlignPower_Scalar16 = (v_scalar16 > MOTOR_STATIC.ALIGN_VPWM_MAX) ? MOTOR_STATIC.ALIGN_VPWM_MAX : v_scalar16; }

static inline uint32_t Motor_Config_GetAlignTime_Cycles(Motor_T * p_motor)                         { return p_motor->Config.AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(Motor_T * p_motor)                         { return _Motor_MillisOf(p_motor->Config.AlignTime_Cycles); }
static inline void Motor_Config_SetAlignTime_Cycles(Motor_T * p_motor, uint32_t cycles)            { p_motor->Config.AlignTime_Cycles = cycles; }
static inline void Motor_Config_SetAlignTime_Millis(Motor_T * p_motor, uint16_t millis)            { p_motor->Config.AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }

// static inline void Motor_Config_SetVoltageBrakeScalar_Frac16(Motor_T * p_motor, uint16_t scalar_Frac16)     { p_motor->Config.VoltageBrakeScalar_InvFrac16 = 65535U - scalar_Frac16; }

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeed_Scalar16(Motor_T * p_motor)                       { return p_motor->Config.OpenLoopSpeed_Scalar16; }
static inline uint16_t Motor_Config_GetOpenLoopPower_Scalar16(Motor_T * p_motor)                       { return p_motor->Config.OpenLoopPower_Scalar16; }
static inline uint32_t Motor_Config_GetOpenLoopAccel_Cycles(Motor_T * p_motor)                         { return p_motor->Config.OpenLoopAccel_Cycles; }

static inline void Motor_Config_SetOpenLoopSpeed_Scalar16(Motor_T * p_motor, uint16_t speed_scalar16)  { p_motor->Config.OpenLoopSpeed_Scalar16 = speed_scalar16; }
static inline void Motor_Config_SetOpenLoopPower_Scalar16(Motor_T * p_motor, uint16_t v_scalar16)      { p_motor->Config.OpenLoopPower_Scalar16 = v_scalar16; }
static inline void Motor_Config_SetOpenLoopAccel_Cycles(Motor_T * p_motor, uint32_t cycles)            { p_motor->Config.OpenLoopAccel_Cycles = cycles; }

static inline uint16_t Motor_Config_GetOpenLoopAccel_Millis(Motor_T * p_motor)                         { return _Motor_MillisOf(p_motor->Config.OpenLoopAccel_Cycles); }
static inline void Motor_Config_SetOpenLoopAccel_Millis(Motor_T * p_motor, uint16_t millis)            { p_motor->Config.OpenLoopAccel_Cycles = _Motor_ControlCyclesOf(millis); }
#endif

#ifdef CONFIG_MOTOR_SIX_STEP_ENABLE
static inline void Motor_Config_SetPhaseModeParam(Motor_T * p_motor, Phase_Mode_T mode)       { p_motor->Config.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->Phase, mode); }
#endif

/* Persistent Base Limits */
static inline uint16_t Motor_Config_GetSpeedLimitForward_Scalar16(Motor_T * p_motor) { return p_motor->Config.SpeedLimitForward_Scalar16; }
static inline uint16_t Motor_Config_GetSpeedLimitReverse_Scalar16(Motor_T * p_motor) { return p_motor->Config.SpeedLimitReverse_Scalar16; }
static inline uint16_t Motor_Config_GetILimitMotoring_Scalar16(Motor_T * p_motor) { return p_motor->Config.ILimitMotoring_Scalar16; }
static inline uint16_t Motor_Config_GetILimitGenerating_Scalar16(Motor_T * p_motor) { return p_motor->Config.ILimitGenerating_Scalar16; }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_Config_SetSpeedFeedbackRef_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_Config_SetSpeedFeedbackRef_Kv(Motor_T * p_motor, uint16_t kv);
extern void Motor_Config_SetSpeedMatchRef_Rpm(Motor_T * p_motor, uint16_t rpm);
extern void Motor_Config_SetKv(Motor_T * p_motor, uint16_t kv);

extern void Motor_Config_SetIaZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIbZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIcZero_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetDirectionCalibration(Motor_T * p_motor, Motor_Direction_T directionForward);
extern void Motor_Config_SetPolePairs(Motor_T * p_motor, uint8_t polePairs);
extern void Motor_Config_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode);

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_Config_SetIPeakRef_Adcu_Debug(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIPeakRef_Adcu(Motor_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIPeakRef_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV);
#endif

extern void Motor_Config_SetSpeedLimitForward_Scalar16(Motor_T * p_motor, uint16_t forward_Frac16);
extern void Motor_Config_SetSpeedLimitReverse_Scalar16(Motor_T * p_motor, uint16_t reverse_Frac16);
extern void Motor_Config_SetSpeedLimit_Scalar16(Motor_T * p_motor, uint16_t forward_Frac16, uint16_t reverse_Frac16);
extern void Motor_Config_SetILimitMotoring_Scalar16(Motor_T * p_motor, uint16_t motoring_Frac16);
extern void Motor_Config_SetILimitGenerating_Scalar16(Motor_T * p_motor, uint16_t generating_Frac16);
extern void Motor_Config_SetILimit_Scalar16(Motor_T * p_motor, uint16_t motoring_Frac16, uint16_t generating_Frac16);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_Config_SetSpeedLimitForward_Rpm(Motor_T * p_motor, uint16_t forward_Rpm);
extern void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_T * p_motor, uint16_t reverse_Rpm);
extern void Motor_Config_SetSpeedLimit_Rpm(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_Config_SetILimitMotoring_Amp(Motor_T * p_motor, uint16_t motoring_Amp);
extern void Motor_Config_SetILimitGenerating_Amp(Motor_T * p_motor, uint16_t generating_Amp);
extern void Motor_Config_SetILimit_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
#endif

#endif
