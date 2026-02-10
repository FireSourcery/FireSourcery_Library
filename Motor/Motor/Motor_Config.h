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

#define MOTOR_TICKS_OF_SPEED_RAMP(RampInterval, UnitsPerS) ((uint32_t)MOTOR_SPEED_LOOP_FREQ * (RampInterval) / (UnitsPerS))
#define MOTOR_TICKS_OF_TORQUE_RAMP(RampInterval, UnitsPerS) ((uint32_t)MOTOR_CONTROL_FREQ * (RampInterval) / (UnitsPerS)) /* V or I */

/* scale to rated max */
#ifndef MOTOR_OPEN_LOOP_MAX_SCALAR
#define MOTOR_OPEN_LOOP_MAX_SCALAR FRACT16(0.1F)
#endif

/******************************************************************************/
/*
    Calibration
*/
/******************************************************************************/
/* inline set */
static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(const Motor_State_T * p_motor) { return p_motor->Config.CommutationMode; }
static inline void Motor_Config_SetCommutationMode(Motor_State_T * p_motor, Motor_CommutationMode_T mode) { p_motor->Config.CommutationMode = mode; }

/* The user direction that is the positive direction */
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward; }
static inline void Motor_Config_SetDirectionCalibration(Motor_State_T * p_motor, Motor_Direction_T directionForward) { p_motor->Config.DirectionForward = directionForward; }
static inline bool Motor_Config_IsCcwPositive(Motor_State_T * p_motor) { return p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW; }
static inline void Motor_Config_SetCcwPositive(Motor_State_T * p_motor, bool isCcwPositive) { p_motor->Config.DirectionForward = (isCcwPositive) ? MOTOR_DIRECTION_CCW : MOTOR_DIRECTION_CW; }

/*
    Set with Propagate
*/
static inline RotorSensor_Id_T Motor_Config_GetSensorMode(const Motor_State_T * p_motor)              { return p_motor->Config.SensorMode; }
static inline uint8_t Motor_Config_GetPolePairs(const Motor_State_T * p_motor)                        { return p_motor->Config.PolePairs; }
static inline uint16_t Motor_Config_GetKv(const Motor_State_T * p_motor)                              { return p_motor->Config.Kv; }
static inline uint16_t Motor_Config_GetSpeedRated(const Motor_State_T * p_motor)                      { return p_motor->Config.SpeedRated_Rpm; }
static inline uint16_t Motor_Config_GetVSpeedScalar_UFract16(const Motor_State_T * p_motor)           { return p_motor->Config.VSpeedScalar_Fract16; }

/* alias */
static inline uint16_t Motor_Config_GetSpeedVRef_Rpm(const Motor_State_T * p_motor)                   { return Motor_GetSpeedVBusRef_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetSpeedVMatchRef_Rpm(const Motor_State_T * p_motor)              { return (p_motor->Config.VSpeedScalar_Fract16 * Motor_GetSpeedVBusRef_Rpm(p_motor)) >> 15U; }
// static inline uint16_t Motor_Config_GetSpeedVSvpwmRef_Rpm(const Motor_State_T * p_motor)              { return Motor_GetSpeedVRefSvpwm_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetSpeedRated_Rpm(const Motor_State_T * p_motor)                  { return Motor_GetSpeedRated_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetVSpeedRated_Fract16(const Motor_State_T * p_motor)             { return Motor_VFract16OfKv(p_motor, Motor_GetSpeedRated_Rpm(p_motor)); }


static inline uint16_t Motor_Config_GetIaZero_Adcu(const Motor_State_T * p_motor)                     { return p_motor->Config.IabcZeroRef_Adcu.A; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(const Motor_State_T * p_motor)                     { return p_motor->Config.IabcZeroRef_Adcu.B; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(const Motor_State_T * p_motor)                     { return p_motor->Config.IabcZeroRef_Adcu.C; }
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
static inline uint16_t Motor_Config_GetSpeedLimitForward_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.SpeedLimitForward_Fract16; }
static inline uint16_t Motor_Config_GetSpeedLimitReverse_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.SpeedLimitReverse_Fract16; }
static inline uint16_t Motor_Config_GetILimitMotoring_Fract16(const Motor_State_T * p_motor)    { return p_motor->Config.ILimitMotoring_Fract16; }
static inline uint16_t Motor_Config_GetILimitGenerating_Fract16(const Motor_State_T * p_motor)  { return p_motor->Config.ILimitGenerating_Fract16; }

/******************************************************************************/
/* Ramps */
/******************************************************************************/
static inline uint32_t Motor_Config_GetSpeedRampTime_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.SpeedRampTime_Cycles; }
static inline uint32_t Motor_Config_GetTorqueRampTime_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.TorqueRampTime_Cycles; }
static inline uint16_t Motor_Config_GetSpeedRampTime_Millis(const Motor_State_T * p_motor) { return p_motor->Config.SpeedRampTime_Cycles; } /* Base Cycle in Millis */
static inline uint16_t Motor_Config_GetTorqueRampTime_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.TorqueRampTime_Cycles); }

/******************************************************************************/
/* OpenLoop Ramp/Limit */
/******************************************************************************/
static inline uint16_t Motor_Config_GetOpenLoopScalarLimit(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopLimitScalar_Fract16; }

static inline uint16_t Motor_Config_GetAlignPowerScalar(const Motor_State_T * p_motor) { return p_motor->Config.AlignScalar_Fract16; }
static inline uint32_t Motor_Config_GetAlignTime_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.AlignTime_Cycles); }

// #if defined(MOTOR_OPEN_LOOP_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeedFinal_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampSpeedFinal_Fract16 ; }
static inline uint32_t Motor_Config_GetOpenLoopSpeedRamp_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampSpeedTime_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopSpeedRamp_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.OpenLoopRampSpeedTime_Cycles); } /*  */

static inline uint16_t Motor_Config_GetOpenLoopIFinal_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampIFinal_Fract16; }
static inline uint32_t Motor_Config_GetOpenLoopIRamp_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampITime_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopIRamp_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.OpenLoopRampITime_Cycles); }
// #endif

/******************************************************************************/
/* Extern */
/******************************************************************************/
// extern void Motor_Config_SetWith(Motor_State_T * p_motor, State_Set_T setter, int32_t value);

// extern void Motor_Config_SetDirectionCalibration(Motor_State_T * p_motor, Motor_Direction_T directionForward);
extern void Motor_Config_SetSensorMode(Motor_State_T * p_motor, RotorSensor_Id_T mode);
extern void Motor_Config_SetPolePairs(Motor_State_T * p_motor, uint8_t polePairs);
extern void Motor_Config_SetKv(Motor_State_T * p_motor, uint16_t kv);
extern void Motor_Config_SetVSpeedScalar_UFract16(Motor_State_T * p_motor, uint16_t scalar);

extern void Motor_Config_SetSpeedRated(Motor_State_T * p_motor, uint16_t controlDeg);
extern void Motor_Config_SetSpeedRated_Rpm(Motor_State_T * p_motor, uint16_t rpm);

extern void Motor_Config_SetSpeedVRef_Rpm(Motor_State_T * p_motor, uint16_t rpm);
extern void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_State_T * p_motor, uint16_t rpm);

extern void Motor_Config_SetIaZero_Adcu(Motor_State_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIbZero_Adcu(Motor_State_T * p_motor, uint16_t adcu);
extern void Motor_Config_SetIcZero_Adcu(Motor_State_T * p_motor, uint16_t adcu);

extern void Motor_Config_SetSpeedLimitForward_Fract16(Motor_State_T * p_motor, uint16_t forward_Fract16);
extern void Motor_Config_SetSpeedLimitReverse_Fract16(Motor_State_T * p_motor, uint16_t reverse_Fract16);
extern void Motor_Config_SetILimitMotoring_Fract16(Motor_State_T * p_motor, uint16_t motoring_Fract16);
extern void Motor_Config_SetILimitGenerating_Fract16(Motor_State_T * p_motor, uint16_t generating_Fract16);

extern void Motor_Config_SetSpeedRampTime_Cycles(Motor_State_T * p_motor, uint32_t cycles);
extern void Motor_Config_SetSpeedRampTime_Millis(Motor_State_T * p_motor, uint16_t millis);
extern void Motor_Config_SetTorqueRampTime_Cycles(Motor_State_T * p_motor, uint32_t cycles);
extern void Motor_Config_SetTorqueRampTime_Millis(Motor_State_T * p_motor, uint16_t millis);

extern void Motor_Config_SetOpenLoopScalarLimit(Motor_State_T * p_motor, uint16_t scalar16);

extern void Motor_Config_SetAlignPowerScalar(Motor_State_T * p_motor, uint16_t i_fract16);
extern void Motor_Config_SetAlignTime_Cycles(Motor_State_T * p_motor, uint32_t cycles);
extern void Motor_Config_SetAlignTime_Millis(Motor_State_T * p_motor, uint16_t millis);

extern void Motor_Config_SetOpenLoopRampIFinal_Fract16(Motor_State_T * p_motor, uint16_t i_fract16);
extern void Motor_Config_SetOpenLoopRampITime_Cycles(Motor_State_T * p_motor, uint32_t cycles);
extern void Motor_Config_SetOpenLoopRampITime_Millis(Motor_State_T * p_motor, uint16_t millis);
extern void Motor_Config_SetOpenLoopRampSpeedFinal_Fract16(Motor_State_T * p_motor, uint16_t speed_fract16);
extern void Motor_Config_SetOpenLoopRampSpeedTime_Cycles(Motor_State_T * p_motor, uint32_t cycles);
extern void Motor_Config_SetOpenLoopRampSpeedTime_Millis(Motor_State_T * p_motor, uint16_t millis);

#if defined(MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_Config_SetSpeedLimitForward_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm);
extern void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_State_T * p_motor, uint16_t reverse_Rpm);
extern void Motor_Config_SetSpeedLimit_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_Config_SetILimitMotoring_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp);
extern void Motor_Config_SetILimitGenerating_Amp(Motor_State_T * p_motor, uint16_t generating_Amp);
extern void Motor_Config_SetILimit_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
#endif

// direct field access without propagate
// int32_t _Motor_Config_Calibration_Get(const Motor_Config_T * p_config, Motor_ConfigId_Calibration_T varId)
// {
//     int32_t value = 0;
//     switch (varId)
//     {
//         case MOTOR_VAR_COMMUTATION_MODE:              value = p_config->CommutationMode;                          break;
//         case MOTOR_VAR_SENSOR_MODE:                   value = p_config->SensorMode;                               break;
//         // case MOTOR_VAR_DIRECTION_CALIBRATION:         value = Motor_Config_GetDirectionCalibration(p_motor);      break;
//         // case MOTOR_VAR_POLE_PAIRS:                    value = Motor_Config_GetPolePairs(p_motor);                 break;
//         // case MOTOR_VAR_KV:                            value = Motor_Config_GetKv(p_motor);                        break;
//         // case MOTOR_VAR_V_SPEED_SCALAR:                value = Motor_Config_GetVSpeedScalar_UFract16(p_motor);     break;
//         // case MOTOR_VAR_SPEED_RATED:               value = Motor_Config_GetSpeedRated(p_motor);                break;
//         // case MOTOR_VAR_IA_ZERO_ADCU:                  value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
//         // case MOTOR_VAR_IB_ZERO_ADCU:                  value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
//         // case MOTOR_VAR_IC_ZERO_ADCU:                  value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
//             // case MOTOR_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
//     }
//     return value;
// }


#endif
