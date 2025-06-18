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
*/
/******************************************************************************/
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "Motor.h"
#include "Motor_StateMachine.h"

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_TICKS_OF_SPEED_RAMP(RatedUnits, UnitsPerS) ((uint32_t)MOTOR_SPEED_LOOP_FREQ * (RatedUnits) / (UnitsPerS))
#define MOTOR_TICKS_OF_TORQUE_RAMP(RatedUnits, UnitsPerS) ((uint32_t)MOTOR_CONTROL_FREQ * (RatedUnits) / (UnitsPerS)) /* V or I */

/* const expr speed_angle16_of_rpm */
#define MOTOR_SPEED_DEG_CONTROL(PolePairs, MechRpm) (((uint32_t)MechRpm * 65536U) / (60U * MOTOR_CONTROL_FREQ / PolePairs))

/* scale to rated max */
#ifndef MOTOR_OPEN_LOOP_MAX_SCALAR
#define MOTOR_OPEN_LOOP_MAX_SCALAR FRACT16(0.1F)
#endif


static inline uint16_t _Motor_Config_GetOpenLoopScalarLimit(const Motor_State_T * p_motor) { return math_min(p_motor->Config.OpenLoopScalarLimit_Fract16, MOTOR_OPEN_LOOP_MAX_SCALAR); }
static inline uint16_t _Motor_Config_GetOpenLoopILimit_Fract16(const Motor_State_T * p_motor) { return fract16_mul(_Motor_Config_GetOpenLoopScalarLimit(p_motor), MotorAnalogRef_GetIRatedPeak_Fract16()); }
static inline uint16_t _Motor_Config_GetOpenLoopVLimit_Fract16(const Motor_State_T * p_motor) { return fract16_mul(_Motor_Config_GetOpenLoopScalarLimit(p_motor), MotorAnalogRef_GetVRated_Fract16()); }

/******************************************************************************/
/*
    Handle set using StateId
*/
/******************************************************************************/
static inline bool Motor_Config_IsConfigState(const Motor_State_T * p_motor)
{
    return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_STOP);
    // || MSM_STATE_ID_FAULT
}

/******************************************************************************/
/*
    Calibration
*/
/******************************************************************************/
static inline MotorSensor_Id_T Motor_Config_GetSensorMode(const Motor_State_T * p_motor)              { return p_motor->Config.SensorMode; }
static inline Motor_Direction_T Motor_Config_GetDirectionCalibration(const Motor_State_T * p_motor)   { return p_motor->Config.DirectionForward; }
static inline uint8_t Motor_Config_GetPolePairs(const Motor_State_T * p_motor)                        { return p_motor->Config.PolePairs; }
static inline uint16_t Motor_Config_GetKv(const Motor_State_T * p_motor)                              { return p_motor->Config.Kv; }
static inline uint16_t Motor_Config_GetSpeedRated(const Motor_State_T * p_motor)                      { return p_motor->Config.SpeedRated_DegPerCycle; }
static inline uint16_t Motor_Config_GetVSpeedScalar_UFract16(const Motor_State_T * p_motor)           { return p_motor->Config.VSpeedScalar_Fract16; }

/* alias */
static inline uint16_t Motor_Config_GetSpeedRated_Rpm(const Motor_State_T * p_motor)                  { return Motor_GetSpeedRatedRef_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetSpeedVRef_Rpm(const Motor_State_T * p_motor)                   { return Motor_GetSpeedVRef_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetSpeedVSvpwmRef_Rpm(const Motor_State_T * p_motor)              { return Motor_GetSpeedVSvpwmRef_Rpm(p_motor); }
static inline uint16_t Motor_Config_GetSpeedVMatchRef_Rpm(const Motor_State_T * p_motor)              { return (p_motor->Config.VSpeedScalar_Fract16 * Motor_GetSpeedVRef_Rpm(p_motor)) >> 15U; }

static inline uint16_t Motor_Config_GetIaZero_Adcu(const Motor_State_T * p_motor)                     { return p_motor->Config.IaZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIbZero_Adcu(const Motor_State_T * p_motor)                     { return p_motor->Config.IbZeroRef_Adcu; }
static inline uint16_t Motor_Config_GetIcZero_Adcu(const Motor_State_T * p_motor)                     { return p_motor->Config.IcZeroRef_Adcu; }
// static inline uint16_t Motor_Config_GetIPeakRef_Adcu(const Motor_State_T * p_motor)                   { return MotorAnalogRef_GetIRatedPeak_Adcu(); }

/* Modes */
static inline Motor_CommutationMode_T Motor_Config_GetCommutationMode(const Motor_State_T * p_motor) { return p_motor->Config.CommutationMode; }
static inline void Motor_Config_SetCommutationMode(Motor_State_T * p_motor, Motor_CommutationMode_T mode) { p_motor->Config.CommutationMode = mode; }

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
static inline uint16_t Motor_Config_GetSpeedRampTime_Millis(const Motor_State_T * p_motor) { /* return _Motor_MillisOf(p_motor->Config.SpeedRampTime_Cycles); */ }
static inline uint16_t Motor_Config_GetTorqueRampTime_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.TorqueRampTime_Cycles); }

/******************************************************************************/
/* OpenLoop Ramp/Limit */
/******************************************************************************/


static inline uint16_t Motor_Config_GetOpenLoopScalarLimit(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopScalarLimit_Fract16; }

static inline uint16_t Motor_Config_GetAlignPowerScalar(const Motor_State_T * p_motor) { return p_motor->Config.AlignScalar_Fract16; }
static inline uint32_t Motor_Config_GetAlignTime_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.AlignTime_Cycles; }
static inline uint16_t Motor_Config_GetAlignTime_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.AlignTime_Cycles); }

// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
static inline uint16_t Motor_Config_GetOpenLoopSpeed_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampSpeedFinal_Fract16; }
static inline uint32_t Motor_Config_GetOpenLoopSpeedRamp_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampSpeed_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopSpeedRamp_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.OpenLoopRampSpeed_Cycles); }

static inline uint16_t Motor_Config_GetOpenLoopI_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampIFinal_Fract16; }
static inline uint32_t Motor_Config_GetOpenLoopIRamp_Cycles(const Motor_State_T * p_motor) { return p_motor->Config.OpenLoopRampI_Cycles; }
static inline uint16_t Motor_Config_GetOpenLoopIRamp_Millis(const Motor_State_T * p_motor) { return _Motor_MillisOf(p_motor->Config.OpenLoopRampI_Cycles); }
// #endif

/******************************************************************************/
/* Extern */
/******************************************************************************/
extern void Motor_Config_Set(Motor_State_T * p_motor, State_Set_T setter, int32_t value);

extern void Motor_Config_SetPolePairs(Motor_State_T * p_motor, uint8_t polePairs);
extern void Motor_Config_SetSensorMode(Motor_State_T * p_motor, MotorSensor_Id_T mode);
extern void Motor_Config_SetDirectionCalibration(Motor_State_T * p_motor, Motor_Direction_T directionForward);
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

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL)
extern void Motor_Config_SetSpeedLimitForward_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm);
extern void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_State_T * p_motor, uint16_t reverse_Rpm);
extern void Motor_Config_SetSpeedLimit_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm);
extern void Motor_Config_SetILimitMotoring_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp);
extern void Motor_Config_SetILimitGenerating_Amp(Motor_State_T * p_motor, uint16_t generating_Amp);
extern void Motor_Config_SetILimit_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp);
#endif


// int32_t _Motor_Config_Calibration_Get(const Motor_Config_T * p_config, Motor_VarConfig_Calibration_T varId)
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
//         // case MOTOR_VAR_SPEED_RATED_DEG:               value = Motor_Config_GetSpeedRated(p_motor);                break;
//         // case MOTOR_VAR_IA_ZERO_ADCU:                  value = Motor_Config_GetIaZero_Adcu(p_motor);               break;
//         // case MOTOR_VAR_IB_ZERO_ADCU:                  value = Motor_Config_GetIbZero_Adcu(p_motor);               break;
//         // case MOTOR_VAR_IC_ZERO_ADCU:                  value = Motor_Config_GetIcZero_Adcu(p_motor);               break;
//             // case MOTOR_VAR_I_PEAK_REF_ADCU:               value = Motor_Config_GetIPeakRef_Adcu(p_motor);             break;
//     }
//     return value;
// }


#endif
