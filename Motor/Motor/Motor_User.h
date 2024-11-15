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
    @file   Motor_User.h
    @author FireSourcery
    @brief  User Input/Output interface function, including error checking
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor_StateMachine.h"
#include "Motor_FOC.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    Inline Getters/Setters
*/
/******************************************************************************/
/*
    Conversion functions only on user call. No periodic proc.
*/
/*! @return [-32767:32767] <=> [-1:1] speed forward as positive. reverse as negative. */
static inline int32_t Motor_User_GetSpeed_Frac16(const MotorPtr_T p_motor) { return Motor_DirectionalValueOf(p_motor, p_motor->Speed_FracS16); }
/*! @return [0:65535] <=> [0:2] */
static inline uint16_t Motor_User_GetSpeed_UFrac16(const MotorPtr_T p_motor) { return math_abs(p_motor->Speed_FracS16); }

/*!
    @return IPhase Zero to Peak.
    iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_Frac16(const MotorPtr_T p_motor) { return Motor_DirectionalValueOf(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_Frac16, 0U)); }
static inline int32_t Motor_User_GetIPhase_UFrac16(const MotorPtr_T p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_UFrac16, 0U); }

/*
    Sampled BEMF during freewheel or VOut during active control
*/
static inline int32_t Motor_User_GetVPhase_Frac16(const MotorPtr_T p_motor) { return Motor_DirectionalValueOf(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_Frac16, 0U)); }
static inline int32_t Motor_User_GetVPhase_UFrac16(const MotorPtr_T p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_UFrac16, 0U); }

/*

*/
static inline int32_t Motor_User_GetVSpeed_Frac16(const MotorPtr_T p_motor) { return Motor_GetVSpeed_Frac16(p_motor); }
static inline int32_t Motor_User_GetVSpeed_UFrac16(const MotorPtr_T p_motor) { return math_abs(Motor_GetVSpeed_Frac16(p_motor)); }

/* Ideal electrical power [0:49152] <=> [0:1.5] */
static inline int32_t Motor_User_GetElectricalPower_UFrac16(const MotorPtr_T p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetElectricalPower_UFrac16, 0U); }

static inline uint16_t Motor_User_GetAdcu(const MotorPtr_T p_motor, MotorAnalog_Channel_T adcChannel)       { return p_motor->AnalogResults.Channels[adcChannel]; }
static inline uint8_t Motor_User_GetAdcu_Msb8(const MotorPtr_T p_motor, MotorAnalog_Channel_T adcChannel)   { return Motor_User_GetAdcu(p_motor, adcChannel) >> (GLOBAL_ANALOG.ADC_BITS - 8U); }

static inline uint16_t Motor_User_GetHeat_Adcu(const MotorPtr_T p_motor)                                    { return p_motor->AnalogResults.Heat_Adcu; }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_User_GetSpeed_Rpm(const MotorPtr_T p_motor)             { return _Motor_ConvertSpeed_FracS16ToRpm(p_motor, Motor_User_GetSpeed_Frac16(p_motor)); }
static inline int16_t Motor_User_GetIPhase_Amps(const MotorPtr_T p_motor)           { return _Motor_ConvertI_FracS16ToAmps(Motor_User_GetIPhase_UFrac16(p_motor)); }
static inline int16_t Motor_User_GetVPhase_Volts(const MotorPtr_T p_motor)          { return _Motor_ConvertV_FracS16ToVolts(Motor_User_GetVPhase_UFrac16(p_motor)); }
static inline int32_t Motor_User_GetElectricalPower_VA(const MotorPtr_T p_motor)    { return _Motor_ConvertPower_FracS16ToWatts(Motor_User_GetElectricalPower_UFrac16(p_motor)); }
static inline int32_t Motor_User_GetHeat_DegCScalar(const MotorPtr_T p_motor, uint16_t scalar)  { return Thermistor_ConvertToDegC_Scalar(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu, scalar); }
// check DC current limit
// Motor_FOC_GetElectricalPower_FracS16Abs(p_motor) / Motor_Static_GetVSource_V() ;
static inline thermal_t Motor_User_GetHeat_DegC(const MotorPtr_T p_motor)           { return Thermistor_ConvertToDegC(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }
#endif

/*
    Read-Only
*/
static inline uint32_t Motor_User_GetControlTimer(const MotorPtr_T p_motor)                     { return p_motor->ControlTimerBase; }
static inline qangle16_t Motor_User_GetElectricalAngle(const MotorPtr_T p_motor)                { return p_motor->ElectricalAngle; }
static inline qangle16_t Motor_User_GetMechanicalAngle(const MotorPtr_T p_motor)                { return Motor_GetMechanicalAngle(p_motor); }
static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(const MotorPtr_T p_motor)      { return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
static inline bool Motor_User_IsStop(const MotorPtr_T p_motor)                                  { return (Motor_User_GetStateId(p_motor) == MSM_STATE_ID_STOP); }
static inline bool Motor_User_IsZeroSpeed(const MotorPtr_T p_motor)                             { return (p_motor->Speed_FracS16 == 0); }
static inline Motor_StatusFlags_T Motor_User_GetStatusFlags(const MotorPtr_T p_motor)           { return p_motor->StatusFlags; }
static inline Motor_FaultFlags_T Motor_User_GetFaultFlags(const MotorPtr_T p_motor)             { return p_motor->FaultFlags; }
/* SubStates */
static inline Motor_OpenLoopState_T Motor_User_GetOpenLoopState(const MotorPtr_T p_motor)       { return p_motor->OpenLoopState; }
static inline Motor_CalibrationState_T Motor_User_GetCalibrationState(const MotorPtr_T p_motor) { return p_motor->CalibrationState; }
static inline uint8_t Motor_User_GetCalibrationStateIndex(const MotorPtr_T p_motor)             { return p_motor->CalibrationStateIndex; }

/*
    Write via interface functions
*/
static inline Motor_Direction_T Motor_User_GetDirection(const MotorPtr_T p_motor)               { return p_motor->Direction; }
static inline bool Motor_User_IsDirectionForward(const MotorPtr_T p_motor)                      { return (p_motor->Config.DirectionForward == p_motor->Direction); }
static inline bool Motor_User_IsDirectionReverse(const MotorPtr_T p_motor)                      { return !Motor_User_IsDirectionForward(p_motor); }
static inline Motor_FeedbackMode_T Motor_User_GetActiveFeedbackMode(const MotorPtr_T p_motor)   { return p_motor->FeedbackMode; }

// todo enum ids, regularize direct/sentinel
static inline uint16_t Motor_User_GetActiveILimit(const MotorPtr_T p_motor)                     { return p_motor->ILimitActiveSentinel_Scalar16; }
static inline uint16_t Motor_User_GetActiveSpeedLimit(const MotorPtr_T p_motor)                 { return p_motor->SpeedLimitDirect_Scalar16; }

/******************************************************************************/
/*!
   by Id
*/
/******************************************************************************/
// typedef enum Motor_User_Id
// {
//     MOTOR_VAR_SPEED,
//     MOTOR_VAR_I_PHASE,
//     MOTOR_VAR_V_PHASE,
//     MOTOR_VAR_POWER,
//     MOTOR_VAR_RAMP_SET_POINT,
//     MOTOR_VAR_ELECTRICAL_ANGLE,
//     MOTOR_VAR_MECHANICAL_ANGLE,
//     MOTOR_VAR_MOTOR_STATE,
//     MOTOR_VAR_MOTOR_STATUS_FLAGS,
//     MOTOR_VAR_MOTOR_FAULT_FLAGS,
//     MOTOR_VAR_MOTOR_HEAT,
//     MOTOR_VAR_MOTOR_ACTIVE_SPEED_LIMIT,
//     MOTOR_VAR_MOTOR_ACTIVE_I_LIMIT,
//     MOTOR_VAR_MOTOR_V_SPEED,
//     MOTOR_VAR_MOTOR_END,
// }
// Motor_User_Id_T;

// #include <assert.h>
// static_assert((MOTOR_VAR_MOTOR_END <= 16U));

// static inline int32_t Motor_GetVar(const MotorPtr_T p_motor, Motor_User_Id_T varId)
// {
//     int32_t value;
//     switch(varId)
//     {
//         case MOTOR_VAR_SPEED:                     value = Motor_User_GetSpeed_UFrac16(p_motor);               break;
//         case MOTOR_VAR_I_PHASE:                   value = Motor_User_GetIPhase_UFrac16(p_motor);              break;
//         case MOTOR_VAR_V_PHASE:                   value = Motor_User_GetVPhase_UFrac16(p_motor);              break;
//         case MOTOR_VAR_POWER:                     value = Motor_User_GetElectricalPower_UFrac16(p_motor);     break;
//         case MOTOR_VAR_RAMP_SET_POINT:            value = Linear_Ramp_GetOutput(&p_motor->Ramp);              break;
//         case MOTOR_VAR_MOTOR_STATE:               value = Motor_User_GetStateId(p_motor);                     break;
//         case MOTOR_VAR_MOTOR_STATUS_FLAGS:        value = Motor_User_GetStatusFlags(p_motor).Word;            break;
//         case MOTOR_VAR_MOTOR_FAULT_FLAGS:         value = Motor_User_GetFaultFlags(p_motor).Word;             break;
//         case MOTOR_VAR_MOTOR_HEAT:                value = Motor_User_GetHeat_Adcu(p_motor);                   break;
//         case MOTOR_VAR_MOTOR_ACTIVE_SPEED_LIMIT:  value = Motor_User_GetActiveSpeedLimit(p_motor);            break;
//         case MOTOR_VAR_MOTOR_ACTIVE_I_LIMIT:      value = Motor_User_GetActiveILimit(p_motor);                break;
//         case MOTOR_VAR_MOTOR_V_SPEED:             value = Motor_GetVSpeed_Frac16(p_motor);                    break;
//         default: value = 0; break;
//     }
//     return value;
// }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern int32_t Motor_User_GetCmd(const MotorPtr_T p_motor);
extern int32_t Motor_User_GetSetPoint(const MotorPtr_T p_motor);
extern void Motor_User_SetVoltageMode(MotorPtr_T p_motor);
extern void Motor_User_SetVoltageCmdValue(MotorPtr_T p_motor, int16_t vCmd);
extern void Motor_User_SetVoltageModeCmd(MotorPtr_T p_motor, int16_t voltage);
extern void Motor_User_SetScalarMode(MotorPtr_T p_motor);
extern void Motor_User_SetScalarCmdValue(MotorPtr_T p_motor, uint32_t scalar);
extern void Motor_User_SetScalarModeCmd(MotorPtr_T p_motor, uint32_t scalar);
extern void Motor_User_SetTorqueMode(MotorPtr_T p_motor);
extern void Motor_User_SetTorqueCmdValue(MotorPtr_T p_motor, int16_t torque);
extern void Motor_User_SetTorqueModeCmd(MotorPtr_T p_motor, int16_t torque);
extern void Motor_User_SetSpeedMode(MotorPtr_T p_motor);
extern void Motor_User_SetSpeedCmdValue(MotorPtr_T p_motor, int16_t speed);
extern void Motor_User_SetSpeedModeCmd(MotorPtr_T p_motor, int16_t speed);
extern void Motor_User_SetPositionCmdValue(MotorPtr_T p_motor, uint16_t angle);
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_User_SetOpenLoopMode(MotorPtr_T p_motor);
extern void Motor_User_SetOpenLoopCmdValue(MotorPtr_T p_motor, int16_t ivCmd);
extern void Motor_User_SetOpenLoopModeCmd(MotorPtr_T p_motor, int16_t ivMagnitude);
#endif
extern void Motor_User_SetActiveCmdValue(MotorPtr_T p_motor, int16_t userCmd);

// extern void Motor_User_ActivateDefaultFeedbackMode(MotorPtr_T p_motor);
// extern void Motor_ActivateControl(MotorPtr_T p_motor);

extern void Motor_User_ForceDisableControl(MotorPtr_T p_motor);
extern bool Motor_User_TryRelease(MotorPtr_T p_motor);
extern bool Motor_User_TryHold(MotorPtr_T p_motor);
extern bool Motor_User_TryDirection(MotorPtr_T p_motor, Motor_Direction_T direction);
extern bool Motor_User_TryDirectionForward(MotorPtr_T p_motor);
extern bool Motor_User_TryDirectionReverse(MotorPtr_T p_motor);

extern bool Motor_User_TrySpeedLimit(MotorPtr_T p_motor, uint16_t scalar16);
extern bool Motor_User_TryILimit(MotorPtr_T p_motor, uint16_t scalar16);
extern bool Motor_User_ClearSpeedLimit(MotorPtr_T p_motor);
extern bool Motor_User_ClearILimit(MotorPtr_T p_motor);

extern void Motor_User_CalibrateSensor(MotorPtr_T p_motor);
extern void Motor_User_CalibrateAdc(MotorPtr_T p_motor);


#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
extern int16_t Motor_User_GetGroundSpeed_Mph(MotorPtr_T p_motor);
extern void Motor_User_SetGroundSpeed_Kmh(MotorPtr_T p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Motor_User_SetGroundSpeed_Mph(MotorPtr_T p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
#endif

#endif




/* Feedback Control Variable Mode  */
// typedef enum Motor_FeedbackModeId
// {
//     MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR,
//     MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT,
//     MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE,
//     MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ,
//     MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE,
//     MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT,
//     MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT,
//     MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT,
// }
// Motor_FeedbackModeId_T;

/* SuperFunction AntiPattern rationale, single variable alias, should be compiler optimizable */
// static inline Motor_FeedbackMode_T Motor_FeedbackModeFlags(Motor_FeedbackModeId_T mode)
// {
//     Motor_FeedbackMode_T flags;
//     switch(mode)
//     {
//         case MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR:          flags.State = MODE_OPEN_LOOP.State;             break;
//         case MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT:         flags.State = MODE_OPEN_LOOP_CURRENT.State;     break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE:          flags.State = MODE_VOLTAGE.State;               break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT:          flags.State = MODE_CURRENT.State;               break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE:    flags.State = MODE_SPEED_VOLTAGE.State;         break;
//         case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT:    flags.State = MODE_SPEED_CURRENT.State;         break;
//         default: flags.State = 0; break;
//     }
//     return flags;
// }

// static inline Motor_FeedbackModeId_T Motor_FeedbackModeId(Motor_FeedbackMode_T mode)
// {
//     Motor_FeedbackModeId_T id;
//     switch((uint32_t)mode.State)
//     {
//         case (uint32_t)MODE_OPEN_LOOP.State:          id = MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR;                 break;
//         case (uint32_t)MODE_OPEN_LOOP_CURRENT.State:  id = MOTOR_FEEDBACK_MODE_OPEN_LOOP_CURRENT;         break;
//         case (uint32_t)MODE_VOLTAGE.State:            id = MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE;          break;
//         case (uint32_t)MODE_CURRENT.State:            id = MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT;          break;
//         case (uint32_t)MODE_SPEED_VOLTAGE.State:      id = MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE;    break;
//         case (uint32_t)MODE_SPEED_CURRENT.State:      id = MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT;    break;
//         default: id = 0; break;
//     }
//     return id;
// }