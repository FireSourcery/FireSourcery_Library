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
    User Get Set Wrappers RAM Variables
*/
/******************************************************************************/
/******************************************************************************/
/*
    Conversion functions only on user call. No periodic proc
*/
/******************************************************************************/
/*!
    Speed_FracS16 set as CCW is positive
    @return speed forward as positive. reverse as negative.
*/
/* [-32767:32767] <=> [-1:1] */
static inline int32_t Motor_User_GetSpeed_Frac16(const MotorPtr_T p_motor) { return Motor_LogicalDirectionCmd(p_motor, p_motor->Speed_FracS16); }
/* [0:65535] <=> [0:2] */
static inline uint16_t Motor_User_GetSpeed_UFrac16(const MotorPtr_T p_motor) { return math_abs(p_motor->Speed_FracS16); }

/*!
    @return IPhase Zero to Peak.

    iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_Frac16(const MotorPtr_T p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_UFrac16, 0U); }
// return Motor_DirectionalCmd(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_Frac16, 0U));

/*
    BEMF during freewheel or VOut during active control
*/
// return Motor_DirectionalCmd(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_Frac16, 0U));
static inline int32_t Motor_User_GetVPhase_Frac16(const MotorPtr_T p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_UFrac16, 0U); }


/* Ideal electrical power [0:49152] <=> [0:1.5] */
static inline int32_t Motor_User_GetElectricalPower_UFrac16(const MotorPtr_T p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetElectricalPower_UFrac16, 0U); }

//check DC current limit
// Motor_FOC_GetElectricalPower_FracS16Abs(p_motor) / Global_Motor_GetVSource_V() ;

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_User_GetSpeed_Rpm(const MotorPtr_T p_motor)            { return _Motor_ConvertSpeed_FracS16ToRpm(p_motor, Motor_User_GetSpeed_Frac16(p_motor)); }
static inline int16_t Motor_User_GetIPhase_Amps(const MotorPtr_T p_motor)          { return _Motor_ConvertI_FracS16ToAmps(Motor_User_GetIPhase_Frac16(p_motor)); }
static inline int16_t Motor_User_GetVPhase_Volts(const MotorPtr_T p_motor)         { return _Motor_ConvertV_FracS16ToVolts(Motor_User_GetVPhase_Frac16(p_motor)); }
static inline int32_t Motor_User_GetElectricalPower_VA(const MotorPtr_T p_motor)   { return _Motor_ConvertPower_FracS16ToWatts(Motor_User_GetElectricalPower_UFrac16(p_motor)); }
#endif

/*
    Writable via interface functions
*/
static inline Motor_Direction_T Motor_User_GetDirection(const MotorPtr_T p_motor)                { return p_motor->Direction; }
static inline bool Motor_User_IsDirectionForward(const MotorPtr_T p_motor)                       { return (p_motor->Parameters.DirectionForward == p_motor->Direction); }
static inline bool Motor_User_IsDirectionReverse(const MotorPtr_T p_motor)                       { return !Motor_User_IsDirectionForward(p_motor); }
static inline Motor_FeedbackMode_T Motor_User_GetActiveFeedbackMode(const MotorPtr_T p_motor)    { return p_motor->FeedbackMode; }
//todo enum version
static inline uint16_t Motor_User_GetActiveILimit(const MotorPtr_T p_motor)                      { return p_motor->ILimitActiveSentinel_Scalar16; }
static inline uint16_t Motor_User_GetActiveSpeedLimit(const MotorPtr_T p_motor)                  { return p_motor->SpeedLimitDirect_Scalar16; }

/*
    Read-Only
*/
static inline qangle16_t Motor_User_GetElectricalAngle(const MotorPtr_T p_motor)                   { return p_motor->ElectricalAngle; }
static inline qangle16_t Motor_User_GetMechanicalAngle(const MotorPtr_T p_motor)                   { return Motor_GetMechanicalAngle(p_motor); }
static inline Motor_StatusFlags_T Motor_User_GetStatusFlags(const MotorPtr_T p_motor)              { return p_motor->StatusFlags; }
static inline Motor_FaultFlags_T Motor_User_GetFaultFlags(const MotorPtr_T p_motor)                { return p_motor->FaultFlags; }
static inline uint32_t Motor_User_GetControlTimer(const MotorPtr_T p_motor)                        { return p_motor->ControlTimerBase; }

static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(const MotorPtr_T p_motor)         { return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
/* select Checks 0 speed or stop state. */
static inline bool Motor_User_CheckStop(const MotorPtr_T p_motor)                                  { return (Motor_User_GetStateId(p_motor) == MSM_STATE_ID_STOP); }

static inline Motor_OpenLoopState_T Motor_User_GetOpenLoopState(const MotorPtr_T p_motor)          { return p_motor->OpenLoopState; }
static inline Motor_CalibrationState_T Motor_User_GetCalibrationState(const MotorPtr_T p_motor)    { return p_motor->CalibrationState; }
static inline uint8_t Motor_User_GetCalibrationStateIndex(const MotorPtr_T p_motor)                { return p_motor->CalibrationStateIndex; }

static inline uint16_t Motor_User_GetAdcu(const MotorPtr_T p_motor, MotorAnalog_Channel_T adcChannel)      { return p_motor->AnalogResults.Channels[adcChannel]; }
static inline uint8_t Motor_User_GetAdcu_Msb8(const MotorPtr_T p_motor, MotorAnalog_Channel_T adcChannel)  { return Motor_User_GetAdcu(p_motor, adcChannel) >> (GLOBAL_ANALOG.ADC_BITS - 8U); }
static inline uint16_t Motor_User_GetHeat_Adcu(const MotorPtr_T p_motor)                                   { return p_motor->AnalogResults.Heat_Adcu; }
static inline int32_t Motor_User_GetHeat_DegC(const MotorPtr_T p_motor, uint16_t scalar)                   { return Thermistor_ConvertToDegC_Int(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu, scalar); }
// static inline float Motor_User_GetHeat_DegCFloat(const MotorPtr_T p_motor        { return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_User_ActivateFeedbackMode(MotorPtr_T p_motor, Motor_FeedbackMode_T mode);
extern void Motor_User_ActivateFeedbackMode_Cast(MotorPtr_T p_motor, uint8_t modeWord);
extern void Motor_User_SetCmd(MotorPtr_T p_motor, int16_t userCmd);
extern int32_t Motor_User_GetCmd(const MotorPtr_T p_motor);
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
extern void Motor_User_SetOpenLoopMode(MotorPtr_T p_motor);
extern void Motor_User_SetOpenLoopCmdValue(MotorPtr_T p_motor, int16_t ivCmd);
extern void Motor_User_SetOpenLoopModeCmd(MotorPtr_T p_motor, int16_t ivMagnitude);
extern void Motor_User_SetActiveCmdValue(MotorPtr_T p_motor, int16_t userCmd);
extern void Motor_User_ActivateDefaultFeedbackMode(MotorPtr_T p_motor);

extern void Motor_User_ReleaseControl(MotorPtr_T p_motor);
extern void Motor_User_DisableControl(MotorPtr_T p_motor);
extern void Motor_User_ActivateControl(MotorPtr_T p_motor);
extern bool Motor_User_TryHold(MotorPtr_T p_motor);
extern bool Motor_User_TryDirection(MotorPtr_T p_motor, Motor_Direction_T direction);
extern bool Motor_User_TryDirectionForward(MotorPtr_T p_motor);
extern bool Motor_User_TryDirectionReverse(MotorPtr_T p_motor);
extern bool Motor_User_CheckFault(const MotorPtr_T p_motor);
extern bool Motor_User_ClearFault(MotorPtr_T p_motor);
extern void Motor_User_SetFault(MotorPtr_T p_motor);
extern void Motor_User_CalibrateSensor(MotorPtr_T p_motor);
extern void Motor_User_CalibrateAdc(MotorPtr_T p_motor);

extern void Motor_User_SetILimitActive(MotorPtr_T p_motor, uint16_t scalar16);
extern void Motor_User_ClearILimitActive(MotorPtr_T p_motor);
extern void Motor_User_SetSpeedLimitActive(MotorPtr_T p_motor, uint16_t scalar_frac16);
extern void Motor_User_ClearSpeedLimitActive(MotorPtr_T p_motor);

extern bool Motor_User_SetSpeedLimitActive_Id(MotorPtr_T p_motor, uint16_t scalar_frac16, uint8_t id);
extern bool Motor_User_ClearSpeedLimitActive_Id(MotorPtr_T p_motor, uint8_t id);
extern bool Motor_User_SetILimitActive_Id(MotorPtr_T p_motor, uint16_t scalar_frac16, uint8_t id);
extern bool Motor_User_ClearILimitActive_Id(MotorPtr_T p_motor, uint8_t id);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
extern int16_t Motor_User_GetGroundSpeed_Mph(MotorPtr_T p_motor);
extern void Motor_User_SetGroundSpeed_Kmh(MotorPtr_T p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Motor_User_SetGroundSpeed_Mph(MotorPtr_T p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
#endif
#endif


