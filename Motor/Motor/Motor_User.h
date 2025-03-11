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
    @version V0

    @brief  User[Interface] Input/Output function, includes error checking.
*/
/******************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor_StateMachine.h"
#include "Motor_Analog.h"
#include "Motor_FOC.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*!
    Inline Getters/Setters
*/
/******************************************************************************/
/* todo wrap functions with intptr_t */

/*! @return [-32767:32767] <=> [-1:1) speed forward as positive. reverse as negative. */
static inline int32_t Motor_User_GetSpeed_Fract16(const Motor_T * p_motor) { return Motor_DirectionalValueOf(p_motor, p_motor->Speed_Fract16); }
/*! @return [0:65535] <=> [0:2) */
static inline uint16_t Motor_User_GetSpeed_UFract16(const Motor_T * p_motor) { return math_abs(p_motor->Speed_Fract16); }

static inline int32_t Motor_User_GetVSpeedEffective_Fract16(const Motor_T * p_motor) { return Motor_GetVSpeed_Fract16(p_motor); }
static inline uint16_t Motor_User_GetVSpeedEffective_UFract16(const Motor_T * p_motor) { return math_abs(Motor_GetVSpeed_Fract16(p_motor)); }

/* Speed as V. Conversion can be handled by host (VSource * Speed_Fract16 / FRACT16_MAX) */
// static inline int32_t Motor_User_GetVSpeedDebug_Fract16(const Motor_T * p_motor)    { return Motor_User_GetSpeed_Fract16(p_motor); }
// static inline uint16_t Motor_User_GetVSpeedDebug_UFract16(const Motor_T * p_motor)  { return Motor_User_GetSpeed_UFract16(p_motor); }

/*
    Conversion functions only on user call. No periodic proc.
    Partial conversion on client side. Alternatively host side conversion using components results in the same values.
*/
/*!
    @return IPhase Zero to Peak.
    iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_Fract16(const Motor_T * p_motor)     { return Motor_DirectionalValueOf(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_Fract16, 0U)); }
static inline uint16_t Motor_User_GetIPhase_UFract16(const Motor_T * p_motor)   { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_UFract16, 0U); }

/*
    Sampled BEMF during freewheel or VOut during active control
*/
static inline int32_t Motor_User_GetVPhase_Fract16(const Motor_T * p_motor)     { return Motor_DirectionalValueOf(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_Fract16, 0U)); }
static inline uint16_t Motor_User_GetVPhase_UFract16(const Motor_T * p_motor)   { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_UFract16, 0U); }

/*
    Ideal electrical power physical VA as UFract16
    [0:49152] <=> [0:1.5]
*/
static inline uint16_t Motor_User_GetElectricalPower_UFract16(const Motor_T * p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetElectricalPower_UFract16, 0U); }

/*  */
static inline uint16_t Motor_User_GetHeat_Adcu(const Motor_T * p_motor) { return Motor_Analog_GetHeat(p_motor); }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_User_GetSpeed_Rpm(const Motor_T * p_motor)             { return _Motor_ConvertSpeed_Fract16ToRpm(p_motor, Motor_User_GetSpeed_Fract16(p_motor)); }
static inline int16_t Motor_User_GetIPhase_Amps(const Motor_T * p_motor)           { return _Motor_ConvertI_Fract16ToAmps(Motor_User_GetIPhase_UFract16(p_motor)); }
static inline int16_t Motor_User_GetVPhase_Volts(const Motor_T * p_motor)          { return _Motor_ConvertV_Fract16ToVolts(Motor_User_GetVPhase_UFract16(p_motor)); }
static inline int32_t Motor_User_GetElectricalPower_VA(const Motor_T * p_motor)    { return _Motor_ConvertPower_Fract16ToWatts(Motor_User_GetElectricalPower_UFract16(p_motor)); }
// check DC current limit // Motor_FOC_GetElectricalPower_Fract16Abs(p_motor) / Motor_Static_GetVSource_V() ;
static inline thermal_t Motor_User_GetHeat_DegC(const Motor_T * p_motor)           { return Thermistor_CelsiusOfAdcu(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }
#endif

static inline angle16_t Motor_User_GetElectricalAngle(const Motor_T * p_motor) { return p_motor->ElectricalAngle; }
static inline angle16_t Motor_User_GetMechanicalAngle(const Motor_T * p_motor) { return Motor_GetMechanicalAngle(p_motor); }

static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(const Motor_T * p_motor) { return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
static inline uint8_t Motor_User_GetSubStateId(const Motor_T * p_motor) { return StateMachine_GetActiveSubStateId(&p_motor->StateMachine); } /* 0 during substate */

static inline Motor_StateFlags_T Motor_User_GetStateFlags(const Motor_T * p_motor) { return p_motor->StateFlags; }
static inline Motor_FaultFlags_T Motor_User_GetFaultFlags(const Motor_T * p_motor) { return p_motor->FaultFlags; }
static inline bool Motor_User_IsStopState(const Motor_T * p_motor) { return Motor_StateMachine_IsState(p_motor, MSM_STATE_ID_STOP); }
static inline bool Motor_User_IsRunState(const Motor_T * p_motor) { return Motor_StateMachine_IsState(p_motor, MSM_STATE_ID_RUN); }


static inline bool Motor_User_IsRampEnabled(const Motor_T * p_motor) { return !_Ramp_IsDisabled(&p_motor->Ramp); }
static inline void Motor_User_SetRampOnOff(Motor_T * p_motor, bool enable) { if (enable) { Motor_EnableRamp(p_motor); } else { Motor_DisableRamp(p_motor); } }

/*
    Set via interface functions
*/

/*! @return [-32767:32767] <=> [-1:1] */
/* Getters satisfy generic use. Setters are specific to control mode. */
static inline int32_t Motor_User_GetCmd(const Motor_T * p_motor)         { return Motor_DirectionalValueOf(p_motor, Ramp_GetTarget(&p_motor->Ramp)); }
static inline int32_t Motor_User_GetSetPoint(const Motor_T * p_motor)    { return Motor_DirectionalValueOf(p_motor, Ramp_GetOutput(&p_motor->Ramp)); }

static inline int32_t Motor_User_GetEffectiveSetPoint(const Motor_T * p_motor)    { return Ramp_GetOutput(&p_motor->Ramp); }



static inline Motor_Direction_T Motor_User_GetDirection(const Motor_T * p_motor)        { return p_motor->Direction; }
static inline Motor_FeedbackMode_T Motor_User_GetFeedbackMode(const Motor_T * p_motor)  { return p_motor->FeedbackMode; }

static inline Phase_Output_T Motor_User_GetPhaseState(const Motor_T * p_motor) { return Phase_ReadOutputState(&p_motor->Phase); }

/* Separate getters for compatibility with StateMachine SetInput and ProcInput */
// static inline bool Motor_User_IsRelease(const Motor_T * p_motor)    { return Phase_IsFloat(&p_motor->Phase); }
// static inline bool Motor_User_IsHold(const Motor_T * p_motor)       { return Phase_IsV0(&p_motor->Phase); }

/*
    User reference direction
    Effective Limits
*/
static inline uint16_t Motor_User_GetSpeedLimitForward(const Motor_T * p_motor) { return p_motor->SpeedLimitForward_Fract16; }
static inline uint16_t Motor_User_GetSpeedLimitReverse(const Motor_T * p_motor) { return p_motor->SpeedLimitReverse_Fract16; }
/* (p_motor->Direction == MOTOR_DIRECTION_CCW) or Speed > 0 */
static inline uint16_t Motor_User_GetSpeedLimit(const Motor_T * p_motor)        { return Motor_GetSpeedLimit(p_motor); }

static inline uint16_t Motor_User_GetILimitMotoring(const Motor_T * p_motor)    { return p_motor->ILimitMotoring_Fract16; }
static inline uint16_t Motor_User_GetILimitGenerating(const Motor_T * p_motor)  { return p_motor->ILimitGenerating_Fract16; }
static inline uint16_t Motor_User_GetILimit(const Motor_T * p_motor)            { return Motor_FOC_GetILimit(p_motor); }

/* SubStates */
// static inline uint32_t Motor_User_GetControlTimer(const Motor_T * p_motor)                      { return p_motor->ControlTimerBase; }
// static inline Motor_OpenLoopState_T Motor_User_GetOpenLoopState(const Motor_T * p_motor)        { return p_motor->OpenLoopState; }
// static inline Motor_CalibrationState_T Motor_User_GetCalibrationState(const Motor_T * p_motor)  { return p_motor->CalibrationState; }
// static inline uint8_t Motor_User_GetCalibrationStateIndex(const Motor_T * p_motor)              { return p_motor->CalibrationStateIndex; }

/*
   User Conditional - set compare with array
*/
static inline bool Motor_User_TrySpeedLimit(Motor_T * p_motor, uint16_t speed_fract16) { return Motor_SetSpeedLimitEntry(p_motor, MOTOR_SPEED_LIMIT_USER, speed_fract16); }
static inline bool Motor_User_ClearSpeedLimit(Motor_T * p_motor)                       { return Motor_ClearSpeedLimitEntry(p_motor, MOTOR_SPEED_LIMIT_USER); }
static inline bool Motor_User_TryILimit(Motor_T * p_motor, uint16_t i_fract16)         { return Motor_SetILimitMotoringEntry(p_motor, MOTOR_I_LIMIT_USER, i_fract16); }
static inline bool Motor_User_ClearILimit(Motor_T * p_motor)                           { return Motor_ClearILimitMotoringEntry(p_motor, MOTOR_I_LIMIT_USER); }

/*
    On poll only
    Consolidate all status flags into a single word
*/
typedef union Motor_User_StatusFlags
{
    struct
    {
        uint16_t HeatWarning    : 1U;
        uint16_t ILimitSet      : 1U;
        uint16_t SpeedLimitSet  : 1U;
        uint16_t ILimited       : 1U;
        uint16_t SpeedLimited   : 1U;
        uint16_t Hold           : 1U;
        uint16_t Release        : 1U;
    };
    uint16_t Value;
}
Motor_User_StatusFlags_T;

static inline Motor_User_StatusFlags_T Motor_User_GetStatusFlags(const Motor_T * p_motor)
{
    return (Motor_User_StatusFlags_T)
    {
        .HeatWarning    = Thermistor_IsWarning(&p_motor->Thermistor),
        .ILimitSet      = Limit_IsUpperActive(&p_motor->ILimit),
        .SpeedLimitSet  = Limit_IsUpperActive(&p_motor->SpeedLimit),
        .ILimited       = p_motor->StateFlags.ILimited,
        .SpeedLimited   = p_motor->StateFlags.SpeedLimited,
        // .Hold           = Motor_User_IsHold(p_motor),
        // .Release        = Motor_User_IsRelease(p_motor),
    };
}

/* buffered copy implementation */
// typedef struct MotorUser_Input
// {
//     int8_t Direction;
//     uint16_t CmdValue;
//     uint16_t FeedbackMode;
//     uint16_t ControlState;
//     uint16_t SpeedLimit;
//     uint16_t ILimit;
//     uint16_t RampOnOff;
// }
// MotorUser_Input_T;7

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_User_ActivateControlWith(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_User_StartControl_Cast(Motor_T * p_motor, uint8_t modeValue);

extern void Motor_User_ActivateControl(Motor_T * p_motor);
extern void Motor_User_Release(Motor_T * p_motor);
extern void Motor_User_Hold(Motor_T * p_motor);
extern void Motor_User_ActivateControlState(Motor_T * p_motor, Phase_Output_T state);
extern void Motor_User_ForceDisableControl(Motor_T * p_motor);
extern void Motor_User_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode);
extern void Motor_User_SetFeedbackMode_Cast(Motor_T * p_motor, uint8_t modeValue);

extern void Motor_User_StartVoltageMode(Motor_T * p_motor);
extern void Motor_User_SetVoltageCmd(Motor_T * p_motor, int16_t volts_fract16);
extern void Motor_User_SetVoltageCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_SetVSpeedScalarCmd(Motor_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_StartIMode(Motor_T * p_motor);
extern void Motor_User_SetICmd(Motor_T * p_motor, int16_t i_fract16);
extern void Motor_User_SetICmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_StartTorqueMode(Motor_T * p_motor);
extern void Motor_User_SetTorqueCmd(Motor_T * p_motor, int16_t torque);
extern void Motor_User_SetTorqueCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_StartSpeedMode(Motor_T * p_motor);
extern void Motor_User_SetSpeedCmd(Motor_T * p_motor, int16_t speed_fract16);
extern void Motor_User_SetSpeedCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_SetPositionCmd(Motor_T * p_motor, uint16_t angle);

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_User_StartOpenLoopMode(Motor_T * p_motor);
extern void Motor_User_SetOpenLoopCmd(Motor_T * p_motor, int16_t ivCmd);
extern void Motor_User_SetOpenLoopSpeed(Motor_T * p_motor, int16_t speed_fract16);
// extern void Motor_User_StartOpenLoopState(Motor_T * p_motor, Motor_OpenLoopState_T state);
#endif

extern void Motor_User_SetActiveCmdValue(Motor_T * p_motor, int16_t userCmd);
extern void Motor_User_SetActiveCmdValue_Scalar(Motor_T * p_motor, int16_t userCmd);
// extern void Motor_User_ProcModeCmd(Motor_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd);

extern void Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_User_SetDirectionForward(Motor_T * p_motor);
extern void Motor_User_SetDirectionReverse(Motor_T * p_motor);

extern void Motor_User_Stop(Motor_T * p_motor);

extern void Motor_User_CalibrateSensor(Motor_T * p_motor);
extern void Motor_User_CalibrateAdc(Motor_T * p_motor);


#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
extern int16_t Motor_User_GetGroundSpeed_Mph(Motor_T * p_motor);
extern void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
#endif

#endif
