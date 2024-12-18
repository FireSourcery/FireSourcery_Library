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
    @brief  User[Interface] Input/Output function, includes error checking.
    @version V0
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
/* Getters satisfy generic use. Setters are specific to control mode. */
/*! @return [-32767:32767] <=> [-1:1] */
static inline int32_t Motor_User_GetCmd(const Motor_T * p_motor)        { return Motor_GetCmd(p_motor); }
static inline int32_t Motor_User_GetSetPoint(const Motor_T * p_motor)   { return Motor_GetSetPoint(p_motor); }


/*! @return [-32767:32767] <=> [-1:1] speed forward as positive. reverse as negative. */
static inline int32_t Motor_User_GetSpeed_Fract16(const Motor_T * p_motor) { return Motor_DirectionalValueOf(p_motor, p_motor->Speed_Fract16); }
/*! @return [0:65535] <=> [0:2] */
static inline uint16_t Motor_User_GetSpeed_UFract16(const Motor_T * p_motor) { return math_abs(p_motor->Speed_Fract16); }

/*
    Conversion functions only on user call. No periodic proc.
*/
/*
    Partial conversion on client side. Alternatively host side conversion using components results in the same values.
*/
/*!
    @return IPhase Zero to Peak.
    iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_Fract16(const Motor_T * p_motor) { return Motor_DirectionalValueOf(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_Fract16, 0U)); }
static inline uint16_t Motor_User_GetIPhase_UFract16(const Motor_T * p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetIPhase_UFract16, 0U); }

/*
    Sampled BEMF during freewheel or VOut during active control
*/
static inline int32_t Motor_User_GetVPhase_Fract16(const Motor_T * p_motor) { return Motor_DirectionalValueOf(p_motor, Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_Fract16, 0U)); }
static inline uint16_t Motor_User_GetVPhase_UFract16(const Motor_T * p_motor) { return Motor_GetCommutationModeInt32(p_motor, Motor_FOC_GetVPhase_UFract16, 0U); }


static inline int32_t Motor_User_GetVSpeed_Fract16(const Motor_T * p_motor) { return Motor_User_GetSpeed_Fract16(p_motor); }
static inline uint16_t Motor_User_GetVSpeed_UFract16(const Motor_T * p_motor) { return Motor_User_GetSpeed_UFract16(p_motor); }

static inline int32_t Motor_User_GetVSpeedEffective_Fract16(const Motor_T * p_motor) { return Motor_GetVSpeed_Fract16(p_motor); }
static inline uint16_t Motor_User_GetVSpeedEffective_UFract16(const Motor_T * p_motor) { return math_abs(Motor_GetVSpeed_Fract16(p_motor)); }

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

/*
    On poll only
    Consolidate all status flags into a single word
*/
typedef union Motor_User_StatusFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U;
        uint16_t ILimitSet          : 1U;
        uint16_t SpeedLimitSet      : 1U;
        uint16_t ILimited           : 1U;
        uint16_t SpeedLimited       : 1U;
    };
    uint16_t Word;
}
Motor_User_StatusFlags_T;

static inline Motor_User_StatusFlags_T Motor_User_GetStatusFlags(const Motor_T * p_motor)
{
    Motor_User_StatusFlags_T status;
    status.HeatWarning = Thermistor_IsWarning(&p_motor->Thermistor);
    status.ILimitSet = Limit_IsUpperActive(&p_motor->ILimit);
    status.SpeedLimitSet = Limit_IsUpperActive(&p_motor->SpeedLimit);
    status.ILimited = p_motor->StateFlags.ILimited;
    status.SpeedLimited = p_motor->StateFlags.SpeedLimited;
}


/*
    Write via interface functions
*/
static inline Motor_Direction_T Motor_User_GetDirection(const Motor_T * p_motor)                { return p_motor->Direction; }
static inline bool Motor_User_IsDirectionForward(const Motor_T * p_motor)                       { return (p_motor->Direction == p_motor->Config.DirectionForward); }
static inline bool Motor_User_IsDirectionReverse(const Motor_T * p_motor)                       { return !Motor_User_IsDirectionForward(p_motor); }
static inline Motor_FeedbackMode_T Motor_User_GetActiveFeedbackMode(const Motor_T * p_motor)    { return p_motor->FeedbackMode; }
static inline uint16_t Motor_User_GetActiveILimit(const Motor_T * p_motor)                      { return Limit_GetUpper(&p_motor->ILimit); }
static inline uint16_t Motor_User_GetActiveSpeedLimit(const Motor_T * p_motor)                  { return Limit_GetUpper(&p_motor->SpeedLimit); }

/*
    Read-Only
*/
static inline uint32_t Motor_User_GetControlTimer(const Motor_T * p_motor)                      { return p_motor->ControlTimerBase; }
static inline angle16_t Motor_User_GetElectricalAngle(const Motor_T * p_motor)                  { return p_motor->ElectricalAngle; }
static inline angle16_t Motor_User_GetMechanicalAngle(const Motor_T * p_motor)                  { return Motor_GetMechanicalAngle(p_motor); }

static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(const Motor_T * p_motor)       { return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
static inline Motor_StateFlags_T Motor_User_GetStateFlags(const Motor_T * p_motor)              { return p_motor->StateFlags; }
static inline Motor_FaultFlags_T Motor_User_GetFaultFlags(const Motor_T * p_motor)              { return p_motor->FaultFlags; }
static inline bool Motor_User_IsStopState(const Motor_T * p_motor)                              { return (Motor_User_GetStateId(p_motor) == MSM_STATE_ID_STOP); }
static inline bool Motor_User_IsZeroSpeed(const Motor_T * p_motor)                              { return (p_motor->Speed_Fract16 == 0); }

/* SubStates */
static inline Motor_OpenLoopState_T Motor_User_GetOpenLoopState(const Motor_T * p_motor)        { return p_motor->OpenLoopState; }
static inline Motor_CalibrationState_T Motor_User_GetCalibrationState(const Motor_T * p_motor)  { return p_motor->CalibrationState; }
static inline uint8_t Motor_User_GetCalibrationStateIndex(const Motor_T * p_motor)              { return p_motor->CalibrationStateIndex; }


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
// Motor_VarId_GetPrimary_T;

// Motor_VarId_Set_T;
// Motor_VarId_IO_T;

// #include <assert.h>
// static_assert((MOTOR_VAR_MOTOR_END <= 16U));

// static inline int32_t Motor_GetVar(const Motor_T * p_motor, Motor_User_Id_T varId)
// {
//     int32_t value;
//     switch(varId)
//     {
//         case MOTOR_VAR_SPEED:                     value = Motor_User_GetSpeed_UFract16(p_motor);               break;
//         case MOTOR_VAR_I_PHASE:                   value = Motor_User_GetIPhase_UFract16(p_motor);              break;
//         case MOTOR_VAR_V_PHASE:                   value = Motor_User_GetVPhase_UFract16(p_motor);              break;
//         case MOTOR_VAR_POWER:                     value = Motor_User_GetElectricalPower_UFract16(p_motor);     break;
//         case MOTOR_VAR_RAMP_SET_POINT:            value = Linear_Ramp_GetOutput(&p_motor->Ramp);              break;
//         case MOTOR_VAR_MOTOR_STATE:               value = Motor_User_GetStateId(p_motor);                     break;
//         case MOTOR_VAR_MOTOR_STATUS_FLAGS:        value = Motor_User_GetStateFlags(p_motor).Word;            break;
//         case MOTOR_VAR_MOTOR_FAULT_FLAGS:         value = Motor_User_GetFaultFlags(p_motor).Word;             break;
//         case MOTOR_VAR_MOTOR_HEAT:                value = Motor_User_GetHeat_Adcu(p_motor);                   break;
//         case MOTOR_VAR_MOTOR_ACTIVE_SPEED_LIMIT:  value = Motor_User_GetActiveSpeedLimit(p_motor);            break;
//         case MOTOR_VAR_MOTOR_ACTIVE_I_LIMIT:      value = Motor_User_GetActiveILimit(p_motor);                break;
//         case MOTOR_VAR_MOTOR_V_SPEED:             value = Motor_GetVSpeed_Fract16(p_motor);                    break;
//         default: value = 0; break;
//     }
//     return value;
// }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
// extern int32_t Motor_User_GetCmd(const Motor_T * p_motor);
// extern int32_t Motor_User_GetSetPoint(const Motor_T * p_motor);
extern void Motor_User_StartVoltageMode(Motor_T * p_motor);
extern void Motor_User_SetVoltageCmdValue(Motor_T * p_motor, int16_t vCmd);
extern void Motor_User_SetScalarMode(Motor_T * p_motor);
extern void Motor_User_SetScalarCmdValue(Motor_T * p_motor, uint32_t scalar);
extern void Motor_User_StartTorqueMode(Motor_T * p_motor);
extern void Motor_User_SetTorqueCmdValue(Motor_T * p_motor, int16_t torque);
extern void Motor_User_StartSpeedMode(Motor_T * p_motor);
extern void Motor_User_SetSpeedCmdValue(Motor_T * p_motor, int16_t speed);
extern void Motor_User_SetPositionCmdValue(Motor_T * p_motor, uint16_t angle);
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_User_StartOpenLoopMode(Motor_T * p_motor);
extern void Motor_User_SetOpenLoopCmdValue(Motor_T * p_motor, int16_t ivCmd);
#endif
extern void Motor_User_SetActiveCmdValue(Motor_T * p_motor, int16_t userCmd);
// extern void Motor_User_ActivateDefaultFeedbackMode(Motor_T * p_motor);

extern void Motor_User_ForceDisableControl(Motor_T * p_motor);
// extern void Motor_ActivateControl(Motor_T * p_motor);
extern bool Motor_User_TryRelease(Motor_T * p_motor);
extern bool Motor_User_TryHold(Motor_T * p_motor);
extern bool Motor_User_TryDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern bool Motor_User_TryDirectionForward(Motor_T * p_motor);
extern bool Motor_User_TryDirectionReverse(Motor_T * p_motor);

extern bool Motor_User_TrySpeedLimit(Motor_T * p_motor, uint16_t percent16);
extern bool Motor_User_TryILimit(Motor_T * p_motor, uint16_t percent16);
extern bool Motor_User_ClearSpeedLimit(Motor_T * p_motor);
extern bool Motor_User_ClearILimit(Motor_T * p_motor);

extern void Motor_User_CalibrateSensor(Motor_T * p_motor);
extern void Motor_User_CalibrateAdc(Motor_T * p_motor);

#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
extern int16_t Motor_User_GetGroundSpeed_Mph(Motor_T * p_motor);
extern void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
extern void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor);
#endif

#endif


