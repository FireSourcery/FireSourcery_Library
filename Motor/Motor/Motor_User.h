#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @brief  User Interface. Outer logic function.
            Input/Output/Cmd functions, includes error checking.
*/
/******************************************************************************/
#include "Motor_StateMachine.h"
#include "Motor/Motor/SubStates/Motor_Calibration.h"
#include "Motor/Motor/SubStates/Motor_OpenLoop.h"

#include "Motor_FOC.h"

#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*!
    Inline Getters/Setters
*/
/******************************************************************************/
static inline Phase_Output_T Motor_User_GetPhaseState(const Motor_T * p_const) { return Phase_ReadOutputState(&p_const->PHASE); }


/*! @return [-32767:32767] <=> [-1:1) speed forward as positive. reverse as negative. */
static inline int32_t Motor_User_GetSpeed_Fract16(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, Motor_GetSpeed_Fract16(p_motor)); } /*  */
/*! @return [0:65535] <=> [0:2) */
static inline uint16_t Motor_User_GetSpeed_UFract16(const Motor_State_T * p_motor) { return math_abs(Motor_GetSpeed_Fract16(p_motor)); }

static inline int16_t Motor_User_GetSpeed_DegPerCycle(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, RotorSensor_GetElectricalAngleSpeed(p_motor->p_ActiveSensor)); }
static inline uint16_t Motor_User_GetSpeed_UDegControl(const Motor_State_T * p_motor) { return math_abs(RotorSensor_GetElectricalAngleSpeed(p_motor->p_ActiveSensor)); }

static inline int32_t Motor_User_GetVSpeedEffective_Fract16(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, Motor_GetVSpeed_Fract16(p_motor)); }
static inline uint16_t Motor_User_GetVSpeedEffective_UFract16(const Motor_State_T * p_motor) { return math_abs(Motor_GetVSpeed_Fract16(p_motor)); }

/*
    Conversion functions only on user call. No periodic proc.
    Partial conversion on client side. Alternatively host side conversion using components results in the same values.
*/
/*!
    @return IPhase Zero to Peak.
    iPhase motoring as positive. generating as negative.
*/
static inline int32_t Motor_User_GetIPhase_Fract16(const Motor_State_T * p_motor)     { return Motor_DirectionalValueOf(p_motor, Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetIPhase_Fract16, 0U)); }
static inline uint16_t Motor_User_GetIPhase_UFract16(const Motor_State_T * p_motor)   { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetIPhase_UFract16, 0U); }

/*
    Sampled BEMF during freewheel or VOut during active control
*/
static inline int32_t Motor_User_GetVPhase_Fract16(const Motor_State_T * p_motor)     { return Motor_DirectionalValueOf(p_motor, Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetVPhase_Fract16, 0U)); }
static inline uint16_t Motor_User_GetVPhase_UFract16(const Motor_State_T * p_motor)   { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetVPhase_UFract16, 0U); }

/*
    Ideal electrical power physical VA as UFract16
    [0:49152] <=> [0:1.5]
*/
static inline uint16_t Motor_User_GetElectricalPower_UFract16(const Motor_State_T * p_motor)  { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetElectricalPower_UFract16, 0U); }
// static inline uint16_t Motor_User_GetPhasePower_UFract16(const Motor_State_T * p_motor)    { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetPhasePower_UFract16, 0U); }

static inline uint16_t Motor_User_GetIdc_UFract16(const Motor_State_T * p_motor) { return fract16_div(Motor_User_GetElectricalPower_UFract16(p_motor), MotorAnalog_GetVSource_Fract16()); }

/*  */
// static inline uint16_t Motor_User_GetHeat_Adcu(const Motor_State_T * p_motor) { return MotorAnalog_GetHeat((MotorAnalog_State_T*)&p_motor->AnalogState); }
static inline uint16_t Motor_User_GetHeat_Adcu(const Motor_State_T * p_motor) { return  Monitor_GetValue(&p_motor->HeatMonitorState); }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_User_GetSpeed_Rpm(const Motor_State_T * p_motor)             { return _Motor_ConvertSpeed_Fract16ToRpm(p_motor, Motor_User_GetSpeed_Fract16(p_motor)); }
static inline int16_t Motor_User_GetIPhase_Amps(const Motor_State_T * p_motor)           { return _Motor_ConvertI_Fract16ToAmps(Motor_User_GetIPhase_UFract16(p_motor)); }
static inline int16_t Motor_User_GetVPhase_Volts(const Motor_State_T * p_motor)          { return _Motor_ConvertV_Fract16ToVolts(Motor_User_GetVPhase_UFract16(p_motor)); }
static inline int32_t Motor_User_GetElectricalPower_VA(const Motor_State_T * p_motor)    { return _Motor_ConvertPower_Fract16ToWatts(Motor_User_GetElectricalPower_UFract16(p_motor)); }
static inline thermal_t Motor_User_GetHeat_DegC(const Motor_State_T * p_motor)           { return HeatMonitor_CelsiusOfAdcu(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }
#endif

static inline angle16_t Motor_User_GetElectricalAngle(const Motor_State_T * p_motor) { return RotorSensor_GetElecticalAngle(p_motor->p_ActiveSensor); }
static inline angle16_t Motor_User_GetMechanicalAngle(const Motor_State_T * p_motor) { return RotorSensor_GetMechanicalAngle(p_motor->p_ActiveSensor); }


/*
    StateMachine Status
    Set via interface functions
*/
/* alternatively use outer context */
static inline Motor_StateId_T Motor_User_GetStateId(const Motor_State_T * p_motor) { return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
/* 0 during substate */
static inline uint8_t Motor_User_GetSubStateId(const Motor_State_T * p_motor) { return _StateMachine_GetActiveSubStateId(&p_motor->StateMachine); }
static inline Motor_Direction_T Motor_User_GetRotaryDirection(const Motor_State_T * p_motor) { return p_motor->Direction; }
static inline Motor_FeedbackMode_T Motor_User_GetFeedbackMode(const Motor_State_T * p_motor) { return p_motor->FeedbackMode; }
static inline Motor_FaultFlags_T Motor_User_GetFaultFlags(const Motor_State_T * p_motor) { return p_motor->FaultFlags; }


/*
    User reference direction
    Effective Limits
*/
static inline uint16_t Motor_User_GetSpeedLimitForward(const Motor_State_T * p_motor)   { return p_motor->SpeedLimitForward_Fract16; }
static inline uint16_t Motor_User_GetSpeedLimitReverse(const Motor_State_T * p_motor)   { return p_motor->SpeedLimitReverse_Fract16; }
static inline uint16_t Motor_User_GetSpeedLimitActive(const Motor_State_T * p_motor)    { return Motor_GetSpeedLimitActive(p_motor); }

static inline uint16_t Motor_User_GetILimitMotoring(const Motor_State_T * p_motor)    { return p_motor->ILimitMotoring_Fract16; }
static inline uint16_t Motor_User_GetILimitGenerating(const Motor_State_T * p_motor)  { return p_motor->ILimitGenerating_Fract16; }
// static inline uint16_t Motor_User_GetILimit(const Motor_State_T * p_motor)         { return Motor_FOC_GetILimit(p_motor); }

/*
*/
static inline bool Motor_User_IsILimitSet(const Motor_State_T * p_motor)
{
    return (p_motor->ILimitMotoring_Fract16 != p_motor->Config.ILimitMotoring_Fract16) || (p_motor->ILimitGenerating_Fract16 != p_motor->Config.ILimitGenerating_Fract16);
}

static inline bool Motor_User_IsSpeedLimitSet(const Motor_State_T * p_motor)
{
    return (p_motor->SpeedLimitForward_Fract16 != p_motor->Config.SpeedLimitForward_Fract16) || (p_motor->SpeedLimitReverse_Fract16 != p_motor->Config.SpeedLimitReverse_Fract16);
}

/* SubStates */
// static inline uint32_t Motor_User_GetControlTimer(const Motor_State_T * p_motor)                      { return p_motor->ControlTimerBase; }
// static inline Motor_OpenLoopState_T Motor_User_GetOpenLoopState(const Motor_State_T * p_motor)        { return p_motor->OpenLoopState; }
// static inline Motor_CalibrationState_T Motor_User_GetCalibrationState(const Motor_State_T * p_motor)  { return p_motor->CalibrationState; }
// static inline uint8_t Motor_User_GetCalibrationStateIndex(const Motor_State_T * p_motor)              { return p_motor->CalibrationStateIndex; }

// static inline bool Motor_User_IsSpeedRampEnabled(const Motor_State_T * p_motor) { return !_Ramp_IsDisabled(&p_motor->SpeedRamp); }

/******************************************************************************/
/*!
    Interfaces
*/
/******************************************************************************/
typedef enum Motor_User_Direction
{
    MOTOR_DIRECTION_REVERSE = -1,
    MOTOR_DIRECTION_STOP = 0,
    MOTOR_DIRECTION_FORWARD = 1,
}
Motor_User_Direction_T;

static inline Motor_User_Direction_T Motor_User_GetDirection(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * p_motor->Direction; }
static inline bool Motor_User_IsDirection(const Motor_State_T * p_motor, Motor_User_Direction_T direction) { return (direction == Motor_User_GetDirection(p_motor)); }

/* Buffered Input for statemachine */
typedef struct Motor_User_Input
{
    uint8_t MotorId;
    int16_t CmdValue; /* [-32768:32767] */
    sign_t Direction;
    Motor_FeedbackMode_T FeedbackMode;
    Phase_Output_T ControlState;
    uint16_t SpeedLimit;
    uint16_t ILimit;
    // uint16_t RampOnOff;
    bool IsUpdated;
}
Motor_User_Input_T;
// move proc buffered

// typedef struct
// {
//     uint32_t PhaseOutput    : 2;   /* [Phase_Output_T] Active/Release/Hold */
//     uint32_t FeedbackMode   : 4;   /* [FeedbackMode_T] flags */
//     uint32_t Direction      : 2;   /* [Motor_Direction_T] Ccw/Cw Start/Stop */
// }
// Motor_ControlInput_T;

// typedef struct
// {
//     int16_t CmdValue;
//     Motor_ControlInput_T ControlInput; /* PhaseOutput, FeedbackMode, Direction */
//     uint8_t MotorId;
// }
// Motor_User_InputPacked_T;

/*
    Consolidate all status flags into a single word
*/
// typedef union Motor_User_StatusFlags
// {
//     struct
//     {
//         uint16_t HeatWarning    : 1U;
//         // uint16_t ILimitSet      : 1U;
//         // uint16_t SpeedLimitSet  : 1U;
//         uint16_t ILimited       : 1U;
//         uint16_t SpeedLimited   : 1U;
//         // uint16_t PhaseOutputState    : 2U;
//         // uint16_t Hold           : 1U;
//         // uint16_t Release        : 1U;
//     };
//     uint16_t Value;
// }
// Motor_User_StatusFlags_T;

// static inline Motor_User_StatusFlags_T Motor_User_GetStatusFlags(const Motor_State_T * p_motor)
// {
//     return (Motor_User_StatusFlags_T)
//     {
//         // .HeatWarning    = HeatMonitor_IsWarning(&p_motor->Thermistor), // alternatively as separate enum
//         // .ILimitSet      = Motor_User_IsILimitSet(p_motor),
//         // .SpeedLimitSet  = Motor_User_IsSpeedLimitSet(p_motor),
//         // .ILimited       = Motor_FOC_IsILimitReached(p_motor),
//         // .SpeedLimited   = Motor_IsSpeedLimitReached(p_motor),
//     };
// }

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_User_ActivateControlWith(const Motor_T * p_const, Motor_FeedbackMode_T mode);

extern void Motor_User_ActivateControl(const Motor_T * p_const);
extern void Motor_User_Release(const Motor_T * p_const);
extern void Motor_User_Hold(const Motor_T * p_const);
extern void Motor_User_ActivatePhaseOutput(const Motor_T * p_const, Phase_Output_T state);

extern void Motor_User_SetFeedbackMode(const Motor_T * p_const, Motor_FeedbackMode_T mode);
extern void Motor_User_SetFeedbackMode_Cast(const Motor_T * p_const, int modeValue);

extern void Motor_User_Stop(const Motor_T * p_const);
extern void Motor_User_ApplyRotaryDirection(const Motor_T * p_const, Motor_Direction_T direction);
extern void Motor_User_ApplyDirectionForward(const Motor_T * p_const);
extern void Motor_User_ApplyDirectionReverse(const Motor_T * p_const);
extern void Motor_User_ApplyDirectionSign(const Motor_T * p_motor, Motor_User_Direction_T sign);
extern void Motor_User_ForceDisableControl(const Motor_T * p_const);

extern void Motor_User_StartVoltageMode(const Motor_T * p_const);
extern void Motor_User_StartIMode(const Motor_T * p_const);
extern void Motor_User_StartTorqueMode(const Motor_T * p_const);
extern void Motor_User_StartSpeedMode(const Motor_T * p_const);

/*
    Ramp Values
*/
extern void Motor_User_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16);
extern void Motor_User_SetVoltageCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_SetVSpeedScalarCmd(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_SetRegenCmd(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_User_SetICmd(Motor_State_T * p_motor, int16_t i_fract16);
extern void Motor_User_SetICmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_User_SetTorqueCmd(Motor_State_T * p_motor, int16_t torque);
extern void Motor_User_SetTorqueCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_User_SetSpeedCmd(Motor_State_T * p_motor, int16_t speed_fract16);
extern void Motor_User_SetSpeedCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_User_SetPositionCmd(Motor_State_T * p_motor, uint16_t angle);


#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_User_EnterOpenLoopState(const Motor_T * p_motor);

extern void Motor_User_SetOpenLoopV(Motor_State_T * p_motor, int16_t volts_fract16);
extern void Motor_User_SetOpenLoopI(Motor_State_T * p_motor, int16_t amps_fract16);
extern void Motor_User_SetOpenLoopCmd(Motor_State_T * p_motor, int16_t ivCmd);
extern void Motor_User_SetOpenLoopCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_User_SetOpenLoopSpeed(Motor_State_T * p_motor, int16_t speed_fract16);
#endif

extern int32_t Motor_User_GetCmd(const Motor_State_T * p_motor);
extern int32_t Motor_User_GetSetPoint(const Motor_State_T * p_motor);
extern int32_t Motor_User_GetSetPoint_Scalar(const Motor_State_T * p_motor);

extern void Motor_User_SetActiveCmdValue(Motor_State_T * p_motor, int16_t userCmd);
extern void Motor_User_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, int16_t userCmd);


extern bool Motor_TryILimit(Motor_State_T * p_motor, uint16_t i_Fract16);
extern bool Motor_TrySpeedLimit(Motor_State_T * p_motor, uint16_t speed_Fract16);
extern void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit);
extern void Motor_SetILimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit);

extern void Motor_User_ProcSyncInput(const Motor_T * p_motor, Motor_User_Input_T * p_input);
