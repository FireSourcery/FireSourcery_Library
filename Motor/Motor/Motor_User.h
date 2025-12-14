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
    For Cmd Set
*/
/******************************************************************************/
typedef enum Motor_UserDirection
{
    MOTOR_USER_DIRECTION_REVERSE = -1,
    MOTOR_USER_DIRECTION_NONE = 0,
    MOTOR_USER_DIRECTION_FORWARD = 1,
}
Motor_UserDirection_T;

// static inline Motor_UserDirection_T Motor_GetUserDirection(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * RotorSensor_GetDirection(p_motor->RotorSensor); }
static inline Motor_UserDirection_T Motor_GetUserDirection(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * p_motor->Direction; }
static inline bool Motor_IsUserDirection(const Motor_State_T * p_motor, Motor_UserDirection_T direction) { return (direction == Motor_GetUserDirection(p_motor)); }

/******************************************************************************/
/*!
    Inline Getters/Setters
*/
/******************************************************************************/
static inline Phase_Output_T Motor_GetPhaseState(const Motor_T * p_const) { return Phase_ReadOutputState(&p_const->PHASE); }

/*! @return [-32767:32767] <=> [-1:1) speed forward as positive. reverse as negative. clamp in case of over saturation */
static inline accum32_t Motor_GetSpeed_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * Motor_GetSpeedFeedback(p_motor); }
/*! @return [0:65535] <=> [0:2) */
static inline ufract16_t Motor_GetSpeed_UFract16(const Motor_State_T * p_motor) { return math_abs(Motor_GetSpeedFeedback(p_motor)); }

/* DigitalSpeed */
static inline angle16_t _Motor_GetSpeed_DegPerCycle(const Motor_State_T * p_motor) { return RotorSensor_GetElectricalSpeed(p_motor->p_ActiveSensor); }
static inline angle16_t Motor_GetSpeed_DegPerCycle(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * RotorSensor_GetElectricalSpeed(p_motor->p_ActiveSensor); }


static inline fract16_t _Motor_GetVSpeedEffective_Fract16(const Motor_State_T * p_motor) { return Motor_GetVSpeed_Fract16(p_motor); }
static inline fract16_t Motor_GetVSpeedEffective_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * Motor_GetVSpeed_Fract16(p_motor); }
static inline ufract16_t Motor_GetVSpeedEffective_UFract16(const Motor_State_T * p_motor) { return math_abs(Motor_GetVSpeed_Fract16(p_motor)); }

/*
    Conversion functions only on user call. No periodic proc.
    Partial conversion local side. Alternatively app side handle
*/
/*!
    @return IPhase Zero to Peak.
    iPhase motoring as positive. generating as negative.
*/
static inline ufract16_t Motor_GetIPhase_UFract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetIPhase_UFract16, NULL); }
static inline fract16_t Motor_GetIPhase_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetIPhase_Fract16, NULL); }

/*
    Sampled BEMF during freewheel or VOut during active control
*/
static inline ufract16_t Motor_GetVPhase_UFract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetVPhase_UFract16, NULL); }
static inline fract16_t Motor_GetVPhase_Fract16(const Motor_State_T * p_motor) { return p_motor->Config.DirectionForward * Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetVPhase_Fract16, NULL); }

/*
    Ideal electrical power physical VA as Q2.14
    [0:49152] <=> [0:1.5]
*/
static inline ufract16_t Motor_GetElectricalPower_UFract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetElectricalPower_UFract16, NULL) / 4; }
static inline ufract16_t Motor_GetIdc_UFract16(const Motor_State_T * p_motor) { return fract16_div(Motor_GetElectricalPower_UFract16(p_motor), Phase_VBus_Fract16()); }

/*  */
static inline uint16_t Motor_GetHeat_Adcu(const Motor_State_T * p_motor) { return Monitor_GetValue(&p_motor->HeatMonitorState); }

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_GetSpeed_Rpm(const Motor_State_T * p_motor)             { return _Motor_ConvertSpeed_Fract16ToRpm(p_motor, Motor_GetSpeed_Fract16(p_motor)); }
static inline int16_t Motor_GetIPhase_Amps(const Motor_State_T * p_motor)           { return _Motor_ConvertI_Fract16ToAmps(Motor_GetIPhase_UFract16(p_motor)); }
static inline int16_t Motor_GetVPhase_Volts(const Motor_State_T * p_motor)          { return _Motor_ConvertV_Fract16ToVolts(Motor_GetVPhase_UFract16(p_motor)); }
static inline int32_t Motor_GetElectricalPower_VA(const Motor_State_T * p_motor)    { return _Motor_ConvertPower_Fract16ToWatts(Motor_GetElectricalPower_UFract16(p_motor)); }
static inline thermal_t Motor_GetHeat_DegC(const Motor_State_T * p_motor)           { return HeatMonitor_CelsiusOfAdcu(&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }
#endif

// static inline angle16_t Motor_GetElectricalAngle(const Motor_State_T * p_motor) { return RotorSensor_GetElectricalAngle(p_motor->p_ActiveSensor); }
// static inline angle16_t Motor_GetMechanicalAngle(const Motor_State_T * p_motor) { return RotorSensor_GetMechanicalAngle(p_motor->p_ActiveSensor); }


/*
    StateMachine State and StateMachine controlled values
    Set via interface functions
*/
static inline Motor_StateId_T Motor_GetStateId(const Motor_State_T * p_motor) { return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
/*
*/
static inline state_t _Motor_GetSubStateId(const Motor_State_T * p_motor) { return _StateMachine_GetActiveSubStateId(&p_motor->StateMachine); }

static inline Motor_Direction_T Motor_GetRotaryDirection(const Motor_State_T * p_motor) { return p_motor->Direction; }
static inline Motor_FeedbackMode_T Motor_GetFeedbackMode(const Motor_State_T * p_motor) { return p_motor->FeedbackMode; }
static inline Motor_FaultFlags_T Motor_GetFaultFlags(const Motor_State_T * p_motor) { return p_motor->FaultFlags; }

/*
    User reference direction
    Effective Limits
*/
static inline uint16_t Motor_GetSpeedLimitForward(const Motor_State_T * p_motor)   { return p_motor->SpeedLimitForward_Fract16; }
static inline uint16_t Motor_GetSpeedLimitReverse(const Motor_State_T * p_motor)   { return p_motor->SpeedLimitReverse_Fract16; }
// static inline uint16_t Motor_GetSpeedLimitActive(const Motor_State_T * p_motor)    { return Motor_GetSpeedLimitActive(p_motor); }

static inline uint16_t Motor_GetILimitMotoring(const Motor_State_T * p_motor)    { return p_motor->ILimitMotoring_Fract16; }
static inline uint16_t Motor_GetILimitGenerating(const Motor_State_T * p_motor)  { return p_motor->ILimitGenerating_Fract16; }
// static inline uint16_t Motor_GetILimit(const Motor_State_T * p_motor)         { return Motor_FOC_GetILimit(p_motor); }


/*
*/
static inline bool Motor_IsILimitSet(const Motor_State_T * p_motor)
{
    return (p_motor->ILimitMotoring_Fract16 != p_motor->Config.ILimitMotoring_Fract16) || (p_motor->ILimitGenerating_Fract16 != p_motor->Config.ILimitGenerating_Fract16);
}

static inline bool Motor_IsSpeedLimitSet(const Motor_State_T * p_motor)
{
    return (p_motor->SpeedLimitForward_Fract16 != p_motor->Config.SpeedLimitForward_Fract16) || (p_motor->SpeedLimitReverse_Fract16 != p_motor->Config.SpeedLimitReverse_Fract16);
}

/* SubStates */
// static inline uint32_t Motor_GetControlTimer(const Motor_State_T * p_motor)                      { return p_motor->ControlTimerBase; }
// static inline Motor_OpenLoopState_T Motor_GetOpenLoopState(const Motor_State_T * p_motor)        { return p_motor->OpenLoopState; }
// static inline Motor_CalibrationState_T Motor_GetCalibrationState(const Motor_State_T * p_motor)  { return p_motor->CalibrationState; }
// static inline uint8_t Motor_GetCalibrationStateIndex(const Motor_State_T * p_motor)              { return p_motor->CalibrationStateIndex; }


//user query
// static inline bool Motor_IsSpeedRampEnabled(const Motor_State_T * p_motor) { return !_Ramp_IsDisabled(&p_motor->SpeedRamp); }
// static inline fract16_t Motor_GetIReq(Motor_State_T * p_motor) { return (p_motor->FeedbackMode.Current == 1U) ? Motor_GetTorqueRamp(p_motor) : 0U; }
// static inline fract16_t Motor_GetVReq(Motor_State_T * p_motor) { return foc.vq; }

/******************************************************************************/
/*!
    Interfaces
*/
/******************************************************************************/
/*
    Move to SyncControl extension
*/
/* Buffered Input for statemachine */
typedef struct Motor_Input
{
    uint8_t MotorId;
    int16_t CmdValue;   /* [-32768:32767] */
    Motor_UserDirection_T Direction;
    Motor_FeedbackMode_T FeedbackMode;
    Phase_Output_T PhaseState;
    /* optionally */
    uint16_t SpeedLimit;
    uint16_t ILimit;
    // uint16_t RampOnOff;
    // stateCmd for on edge
}
Motor_Input_T;


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_ActivateControl(const Motor_T * p_const);
extern void Motor_Release(const Motor_T * p_const);
extern void Motor_Hold(const Motor_T * p_const);
extern void Motor_ActivatePhaseOutput(const Motor_T * p_const, Phase_Output_T state);

extern void Motor_ApplyFeedbackMode(const Motor_T * p_const, Motor_FeedbackMode_T mode);

extern void Motor_Stop(const Motor_T * p_const);
extern void Motor_ApplyRotaryDirection(const Motor_T * p_const, Motor_Direction_T direction);
extern void Motor_ApplyDirectionForward(const Motor_T * p_const);
extern void Motor_ApplyDirectionReverse(const Motor_T * p_const);
extern void Motor_ApplyUserDirection(const Motor_T * p_motor, Motor_UserDirection_T sign);

extern void Motor_ForceDisableControl(const Motor_T * p_const);

extern void Motor_StartVoltageMode(const Motor_T * p_const);
extern void Motor_StartIMode(const Motor_T * p_const);
extern void Motor_StartTorqueMode(const Motor_T * p_const);
extern void Motor_StartSpeedMode(const Motor_T * p_const);

/*
    Ramp Values
*/
extern void Motor_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16);
extern void Motor_SetVoltageCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_SetVSpeedScalarCmd(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_SetRegenCmd(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetICmd(Motor_State_T * p_motor, int16_t i_fract16);
extern void Motor_SetICmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetTorqueCmd(Motor_State_T * p_motor, int16_t torque);
extern void Motor_SetTorqueCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetSpeedCmd_Fract16(Motor_State_T * p_motor, int16_t speed_fract16);
extern void Motor_SetSpeedCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetPositionCmd(Motor_State_T * p_motor, uint16_t angle);


#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(MOTOR_SENSOR_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
extern void Motor_EnterOpenLoopState(const Motor_T * p_motor);

extern void Motor_SetOpenLoopV(Motor_State_T * p_motor, int16_t volts_fract16);
extern void Motor_SetOpenLoopI(Motor_State_T * p_motor, int16_t amps_fract16);
extern void Motor_SetOpenLoopCmd(Motor_State_T * p_motor, int16_t ivCmd);
extern void Motor_SetOpenLoopCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_SetOpenLoopSpeed(Motor_State_T * p_motor, int16_t speed_fract16);
#endif

extern int16_t Motor_GetCmd(const Motor_State_T * p_motor);
extern int16_t Motor_GetSetpoint(const Motor_State_T * p_motor);
extern int16_t Motor_GetSetpoint_Scalar(const Motor_State_T * p_motor);

extern void Motor_SetActiveCmdValue(Motor_State_T * p_motor, int16_t userCmd);
extern void Motor_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, int16_t userCmd);


extern bool Motor_TryILimit(Motor_State_T * p_motor, uint16_t i_Fract16);
extern bool Motor_TrySpeedLimit(Motor_State_T * p_motor, uint16_t speed_Fract16);
extern void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit);
extern void Motor_SetILimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit);

extern void Motor_ProcSyncInput(const Motor_T * p_motor, Motor_Input_T * p_input);
