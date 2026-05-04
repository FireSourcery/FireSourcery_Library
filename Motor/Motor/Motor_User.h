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
static inline Phase_Output_T Motor_GetPhaseState(const Motor_T * p_const) { return Phase_ReadOutputState(&p_const->PHASE); }

/*! @return [-32767/2:32767/2] <=> [-1:1) */
static inline accum32_t Motor_GetSpeed_Fract16(const Motor_State_T * p_motor) { return Motor_GetSpeedFeedback(p_motor) * p_motor->Config.DirectionForward; }
// /*! @return [0:65535] <=> [0:4) */
// static inline ufract16_t Motor_GetSpeed_UFract16(const Motor_State_T * p_motor) { return math_abs(Motor_GetSpeedFeedback(p_motor)); }

static inline angle16_t Motor_GetSpeed_DegPerCycle(const Motor_State_T * p_motor) { return  RotorSensor_GetElectricalDelta(p_motor->p_ActiveSensor) * p_motor->Config.DirectionForward; }

static inline fract16_t _Motor_GetVSpeedEffective_Fract16(const Motor_State_T * p_motor) { return Motor_GetVSpeed_Fract16(p_motor); }
static inline fract16_t Motor_GetVSpeedEffective_Fract16(const Motor_State_T * p_motor) { return  Motor_GetVSpeed_Fract16(p_motor) * p_motor->Config.DirectionForward; }
static inline ufract16_t Motor_GetVSpeedEffective_UFract16(const Motor_State_T * p_motor) { return math_abs(Motor_GetVSpeed_Fract16(p_motor)); }

/*
    Conversion functions only on user call. No periodic proc.
    Partial conversion local side. Alternatively app side handle
*/
/*!
    @return IPhase Zero to Peak.
*/
static inline ufract16_t Motor_GetIPhase_UFract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetIPhase_UFract16, NULL); }
/* iPhase motoring as positive. generating as negative. */
static inline fract16_t Motor_GetIPhase_Fract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetIPhase_Fract16, NULL) * p_motor->Direction; ; }

/*
    Sampled BEMF during freewheel or VOut during active control
*/
static inline ufract16_t Motor_GetVPhase_UFract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetVPhase_UFract16, NULL); }
static inline fract16_t Motor_GetVPhase_Fract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetVPhase_Fract16, NULL) * p_motor->Direction; ; }

/*
    Ideal electrical power physical VA
*/
/* [1:1.5] */
static inline uint16_t Motor_GetElectricalPower_UFract16(const Motor_State_T * p_motor) { return Motor_CommutationModeFn_Call(p_motor, Motor_FOC_GetElectricalPower_UFract16, NULL); }
static inline ufract16_t Motor_GetIBus_UFract16(const Motor_State_T * p_motor) { return FOC_GetIBus(&p_motor->Foc, Phase_VBus_Fract16()); }

/*  */
static inline uint16_t Motor_GetHeat_Adcu(const Motor_State_T * p_motor) { return Monitor_GetValue(&p_motor->HeatMonitorState); }

#ifdef MOTOR_UNIT_CONVERSION_LOCAL
static inline int16_t Motor_GetSpeed_Rpm(const Motor_State_T * p_motor)             { return  (p_motor, Motor_GetSpeed_Fract16(p_motor)); }
static inline int16_t Motor_GetIPhase_Amps(const Motor_State_T * p_motor)           { return  (Motor_GetIPhase_UFract16(p_motor)); }
static inline int16_t Motor_GetVPhase_Volts(const Motor_State_T * p_motor)          { return  (Motor_GetVPhase_UFract16(p_motor)); }
static inline int32_t Motor_GetElectricalPower_VA(const Motor_State_T * p_motor)    { return  (Motor_GetElectricalPower_UFract16(p_motor)); }
static inline thermal_t Motor_GetHeat_DegC(const Motor_State_T * p_motor)           { return  (&p_motor->Thermistor, p_motor->AnalogResults.Heat_Adcu); }
#endif

/******************************************************************************/
/*!
    Control State
*/
/******************************************************************************/
/*
    Caller maintain feedback state to determine units
*/
static inline int16_t _Motor_GetIVSetpoint(const Motor_State_T * p_motor)               { return Ramp_GetOutput(&p_motor->TorqueRamp) * p_motor->Direction; }
static inline int16_t _Motor_GetSpeedMotoringSetpoint(const Motor_State_T * p_motor)    { return Ramp_GetOutput(&p_motor->SpeedRamp) * p_motor->Direction; }

static inline int16_t _Motor_GetTorqueSetpoint(const Motor_State_T * p_motor)           { return Ramp_GetOutput(&p_motor->TorqueRamp) * p_motor->Config.DirectionForward; }
static inline int16_t _Motor_GetSpeedSetpoint(const Motor_State_T * p_motor)            { return Ramp_GetOutput(&p_motor->SpeedRamp) * p_motor->Config.DirectionForward; }

static inline fract16_t Motor_GetVSetpoint(const Motor_State_T * p_motor) { return (p_motor->FeedbackMode.Current == 0U) ? _Motor_GetTorqueSetpoint(p_motor) : 0; }
static inline fract16_t Motor_GetISetpoint(const Motor_State_T * p_motor) { return (p_motor->FeedbackMode.Current == 1U) ? _Motor_GetTorqueSetpoint(p_motor) : 0; }
static inline fract16_t Motor_GetSpeedSetpoint(const Motor_State_T * p_motor) { return (p_motor->FeedbackMode.Speed == 1U) ? _Motor_GetSpeedSetpoint(p_motor) : 0; }
// static inline bool Motor_IsSpeedRampEnabled(const Motor_State_T * p_motor) { return !_Ramp_IsDisabled(&p_motor->SpeedRamp); }

/* quick derive view */
static inline ufract16_t Motor_User_ILimitMotoring(const Motor_State_T * p_motor) { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? p_motor->ILimitCcw_Fract16 : -p_motor->ILimitCw_Fract16; }
static inline ufract16_t Motor_User_ILimitGenerating(const Motor_State_T * p_motor) { return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? -p_motor->ILimitCw_Fract16 : p_motor->ILimitCcw_Fract16; }
/* Direction-forward speed limit, virtual via Ccw/Cw. */
static inline ufract16_t Motor_User_SpeedLimit(const Motor_State_T * p_motor) { return (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? (ufract16_t)p_motor->SpeedLimitCcw_Fract16 : (ufract16_t)(-p_motor->SpeedLimitCw_Fract16); }

/*

*/
// static inline bool Motor_IsILimitSet(const Motor_State_T * p_motor)
// {
//     return (Motor_User_ILimitMotoring(p_motor) != p_motor->Config.ILimitMotoring_Fract16) || (Motor_User_ILimitGenerating(p_motor) != p_motor->Config.ILimitGenerating_Fract16);
// }
// static inline bool Motor_IsSpeedLimitSet(const Motor_State_T * p_motor)
// {
//     return (Motor_SpeedLimitForward(p_motor) != p_motor->Config.SpeedLimitForward_Fract16) || (Motor_SpeedLimitReverse(p_motor) != p_motor->Config.SpeedLimitReverse_Fract16);
// }




/******************************************************************************/
/*!
    Data Interfaces
*/
/******************************************************************************/
/* Move to SyncControl extension */
/* Buffered Sync Input for statemachine */
typedef struct Motor_InputData
{
    uint8_t MotorId;
    int16_t CmdValue;   /* [-32768:32767] */
    Motor_Direction_T Direction;
    Motor_FeedbackMode_T FeedbackMode;
    Phase_Output_T PhaseOutput;
    /* optionally */
    uint16_t SpeedLimit;
    uint16_t ILimit;
    // uint16_t RampOnOff;
    // stateCmd for on edge
}
Motor_Input_T;


// typedef int16_t motor_cmd_t;
// typedef const struct Motor_Drive
// {
//     Motor_FeedbackMode_T FEEDBACK_MODE;
//     void (*APPLY_CMD)(Motor_State_T * p_motor, motor_cmd_t value);
// }
// Motor_Drive_T;

/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Motor_ActivateControl(const Motor_T * p_const);
extern void Motor_ReleaseVZ(const Motor_T * p_const);
extern void Motor_ReleaseV0(const Motor_T * p_const);
extern void Motor_ApplyControlState(const Motor_T * p_const, Phase_Output_T state);

extern void Motor_Disable(const Motor_T * p_const);
extern void Motor_Enable(const Motor_T * p_motor);

extern void Motor_ApplyFeedbackMode(const Motor_T * p_const, Motor_FeedbackMode_T mode);

extern void Motor_ApplyVirtualDirection(const Motor_T * p_const, Motor_Direction_T direction);
extern void Motor_ApplyUserDirection(const Motor_T * p_motor, Motor_Direction_T sign);
extern void Motor_ApplyDirectionForward(const Motor_T * p_const);
extern void Motor_ApplyDirectionReverse(const Motor_T * p_const);

extern void Motor_ForceDisableControl(const Motor_T * p_const);

extern void Motor_StartVoltageMode(const Motor_T * p_const);
extern void Motor_StartIMode(const Motor_T * p_const);
extern void Motor_StartTorqueMode(const Motor_T * p_const);
extern void Motor_StartSpeedMode(const Motor_T * p_const);

/*
    Ramp Values
*/
extern void Motor_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16);
extern void Motor_SetVoltageCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_SetVSpeedScalarCmd(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_SetRegenCmd(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetICmd(Motor_State_T * p_motor, int16_t i_fract16);
extern void Motor_SetICmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_ApplyTorque0(const Motor_T * p_motor);

extern void Motor_SetTorqueCmd(Motor_State_T * p_motor, int16_t torque);
extern void Motor_SetTorqueCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetSpeedMotoringCmd(Motor_State_T * p_motor, int16_t speed_fract16);
extern void Motor_SetSpeedMotoringCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16);
extern void Motor_SetSpeedCmd(Motor_State_T * p_motor, int16_t speed_fract16);
extern void Motor_SetSpeedCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16);

extern void Motor_SetPositionCmd(Motor_State_T * p_motor, uint16_t angle);


extern void Motor_SetActiveCmdScalar(Motor_State_T * p_motor, int16_t userCmd);


extern bool Motor_TryILimit(Motor_State_T * p_motor, uint16_t i_Fract16);
extern bool Motor_TrySpeedLimit(Motor_State_T * p_motor, uint16_t speed_Fract16);

/* Single-pointer Apply path — pulls system derate from Motor_T.P_SYSTEM_* and composes with inline local. */
void Motor_SetILimitWith(const Motor_T * p_motor);
void Motor_SetSpeedLimitWith(const Motor_T * p_motor);

/* Old two-array forwarders preserved as comments for migration reference. */
// void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_local, LimitArray_T * p_system);
// void Motor_SetILimitMotoringWith(Motor_State_T * p_motor, LimitArray_T * p_local, LimitArray_T * p_system);
// void Motor_SetILimitGeneratingWith(Motor_State_T * p_motor, LimitArray_T * p_local, LimitArray_T * p_system);

extern void Motor_ProcSyncInput(const Motor_T * p_motor, Motor_Input_T * p_input);
