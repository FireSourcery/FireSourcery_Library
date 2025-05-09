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
    @file   Motor_User.c
    @author FireSourcery
    @version V0

    @brief
*/
/******************************************************************************/
#include "Motor_User.h"
#include "System/Critical/Critical.h"
#include "Motor_StateMachine.h"

/******************************************************************************/
/*!
    StateMachine handled inputs

    Motor State Machine Thread Safety
    State Proc in PWM thread.
    User Input [Motor_User_ActivateControlWith] in Main thread.

    Sync Mode -
        Inputs do not directly proc transition, set for sync proc
        Must check input flags every control cycle.
        Multiple calls to [StateMachine_SetInput] overwrites within 1 control cycle may be lost (e.g within 1 packet)
            handle with input image buffer, input queue, spin-lock delegates to Xcvr buffer, or client side constraints

    Async Mode -
        Need critical section during input/ input transition
        No Critical during transistion -> Prev State Proc may overwrite new State Entry.

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Control State On/Off using Phase Output On/Off
*/
/******************************************************************************/
/*
    Set [FeedbackMode] and Transition to Run State
*/
inline void Motor_User_ActivateControlWith(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    // todo merge or deprecate
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FEEDBACK_MODE, mode.Value);
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_STATE, PHASE_OUTPUT_VPWM);
}

/* Generic array functions use */
// inline void Motor_User_StartControlMode_Cast(Motor_T * p_motor, int modeValue)
// {
//     Motor_User_ActivateControlWith(p_motor, Motor_FeedbackMode_Cast(modeValue));
// }

/*
    Phase_Output_T as ControlState
    alternatively use Motor_StateMachine_StateId_T
    Run
*/
inline void Motor_User_ActivateControl(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_STATE, PHASE_OUTPUT_VPWM);
}

/*
    StateMachine checked disable
*/
inline void Motor_User_Release(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_STATE, PHASE_OUTPUT_FLOAT);
}

inline void Motor_User_Hold(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_STATE, PHASE_OUTPUT_V0);
}

inline void Motor_User_ActivateControlState(Motor_T * p_motor, Phase_Output_T state)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_STATE, state);
}

/*
   Disable control non StateMachine checked
*/
void Motor_User_ForceDisableControl(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    Motor_User_Release(p_motor);
    Motor_User_Stop(p_motor);
}


/******************************************************************************/
/*!
    Feedback Control Modes: torque/speed/position, above Commutation layer: foc/six-step

    UserCmdValue functions
    UserCmdValue sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
        Handle direction and limits check on input.
        Limits check again in feedback loop, as input may have stopped
    Call regularly to update cmd value, lower frequency compared to control loop, ~100-1000Hz.

    SetMode       - Invokes StateMachine - Sets control mode only
    SetCmd[Value] - Without invoking StateMachine - Sets buffered cmd value, sets on all states even when inactive

    Cmd as scalar of Calibration Ref, clamped by Config.Limit
    _Scalar as scalar of Config.Limit
*/
/******************************************************************************/
/* User set [FeedbackMode] without starting Run */
inline void Motor_User_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    // if (mode.Word != p_motor->FeedbackMode.Word)
    StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_FEEDBACK_MODE, mode.Value);
}

inline void Motor_User_SetFeedbackMode_Cast(Motor_T * p_motor, int modeValue)
{
    Motor_User_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(modeValue));
}

/*
    Cmd Value
    Bypasses User Mode restrictions, bounds
    Concurrency note: only 1 thread updates RampTarget. StateMachine_Proc thread only updates OutputState
    Ramp input allows over saturated input
*/
// static inline void _Motor_User_SetCmd(Motor_T * p_motor, int32_t userCmd)
// {
//     Ramp_SetTarget(&p_motor->Ramp, Motor_DirectionalValueOf(p_motor, userCmd));
// }

/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_StartVoltageMode(Motor_T * p_motor) { Motor_User_ActivateControlWith(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE); }

/*!
    @param[in] voltage [-32768:32767]
*/
void Motor_User_SetVoltageCmd(Motor_T * p_motor, int16_t volts_fract16)
{
    int32_t limitedCmd = math_clamp(volts_fract16, 0, MotorAnalogRef_GetVSource_Fract16() / 2);     /* Reverse voltage set direction, no plugging */
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}

void Motor_User_SetVoltageCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    int32_t limitedCmd = (scalar_fract16 > 0) ? fract16_mul(MotorAnalogRef_GetVSource_Fract16() / 2, scalar_fract16) : 0;
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));

}

// [v/2 +/- scalar * ]
void Motor_User_SetVSpeedScalarCmd(Motor_T * p_motor, int16_t scalar_fract16)
{
    // Motor_User_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 2);
}

/******************************************************************************/
/*!
    Current Mode
*/
/******************************************************************************/
void Motor_User_StartIMode(Motor_T * p_motor) { Motor_User_ActivateControlWith(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); }

/*!
    @param[in] i [-32768:32767]
*/
/* Scalar of [IRatedPeak_Fract16] */
void Motor_User_SetICmd(Motor_T * p_motor, int16_t i_Fract16)
{
    int32_t limitedCmd = math_clamp(i_Fract16, (int32_t)0 - Motor_User_GetILimitGenerating(p_motor), Motor_User_GetILimitMotoring(p_motor));
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}

/* Scalar of Config.Limit */
void _Motor_User_SetICmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    // int32_t limitedCmd = fract16_mul((scalar_fract16 > 0) ? Motor_User_GetILimitMotoring(p_motor) : Motor_User_GetILimitGenerating(p_motor), scalar_fract16);
    int32_t limitedCmd = fract16_mul((scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16, scalar_fract16);
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}

void Motor_User_SetICmd_Scalar(Motor_T * p_motor, motor_value_t scalar_fract16)
{
    _Motor_User_SetICmd_Scalar(p_motor, scalar_fract16);
}

/******************************************************************************/
/*!
    Torque Mode
    case for additional process
*/
/******************************************************************************/
void Motor_User_StartTorqueMode(Motor_T * p_motor) { Motor_User_StartIMode(p_motor); }

void Motor_User_SetTorqueCmd(Motor_T * p_motor, int16_t value_Fract16)
{
    Motor_User_SetICmd(p_motor, value_Fract16);
}

/*!
    @param[in] torque [-32768:32767]
*/
void Motor_User_SetTorqueCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetICmd_Scalar(p_motor, scalar_fract16);
    // if (scalar_fract16 < 0)
    // if (scalar_fract16 > 0)
    // if (scalar_fract16 == 0)
}

// void Motor_User_SetIFollow(Motor_T * p_motor)
// {
//     Motor_User_SetTorqueCmd(p_motor, 0U);
// }


/******************************************************************************/
/*!
    Speed Mode
*/
/******************************************************************************/
/*!
    Default speed mode is speed torque mode
*/
void Motor_User_StartSpeedMode(Motor_T * p_motor) { Motor_User_ActivateControlWith(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT); }

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION,
        Hall sensor speed interpolation, Limit Directions
    @param[in] speed [-32768:32767]
*/
void Motor_User_SetSpeedCmd(Motor_T * p_motor, int16_t speed_fract16)
{
    int32_t limitedCmd = math_clamp(speed_fract16, 0, Motor_User_GetSpeedLimit(p_motor));
    Ramp_SetTarget(&p_motor->SpeedRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}

static inline uint16_t _Motor_GetSpeedLimitConfig(const Motor_T * p_motor) { return (Motor_IsDirectionForward(p_motor) ? p_motor->Config.SpeedLimitForward_Fract16 : p_motor->Config.SpeedLimitReverse_Fract16); }

void Motor_User_SetSpeedCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    int32_t limitedCmd = (scalar_fract16 > 0) ? fract16_mul(_Motor_GetSpeedLimitConfig(p_motor), scalar_fract16) : 0;
    Ramp_SetTarget(&p_motor->SpeedRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}

void Motor_User_SetSpeedCmdScalar(Motor_T * p_motor, motor_value_t scalar_fract16)
{
    Motor_User_SetSpeedCmd_Scalar(p_motor, scalar_fract16);
}


/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
/*!
    @param[in] angle [0:65535]
*/
void Motor_User_SetPositionCmd(Motor_T * p_motor, uint16_t angle)
{
    // _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
    // _Motor_User_SetCmd(p_motor, angle);
}

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
/*!

*/
// void Motor_User_StartOpenLoopMode(Motor_T * p_motor) { Motor_User_ActivateControlWith(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR); }
/*
*/
void Motor_User_StartOpenLoopState(Motor_T * p_motor)
{
    // StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_OPEN_LOOP, MOTOR_OPEN_LOOP_STATE_ENTER);
    // StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_OPEN_LOOP, state);
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_OPEN_LOOP, (uintptr_t)&MOTOR_STATE_OPEN_LOOP); /* Set FeedbackMode on Entry */
}

/*
    Open Loop Align
*/
void Motor_User_SetOpenLoopV(Motor_T * p_motor, int16_t volts_fract16)
{
    int32_t limitedCmd = math_clamp(volts_fract16, 0, fract16_mul(MotorAnalogRef_GetVSource_Fract16() / 2, FRACT16(0.1)));
    Motor_User_SetICmd(p_motor, limitedCmd);
}

/* Calibration */
void Motor_User_SetOpenLoopI(Motor_T * p_motor, int16_t amps_fract16)
{
    int32_t limitedCmd = math_clamp(amps_fract16, 0, p_motor->Config.OpenLoopIFinal_Fract16);
    Motor_User_SetICmd(p_motor, limitedCmd);
}

/*!

*/
/* Motor_User_SetOpenLoopPower */
/* Scalar of Saturation Max */
void Motor_User_SetOpenLoopCmd(Motor_T * p_motor, int16_t ivCmd)
{
    if (p_motor->FeedbackMode.Current == 1U) { Motor_User_SetOpenLoopI(p_motor, ivCmd); }
    else { Motor_User_SetOpenLoopV(p_motor, ivCmd); }
}

/* Scalar of Config Limits */
void Motor_User_SetOpenLoopCmd_Scalar(Motor_T * p_motor, int16_t scalar_fract16)
{
    if (p_motor->FeedbackMode.Current == 1U)
    {
        int32_t limitedCmd = fract16_mul((scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16, scalar_fract16);
        Motor_User_SetOpenLoopI(p_motor, limitedCmd);
    }
}

/*
    Open Loop Start Up
    Preset Ramp
*/
void Motor_User_SetOpenLoopSpeed(Motor_T * p_motor, int16_t speed_fract16)
{
    int32_t limitedCmd = math_clamp(speed_fract16, 0, p_motor->Config.OpenLoopSpeedFinal_Fract16);
    Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}

#endif


/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/*!
    Untyped scalar to calibration ref, clamp by limit of the active mode
    @param[in] userCmd[-32768:32767] mixed units
*/
void Motor_User_SetActiveCmdValue(Motor_T * p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmd(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmd(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetICmd(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmd(p_motor, userCmd); }
}

/* Scalar to limit */
void Motor_User_SetActiveCmdValue_Scalar(Motor_T * p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd_Scalar(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmd(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmd_Scalar(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetICmd_Scalar(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmd_Scalar(p_motor, userCmd); }
}


void Motor_User_SetActiveCmdValue_Scalar_Cast(Motor_T * p_motor, int userCmd)
{
    Motor_User_SetActiveCmdValue_Scalar(p_motor, (int16_t)userCmd);
}

/*
    Alternatively store Input Value
*/

/*! @return [-32767:32767] <=> [-1:1] */
int32_t _Motor_User_GetCmd(const Motor_T * p_motor)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;
    int32_t cmd;
    if (flags.Speed == 1U)  { cmd = Ramp_GetTarget(&p_motor->SpeedRamp); }
    else                    { cmd = Ramp_GetTarget(&p_motor->TorqueRamp); }
    return cmd;
}

/*! @return [-32767:32767] <=> [-1:1] */
int32_t Motor_User_GetCmd(const Motor_T * p_motor)
{
    return Motor_DirectionalValueOf(p_motor, _Motor_User_GetCmd(p_motor));
}

int32_t _Motor_User_GetSetPoint(const Motor_T * p_motor)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;
    int32_t cmd;
    if (flags.Speed == 1U)  { cmd = Ramp_GetOutput(&p_motor->SpeedRamp); }
    else                    { cmd = Ramp_GetOutput(&p_motor->TorqueRamp); }
    return cmd;
}

int32_t Motor_User_GetSetPoint(const Motor_T * p_motor)
{
    return Motor_DirectionalValueOf(p_motor, _Motor_User_GetSetPoint(p_motor));
}

// int32_t _Motor_User_GetSetPoint_Scalar(const Motor_T * p_motor)
// {
//     Motor_FeedbackMode_T flags = p_motor->FeedbackMode;
//     int32_t cmd;
//     if (flags.Speed == 1U)  { cmd = fract16_div(Ramp_GetOutput(&p_motor->SpeedRamp), INT16_MAX); }
//     else                    { cmd = fract16_div(Ramp_GetOutput(&p_motor->TorqueRamp), MotorAnalogRef_GetIRatedPeak_Fract16()); }
//     return cmd;
// }

// int32_t Motor_User_GetSetPoint_Scalar(const Motor_T * p_motor)
// {
//     return Motor_DirectionalValueOf(p_motor, _Motor_User_GetSetPoint_Scalar(p_motor));
// }



/******************************************************************************/
/*!
    Direction
*/
/******************************************************************************/
/*
    ProcInput use Critical, as States may transition during input
*/
void Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if (p_motor->Direction != direction) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
}

void Motor_User_SetDirectionForward(Motor_T * p_motor) { Motor_User_SetDirection(p_motor, p_motor->Config.DirectionForward); }
void Motor_User_SetDirectionReverse(Motor_T * p_motor) { Motor_User_SetDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }


bool Motor_User_TryDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    if (p_motor->Direction != direction) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction); }
    return (direction == p_motor->Direction);
}

bool Motor_User_TryDirectionForward(Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, p_motor->Config.DirectionForward); }
bool Motor_User_TryDirectionReverse(Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }

/* Transition to Stop */
void Motor_User_Stop(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, MOTOR_DIRECTION_NULL);
}


/******************************************************************************/
/*
    Ground Speed
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
int16_t Motor_User_GetGroundSpeed_Kmh(Motor_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     speed = Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             speed = 0;     break;
    }

    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

int16_t Motor_User_GetGroundSpeed_Mph(Motor_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             speed = 0;     break;
    }

    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

void Motor_User_SetGroundSpeed_Kmh(Motor_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:     break;
    }
}

void Motor_User_SetGroundSpeed_Mph(Motor_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:      Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             break;
    }
}
#endif


