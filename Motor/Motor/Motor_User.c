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
    @file   Motor_User.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Motor_User.h"


/******************************************************************************/
/*!
    StateMachine handled inputs

    Motor State Machine Thread Safety
    State Proc in PWM thread.
    User Input [StateMachine_ProcInput] in Main thread.

    Sync Mode -
        Inputs do not directly proc transition, set for sync proc
        overheader - check input flags every control cycle.
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
    StateMachine Inputs use full context
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Phase Output State
    Phase Output directly mapping to a Control State: Feedback Run, Freewheel, Hold
*/
/******************************************************************************/
inline void Motor_User_ActivateControl(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_VPWM); }

inline void Motor_User_Release(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_FLOAT); }

inline void Motor_User_Hold(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_V0); }

inline void Motor_User_ActivatePhaseOutput(const Motor_T * p_motor, Phase_Output_T state) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, state); }

/******************************************************************************/
/*!
    Feedback Control Mode: torque/speed/position, above Commutation layer: foc/six-step

    UserCmdValue functions
    UserCmdValue sign +/- indicates along or against Direction selected. NOT virtual CW/CCW.
        Handle direction and limits check on input.
        Limits checked again in feedback loop, as input may have stopped
    Update ~100-1000Hz.

    SetMode       - Invokes StateMachine - Sets control mode only
    SetCmd[Value] - Without invoking StateMachine - Sets buffered cmd value, sets on all states even when inactive

    Caller handle input state to cmd.
    Ramp is the only input that stores input state. other input to Motor proc async. Additional handling by caller

    Cmd as scalar of Calibration Ref, clamped by Config.Limit
    Cmd_Scalar as scalar of Config.Limit
*/
/******************************************************************************/
/*
    User set [FeedbackMode] without starting Run
    Using sync modde. activates the next pwm cycle
*/
inline void Motor_User_SetFeedbackMode(const Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    if (mode.Value != p_motor->P_MOTOR_STATE->FeedbackMode.Value) { StateMachine_SetInput(&p_motor->STATE_MACHINE, MSM_INPUT_FEEDBACK_MODE, mode.Value); }
}

inline void Motor_User_SetFeedbackMode_Cast(const Motor_T * p_motor, int modeValue)
{
    Motor_User_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(modeValue));
}


/*
    Set [FeedbackMode] and Transition to Run State
*/
// merge or deprecate /* input buffer feedback mode or pass with combined cmd */
// inline void Motor_User_ActivateControlWith(const Motor_T * p_motor, Motor_FeedbackMode_T mode)
// {
//     Motor_User_SetFeedbackMode(p_motor, mode);
//     StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_VPWM);
// }

// /* Generic array functions use */
// void Motor_User_ActivateControlWith_Cast(const Motor_T * p_motor, int modeValue)
// {
//     Motor_User_ActivateControlWith(p_motor, Motor_FeedbackMode_Cast(modeValue));
// }

/******************************************************************************/
/*!
    Direction/Stop
*/
/******************************************************************************/
/*
    Transition to Stop
    Release stays in ready mode. Stop disables inputs until next Start.
*/
void Motor_User_Stop(const Motor_T * p_motor) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, MOTOR_DIRECTION_NULL); }

/*
    ProcInput use Critical, as States may transition during input
    SetInput use async polling for status
*/
void Motor_User_ApplyRotaryDirection(const Motor_T * p_motor, Motor_Direction_T direction)
{
    if (p_motor->P_MOTOR_STATE->Direction != direction) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, direction); }
}

void Motor_User_ApplyDirectionForward(const Motor_T * p_motor) { Motor_User_ApplyRotaryDirection(p_motor, p_motor->P_MOTOR_STATE->Config.DirectionForward); }
void Motor_User_ApplyDirectionReverse(const Motor_T * p_motor) { Motor_User_ApplyRotaryDirection(p_motor, Motor_GetDirectionReverse(p_motor->P_MOTOR_STATE)); }


/* 1, 0, -1 */
void Motor_User_ApplyDirectionSign(const Motor_T * p_motor, int sign) { Motor_User_ApplyRotaryDirection(p_motor, p_motor->P_MOTOR_STATE->Config.DirectionForward * sign); }

/******************************************************************************/
/*
    Alternatively Caller query
*/
/******************************************************************************/
// bool Motor_User_TryDirection(const Motor_T * p_motor, Motor_Direction_T direction)
// {
//     if (p_motor->P_MOTOR_STATE->Direction != direction) { StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, direction); }
//     return (direction == p_motor->P_MOTOR_STATE->Direction);
// }

// bool Motor_User_TryDirectionForward(const Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, p_motor->P_MOTOR_STATE->Config.DirectionForward); }
// bool Motor_User_TryDirectionReverse(const Motor_T * p_motor) { return Motor_User_TryDirection(p_motor, Motor_GetDirectionReverse(p_motor->P_MOTOR_STATE)); }


/******************************************************************************/
/*
   Disable control non StateMachine checked
*/
/******************************************************************************/
void Motor_User_ForceDisableControl(const Motor_T * p_motor)
{
    Phase_Float(&p_motor->PHASE);
    Motor_User_Release(p_motor);
    Motor_User_Stop(p_motor);
}

/******************************************************************************/
/*
    Cmd Modes
*/
/******************************************************************************/


// /* alternatively using cmd pattern */
// typedef const struct Motor_Cmd
// {
//     Motor_FeedbackMode_T FEEDBACK_MODE; /* Feedback Mode */
//     void(*SET_CMD)(Motor_State_T * p_motor, int16_t value_fract16);
//     void(*SET_CMD_SCALAR)(Motor_State_T * p_motor, int16_t scalar_fract16);
// }
// Motor_Cmd_T;

// static inline void Motor_User_StartCmdMode(Motor_State_T * p_motor, Motor_Cmd_T cmd) { Motor_User_SetFeedbackMode(p_motor, cmd.FEEDBACK_MODE); }
// static inline void Motor_User_SetCmdValue(Motor_State_T * p_motor, Motor_Cmd_T cmd, int16_t value) { cmd.SET_CMD(p_motor, value); }
// static inline void Motor_User_SetCmdValueScalar(Motor_State_T * p_motor, Motor_Cmd_T cmd, int16_t value) { cmd.SET_CMD_SCALAR(p_motor, value); }

/******************************************************************************/
/*
    Cmd Value
    Concurrency note: only 1 thread updates RampTarget. StateMachine_Proc thread only updates OutputState

    alternatively split ramp by thread of execution
*/
/******************************************************************************/

/*
    Helper
*/
static inline void _Motor_User_SetTorqueCmd(Motor_State_T * p_motor, int16_t userCmd) { Ramp_SetTarget(&p_motor->TorqueRamp, Motor_DirectionalValueOf(p_motor, userCmd)); }
static inline int32_t _Motor_User_GetTorqueCmd(const Motor_State_T * p_motor) { return Ramp_GetTarget(&p_motor->TorqueRamp); }
static inline int32_t _Motor_User_GetTorqueSetPoint(const Motor_State_T * p_motor) { return Ramp_GetOutput(&p_motor->TorqueRamp); }
static inline int32_t Motor_User_GetTorqueCmd(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)); }
static inline int32_t Motor_User_GetTorqueSetPoint(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, Ramp_GetOutput(&p_motor->TorqueRamp)); }


/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_StartVoltageMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE); }

/*!
    @param[in] voltage [0:32767]
*/
void Motor_User_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16)
{
    int32_t limitedCmd = math_clamp(volts_fract16, 0, MotorAnalog_GetVSource_Fract16() / 2);     /* Reverse voltage set direction, no plugging */
    _Motor_User_SetTorqueCmd(p_motor, limitedCmd);
}

void Motor_User_SetVoltageCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    int32_t limitedCmd = (scalar_fract16 > 0) ? fract16_mul(MotorAnalog_GetVSource_Fract16() / 2, scalar_fract16) : 0;
    _Motor_User_SetTorqueCmd(p_motor, limitedCmd);
}

// [v/2 +/- scalar * ]
// void Motor_User_SetVSpeedScalarCmd(Motor_State_T * p_motor, int16_t scalar_fract16)
// {
//     // Motor_User_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 2);
// }

/* todo */
void _Motor_User_SetVRegenCmd(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    // Motor_User_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 4);
}


/******************************************************************************/
/*!
    Current Mode
*/
/******************************************************************************/
/* Match to 0 torque on start */
void Motor_User_StartIMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); }

/*!
    @param[in] current [-32768:32767]
*/
/* Scalar of [IRatedPeak_Fract16] */
void Motor_User_SetICmd(Motor_State_T * p_motor, int16_t i_Fract16)
{
    int32_t limitedCmd = math_clamp(i_Fract16, (int32_t)0 - Motor_User_GetILimitGenerating(p_motor), Motor_User_GetILimitMotoring(p_motor));
    _Motor_User_SetTorqueCmd(p_motor, limitedCmd);
}

/* Scalar of Config.Limit */
void Motor_User_SetICmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    int32_t limitedCmd = fract16_mul(((scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16), scalar_fract16);
    _Motor_User_SetTorqueCmd(p_motor, limitedCmd);
}

/******************************************************************************/
/*!
    Torque Mode
    case for additional process
*/
/******************************************************************************/
// void Motor_User_StartTorqueMode(const Motor_T * p_motor)
// {
//     if (p_motor->P_MOTOR_STATE->FeedbackMode.Current == 1U) { Motor_User_StartIMode(p_motor); }
//     else { Motor_User_StartVoltageMode(p_motor); }
// }

void Motor_User_SetTorqueCmd(Motor_State_T * p_motor, int16_t value_Fract16)
{
    if (p_motor->FeedbackMode.Current == 1U)    { Motor_User_SetICmd(p_motor, value_Fract16); }
    else                                        { Motor_User_SetVoltageCmd(p_motor, value_Fract16); }
    // Motor_User_SetICmd(p_motor, value_Fract16);
}

/*!
    @param[in] torque [-32768:32767]
*/
void Motor_User_SetTorqueCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    if (p_motor->FeedbackMode.Current == 1U) { Motor_User_SetICmd_Scalar(p_motor, scalar_fract16); }
    else                                     { Motor_User_SetVoltageCmd_Scalar(p_motor, scalar_fract16); }

    // if (scalar_fract16 < 0)
    // if (scalar_fract16 > 0)
    // if (scalar_fract16 == 0)
}

// void Motor_User_SetIFollow(Motor_State_T * p_motor)
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
void Motor_User_StartSpeedMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT); }

/*

*/
static inline void _Motor_User_SetSpeedCmd(Motor_State_T * p_motor, int16_t userCmd) { Ramp_SetTarget(&p_motor->SpeedRamp, Motor_DirectionalValueOf(p_motor, userCmd)); }

static inline int32_t Motor_User_GetSpeedCmd(const Motor_State_T * p_motor) { return Ramp_GetTarget(&p_motor->SpeedRamp); }
static inline int32_t Motor_User_GetSpeedSetPoint(const Motor_State_T * p_motor) { return Ramp_GetOutput(&p_motor->SpeedRamp); }

/*!
    Only allow forward direction, reverse direction use MOTOR_DIRECTION, for now
        Hall sensor speed interpolation, Limit Directions

    @param[in] speed [0:32767]
    todo as angle speed
*/
void Motor_User_SetSpeedCmd(Motor_State_T * p_motor, int16_t speed_fract16)
{
    int32_t limitedCmd = math_clamp(speed_fract16, 0, Motor_GetSpeedLimitActive(p_motor));
    _Motor_User_SetSpeedCmd(p_motor, limitedCmd);
}

/* positive ad along direction selected */
void Motor_User_SetSpeedCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    // if (p_motor->FeedbackMode.Speed == 1U)
    int32_t limitedCmd = (scalar_fract16 > 0) ? fract16_mul(Motor_GetSpeedLimitActive(p_motor), scalar_fract16) : 0;
    _Motor_User_SetSpeedCmd(p_motor, limitedCmd);
}

// static inline int32_t Motor_User_GetSpeedSetPoint_Scalar(const Motor_State_T * p_motor) { return fract16_mul(Motor_GetSpeedLimitActive(p_motor), Ramp_GetOutput(&p_motor->SpeedRamp)); }

/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
/*!
    @param[in] angle [0:65535]
*/
void Motor_User_SetPositionCmd(Motor_State_T * p_motor, uint16_t angle)
{
    // _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
    // _Motor_User_SetCmd(p_motor, angle);
}

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
/*!

*/
// void Motor_User_StartOpenLoopMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR); }

/*
*/
void Motor_User_StartOpenLoopState(const Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, (uintptr_t)&MOTOR_STATE_OPEN_LOOP); /* Set FeedbackMode on Entry */
    // StateMachine_SetInput(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, state);
}

/* todo align as positive only, rotation cmd use limits */
/*
    Open Loop Align
*/
void Motor_User_SetOpenLoopV(Motor_State_T * p_motor, int16_t volts_fract16)
{
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_OpenLoopVLimitOf(p_motor, volts_fract16));
}

/* Calibration - set CCW first */
void Motor_User_SetOpenLoopI(Motor_State_T * p_motor, int16_t amps_fract16)
{
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_OpenLoopILimitOf(p_motor, amps_fract16)); /* setting as directional, altneratively clamp wiht 0 */
}

/*!

*/
/* Motor_User_SetOpenLoopPower */
/* Scalar of Saturation Max */
void Motor_User_SetOpenLoopCmd(Motor_State_T * p_motor, int16_t ivCmd)
{
    if (p_motor->FeedbackMode.Current == 1U)    { Motor_User_SetOpenLoopI(p_motor, ivCmd); }
    else                                        { Motor_User_SetOpenLoopV(p_motor, ivCmd); }
}

/* Scalar of Config Limits */
void Motor_User_SetOpenLoopCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    if (p_motor->FeedbackMode.Current == 1U)
    {
        Motor_User_SetOpenLoopI(p_motor, fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_fract16));
    }
    else
    {
        Motor_User_SetOpenLoopV(p_motor, fract16_mul(MotorAnalog_GetVSource_Fract16() / 2, scalar_fract16));
    }
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    On the fly adjustment of Open Loop Speed
    Open Loop Start Up
    Preset Ramp
*/
void Motor_User_SetOpenLoopSpeed(Motor_State_T * p_motor, int16_t speed_fract16)
{
    int32_t limitedCmd = math_clamp(speed_fract16, 0, p_motor->Config.OpenLoopRampSpeedFinal_Fract16);
    Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, Motor_DirectionalValueOf(p_motor, limitedCmd));
}



/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/*!
    Untyped scalar to calibration ref, clamp by limit of the active mode
    @param[in] userCmd[-32768:32767] mixed units
*/
void Motor_User_SetActiveCmdValue(Motor_State_T * p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;

    // if (_Motor_StateMachine_IsOpenLoop(p_motor)) { Motor_User_SetOpenLoopCmd(p_motor, userCmd); }

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmd(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmd(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetICmd(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmd(p_motor, userCmd); }
}

/* Scalar to limit */
void Motor_User_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, int16_t userCmd)
{
    Motor_FeedbackMode_T flags = p_motor->FeedbackMode;

    if      (flags.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd_Scalar(p_motor, userCmd); }
    // else if (flags.Position == 1U)  { Motor_User_SetPositionCmd(p_motor, userCmd); }
    else if (flags.Speed == 1U)     { Motor_User_SetSpeedCmd_Scalar(p_motor, userCmd); }
    else if (flags.Current == 1U)   { Motor_User_SetICmd_Scalar(p_motor, userCmd); }
    else                            { Motor_User_SetVoltageCmd_Scalar(p_motor, userCmd); }
}


// void Motor_User_SetActiveCmdValue_ScalarCast(Motor_State_T * p_motor, int userCmd)
// {
//     Motor_User_SetActiveCmdValue_Scalar(p_motor, (int16_t)userCmd);
// }

/* Alternatively store Input Value */
static inline const Ramp_T * _Motor_User_GetActiveRamp(const Motor_State_T * p_motor)
{
    const Ramp_T * p_active;
    if (p_motor->FeedbackMode.Speed == 1U)  { p_active = &p_motor->SpeedRamp; }
    else                                    { p_active = &p_motor->TorqueRamp; }
    return p_active;
}

/*
    Mixed units
*/
static inline int32_t _Motor_User_GetCmd(const Motor_State_T * p_motor) { return Ramp_GetTarget(_Motor_User_GetActiveRamp(p_motor)); }
static inline int32_t _Motor_User_GetSetPoint(const Motor_State_T * p_motor) { return Ramp_GetOutput(_Motor_User_GetActiveRamp(p_motor)); }

int32_t Motor_User_GetCmd(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, _Motor_User_GetCmd(p_motor)); }
int32_t Motor_User_GetSetPoint(const Motor_State_T * p_motor) { return Motor_DirectionalValueOf(p_motor, _Motor_User_GetSetPoint(p_motor)); }

// /*! @return [-32767:32767] <=> [-1:1] */
// int32_t _Motor_User_GetSetPoint_Scalar(const Motor_State_T * p_motor)
// {
//     int32_t cmd;
//     if (p_motor->FeedbackMode.Speed == 1U)  { cmd = fract16_div(Ramp_GetOutput(&p_motor->SpeedRamp), INT16_MAX); }
//     else                                    { cmd = fract16_div(Ramp_GetOutput(&p_motor->TorqueRamp), MotorAnalogRef_GetIRatedPeak_Fract16()); }
//     return cmd;
// }

// int32_t Motor_User_GetSetPoint_Scalar(const Motor_State_T * p_motor)
// {
//     return Motor_DirectionalValueOf(p_motor, _Motor_User_GetSetPoint_Scalar(p_motor));
// }

bool Motor_User_IsRampEnabled(const Motor_State_T * p_motor) { return _Ramp_IsEnabled(_Motor_User_GetActiveRamp(p_motor)); }
// static inline void Motor_User_SetRampOnOff(Motor_State_T * p_motor, bool enable) { if (enable) { Motor_EnableRamp(p_motor); } else { Motor_DisableRamp(p_motor); } }


/******************************************************************************/
/*
    Ground Speed
*/
/******************************************************************************/
#if defined(CONFIG_MOTOR_UNIT_CONVERSION_LOCAL) && defined(CONFIG_MOTOR_SURFACE_SPEED_ENABLE)
int16_t Motor_User_GetGroundSpeed_Kmh(Motor_State_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Kmh(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     speed = Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             speed = 0;     break;
    }

    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

int16_t Motor_User_GetGroundSpeed_Mph(Motor_State_T * p_motor)
{
    int16_t speed;

    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     speed = Encoder_DeltaD_GetGroundSpeed_Mph(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     speed =  Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             speed = 0;     break;
    }

    return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? speed : 0 - speed;
}

void Motor_User_SetGroundSpeed_Kmh(Motor_State_T * p_motor, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:        Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     Encoder_SetGroundRatio_Metric(&p_motor->Encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
#if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:     break;
    }
}

void Motor_User_SetGroundSpeed_Mph(Motor_State_T * p_motor, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:         Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
        case MOTOR_SENSOR_MODE_ENCODER:     Encoder_SetGroundRatio_US(&p_motor->Encoder, wheelDiameter_Inch10, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);    break;
#if defined(CONFIG_MOTOR_SENSOR_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:      Linear_Speed_CalcGroundSpeed(&p_motor->Units, p_motor->Speed_Fixed32); break;
#endif
#if defined(CONFIG_MOTOR_SENSOR_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     speed = 0;     break;
#endif
        default:                             break;
    }
}
#endif


