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
    StateMachine Inputs use full [Motor_T] context

    Motor State Machine Thread Safety
    State Proc in PWM thread.
    User Input [StateMachine_ApplyInput] in Main thread.

    Sync Mode -
        Inputs do not directly proc transition, set for sync proc
        overheader - check input flags every control cycle.
        Multiple calls to [StateMachine_SetInput] overwrites within 1 control cycle may be lost (e.g within 1 packet)
            handle with input image buffer, input queue, spin-lock delegates to Xcvr buffer, or client side constraints

    Async Mode -
        Block or delay PWM while processing input
        Need critical section during input/ input transition
        No Critical during transition -> Prev State Proc may overwrite new State Entry.

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Phase Output State
    Phase Output directly mapping to a Control State: Feedback Run, Freewheel, Hold
*/
/******************************************************************************/
inline void Motor_User_ActivateControl(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_VPWM); }

inline void Motor_User_Release(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_FLOAT); }

inline void Motor_User_Hold(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_OUTPUT_V0); }

// validate enum
inline void Motor_User_ActivatePhaseOutput(const Motor_T * p_motor, Phase_Output_T state) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, state); }

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

/******************************************************************************/
/*!
    Direction/Stop
*/
/******************************************************************************/
/*
    ProcInput use Critical, as States may transition during input
    SetInput use async polling for status
*/
void Motor_User_ApplyRotaryDirection(const Motor_T * p_motor, Motor_Direction_T direction)
{
    if (p_motor->P_MOTOR_STATE->Direction != direction)
    {
        //validate enum
        StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, direction);
    }
}


void Motor_User_ApplyDirectionSign(const Motor_T * p_motor, Motor_User_Direction_T sign) { Motor_User_ApplyRotaryDirection(p_motor, p_motor->P_MOTOR_STATE->Config.DirectionForward * sign); }

void Motor_User_ApplyDirectionForward(const Motor_T * p_motor) { Motor_User_ApplyRotaryDirection(p_motor, p_motor->P_MOTOR_STATE->Config.DirectionForward); }
void Motor_User_ApplyDirectionReverse(const Motor_T * p_motor) { Motor_User_ApplyRotaryDirection(p_motor, Motor_GetDirectionReverse(p_motor->P_MOTOR_STATE)); }

/*
    Transition to Stop
    Release stays in ready mode. Stop disables inputs until next Start.
*/
void Motor_User_Stop(const Motor_T * p_motor) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, MOTOR_DIRECTION_NULL); }

/******************************************************************************/
/*
   Disable control. non StateMachine checked
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
/******************************************************************************/
/*
    Cmd Value
    Concurrency note:
        Input thread updates Ramp Target.
        StateMachine_Proc thread updates Ramp OutputState
*/
/******************************************************************************/
/*
    Helper
*/
// static inline int16_t _Motor_User_GetTorqueCmd(const Motor_State_T * p_motor)           { return Ramp_GetTarget(&p_motor->TorqueRamp); }
// static inline int16_t _Motor_User_GetTorqueSetpoint(const Motor_State_T * p_motor)      { return Ramp_GetOutput(&p_motor->TorqueRamp); }
// static inline int16_t _Motor_User_GetSpeedCmd(const Motor_State_T * p_motor)            { return Ramp_GetTarget(&p_motor->SpeedRamp); }
// static inline int16_t _Motor_User_GetSpeedSetpoint(const Motor_State_T * p_motor)       { return Ramp_GetOutput(&p_motor->SpeedRamp); }

// static inline void _Motor_User_SetTorqueMotoring(Motor_State_T * p_motor, int16_t userCmd) { Ramp_SetTarget(&p_motor->TorqueRamp, p_motor->Direction * (int32_t)userCmd); }
// static inline void _Motor_User_SetTorqueForward(Motor_State_T * p_motor, int16_t userCmd)  { Ramp_SetTarget(&p_motor->TorqueRamp, p_motor->Config.DirectionForward * (int32_t)userCmd); }
// // static inline int16_t Motor_User_GetTorqueCmd(const Motor_State_T * p_motor)            { return p_motor->Direction * Ramp_GetTarget(&p_motor->TorqueRamp); }
// // static inline int16_t Motor_User_GetTorqueSetpoint(const Motor_State_T * p_motor)       { return p_motor->Direction * Ramp_GetOutput(&p_motor->TorqueRamp); }
// // static inline void _Motor_User_SetTorqueCmd(Motor_State_T * p_motor, int16_t userCmd)   { Ramp_SetTarget(&p_motor->TorqueRamp, p_motor->Config.DirectionForward * (int32_t)userCmd); }
// // static inline int16_t Motor_User_GetTorqueCmd(const Motor_State_T * p_motor)            { return p_motor->Config.DirectionForward * Ramp_GetTarget(&p_motor->TorqueRamp); }
// // static inline int16_t Motor_User_GetTorqueSetpoint(const Motor_State_T * p_motor)       { return p_motor->Config.DirectionForward * Ramp_GetOutput(&p_motor->TorqueRamp); }

// static inline void _Motor_User_SetSpeedCmd(Motor_State_T * p_motor, int16_t userCmd)    { Ramp_SetTarget(&p_motor->SpeedRamp, p_motor->Config.DirectionForward * (int32_t)userCmd); }
// static inline int16_t Motor_User_GetSpeedCmd(const Motor_State_T * p_motor)             { return p_motor->Config.DirectionForward * Ramp_GetTarget(&p_motor->SpeedRamp); }
// static inline int16_t Motor_User_GetSpeedSetpoint(const Motor_State_T * p_motor)        { return p_motor->Config.DirectionForward * Ramp_GetOutput(&p_motor->SpeedRamp); }

/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_User_StartVoltageMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE); }

/*!
    @param[in] voltage [0:32767]
*/
/* Aligned to applied direction */
/* Reverse voltage set direction, no plugging */
void Motor_User_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16)
{
    Ramp_SetTarget(&p_motor->TorqueRamp, p_motor->Direction * math_clamp(volts_fract16, 0, Phase_VBus_GetVRef()));
}

void Motor_User_SetVoltageCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetVoltageCmd(p_motor, fract16_mul(scalar_fract16, Phase_VBus_GetVRef()));
}

// void _Motor_User_SetVRegenCmd(Motor_State_T * p_motor, int16_t scalar_fract16)
// {
//     // Motor_User_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 4);
// }

/******************************************************************************/
/*!
    Current Mode
*/
/******************************************************************************/
/* Match to 0 torque on start */
void Motor_User_StartIMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); }

/*!
    @param[in] current [-32768:32767] as "Amps Fract16"
    +/- Aligned to applied direction, Motoring/Generating
*/
void Motor_User_SetICmd(Motor_State_T * p_motor, int16_t i_Fract16)
{
    Ramp_SetTarget(&p_motor->TorqueRamp, p_motor->Direction * math_clamp(i_Fract16, (int32_t)0 - Motor_GetILimitGeneratingActive(p_motor), Motor_GetILimitMotoringActive(p_motor)));
}

/* Scalar of Config.Limit - maintain same proportion through runtime. set by user. */
void Motor_User_SetICmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetICmd(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16));
}

// void Motor_User_SetIFollow(Motor_State_T * p_motor)
// {
//     Motor_User_SetICmd(p_motor, 0U);
// }

/******************************************************************************/
/*!
    Torque Mode
*/
/******************************************************************************/
/* Aligned to configured forward */
/* TorqueV */
void Motor_User_SetTorqueVCmd(Motor_State_T * p_motor, int16_t volts_fract16)
{
    // assert(p_motor->FeedbackMode.Value == MOTOR_FEEDBACK_MODE_VOLTAGE.Value);
    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_VReqLimitOf(p_motor, p_motor->Config.DirectionForward * (int32_t)volts_fract16));
}

void Motor_User_SetTorqueVCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetTorqueVCmd(p_motor, fract16_mul(scalar_fract16, Phase_VBus_GetVRef()));
}

/* +/- Aligned to configured forward */
void Motor_User_SetTorqueCmd(Motor_State_T * p_motor, int16_t value_Fract16)
{

    Ramp_SetTarget(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, p_motor->Config.DirectionForward * (int32_t)value_Fract16));
}

/*!
    @param[in] torque [-32768:32767]
*/
/* assume forward is motoring. holds for no plugging */
void Motor_User_SetTorqueCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetICmd(p_motor, fract16_mul(scalar_fract16, p_motor->Config.ILimitMotoring_Fract16));
}


// if (p_motor->FeedbackMode.Current == 1U) { Motor_User_SetICmd(p_motor, value_Fract16); } else { Motor_User_SetVoltageCmd(p_motor, value_Fract16); }
// if (p_motor->FeedbackMode.Current == 1U) { Motor_User_SetICmd_Scalar(p_motor, scalar_fract16); } else { Motor_User_SetVoltageCmd_Scalar(p_motor, scalar_fract16); }
// Motor_User_SetICmd(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16));
/******************************************************************************/
/*!
    Speed Mode
*/
/******************************************************************************/
/*!
    Default speed mode is speed torque mode
*/
void Motor_User_StartSpeedMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT); }

/*!
    Only allow forward direction, reverse direction set Direction first
        Hall sensor speed interpolation, Limit Directions

    @param[in] speed [-32768:32767] - "Rpm Fract16". positive as along direction selected by config
*/
void Motor_User_SetSpeedCmd_Fract16(Motor_State_T * p_motor, int16_t speed_fract16)
{
    Ramp_SetTarget(&p_motor->SpeedRamp, p_motor->Config.DirectionForward * math_clamp(speed_fract16, (int32_t)0 - p_motor->SpeedLimitReverse_Fract16, p_motor->SpeedLimitForward_Fract16));
}

/* p_motor->Config.SpeedLimitForward_Fract16 as consistent ref, including reverse */
void Motor_User_SetSpeedCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_User_SetSpeedCmd_Fract16(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.SpeedLimitForward_Fract16 : p_motor->Config.SpeedLimitReverse_Fract16));
}

/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
// /*!
//     @param[in] angle [0:65535]
// */
// void Motor_User_SetPositionCmd(Motor_State_T * p_motor, uint16_t angle)
// {
//     // _Motor_User_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
//     // _Motor_User_SetCmd(p_motor, angle);
// }

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
/*
*/
void Motor_User_EnterOpenLoopState(const Motor_T * p_motor)
{
    StateMachine_ApplyInput(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, (uintptr_t)&MOTOR_STATE_OPEN_LOOP); /* Set FeedbackMode on Entry */
}
/*!
    Set by State Transition
*/
// void _Motor_User_StartOpenLoopMode(const Motor_T * p_motor) { Motor_User_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR); }

/* Also clamps on Proc */
/* todo align as positive only, rotation cmd use limits */
/*
    Open Loop Align
    always positive. no direction correction, applied to d-axis
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
    if (p_motor->FeedbackMode.Current == 1U) { Motor_User_SetOpenLoopI(p_motor, ivCmd); } else { Motor_User_SetOpenLoopV(p_motor, ivCmd); }
}

/* Scalar of Config Limits */
void Motor_User_SetOpenLoopCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    if (p_motor->FeedbackMode.Current == 1U)
    {
        Motor_User_SetOpenLoopI(p_motor, fract16_mul(scalar_fract16, p_motor->Config.ILimitMotoring_Fract16));
    }
    else
    {
        Motor_User_SetOpenLoopV(p_motor, fract16_mul(scalar_fract16, Phase_VBus_GetVRef()));
    }
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    On the fly adjustment of Preset Open Loop Speed
    Open Loop Start Up
    Preset Ramp
*/
void Motor_User_SetOpenLoopSpeed(Motor_State_T * p_motor, int16_t speed_degPerCycle)
{
    int32_t limitedCmd = math_clamp(speed_degPerCycle, 0 - p_motor->Config.OpenLoopRampSpeedFinal_Fract16, p_motor->Config.OpenLoopRampSpeedFinal_Fract16);
    Ramp_SetTarget(&p_motor->OpenLoopSpeedRamp, p_motor->Config.DirectionForward * limitedCmd);
}

/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/* Alternatively store Input Value separately */
static inline const Ramp_T * _Motor_User_GetInputRamp(const Motor_State_T * p_motor)
{
    const Ramp_T * p_active;
    if (p_motor->FeedbackMode.Speed == 1U) { p_active = &p_motor->SpeedRamp; } else { p_active = &p_motor->TorqueRamp; }
    return p_active;
}

static inline int16_t _Motor_User_GetCmd(const Motor_State_T * p_motor)         { return Ramp_GetTarget(_Motor_User_GetInputRamp(p_motor)); }
static inline int16_t _Motor_User_GetSetpoint(const Motor_State_T * p_motor)    { return Ramp_GetOutput(_Motor_User_GetInputRamp(p_motor)); }

bool Motor_User_IsRampEnabled(const Motor_State_T * p_motor) { return _Ramp_IsEnabled(_Motor_User_GetInputRamp(p_motor)); }
// static inline void Motor_User_SetRampOnOff(Motor_State_T * p_motor, bool enable) { if (enable) { Motor_EnableRamp(p_motor); } else { Motor_DisableRamp(p_motor); } }

/*!
    Mixed units
    Untyped scalar to calibration ref, clamp by limit of the active mode
    @param[in] userCmd[-32768:32767] mixed units
*/
inline void _Motor_User_SetActiveCmdValue(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    // if (_Motor_StateMachine_IsOpenLoop(p_motor)) { Motor_User_SetOpenLoopCmd(p_motor, userCmd); }
    if      (mode.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd(p_motor, userCmd); }
    // else if (mode.Position == 1U)  { Motor_User_SetPositionCmd(p_motor, userCmd); }
    else if (mode.Speed == 1U)     { Motor_User_SetSpeedCmd_Fract16(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_User_SetTorqueCmd(p_motor, userCmd); } /* align to direction by default */
    else                           { Motor_User_SetTorqueVCmd(p_motor, userCmd); }
    // else if (mode.Current == 1U)   { Motor_User_SetICmd(p_motor, userCmd); }
    // else                           { Motor_User_SetVoltageCmd(p_motor, userCmd); }
}

void Motor_User_SetActiveCmdValue(Motor_State_T * p_motor, int16_t userCmd)
{
    // if (p_input->CmdValue != Motor_User_GetCmd(p_motor )
    _Motor_User_SetActiveCmdValue(p_motor, p_motor->FeedbackMode, userCmd);
}



/*
    Unitless scalar
*/
/*
    Scalar to Config value for consistent user handling
*/
inline void _Motor_User_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    if      (mode.OpenLoop == 1U)  { Motor_User_SetOpenLoopCmd_Scalar(p_motor, userCmd); }
    // else if (mode.Position == 1U)  { Motor_User_SetPositionCmd_Scalar(p_motor, userCmd); }
    else if (mode.Speed == 1U)     { Motor_User_SetSpeedCmd_Scalar(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_User_SetTorqueCmd_Scalar(p_motor, userCmd); } /* align to direction by default */
    else                           { Motor_User_SetTorqueVCmd_Scalar(p_motor, userCmd); }
    // else if (mode.Current == 1U)   { Motor_User_SetICmd_Scalar(p_motor, userCmd); }
    // else                           { Motor_User_SetVoltageCmd_Scalar(p_motor, userCmd); }
}

void Motor_User_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, int16_t userCmd)
{
    _Motor_User_SetActiveCmdValue_Scalar(p_motor, p_motor->FeedbackMode, userCmd);
}

/*
    call other mode versions when possible.
*/
/*! @return [-32767:32767] <=> [-1:1] */
// int16_t _Motor_User_GetSetpoint_Scalar(const Motor_State_T * p_motor)
// {
//     int16_t cmd;
//     if (p_motor->FeedbackMode.Speed == 1U) { cmd = fract16_div(Ramp_GetOutput(&p_motor->SpeedRamp), p_motor->Config.SpeedLimitForward_Fract16); }
//     else
//     {
//         if (p_motor->FeedbackMode.Current == 1U) { cmd = fract16_div(Ramp_GetOutput(&p_motor->TorqueRamp), p_motor->Config.ILimitMotoring_Fract16); }
//         else                                     { cmd = fract16_div(Ramp_GetOutput(&p_motor->TorqueRamp), (Phase_VBus_Fract16() + 1) / 2); }
//     }
//     return cmd;
// }

// int16_t Motor_User_GetSetpoint_Scalar(const Motor_State_T * p_motor)
// {
//     return Motor_DirectionalValueOf(p_motor, _Motor_User_GetSetpoint_Scalar(p_motor));
// }


/******************************************************************************/
/*
*/
/******************************************************************************/
bool Motor_TrySpeedLimit(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    bool isLimit = false;
    // switch (Motor_User_GetDirection(p_motor))
    // {
    //     case 1:
    //         if (speed_ufract16 > p_motor->SpeedLimitForward_Fract16) { Motor_SetSpeedLimitForward(p_motor, speed_ufract16); isLimit = true; }
    //         break;
    //     case -1:
    //         if (speed_ufract16 > p_motor->SpeedLimitReverse_Fract16) { Motor_SetSpeedLimitReverse(p_motor, speed_ufract16); isLimit = true; }
    //         break;
    //     default: break;
    // }

    return isLimit;
}

bool Motor_TryILimit(Motor_State_T * p_motor, uint16_t i_Fract16)
{
    bool isLimit = false;
    // if (i_Fract16 < p_motor->ILimitMotoring_Fract16) { p_motor->ILimitMotoring_Fract16 = i_Fract16; isLimit = true; }
    // if (i_Fract16 < p_motor->ILimitGenerating_Fract16) { p_motor->ILimitGenerating_Fract16 = i_Fract16; isLimit = true; }
    // if (isLimit == true) { ApplyILimit(p_motor); }
    return isLimit;
}

/*
    Interface
    Set using comparison struct
*/
void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_GetUpper(p_limit)); }
    // else                                        { Motor_ClearSpeedLimit(p_motor); }
}

void Motor_SetILimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TryILimit(p_motor, LimitArray_GetUpper(p_limit)); }
    // else                                        { Motor_ClearILimit(p_motor); }
}





/******************************************************************************/
/*
    Specialized Sync input
*/
/******************************************************************************/
void Motor_User_ProcSyncInput(const Motor_T * p_motor, Motor_User_Input_T * p_input)
{
    if (p_input->PhaseState != Motor_User_GetPhaseState(p_motor))
    {
        Motor_User_ActivatePhaseOutput(p_motor, p_input->PhaseState);
    }
    // Motor_User_ActivatePhaseOutput(p_motor, p_input->PhaseState);
    Motor_User_SetFeedbackMode(p_motor, p_input->FeedbackMode);
    Motor_User_ApplyDirectionSign(p_motor, (Motor_User_Direction_T)p_input->Direction);

    // Flags should update even if State has not transitioned
    // overwritten by match in case FeedbackMode changed.
    Motor_User_SetActiveCmdValue_Scalar(p_motor->P_MOTOR_STATE, p_input->CmdValue);

    // if (p_input->SpeedLimit != p_prev->SpeedLimit)
    // {
    //     p_prev->SpeedLimit = p_input->SpeedLimit;
    //     MotorController_SetSpeedLimitAll(p_context, MOT_SPEED_LIMIT_USER, p_input->SpeedLimit);
    // }

    // if (p_input->ILimit != p_prev->ILimit)
    // {
    //     p_prev->ILimit = p_input->ILimit;
    //     MotorController_SetILimitAll(p_context, MOT_I_LIMIT_USER, p_input->ILimit);
    // }
}

/* units as FeedbackMode Set */
void Motor_User_ProcInputSetpoint(const Motor_T * p_motor, Motor_User_Input_T * p_input)
{
    if (p_input->CmdValue != Motor_User_GetCmd(p_motor->P_MOTOR_STATE)) /* compare applicable to mixed units values */
    {
        Motor_User_SetActiveCmdValue(p_motor->P_MOTOR_STATE, p_input->CmdValue);
    }
}


