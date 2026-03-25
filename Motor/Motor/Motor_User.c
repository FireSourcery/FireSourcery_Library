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
#include "Motor_User.h"


/******************************************************************************/
/*!
    StateMachine handled inputs
    Inputs are not instantaneous state assignments:
        User Request → StateMachine Input → State Transition Guard → Inner Set

    StateMachine Inputs use full [Motor_T] context

    Motor State Machine Thread Safety
    State Proc in PWM thread.
    User Input [StateMachine_Tree_Input] in Main thread.

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

    ProcInput use Critical, as States may transition during input
    SetInput use async polling for status
*/
/******************************************************************************/
/*
    Caller / Var layer validate enum bounds
*/

/******************************************************************************/
/*
    Transition to Stop / Runtime inactive
    Stop disables inputs until next Start.
*/
/******************************************************************************/
static State_T * _Motor_InputStop(const Motor_T * p_motor, state_value_t value)
{
    (void)value;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR_STATE) == 0U) { return &MOTOR_STATE_STOP; }
    return NULL;
}

void Motor_Stop(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_PASSIVE, .NEXT = (State_Input_T)_Motor_InputStop };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

static State_T * _Motor_InputStart(const Motor_T * p_motor, state_value_t value)
{
    (void)value;
    return &MOTOR_STATE_PASSIVE;
}

void Motor_Start(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_STOP, .NEXT = (State_Input_T)_Motor_InputStart };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
    // { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_STATE_CMD, MSM_INPUT_STATE_START); }
}

/******************************************************************************/
/*!
    Phase Output State
    Phase Output directly mapping to a Control State: Feedback Run, Freewheel, Hold
    Release stays in ready mode.
*/
/******************************************************************************/
inline void Motor_ActivateControl(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_VOUT_PWM); }

inline void Motor_ReleaseVZ(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_VOUT_Z); }

inline void Motor_ReleaseV0(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, PHASE_VOUT_0); }

inline void Motor_ApplyControlState(const Motor_T * p_motor, Phase_Output_T state) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_PHASE_OUTPUT, state); }


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
    Using sync mode. activates the next pwm cycle
*/
inline void Motor_ApplyFeedbackMode(const Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    if (mode.Value != p_motor->P_MOTOR_STATE->FeedbackMode.Value) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_FEEDBACK_MODE, mode.Value); }
}

/******************************************************************************/
/*!
    Direction/Stop
    Applied Voltage Direction - sign of cmd, directional limits
    Handle direction change and stop through StateMachine to ensure safe transition and consistent state.
*/
/******************************************************************************/
/*
    Apply virtual
*/
void Motor_ApplyVirtualDirection(const Motor_T * p_motor, Motor_Direction_T direction)
{
    if (direction != p_motor->P_MOTOR_STATE->Direction) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_DIRECTION, direction); }
}

/* calibrated positive or ccw */
void Motor_ApplyUserDirection(const Motor_T * p_motor, Motor_Direction_T direction) { Motor_ApplyVirtualDirection(p_motor, p_motor->P_MOTOR_STATE->Config.DirectionForward * direction); }



/******************************************************************************/
/*
   Disable control. non StateMachine checked
*/
/******************************************************************************/
void Motor_ForceDisableControl(const Motor_T * p_motor)
{
    Phase_Deactivate(&p_motor->PHASE);
    Motor_ReleaseVZ(p_motor);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR_STATE);
    p_motor->P_MOTOR_STATE->UserSpeedReq = 0;
    p_motor->P_MOTOR_STATE->UserTorqueReq = 0;
    // Motor_Stop(p_motor); /* optionally transition to Stop, or stay in current state with control disabled */
}

/******************************************************************************/
/*
    Cmd Modes
*/
/******************************************************************************/
/******************************************************************************/
/*
    Cmd Value
    ~1ms-50ms. Directly set buffer without calling StateMachine.
    Concurrency note:
        Input thread updates Ramp Target.
        StateMachine_Proc thread updates Ramp OutputState
*/
/******************************************************************************/
static inline void _Motor_SetTorqueMotoringCmd(Motor_State_T * p_motor, int16_t userCmd) { p_motor->UserTorqueReq = p_motor->Direction * userCmd; }
static inline void _Motor_SetTorqueForwardCmd(Motor_State_T * p_motor, int16_t userCmd) { p_motor->UserTorqueReq = p_motor->Config.DirectionForward * userCmd; }
/* limit on feedback */
static inline void _Motor_SetSpeedMotoringCmd_Fract16(Motor_State_T * p_motor, int16_t speed_fract16) { p_motor->UserSpeedReq = p_motor->Direction * speed_fract16; }
static inline void _Motor_SetVelocityCmd_Fract16(Motor_State_T * p_motor, int16_t speed_fract16) { p_motor->UserSpeedReq = p_motor->Config.DirectionForward * speed_fract16; }


/******************************************************************************/
/*!
    Voltage Mode
*/
/******************************************************************************/
void Motor_StartVoltageMode(const Motor_T * p_motor) { Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_VOLTAGE); }

/*!
    @param[in] voltage [0:32767]
*/
/* Aligned to applied direction */
/* Reverse voltage set direction, no plugging */
void Motor_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16)
{
    _Motor_SetTorqueMotoringCmd(p_motor, math_clamp(volts_fract16, 0, Phase_VBus_GetVRef()));
}

void Motor_SetVoltageCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_SetVoltageCmd(p_motor, fract16_mul(scalar_fract16, Phase_VBus_GetVRef()));
}

// void _Motor_SetVRegenCmd(Motor_State_T * p_motor, int16_t scalar_fract16)
// {
//     // Motor_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 4);
// }

/******************************************************************************/
/*!
    Current Mode
*/
/******************************************************************************/
/* Match to 0 torque on start */
void Motor_StartIMode(const Motor_T * p_motor) { Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); }


/*!
    @param[in] current [-32768:32767] as "Amps Fract16"
    +/- Aligned to applied direction, Motoring/Generating
*/
void Motor_SetICmd(Motor_State_T * p_motor, int16_t i_fract16)
{
    _Motor_SetTorqueMotoringCmd(p_motor, math_clamp(i_fract16, (int32_t)0 - Motor_ILimitGenerating(p_motor), Motor_ILimitMotoring(p_motor)));
}

/* Scalar of Config.Limit - maintain same proportion through runtime. set by user. */
void Motor_SetICmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_SetICmd(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16));
}

/* Call handles 0 cmd values. alternatively intervention state */
void Motor_ApplyTorque0(const Motor_T * p_motor)
{
    Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);
    _Motor_SetTorqueMotoringCmd(p_motor->P_MOTOR_STATE, 0);
}

/******************************************************************************/
/*!
    Torque Mode
*/
/******************************************************************************/
/*
    +/- Aligned to velocity also configured forward
*/
void Motor_SetTorqueCmd(Motor_State_T * p_motor, int16_t i_fract16)
{
    _Motor_SetTorqueForwardCmd(p_motor, math_clamp(i_fract16, (int32_t)0 - Motor_ILimitReverse(p_motor), Motor_ILimitForward(p_motor)));
}

/* scale to motoring limit, larger of motoring/generating in most cases */
void Motor_SetTorqueCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_SetTorqueCmd(p_motor, fract16_mul(scalar_fract16, p_motor->Config.ILimitMotoring_Fract16));
}

/******************************************************************************/
/*!
    Speed Mode
*/
/******************************************************************************/
/*!
    Default speed mode is speed with current loop
*/
void Motor_StartSpeedMode(const Motor_T * p_motor) { Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_SPEED_CURRENT); }

/*!
    @param[in] speed [-32768:32767] - Rpm Fract16
*/

/* + as selected direction. Speed reduce to 0 only. clamped by anti-plugging on request */
/* Optionally only allow selected direction, reverse direction set Direction first */
/* Sensor speed interpolation use auto detect */
void Motor_SetSpeedCmd_Fract16(Motor_State_T * p_motor, int16_t speed_fract16)
{
    _Motor_SetSpeedMotoringCmd_Fract16(p_motor, math_clamp(speed_fract16, (int32_t)0 - Motor_SpeedLimitGenerating(p_motor), Motor_SpeedLimitMotoring(p_motor))); /* or max(fwd, rev) as consistent ref to user input  */
}

void Motor_SetSpeedCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_SetSpeedCmd_Fract16(p_motor, fract16_mul(scalar_fract16, p_motor->Config.SpeedLimitForward_Fract16)); /* or max(fwd, rev) as consistent ref to user input  */
}

/* + as absolute direction, selected by config. anti-plugging must be on feedback */
void Motor_SetVelocityCmd_Fract16(Motor_State_T * p_motor, int16_t speed_fract16)
{
    _Motor_SetVelocityCmd_Fract16(p_motor, math_clamp(speed_fract16, (int32_t)0 - p_motor->SpeedLimitReverse_Fract16, p_motor->SpeedLimitForward_Fract16));
}

void Motor_SetVelocityCmd_Scalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_SetVelocityCmd_Fract16(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.SpeedLimitForward_Fract16 : p_motor->Config.SpeedLimitReverse_Fract16));
}


/******************************************************************************/
/*!
    Position Mode
*/
/******************************************************************************/
// /*!
//     @param[in] angle [0:65535]
// */
// void Motor_SetPositionCmd(Motor_State_T * p_motor, uint16_t angle)
// {
//     // _Motor_ActivateControlMode(p_motor, MOTOR_FEEDBACK_MODE_POSITION_SPEED_CURRENT);
//     // _Motor_SetCmd(p_motor, angle);
// }

/******************************************************************************/
/*!
    Open Loop
*/
/******************************************************************************/
/*
*/
void Motor_EnterOpenLoopState(const Motor_T * p_motor)
{
    StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MSM_INPUT_OPEN_LOOP, (uintptr_t)&MOTOR_STATE_OPEN_LOOP); /* Set FeedbackMode on Entry */
}

/*
    Open Loop Cmd Limit clamp on proc.
*/
/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    On the fly adjustment of Preset Open Loop Speed
    Open Loop Start Up
    Preset Ramp
*/
// void Motor_SetOpenLoopSpeed(Motor_State_T * p_motor, int16_t speed)
// {
//     int32_t limitedCmd = math_clamp(speed, 0 - p_motor->Config.OpenLoopRampSpeedFinal_Fract16, p_motor->Config.OpenLoopRampSpeedFinal_Fract16) * p_motor->Direction;
// }

/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/*
    Unitless scalar as Motoring Direction
*/
/*
    Scalar to Config value for consistent user handling
    OpenLoop clamp on Proc TorqueRamp
*/
void _Motor_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    if (mode.Speed == 1U)          { Motor_SetSpeedCmd_Scalar(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_SetICmd_Scalar(p_motor, userCmd); }
    else                           { Motor_SetVoltageCmd_Scalar(p_motor, userCmd); }
    // else if (mode.Current == 1U)   { Motor_SetTorqueCmd_Scalar(p_motor, userCmd); } /* align to direction by default */
    // else                           { Motor_SetTorqueVCmd_Scalar(p_motor, userCmd); }
}

void Motor_SetActiveCmdValue_Scalar(Motor_State_T * p_motor, int16_t userCmd)
{
    _Motor_SetActiveCmdValue_Scalar(p_motor, p_motor->FeedbackMode, userCmd);
}

/*!
    Mixed units
    Untyped scalar to calibration ref, clamp by limit of the active mode
    @param[in] userCmd[-32768:32767] mixed units
*/
// inline void _Motor_SetActiveCmdValue(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
// {
//     // if (_Motor_StateMachine_IsOpenLoop(p_motor)) { Motor_SetOpenLoopCmd(p_motor, userCmd); }
//     if      (mode.OpenLoop == 1U)  { Motor_SetOpenLoopCmd(p_motor, userCmd); }
//     // else if (mode.Position == 1U)  { Motor_SetPositionCmd(p_motor, userCmd); }
//     else if (mode.Speed == 1U)     { Motor_SetSpeedCmd_Fract16(p_motor, userCmd); }
//     // else if (mode.Current == 1U)   { Motor_SetTorqueCmd(p_motor, userCmd); } /* align to direction by default */
//     // else                           { Motor_SetTorqueVCmd(p_motor, userCmd); }
//     else if (mode.Current == 1U)   { Motor_SetICmd(p_motor, userCmd); }
//     else                           { Motor_SetVoltageCmd(p_motor, userCmd); }
// }

// void Motor_SetActiveCmdValue(Motor_State_T * p_motor, int16_t userCmd)
// {
//     // if (p_input->CmdValue != Motor_GetCmd(p_motor )
//     _Motor_SetActiveCmdValue(p_motor, p_motor->FeedbackMode, userCmd);
// }


/******************************************************************************/
/*
    Set system "user" layer
*/
/******************************************************************************/
/* Handle remaining comparison with Motor heat I limits if its not handled by arbitration array */
bool Motor_TrySpeedLimit(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    bool isLimit = true;
    Motor_SetSpeedLimits(p_motor, speed_ufract16);
    return isLimit;
}

bool Motor_TryClearSpeedLimit(Motor_State_T * p_motor)
{
    bool isLimit = true;
    Motor_ResetSpeedLimit(p_motor);
    return isLimit;
}

bool Motor_TryILimit(Motor_State_T * p_motor, uint16_t i_Fract16)
{
    bool isLimit = true;
    Motor_SetILimits(p_motor, i_Fract16);
    return isLimit;
}

bool Motor_TryClearILimit(Motor_State_T * p_motor)
{
    bool isLimit = true;
    Motor_ResetILimit(p_motor);
    return isLimit;
}

/*
    Interface
    Set using comparison struct
*/
void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_GetUpper(p_limit)); }
    else { Motor_TryClearSpeedLimit(p_motor); }
}

void Motor_SetILimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TryILimit(p_motor, LimitArray_GetUpper(p_limit)); }
    else { Motor_TryClearILimit(p_motor); }
}





/******************************************************************************/
/*
    Specialized Sync input
*/
/******************************************************************************/
// void Motor_ProcSyncInput(const Motor_T * p_motor, Motor_Input_T * p_input)
// {
//     if (p_input->PhaseOutput != Motor_GetPhaseState(p_motor))
//     {
//         Motor_ApplyControlState(p_motor, p_input->PhaseOutput);
//     }
//     // Motor_ApplyControlState(p_motor, p_input->PhaseOutput);
//     Motor_ApplyFeedbackMode(p_motor, p_input->FeedbackMode);
//     Motor_ApplyUserDirection(p_motor, p_input->Direction);

//     // Flags should update even if State has not transitioned
//     // overwritten by match in case FeedbackMode changed.
//     Motor_SetActiveCmdValue_Scalar(p_motor->P_MOTOR_STATE, p_input->CmdValue);

//     // if (p_input->SpeedLimit != p_prev->SpeedLimit)
//     // {
//     //     p_prev->SpeedLimit = p_input->SpeedLimit;
//     //     MotorController_SetUserSpeedLimitAll(p_context, MOT_SPEED_LIMIT_USER, p_input->SpeedLimit);
//     // }

//     // if (p_input->ILimit != p_prev->ILimit)
//     // {
//     //     p_prev->ILimit = p_input->ILimit;
//     //     MotorController_SetUserILimitAll(p_context, MOT_I_LIMIT_USER, p_input->ILimit);
//     // }
// }

// /* units as FeedbackMode Set */
// void Motor_ProcInputSetpoint(const Motor_T * p_motor, Motor_Input_T * p_input)
// {
//     if (p_input->CmdValue != Motor_GetCmd(p_motor->P_MOTOR_STATE)) /* compare applicable to mixed units values */
//     {
//         Motor_SetActiveCmdValue(p_motor->P_MOTOR_STATE, p_input->CmdValue);
//     }
// }


