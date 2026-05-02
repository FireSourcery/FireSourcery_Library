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
/*!
    Phase Output State
    Phase Output directly mapping to a Control State: Feedback Run, Freewheel, Hold
    Release stays in ready mode.
*/
/******************************************************************************/
inline void Motor_ActivateControl(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_PHASE_OUTPUT, PHASE_VOUT_PWM); }

inline void Motor_ReleaseVZ(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_PHASE_OUTPUT, PHASE_VOUT_Z); }

inline void Motor_ReleaseV0(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_PHASE_OUTPUT, PHASE_VOUT_0); }

inline void Motor_ApplyControlState(const Motor_T * p_motor, Phase_Output_T state) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_PHASE_OUTPUT, state); }


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
    if (mode.Value != p_motor->P_MOTOR->FeedbackMode.Value) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_FEEDBACK_MODE, mode.Value); }
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
    if (direction != p_motor->P_MOTOR->Direction) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_DIRECTION, direction); }
}

/* calibrated positive or ccw */
void Motor_ApplyUserDirection(const Motor_T * p_motor, Motor_Direction_T direction) { Motor_ApplyVirtualDirection(p_motor, p_motor->P_MOTOR->Config.DirectionForward * direction); }



/******************************************************************************/
/*
   Disable control. non StateMachine checked
*/
/******************************************************************************/
void Motor_ForceDisableControl(const Motor_T * p_motor)
{
    Phase_Deactivate(&p_motor->PHASE);
    Motor_ReleaseVZ(p_motor);
    Motor_FOC_ClearFeedbackState(p_motor->P_MOTOR);
    p_motor->P_MOTOR->UserSpeedReq = 0;
    p_motor->P_MOTOR->UserTorqueReq = 0;
    // Motor_Disable(p_motor); /* optionally transition to Stop, or stay in current state with control disabled */
}

/******************************************************************************/
/*
    Cmd Modes
*/
/******************************************************************************/
/*
    User frame
    forward / reverse, throttle, brake
    in the calibrated / physical frame (signed by Config.DirectionForward)
    Caller: protocol handler, Var interface

    Motor_Set[Quantity][Frame]Cmd[InputType]
*/
/******************************************************************************/
/*
    Cmd Value
    ~1ms-50ms. Directly set buffer without calling StateMachine.
    Concurrency note:
        Input thread updates Ramp Target.
        StateMachine_Proc thread updates Ramp OutputState
*/
/******************************************************************************/
static inline void _Motor_SetTorqueCmd(Motor_State_T * p_motor, int16_t userCmd) { p_motor->UserTorqueReq = p_motor->Config.DirectionForward * userCmd; }
static inline void _Motor_SetSpeedCmd(Motor_State_T * p_motor, int16_t speed_fract16) { p_motor->UserSpeedReq = p_motor->Config.DirectionForward * speed_fract16; }
static inline void _Motor_SetTorqueMotoringCmd(Motor_State_T * p_motor, int16_t userCmd) { p_motor->UserTorqueReq = p_motor->Direction * userCmd; }
static inline void _Motor_SetSpeedMotoringCmd(Motor_State_T * p_motor, int16_t speed_fract16) { p_motor->UserSpeedReq = p_motor->Direction * speed_fract16; }


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
// _Motor_SetTorqueMotoringCmd(p_motor, math_clamp(volts_fract16, 0, Phase_VBus_GetVRef()));
void Motor_SetVoltageCmd(Motor_State_T * p_motor, int16_t volts_fract16) { _Motor_SetTorqueMotoringCmd(p_motor, volts_fract16); }
void Motor_SetVoltageCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16) { Motor_SetVoltageCmd(p_motor, fract16_mul(scalar_fract16, Phase_VBus_GetVRef())); }
// void _Motor_SetVFreqCmd(Motor_State_T * p_motor, int16_t scalar_fract16) {   Motor_SetVoltageCmd(p_motor, Motor_GetVSpeed_Fract16(p_motor) / 4); }

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
    Per-unit of calibration reference (I_CALIB_AMPS)
*/
// _Motor_SetTorqueMotoringCmd(p_motor, math_clamp(i_fract16, (int32_t)0 - Motor_ILimitGenerating(p_motor), Motor_ILimitMotoring(p_motor)));
void Motor_SetICmd(Motor_State_T * p_motor, int16_t i_fract16) { _Motor_SetTorqueMotoringCmd(p_motor, i_fract16); }

/*  Per-unit of limit reference Config.Limit - maintain same proportion through runtime. set by user. */
void Motor_SetICmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16) { Motor_SetICmd(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.ILimitMotoring_Fract16 : p_motor->Config.ILimitGenerating_Fract16)); }

/* Call handles 0 cmd values. alternatively intervention state */
void Motor_ApplyTorque0(const Motor_T * p_motor)
{
    Motor_ApplyFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT);
    _Motor_SetTorqueMotoringCmd(p_motor->P_MOTOR, 0);
}


/******************************************************************************/
/*!
    Calibrated Referemce Frame
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Torque Mode
*/
/******************************************************************************/
/*
    +/- Aligned to velocity also configured forward
*/
// _Motor_SetTorqueCmd(p_motor, math_clamp(i_fract16, (int32_t)0 - Motor_ILimitReverse(p_motor), Motor_ILimitForward(p_motor)));
void Motor_SetTorqueCmd(Motor_State_T * p_motor, int16_t i_fract16) { _Motor_SetTorqueCmd(p_motor, i_fract16); }

/* scale to motoring limit, larger of motoring/generating in most cases */
void Motor_SetTorqueCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16) { Motor_SetTorqueCmd(p_motor, fract16_mul(scalar_fract16, p_motor->Config.ILimitMotoring_Fract16)); }

void Motor_SetTorqueVCmd(Motor_State_T * p_motor, int16_t i_fract16) { _Motor_SetTorqueCmd(p_motor, i_fract16); }
void Motor_SetTorqueVCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16) { Motor_SetTorqueVCmd(p_motor, fract16_mul(scalar_fract16, Phase_VBus_GetVRefSvpwm())); }

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
void Motor_SetSpeedCmd(Motor_State_T * p_motor, int16_t speed_fract16) { _Motor_SetSpeedCmd(p_motor, speed_fract16); }
/* Optionally only allow selected direction, reverse direction set Direction first */
// _Motor_SetSpeedCmd(p_motor, math_clamp(speed_fract16, (int32_t)0 - p_motor->SpeedLimitReverse_Fract16, p_motor->SpeedLimitForward_Fract16));

/* ofLimit / Percent */
void Motor_SetSpeedCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16)
{
    Motor_SetSpeedCmd(p_motor, fract16_mul(scalar_fract16, (scalar_fract16 > 0) ? p_motor->Config.SpeedLimitForward_Fract16 : p_motor->Config.SpeedLimitReverse_Fract16));
}


/******************************************************************************/
/*!
    Motoring
*/
/******************************************************************************/
// void Motor_SetTorqueDriveCmd(Motor_State_T * p_motor, int16_t i_fract16)
// {
//     // _Motor_SetTorqueCmd(p_motor, math_clamp(i_fract16, (int32_t)0 - Motor_ILimitReverse(p_motor), Motor_ILimitForward(p_motor)));
//     _Motor_SetTorqueMotoringCmd(p_motor, i_fract16);
// }

// void Motor_SetTorqueDriveCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16)
// {
//     Motor_SetTorqueDriveCmd(p_motor, fract16_mul(scalar_fract16, p_motor->Config.ILimitMotoring_Fract16));
// }

/*
    alternatively throttle layer handle
*/
/* + as selected motoring direction. Speed reduce to 0 only. clamped by anti-plugging in feedback loop */
// _Motor_SetSpeedMotoringCmd(p_motor, math_clamp(speed_fract16, (int32_t)0 - Motor_SpeedLimitGenerating(p_motor), Motor_SpeedLimitMotoring(p_motor)));
void Motor_SetSpeedMotoringCmd(Motor_State_T * p_motor, int16_t speed_fract16) { _Motor_SetSpeedMotoringCmd(p_motor, speed_fract16); }

/* select higher value as consistent reference */
void Motor_SetSpeedMotoringCmdScalar(Motor_State_T * p_motor, int16_t scalar_fract16) { Motor_SetSpeedMotoringCmd(p_motor, fract16_mul(scalar_fract16, p_motor->Config.SpeedLimitForward_Fract16)); }

/******************************************************************************/
/*!
    Generic Mode
*/
/******************************************************************************/
/*
    Unitless scalar as Motoring Direction
    Scalar to Config value for consistent user handling
*/
void _Motor_SetMotoringCmdScalar(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    if (mode.Speed == 1U)          { Motor_SetSpeedMotoringCmdScalar(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_SetICmdScalar(p_motor, userCmd); }
    else                           { Motor_SetVoltageCmdScalar(p_motor, userCmd); }
}

void _Motor_SetUserCmdScalar(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    if (mode.Speed == 1U)          { Motor_SetSpeedCmdScalar(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_SetTorqueCmdScalar(p_motor, userCmd); }
    else                           { Motor_SetTorqueVCmdScalar(p_motor, userCmd); }
}

void Motor_SetActiveCmdScalar(Motor_State_T * p_motor, int16_t userCmd)
{
    _Motor_SetUserCmdScalar(p_motor, p_motor->FeedbackMode, userCmd);
}


/* mixed units */
void _Motor_SetMotoringCmdUnits(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    if (mode.Speed == 1U)          { Motor_SetSpeedMotoringCmd(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_SetICmd(p_motor, userCmd); }
    else                           { Motor_SetVoltageCmd(p_motor, userCmd); }
}

void _Motor_SetUserCmdUnits(Motor_State_T * p_motor, Motor_FeedbackMode_T mode, int16_t userCmd)
{
    if (mode.Speed == 1U)          { Motor_SetSpeedCmd(p_motor, userCmd); }
    else if (mode.Current == 1U)   { Motor_SetTorqueCmd(p_motor, userCmd); }
    else                           { Motor_SetTorqueVCmd(p_motor, userCmd); }
}

typedef struct
{
    uint16_t Magnitude : 15;
    uint16_t Reverse : 1;
    uint16_t Frame : 1;
}
Motor_QuadrantCmd_T;

typedef struct
{
    Motor_FeedbackMode_T FeedbackMode;
    Motor_QuadrantCmd_T Cmd;
}
Motor_DriveCmd_T;

// #include "Motor_Drive.h"
// void Motor_SetDriveCmd(Motor_T * p_motor, Motor_FeedbackMode_T mode, Motor_QuadrantCmd_T cmd)
// {
//     if (cmd.Frame == 0) { _Motor_SetUserCmdUnits(p_motor, mode, cmd.Magnitude); }
//     else { _Motor_SetMotoringCmdUnits(p_motor, mode, cmd.Magnitude); }
// }

// typedef struct
// {
//     uint8_t Motoring : 1;
//     uint8_t Forward : 1;
// }
// Motor_QuadrantSign_T;
// typedef struct
// {
//     int16_t Cmd ;
//     Motor_QuadrantSign_T QuadrantSign;
// }
// Motor_QuadrantCmd_T;

// void Motor_SetDriveCmd(Motor_T * p_motor, Motor_FeedbackMode_T mode, Motor_QuadrantCmd_T cmd)
// {
//     switch ((uint8_t)cmd.QuadrantSign)
//     {
//         case  : /* Forward Motoring */
//             _Motor_SetMotoringCmdUnits(p_motor, mode, cmd.Cmd);
//             break;
//         case  : /* Reverse Motoring */
//             _Motor_SetMotoringCmdUnits(p_motor, mode, cmd.Cmd);
//             break;
//         case  : /* Forward Generating */
//             _Motor_SetUserCmdUnits(p_motor, mode, cmd.Cmd);
//             break;
//         case  : /* Reverse Generating */
//             _Motor_SetUserCmdUnits(p_motor, mode, cmd.Cmd);
//             break;
//         default:
//             break;
//     }
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
    StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_OPEN_LOOP, (uintptr_t)&MOTOR_STATE_OPEN_LOOP); /* Set FeedbackMode on Entry */
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
/*
    Set system "user" layer
*/
/******************************************************************************/
/* Handle remaining comparison with Motor heat I limits if its not handled by arbitration array */
/*     // Motor_SetILimits(p_motor, math_min(i_Fract16, LimitArray_Upper(p_local))); */
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


void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_local, LimitArray_T * p_system)
{
    // if (LimitArray_IsUpperActive(p_local) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_Upper(p_local)); }
    // else { Motor_TryClearSpeedLimit(p_motor); }

    Motor_SetSpeedLimits(p_motor, math_min(LimitArray_Upper(p_system), LimitArray_Upper(p_local)));
}

void Motor_SetILimitMotoringWith(Motor_State_T * p_motor, LimitArray_T * p_local, LimitArray_T * p_system)
{
    // if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TryILimit(p_motor, LimitArray_Upper(p_limit)); }
    // else { Motor_TryClearILimit(p_motor); }
    Motor_SetILimitMotoring(p_motor, math_min(LimitArray_Upper(p_system), LimitArray_Upper(p_local)));
}

void Motor_SetILimitGeneratingWith(Motor_State_T * p_motor, LimitArray_T * p_local, LimitArray_T * p_system)
{
    // if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TryILimit(p_motor, LimitArray_Upper(p_limit)); }
    // else { Motor_TryClearILimit(p_motor); }
    Motor_SetILimitGenerating(p_motor, math_min(LimitArray_Upper(p_system), LimitArray_Upper(p_local)));
}

/*
    Interface
    Set using comparison struct
*/
// void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
// {
//     if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_Upper(p_limit)); }
//     else { Motor_TryClearSpeedLimit(p_motor); }
// }

// void Motor_SetILimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
// {
//     // LimitArray_T * p_local = &p_motor->I_LIMIT_SOURCES;
//     if (LimitArray_IsUpperActive(p_limit) == true)
//     {
//         Motor_TryILimit(p_motor, fract16_mul(LimitArray_UpperComposed(p_local, p_limit), Phase_Calibration_GetIRatedPeak_Fract16()));
//     }
//     else { Motor_TryClearILimit(p_motor); }
// }

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
//     Motor_SetActiveCmdScalar(p_motor->P_MOTOR, p_input->CmdValue);

//     // if (p_input->SpeedLimit != p_prev->SpeedLimit)
//     // {
//     //     p_prev->SpeedLimit = p_input->SpeedLimit;
//     //     MotorController_SetUserSpeedLimitAll(p_dev, MOT_SPEED_LIMIT_USER, p_input->SpeedLimit);
//     // }

//     // if (p_input->ILimit != p_prev->ILimit)
//     // {
//     //     p_prev->ILimit = p_input->ILimit;
//     //     MotorController_SetUserILimitAll(p_dev, MOT_I_LIMIT_USER, p_input->ILimit);
//     // }
// }

// /* units as FeedbackMode Set */
// void Motor_ProcInputSetpoint(const Motor_T * p_motor, Motor_Input_T * p_input)
// {
//     if (p_input->CmdValue != Motor_GetCmd(p_motor->P_MOTOR)) /* compare applicable to mixed units values */
//     {
//         Motor_SetActiveCmdValue(p_motor->P_MOTOR, p_input->CmdValue);
//     }
// }


