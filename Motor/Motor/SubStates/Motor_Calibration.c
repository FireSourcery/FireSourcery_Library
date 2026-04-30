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
    @file   Motor_Calibration.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_Calibration.h"

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/

// /*
//     Same as run
// */
// static void Tuning_Entry(const Motor_T * p_motor)
// {
//     Phase_ActivateT0(&p_motor->PHASE);

//     /* reload from */
//     p_motor->P_MOTOR->Config.PidSpeed = p_motor->P_NVM_CONFIG->PidSpeed;
//     p_motor->P_MOTOR->Config.PidI = p_motor->P_NVM_CONFIG->PidI;
//     Motor_ResetSpeedPid(p_motor->P_MOTOR);
//     Motor_ResetIPid(p_motor->P_MOTOR);
// }

// static void Tuning_Proc(const Motor_T * p_motor)
// {
//     Motor_ProcOuterFeedback(p_motor->P_MOTOR);
//     Motor_FOC_ProcAngleControl(p_motor->P_MOTOR);
//     Motor_FOC_WriteDuty(p_motor);
// }

// /*
// */
// static State_T * Tuning_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
// {
//     Motor_SetFeedbackMode(p_motor->P_MOTOR, Motor_FeedbackMode_Cast(feedbackMode));
//     Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR);
//     return NULL;
// }

// static const State_Input_T TUNING_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
// {
//     [MOTOR_STATE_INPUT_FEEDBACK_MODE] = (State_Input_T)Tuning_InputFeedbackMode,
//     [MOTOR_STATE_INPUT_CALIBRATION] = NULL,
//     [MOTOR_STATE_INPUT_OPEN_LOOP] = NULL,
// };

// const State_T CALIBRATION_STATE_TUNING =
// {
//     // .ID         = STATE_PATH_ID(MOTOR_STATE_ID_CALIBRATION, MOTOR_CALIBRATION_STATE_TUNING),
//     .P_PARENT   = &MOTOR_STATE_CALIBRATION,
//     .P_TOP      = &MOTOR_STATE_CALIBRATION,
//     .DEPTH      = 1U,
//     .ENTRY      = (State_Action_T)Tuning_Entry,
//     .LOOP       = (State_Action_T)Tuning_Proc,
//     .P_TRANSITION_TABLE = TUNING_TRANSITION_TABLE,
// };

// // void Motor_Calibration_EnterTuning(const Motor_T * p_motor)
// // {
// //     StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_TUNING);
// // }

// static State_T * Tuning_Start(const Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE_TUNING; }

// void Motor_Calibration_EnterTuning(const Motor_T * p_motor)
// {
//     static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Tuning_Start };
//     StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
// }


// bool Motor_Calibration_IsTuning(const Motor_T * p_motor) { return (StateMachine_IsActiveBranch(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_TUNING)); }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/*
    Excite + Capture state. Single owner — only one motor tunes at a time.
    File-scope keeps Motor_State_T unchanged; ownership pointer guards cross-motor access.
*/
static Motor_Tuning_Config_T  s_TuningConfig = { .Shape = MOTOR_TUNING_EXCITE_TRACK };
static Motor_Tuning_Capture_T s_TuningCapture;
static const Motor_State_T *  sp_TuningOwner;
static uint32_t               s_TuningTick;

static int16_t Excite_Eval(const Motor_Tuning_Config_T * p_cfg, uint32_t tick)
{
    switch (p_cfg->Shape)
    {
        case MOTOR_TUNING_EXCITE_STEP:   return (tick < p_cfg->StepDelay) ? 0 : p_cfg->Amplitude;
        case MOTOR_TUNING_EXCITE_SQUARE: return ((tick / p_cfg->HalfPeriod) & 1U) ? -p_cfg->Amplitude : p_cfg->Amplitude;
        default:                         return 0;
    }
}

static int16_t Capture_Feedback(const Motor_State_T * p_motor, Motor_Tuning_Channel_T ch)
{
    return (ch == MOTOR_TUNING_CHANNEL_IQ) ? FOC_Iq(&p_motor->Foc) : (int16_t)Motor_GetSpeedFeedback(p_motor);
}

static void Capture_Sample(Motor_Tuning_Capture_T * p_cap, int16_t sp, int16_t fb)
{
    if (p_cap->Full == true) { return; }
    p_cap->Setpoint[p_cap->Index] = sp;
    p_cap->Feedback[p_cap->Index] = fb;
    p_cap->Index++;
    if (p_cap->Index >= MOTOR_TUNING_CAPTURE_LEN) { p_cap->Full = true; }
}

/*
    Same loop as Run; if armed for Excite, override the active loop's setpoint and capture (sp, fb).
*/
static void AutoTuning_Entry(const Motor_T * p_motor)
{
    Phase_ActivateT0(&p_motor->PHASE);

    /* reload from NVM so live edits are discarded on (re)entry */
    p_motor->P_MOTOR->Config.PidSpeed = p_motor->P_NVM_CONFIG->PidSpeed;
    p_motor->P_MOTOR->Config.PidI = p_motor->P_NVM_CONFIG->PidI;
    Motor_ResetSpeedPid(p_motor->P_MOTOR);
    Motor_ResetIPid(p_motor->P_MOTOR);

    s_TuningConfig.Shape = MOTOR_TUNING_EXCITE_TRACK;
    s_TuningCapture.Index = 0U;
    s_TuningCapture.Full = false;
    s_TuningTick = 0U;
    sp_TuningOwner = p_motor->P_MOTOR;
}

static void AutoTuning_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    if ((s_TuningConfig.Shape != MOTOR_TUNING_EXCITE_TRACK) && (sp_TuningOwner == p_state))
    {
        int16_t sp = Excite_Eval(&s_TuningConfig, s_TuningTick);
        int16_t fb = Capture_Feedback(p_state, s_TuningConfig.Channel);

        if (s_TuningConfig.Channel == MOTOR_TUNING_CHANNEL_SPEED) { p_state->UserSpeedReq = sp; }
        else                                                      { p_state->UserTorqueReq = sp; }

        Capture_Sample(&s_TuningCapture, sp, fb);
        s_TuningTick++;
    }

    Motor_ProcOuterFeedback(p_state);
    Motor_FOC_ProcAngleControl(p_state);
    Motor_FOC_WriteDuty(p_motor);
}


// static State_T * Tuning_InputControl(const Motor_T * p_motor, state_value_t phaseOutput)
// {
//     State_T * p_nextState = NULL;
//     switch ((Phase_Output_T)phaseOutput)
//     {
//         // case PHASE_VOUT_Z: p_nextState = &MOTOR_STATE_PASSIVE; break;
//         case PHASE_VOUT_Z: Phase_Deactivate(&p_motor->PHASE); break;
//         case PHASE_VOUT_0: Phase_ActivateV0(&p_motor->PHASE); break;
//         case PHASE_VOUT_PWM: Phase_ActivateT0(&p_motor->PHASE); break;
//         default: break;
//     }
//     return p_nextState;
//     // return &MOTOR_STATE_CALIBRATION;
// }

const State_T CALIBRATION_STATE_AUTO_TUNING =
{
    // .ID         = STATE_PATH_ID(MOTOR_STATE_ID_CALIBRATION, MOTOR_CALIBRATION_STATE_TUNING),
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)AutoTuning_Entry,
    .LOOP       = (State_Action_T)AutoTuning_Proc,
};



// void Motor_Calibration_EnterTuning(const Motor_T * p_motor)
// {
//     StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_TUNING);
// }

static State_T * AutoTuning_Start(const Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE_AUTO_TUNING; }

void Motor_Calibration_EnterAutoTuning(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)AutoTuning_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}


/*
    Arm an excite + capture run. No-op if motor isn't currently in TUNING.
    Re-arming during a run resets the capture buffer and tick counter.
*/
void Motor_Calibration_Tuning_ArmExcite(const Motor_T * p_motor, const Motor_Tuning_Config_T * p_config)
{
    if (Motor_Calibration_IsTuning(p_motor) == false) { return; }
    s_TuningConfig = *p_config;
    s_TuningCapture.Index = 0U;
    s_TuningCapture.Full = false;
    s_TuningTick = 0U;
    sp_TuningOwner = p_motor->P_MOTOR;
}

void Motor_Calibration_Tuning_Disarm(const Motor_T * p_motor)
{
    if (sp_TuningOwner != p_motor->P_MOTOR) { return; }
    s_TuningConfig.Shape = MOTOR_TUNING_EXCITE_TRACK;
    p_motor->P_MOTOR->UserSpeedReq = 0;
    p_motor->P_MOTOR->UserTorqueReq = 0;
}

bool Motor_Calibration_Tuning_IsCaptureDone(const Motor_T * p_motor)
{
    return (sp_TuningOwner == p_motor->P_MOTOR) && s_TuningCapture.Full;
}

const Motor_Tuning_Capture_T * Motor_Calibration_Tuning_GetCapture(const Motor_T * p_motor)
{
    return (sp_TuningOwner == p_motor->P_MOTOR) ? &s_TuningCapture : NULL;
}

/******************************************************************************/
/*!
    Homing SubState, alternatively move to OpenLoop
    Start from Config State only
*/
/******************************************************************************/
/*
    with position sensor without position feedback loop
*/
static void Homing_Entry(const Motor_T * p_motor)
{
    // for now
    p_motor->P_MOTOR->ControlTimerBase = 0U;
    p_motor->P_MOTOR->CalibrationStateIndex = 0U;
    p_motor->P_MOTOR->FeedbackMode.Current = 0U;

    // Phase_ActivateV0(&p_motor->PHASE);
    // Timer_StartPeriod_Millis(&p_motor->P_MOTOR->ControlTimer, 20); // ~1rpm

    // p_motor->P_MOTOR->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    // p_motor->P_MOTOR->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    Motor_FOC_StartOpenLoop(p_motor->P_MOTOR);
}

/*

*/
static void Homing_Proc(const Motor_T * p_motor)
{
    (void)p_motor;
    // uint16_t angleDelta = Encoder_GetHomingAngle(&p_motor->Encoder); // * direction
    /* alternatively use openloop speed */
    // uint16_t angleDelta = 65536/1000;

    // RotorSensor_GetMechanicalAngle(p_motor->Sensor) get direction

    // if (Timer_Periodic_Poll(&p_motor->P_MOTOR->ControlTimer) == true)
    // {
    //     Motor_PollSensorAngle(p_motor); /*  */
        // RotorSensor_CaptureAngle(p_motor->P_MOTOR->p_ActiveSensor);

        // p_motor->P_MOTOR->ElectricalAngle = (Motor_GetMechanicalAngle(p_motor) + angleDelta) * p_motor->P_MOTOR->Config.PolePairs;
        // p_motor->P_MOTOR->ElectricalAngle += (angleDelta * p_motor->P_MOTOR->Config.PolePairs);
        // Motor_FOC_AngleControl(p_motor, p_motor->P_MOTOR->ElectricalAngle, Ramp_ProcOutput(&p_motor->AuxRamp), 0);
        // Motor_FOC_AngleControl(p_motor, p_motor->P_MOTOR->ElectricalAngle, p_motor->P_MOTOR->Config.OpenLoopRampIFinal_Fract16 * 2, 0);
        // Motor_FOC_ProcOpenLoop(p_motor->P_MOTOR);
    // }

    // /* error on full rev todo */
    // if ( Home(p_motor), Motor_GetMechanicalAngle(p_motor) + angleDelta,

    // p_motor->P_MOTOR->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

}



static const State_T CALIBRATION_STATE_HOMING =
{
    // .ID         = STATE_PATH_ID(MOTOR_STATE_ID_CALIBRATION, MOTOR_CALIBRATION_STATE_HOMING),
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)Homing_Entry,
    .LOOP       = (State_Action_T)Homing_Proc,
};

static State_T * Homing_Start(const Motor_T * p_motor, state_value_t null)
{
    (void)p_motor; (void)null;
    // if (RotorSensor_IsAngleHomeSet(p_motor->Sensor) == false) { return &MOTOR_STATE_CALIBRATION; }
    return &CALIBRATION_STATE_HOMING;
}

/* Transition from any Calibration State */
void Motor_Calibration_StartHome(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CALIBRATION_STATE_HOMING_TRANSITION = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Homing_Start, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CALIBRATION_STATE_HOMING_TRANSITION, 0U);

    // StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_HOMING);
}

