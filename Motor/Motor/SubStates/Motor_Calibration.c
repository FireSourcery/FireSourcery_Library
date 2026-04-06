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
    @brief Tuning
    Marker for Var Set +
*/
/******************************************************************************/
extern const State_T CALIBRATION_STATE_TUNING;

/*
    Same as run
*/
static void Tuning_Entry(const Motor_T * p_motor)
{
    Phase_ActivateT0(&p_motor->PHASE);

    /* reload from */
    p_motor->P_MOTOR_STATE->Config.PidSpeed = p_motor->P_NVM_CONFIG->PidSpeed;
    p_motor->P_MOTOR_STATE->Config.PidI = p_motor->P_NVM_CONFIG->PidI;
    Motor_ResetSpeedPid(p_motor->P_MOTOR_STATE);
    Motor_ResetCurrentPid(p_motor->P_MOTOR_STATE);
}

static void Tuning_Proc(const Motor_T * p_motor)
{
    Motor_ProcOuterFeedback(p_motor->P_MOTOR_STATE);
    Motor_FOC_ProcAngleControl(p_motor->P_MOTOR_STATE);
    Motor_FOC_WriteDuty(p_motor);
}

static void Tuning_Exit(const Motor_T * p_motor)
{
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

static State_T * Tuning_InputFeedbackMode(const Motor_T * p_motor, state_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor->P_MOTOR_STATE, feedbackMode);
    Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR_STATE);
    return NULL;
}

const State_T CALIBRATION_STATE_TUNING =
{
    // .ID         = STATE_PATH_ID(MOTOR_STATE_ID_CALIBRATION, MOTOR_CALIBRATION_STATE_TUNING),
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)Tuning_Entry,
    .LOOP       = (State_Action_T)Tuning_Proc,
};

static const State_Input_T TUNING_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_FEEDBACK_MODE]   = (State_Input_T)Tuning_InputFeedbackMode,
    [MOTOR_STATE_INPUT_CALIBRATION]     = NULL,
    [MOTOR_STATE_INPUT_OPEN_LOOP]       = NULL,
};


// void Motor_Calibration_EnterTuning(const Motor_T * p_motor)
// {
//     StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE_TUNING);
// }

static State_T * Tuning_Start(const Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE_TUNING; }

void Motor_Calibration_EnterTuning(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Tuning_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}


bool Motor_Calibration_IsTuning(const Motor_T * p_motor) { return (StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &CALIBRATION_STATE_TUNING)); }

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
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U;
    p_motor->P_MOTOR_STATE->CalibrationStateIndex = 0U;
    p_motor->P_MOTOR_STATE->FeedbackMode.Current = 0U;

    // Phase_ActivateV0(&p_motor->PHASE);
    // Timer_StartPeriod_Millis(&p_motor->P_MOTOR_STATE->ControlTimer, 20); // ~1rpm

    // p_motor->P_MOTOR_STATE->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    // p_motor->P_MOTOR_STATE->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    Motor_FOC_StartOpenLoop(p_motor->P_MOTOR_STATE);
}

/*

*/
static void Homing_Proc(const Motor_T * p_motor)
{
    // uint16_t angleDelta = Encoder_GetHomingAngle(&p_motor->Encoder); // * direction
    /* alternatively use openloop speed */
    // uint16_t angleDelta = 65536/1000;

    // RotorSensor_GetMechanicalAngle(p_motor->Sensor) get direction

    // if (Timer_Periodic_Poll(&p_motor->P_MOTOR_STATE->ControlTimer) == true)
    // {
    //     Motor_PollSensorAngle(p_motor); /*  */
        // RotorSensor_CaptureAngle(p_motor->P_MOTOR_STATE->p_ActiveSensor);

        // p_motor->P_MOTOR_STATE->ElectricalAngle = (Motor_GetMechanicalAngle(p_motor) + angleDelta) * p_motor->P_MOTOR_STATE->Config.PolePairs;
        // p_motor->P_MOTOR_STATE->ElectricalAngle += (angleDelta * p_motor->P_MOTOR_STATE->Config.PolePairs);
        // Motor_FOC_AngleControl(p_motor, p_motor->P_MOTOR_STATE->ElectricalAngle, Ramp_ProcOutput(&p_motor->AuxRamp), 0);
        // Motor_FOC_AngleControl(p_motor, p_motor->P_MOTOR_STATE->ElectricalAngle, p_motor->P_MOTOR_STATE->Config.OpenLoopRampIFinal_Fract16 * 2, 0);
        // Motor_FOC_ProcOpenLoop(p_motor->P_MOTOR_STATE);
    // }

    // /* error on full rev todo */
    // if ( Home(p_motor), Motor_GetMechanicalAngle(p_motor) + angleDelta,

    // p_motor->P_MOTOR_STATE->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

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

