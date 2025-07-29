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
    Homing SubState, alternatively move to OpenLoop
    Start from Config State only
*/
/******************************************************************************/
/*
    with position sensor without position feedback loop
*/
static void Calibration_HomeEntry(const Motor_T * p_motor)
{
    // // for now
    // p_motor->P_MOTOR_STATE->ControlTimerBase = 0U;
    // p_motor->P_MOTOR_STATE->CalibrationStateIndex = 0U;
    // p_motor->P_MOTOR_STATE->FeedbackMode.Current = 0U;

    // Phase_ActivateOutputV0(&p_motor->PHASE);
    // Timer_StartPeriod_Millis(&p_motor->P_MOTOR_STATE->ControlTimer, 20); // ~1rpm
    // Ramp_Set(&p_motor->P_MOTOR_STATE->OpenLoopIRamp, p_motor->P_MOTOR_STATE->Config.OpenLoopRampITime_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->P_MOTOR_STATE->Config.OpenLoopRampIFinal_Fract16));

    // p_motor->P_MOTOR_STATE->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    // p_motor->P_MOTOR_STATE->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    // // Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, MOTOR_DIRECTION_CCW);
    // Motor_FOC_SetDirection_Cast(p_motor, p_motor->P_MOTOR_STATE->Direction);
}

/*

*/
// todo step speed calc
static void Calibration_ProcHome(const Motor_T * p_motor)
{
    // // uint16_t angleDelta = Encoder_GetHomingAngle(&p_motor->Encoder); // * direction
    // /* alternatively use openloop speed */
    // uint16_t angleDelta = 65536/1000;

    // // RotorSensor_GetMechanicalAngle(p_motor->Sensor) get direction

    // if (Timer_Periodic_Poll(&p_motor->P_MOTOR_STATE->ControlTimer) == true)
    // {
    //     Motor_PollSensorAngle(p_motor); /*  */

    //     // p_motor->P_MOTOR_STATE->ElectricalAngle = (Motor_GetMechanicalAngle(p_motor) + angleDelta) * p_motor->P_MOTOR_STATE->Config.PolePairs;
    //     p_motor->P_MOTOR_STATE->ElectricalAngle += (angleDelta * p_motor->P_MOTOR_STATE->Config.PolePairs);
    //     // Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->P_MOTOR_STATE->ElectricalAngle, Ramp_ProcOutput(&p_motor->AuxRamp), 0);
    //     Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->P_MOTOR_STATE->ElectricalAngle, p_motor->P_MOTOR_STATE->Config.OpenLoopRampIFinal_Fract16 * 2, 0);
    // }
}

static State_T * Calibration_HomeEnd(const Motor_T * p_motor)
{
    // State_T * p_nextState = NULL;
    // uint16_t angleDelta = 65536 / 1000;

    // /* error on full rev todo */
    // if (angle16_cycle(Motor_GetMechanicalAngle(p_motor), Motor_GetMechanicalAngle(p_motor) + angleDelta, (p_motor->P_MOTOR_STATE->Direction == MOTOR_DIRECTION_CCW)) == true)
    // {
    //     Phase_ActivateOutputV0(&p_motor->PHASE);
    //     _StateMachine_EndSubState(p_motor->STATE_MACHINE.P_MOTOR_STATE);
    //     p_nextState = &MOTOR_STATE_CALIBRATION;
    // }

    // p_motor->P_MOTOR_STATE->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    return NULL;
}

static const State_T CALIBRATION_STATE_HOMING =
{
    // .ID         = MSM_STATE_ID_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)Calibration_HomeEntry,
    .LOOP       = (State_Action_T)Calibration_ProcHome,
    .NEXT       = (State_InputVoid_T)Calibration_HomeEnd,
};

static State_T * Calibration_StartHome(const Motor_T * p_motor, state_value_t null)
{
    // if (RotorSensor_IsAngleHomeSet(p_motor->Sensor) == false) { return &MOTOR_STATE_CALIBRATION; }
    return &CALIBRATION_STATE_HOMING;
}


/* Transition from any Calibration State */
void Motor_Calibration_StartHome(const Motor_T * p_motor)
{
    static const StateMachine_TransitionInput_T CALIBRATION_STATE_HOMING_TRANSITION = { .P_START = &MOTOR_STATE_CALIBRATION, .TRANSITION = (State_Input_T)Calibration_StartHome, };
    StateMachine_InvokeBranchTransition(&p_motor->STATE_MACHINE, &CALIBRATION_STATE_HOMING_TRANSITION, 0U);
}


/******************************************************************************/
/*!
    @brief Calibration
*/
/******************************************************************************/
// static const State_T CALIBRATION_STATE_CONFIG =
// {
//     // .ID         = MSM_STATE_ID_CALIBRATION,
//     .P_PARENT = &MOTOR_STATE_CALIBRATION,
//     .P_TOP = &MOTOR_STATE_CALIBRATION,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)Calibration_TuningEntry,
//     .LOOP = (State_Action_T)Calibration_TuningLoop,
//     .NEXT = (State_InputVoid_T)Calibration_TuningEnd,
// };


// static const State_T RUN_STATE_TUNNING =
// {
//     // .ID         = MSM_STATE_ID_CALIBRATION,
//     .P_PARENT = &MOTOR_STATE_RUN,
//     .P_TOP = &MOTOR_STATE_RUN,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)Calibration_TuningEntry,
//     .LOOP = (State_Action_T)Calibration_TuningLoop,
//     .NEXT = (State_InputVoid_T)Calibration_TuningEnd,
// };