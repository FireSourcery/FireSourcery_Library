/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Motor_Intervention.c
    @author FireSourcery
    @brief  Motor Intervention SubStates
*/
/******************************************************************************/
#include "Motor_Intervention.h"


/******************************************************************************/
/******************************************************************************/
/*
    Ramp Target is owned by user setters
    Intervention States used adjusted or alternate target.
*/
/******************************************************************************/
/*!
    @brief SubState: Torque Zero (SS0)

    Immediate torque removal with active current control.
    Generates braking current only (no motoring).
    User may resume to Run by re-applying VOUT_PWM.

    Auto-escalates to RampSafe on timeout if speed remains above freewheel limit.
*/
/******************************************************************************/
static void TorqueZero_Entry(Motor_T * p_motor)
{
    Motor_State_T * p_context = p_motor->P_MOTOR;
    p_context->ControlTimerBase = 0U;
    // Motor_SetFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CURRENT); /* in case of voltage mode */
    p_context->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT; /* Torque control mode for active braking */

    FOC_MatchIVState(&p_context->Foc, FOC_Vd(&p_context->Foc), FOC_Vq(&p_context->Foc));
    // Ramp_SetOutputLimit(&p_context->TorqueRamp, Motor_ILimitCw(p_context), Motor_ILimitCcw(p_context));
    // Ramp_SetOutputState(&p_context->TorqueRamp, FOC_Iq(&p_context->Foc));
    // Ramp_SetTarget(&p_context->TorqueRamp, 0); /* also set by user */

    // Motor_FOC_MatchIVState(p_state);
    // Ramp_SetOutputState(&p_state->TorqueRamp, 0);
    // Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR);

    /* seed Target to a small generating bias */
    // Ramp_SetTarget(&p_state->TorqueRamp, -1 * p_state->Direction * fract16_mul(Motor_ILimitGenerating(p_motor), 32768 / 20));
}

static void TorqueZero_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    /* Filter the (Entry-seeded or user-updated) Target: pass generating-side magnitude only;
       motoring-side requests are zeroed. Output ramps from 0 (set in Entry) toward the filtered value. */
    Motor_FOC_ProcTorqueReq(p_state, 0);    // user cmd to resume only
    // Motor_FOC_ProcTorqueReq(p_state, _Motor_GeneratingOnly(p_state, Ramp_GetTarget(&p_state->TorqueRamp)));
    /* optionally speed mod c */
}

// static State_T * TorqueZero_VOut(Motor_T * p_motor, state_value_t phaseOutput)
// {
//     switch ((Phase_VOutMode_T)phaseOutput)
//     {
//         case PHASE_VOUT_Z:
//         case PHASE_VOUT_0: return Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR) ? &MOTOR_STATE_PASSIVE : &INTERVENTION_STATE_RAMP_SAFE;
//         case PHASE_VOUT_PWM: return Motor_IsFeedbackAvailable(p_motor->P_MOTOR) ? //same as parent
//         default: return NULL;
//     }
//     return NULL;
// }

static State_T * TorqueZero_Next(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    // if (Motor_IsSpeedFreewheelLimitRange(p_state)) { return &MOTOR_STATE_PASSIVE; }
    if (p_state->ControlTimerBase > MOTOR_INTERVENTION_COAST_TIMEOUT) { return &INTERVENTION_STATE_RAMP_SAFE; }
    return NULL;
}

const State_T INTERVENTION_STATE_TORQUE_ZERO =
{
    .P_TOP      = &MOTOR_STATE_INTERVENTION,
    .P_PARENT   = &MOTOR_STATE_INTERVENTION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)TorqueZero_Entry,
    .LOOP       = (State_Action_T)TorqueZero_Proc,
    .NEXT       = (State_Input0_T)TorqueZero_Next,
};


/******************************************************************************/
/*!
    @brief SubState: Ramp to Safe Speed (SS1)

    Active deceleration using speed PID targeting zero speed.
    User resume is rejected — committed to safe stop.
    Monitors deceleration progress; escalates to Fault on watchdog timeout.
*/
/******************************************************************************/
static void RampSafe_Entry(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    /* Match speed ramp to current speed for bumpless transfer */
    Ramp_SetOutputState(&p_state->SpeedRamp, Motor_GetSpeedFeedback(p_state));
    PID_SetOutputState(&p_state->PidSpeed, Ramp_GetOutput(&p_state->TorqueRamp));

    p_state->ControlTimerBase = 0U;
}

static void RampSafe_Proc(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    if (p_state->SpeedUpdateFlag == true)
    {
        p_state->SpeedUpdateFlag = false;
        Motor_ProcSpeedControlOf(p_state, 0); /* drive to zero speed */
    }

    Motor_FOC_ProcTorqueReq(p_state, PID_GetOutput(&p_state->PidSpeed)); /* bypassing user torque ramp */
    //Motor_FOC_WriteDuty(p_motor);
}

static State_T * RampSafe_Next(Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    if (Motor_IsSpeedFreewheelLimitRange(p_state)) { return &MOTOR_STATE_PASSIVE; }
    /* Watchdog: if speed not decreasing after timeout, escalate to fault */
    if (p_state->ControlTimerBase > MOTOR_INTERVENTION_RAMP_WATCHDOG) { return &MOTOR_STATE_FAULT; }

    return NULL;
}


static State_T * Intervention_InputControl(Motor_T * p_motor, state_value_t phaseOutput)
{
    switch ((Phase_VOutMode_T)phaseOutput)
    {
        case PHASE_VOUT_Z:
        case PHASE_VOUT_0: return Motor_IsSpeedFreewheelLimitRange(p_motor->P_MOTOR) ? &MOTOR_STATE_PASSIVE : NULL;  /* same as parent */
        case PHASE_VOUT_PWM: return NULL; /* Reject */
        default: return NULL;
    }
    return NULL;
}

static const State_Input_T RAMP_SAFE_TRANSITION_TABLE[MOTOR_TRANSITION_TABLE_LENGTH] =
{
    [MOTOR_STATE_INPUT_PHASE_OUTPUT]    = (State_Input_T)Intervention_InputControl, /* override */
};

const State_T INTERVENTION_STATE_RAMP_SAFE =
{
    .P_TOP      = &MOTOR_STATE_INTERVENTION,
    .P_PARENT   = &MOTOR_STATE_INTERVENTION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)RampSafe_Entry,
    .LOOP       = (State_Action_T)RampSafe_Proc,
    .NEXT       = (State_Input0_T)RampSafe_Next,
    .P_TRANSITION_TABLE = &RAMP_SAFE_TRANSITION_TABLE[0U],
};
