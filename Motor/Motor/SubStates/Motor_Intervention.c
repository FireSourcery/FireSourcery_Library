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
/*!
    @brief SubState: Torque Zero (SS0)

    Immediate torque removal with active current control.
    Generates braking current only (no motoring).
    User may resume to Run by re-applying VOUT_PWM.

    Auto-escalates to RampSafe on timeout if speed remains above freewheel limit.
*/
/******************************************************************************/
static void TorqueZero_Entry(const Motor_T * p_motor)
{
    p_motor->P_MOTOR_STATE->ControlTimerBase = 0U;
    Motor_SetFeedbackMode(p_motor->P_MOTOR_STATE, MOTOR_FEEDBACK_MODE_CURRENT); /* in case of voltage mode */
}

static void TorqueZero_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;
    // if (p_state->FeedbackMode.Current == 1U)
    // {
    //     p_state->UserTorqueReq = _Motor_GeneratingOnly(p_state, 0);
    // }
    Motor_FOC_ProcTorqueReq(p_state, 0, _Motor_GeneratingOnly(p_state, p_state->UserTorqueReq));
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * TorqueZero_Next(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;

    if (Motor_IsSpeedFreewheelLimitRange(p_state)) { return &MOTOR_STATE_PASSIVE; }
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
static void RampSafe_Entry(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;

    /* Match speed ramp to current speed for bumpless transfer */
    Ramp_SetOutputState(&p_state->SpeedRamp, Motor_GetSpeedFeedback(p_state));
    PID_SetOutputState(&p_state->PidSpeed, Ramp_GetOutput(&p_state->TorqueRamp));

    p_state->ControlTimerBase = 0U;
}

static void RampSafe_Proc(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;

    if (p_state->SpeedUpdateFlag == true)
    {
        p_state->SpeedUpdateFlag = false;
        p_state->UserTorqueReq = _Motor_GeneratingOnly(p_state, Motor_SpeedControlOf(p_state, 0));
    }

    Motor_FOC_ProcTorqueReq(p_state, 0, Motor_IRampOf(p_state, p_state->UserTorqueReq));
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * RampSafe_Next(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR_STATE;

    if (Motor_IsSpeedFreewheelLimitRange(p_state)) { return &MOTOR_STATE_PASSIVE; }
    /* Watchdog: if speed not decreasing after timeout, escalate to fault */
    if (p_state->ControlTimerBase > MOTOR_INTERVENTION_RAMP_WATCHDOG) { return &MOTOR_STATE_FAULT; }

    return NULL;
}

const State_T INTERVENTION_STATE_RAMP_SAFE =
{
    .P_TOP      = &MOTOR_STATE_INTERVENTION,
    .P_PARENT   = &MOTOR_STATE_INTERVENTION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)RampSafe_Entry,
    .LOOP       = (State_Action_T)RampSafe_Proc,
    .NEXT       = (State_Input0_T)RampSafe_Next,
};
