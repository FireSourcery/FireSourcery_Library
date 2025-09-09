
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
    @file   MotDrive_StateMachine.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "MotDrive_StateMachine.h"
#include "MotDrive.h"


/******************************************************************************/
/*
    Drive Mode
    Staet Mode use full context
*/
/******************************************************************************/
/* Alternatively as substates */
void MotDrive_StartThrottleMode(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE_STATE->Config.ThrottleMode)
    {
        case MOT_DRIVE_THROTTLE_MODE_SPEED:  MotMotors_SetFeedbackMode(&p_motDrive->MOTORS, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);     break;
        case MOT_DRIVE_THROTTLE_MODE_TORQUE: MotMotors_SetFeedbackMode(&p_motDrive->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);           break;
        default: break;
    }

    /* alternatively from Release only */
    MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_VPWM);
}

void MotDrive_SetThrottleValue(const MotDrive_T * p_motDrive, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;

    switch (p_motDrive->P_MOT_DRIVE_STATE->Config.ThrottleMode)
    {
        case MOT_DRIVE_THROTTLE_MODE_SPEED:  MotMotors_SetCmdWith(&p_motDrive->MOTORS, Motor_User_SetSpeedCmd_Scalar, (int32_t)cmdValue);     break;
        case MOT_DRIVE_THROTTLE_MODE_TORQUE: MotMotors_SetCmdWith(&p_motDrive->MOTORS, Motor_User_SetICmd_Scalar, (int32_t)cmdValue);         break;
        default: break;
    }
}

// apply hold on low speed
void MotDrive_StartBrakeMode(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE_STATE->Config.BrakeMode)
    {
        case MOT_DRIVE_BRAKE_MODE_TORQUE:  MotMotors_SetFeedbackMode(&p_motDrive->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);  break;
        // case MOT_DRIVE_BRAKE_MODE_VOLTAGE: MotMotors_SetFeedbackMode(&p_motDrive->MOTORS, MOTOR_FEEDBACK_MODE_VOLTAGE);  break;
        default: break;
    }

    /* alternatively from Release only */
    MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_VPWM);
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void MotDrive_SetBrakeValue(const MotDrive_T * p_motDrive, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_motDrive->P_MOT_DRIVE_STATE->Config.BrakeMode)
    {
        case MOT_DRIVE_BRAKE_MODE_TORQUE: MotMotors_SetCmdWith(&p_motDrive->MOTORS, Motor_User_SetICmd_Scalar, cmdValue); break;
        // case MOT_DRIVE_BRAKE_MODE_VOLTAGE: MotMotors_SetCmdWith(&p_motDrive->MOTORS, Motor_User_SetRegenCmd, 0); break;
        default: break;
    }
}

/* an alternate cmd for float is required */
void MotDrive_StartDriveZero(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE_STATE->Config.ZeroMode)
    {
        case MOT_DRIVE_ZERO_MODE_FLOAT:     MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_FLOAT);        break;
        case MOT_DRIVE_ZERO_MODE_CRUISE:    MotMotors_SetFeedbackMode(&p_motDrive->MOTORS, MOTOR_FEEDBACK_MODE_CURRENT);    break;
        case MOT_DRIVE_ZERO_MODE_REGEN: /* MotDrive_SetRegenMotorAll(p_this); */ break;
        default: break;
    }
}

/*
    Check Stop / Zero Throttle
    Eventually release for stop transition
*/
void MotDrive_ProcDriveZero(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE_STATE->Config.ZeroMode)
    {
        case MOT_DRIVE_ZERO_MODE_FLOAT: break;
        case MOT_DRIVE_ZERO_MODE_CRUISE: MotMotors_SetCmdWith(&p_motDrive->MOTORS, Motor_User_SetICmd_Scalar, 0); break;
        case MOT_DRIVE_ZERO_MODE_REGEN: break;
        default: break;
    }
}



/******************************************************************************/
/*
    State Machine
*/
/******************************************************************************/
// alternatively neutral moves to top state. change to substate, and keep module as input filter/mapper

static const State_T STATE_DRIVE;
static const State_T STATE_NEUTRAL;
// static const State_T STATE_DISABLED;

#define MOT_DRIVE_TRANSITION_TABLE_LENGTH (2U)

const StateMachine_Machine_T MOT_DRIVE_MACHINE =
{
    .P_STATE_INITIAL = &STATE_NEUTRAL,
    .TRANSITION_TABLE_LENGTH = MOT_DRIVE_TRANSITION_TABLE_LENGTH,
};



/******************************************************************************/
/*!
    @brief Drive State

    Motor States: Run, Freewheel
    Accepted Inputs: Throttle, Brake
    Associate a drive release mode to cmd value 0, on top of motor layer
    SubStates: Fwd/Rev,

    neutral and drive share stop via motor stop state.
    DriveZero must release control, to transition into Park State
*/
/******************************************************************************/
static void Drive_Entry(const MotDrive_T * p_motDrive)
{
    // assert(MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) || MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse));

    // if(!(MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) || MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse)))
    // {
    //     Mot
    // }

    // MotMotors_SetCmdValue(&p_motDrive->MOTORS, 0);
    // MotDrive_StartControlAll(p_this);
    // p_this->StateFlags.IsStopped = 0U;
    p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd = MOT_DRIVE_CMD_RELEASE; // next input is edge transition

    // if (p_motDrive->P_MOT_DRIVE_STATE->Input.Direction != _MotMotors_GetDirectionAll(p_motDrive))
    // {

    // }
}

/* periodic on values ~ 1-10ms */
static void Drive_Proc(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd)
    {
        case MOT_DRIVE_CMD_BRAKE: MotDrive_SetBrakeValue(p_motDrive, p_motDrive->P_MOT_DRIVE_STATE->Input.BrakeValue); break;
        case MOT_DRIVE_CMD_THROTTLE: MotDrive_SetThrottleValue(p_motDrive, p_motDrive->P_MOT_DRIVE_STATE->Input.ThrottleValue); break;
        case MOT_DRIVE_CMD_RELEASE: MotDrive_ProcDriveZero(p_motDrive); break;
        default: break;
    }
}

/* handle on edge */
/* Externally detect cmd edge */
static State_T * Drive_InputCmdStart(const MotDrive_T * p_motDrive, state_value_t mode)
{
    /* optionally use prevCmd or substate to handle feedback only or activeOutputWith feedback */
    // start control from disabled output onyl,
    // only feedback from active output
    switch ((MotDrive_Cmd_T)mode)
    {
        case MOT_DRIVE_CMD_BRAKE: MotDrive_StartBrakeMode(p_motDrive); break;
        case MOT_DRIVE_CMD_THROTTLE: MotDrive_StartThrottleMode(p_motDrive); break;
        case MOT_DRIVE_CMD_RELEASE: MotDrive_StartDriveZero(p_motDrive); break;
        default: break;
    }
    return NULL;
}

static State_T * Drive_InputDirection(const MotDrive_T * p_motDrive, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch((Motor_User_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NONE:      p_nextState = &STATE_NEUTRAL;   break;
        case MOTOR_DIRECTION_FORWARD:   p_nextState = NULL;             break;
        case MOTOR_DIRECTION_REVERSE:   p_nextState = NULL;             break;
        default: break;
    }

    return p_nextState;
}

// static State_T * Drive_InputBuffer(const MotDrive_T * p_motDrive, state_value_t inputsPtr)
// {
//     State_T * p_nextState = NULL;
//     // Motor_User_Input_T * p_inputs = (Motor_User_Input_T *)inputsPtr;
//     return p_nextState;
// }

static const State_Input_T DRIVE_TRANSITION_TABLE[MOT_DRIVE_TRANSITION_TABLE_LENGTH] =
{
    [MOT_DRIVE_STATE_INPUT_DIRECTION] = (State_Input_T)Drive_InputDirection,
    [MOT_DRIVE_STATE_INPUT_CMD_START] = (State_Input_T)Drive_InputCmdStart,
};

static const State_T STATE_DRIVE =
{
    .ID     = MOT_DRIVE_STATE_ID_DRIVE,
    .ENTRY  = (State_Action_T)Drive_Entry,
    .LOOP   = (State_Action_T)Drive_Proc,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
};



/******************************************************************************/
/*!
    @brief  Neutral State
    Drive extension
    Motor States: Run (Braking only), Freewheel
        Motor maybe in Run when Braking
        MOT_DRIVE remains in Neutral state
    Accepted Inputs: Brake only, Throttle no effect.
    Motor Direction unchanged upon entering MC Neutral
*/
/******************************************************************************/
static void Neutral_Entry(const MotDrive_T * p_motDrive)
{
    // if (p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd != MOT_DRIVE_CMD_BRAKE)
    //     { MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_FLOAT); }  /* If enter neutral while braking, handle discontinuity */

    MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_FLOAT);
}

static void Neutral_Proc(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE_STATE->Input.Cmd)
    {
        case MOT_DRIVE_CMD_RELEASE:
            // assert (MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_PASSIVE)); // check for consistency
            break;
        case MOT_DRIVE_CMD_BRAKE: MotDrive_SetBrakeValue(p_motDrive, p_motDrive->P_MOT_DRIVE_STATE->Input.BrakeValue); break;
        case MOT_DRIVE_CMD_THROTTLE: break;
        default: break;
    }
}

/*
    Motor Keeps Direction
    // alternatively user motorcotroller tree
*/
static State_T * Neutral_InputForward(const MotDrive_T * p_motDrive)
{
    if (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) == true) { return &STATE_DRIVE; }
    if (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_motDrive->MOTORS, MOTOR_DIRECTION_FORWARD); return &STATE_DRIVE; }
    return NULL;
    // return (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) == true) ? &STATE_DRIVE : NULL;
}

static State_T * Neutral_InputReverse(const MotDrive_T * p_motDrive)
{
    if (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse) == true) { return &STATE_DRIVE; }
    if (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_motDrive->MOTORS, MOTOR_DIRECTION_REVERSE); return &STATE_DRIVE; }
    return NULL;
    // return (MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse) == true) ? &STATE_DRIVE : NULL;
}

static State_T * Neutral_InputDirection(const MotDrive_T * p_motDrive, state_value_t direction)
{
    State_T * p_nextState = NULL;

    // if
    // (
    //     MotMotors_IsEveryState(&p_motDrive->MOTORS, MSM_STATE_ID_RUN) ||
    //     MotMotors_IsEveryState(&p_motDrive->MOTORS, MSM_STATE_ID_PASSIVE) ||
    //     MotMotors_IsEveryState(&p_motDrive->MOTORS, MSM_STATE_ID_STOP)
    // )
    // {
        switch ((Motor_Direction_T)direction)
        {
            case MOTOR_DIRECTION_FORWARD: p_nextState = Neutral_InputForward(p_motDrive); break;
            case MOTOR_DIRECTION_REVERSE: p_nextState = Neutral_InputReverse(p_motDrive); break;
            case MOTOR_DIRECTION_NONE: p_nextState = &STATE_NEUTRAL; break;
            default: break;
        }
    // }

    return p_nextState;
}

static State_T * Neutral_InputBrake(const MotDrive_T * p_motDrive, state_value_t mode)
{
    switch ((MotDrive_Cmd_T)mode)
    {
        case MOT_DRIVE_CMD_RELEASE: MotMotors_ActivateControlState(&p_motDrive->MOTORS, PHASE_OUTPUT_FLOAT); break;
        case MOT_DRIVE_CMD_BRAKE: MotDrive_StartBrakeMode(p_motDrive); break;
        case MOT_DRIVE_CMD_THROTTLE: break;
        default: break;
    }
    return NULL;
}

static const State_Input_T NEUTRAL_TRANSITION_TABLE[MOT_DRIVE_TRANSITION_TABLE_LENGTH] =
{
    [MOT_DRIVE_STATE_INPUT_DIRECTION]   = (State_Input_T)Neutral_InputDirection,
    [MOT_DRIVE_STATE_INPUT_CMD_START]   = (State_Input_T)Neutral_InputBrake,
};

static const State_T STATE_NEUTRAL =
{
    .ID     = MOT_DRIVE_STATE_ID_NEUTRAL,
    .ENTRY  = (State_Action_T)Neutral_Entry,
    .LOOP   = (State_Action_T)Neutral_Proc,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
};



/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/*
    also returns NEUTRAL on error
*/
