
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
void MotDrive_StartThrottleMode(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE->Config.ThrottleMode)
    {
        case MOT_DRIVE_THROTTLE_MODE_SPEED:  MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_StartSpeedMode);     break;
        case MOT_DRIVE_THROTTLE_MODE_TORQUE: MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_StartTorqueMode);    break;
        default: break;
    }
}

void MotDrive_SetThrottleValue(const MotDrive_T * p_motDrive, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;
    switch (p_motDrive->P_MOT_DRIVE->Config.ThrottleMode)
    {
        case MOT_DRIVE_THROTTLE_MODE_SPEED:  MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_User_SetSpeedCmd_Scalar, cmdValue);     break;
        case MOT_DRIVE_THROTTLE_MODE_TORQUE: MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_User_SetICmd_Scalar, cmdValue);         break;
        default: break;
    }
}

// apply hold on low speed
void MotDrive_StartBrakeMode(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE->Config.BrakeMode)
    {
        case MOT_DRIVE_BRAKE_MODE_TORQUE:  MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_StartTorqueMode);   break;
        case MOT_DRIVE_BRAKE_MODE_VOLTAGE: MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_StartVoltageMode);  break;
        default: break;
    }
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

    switch (p_motDrive->P_MOT_DRIVE->Config.BrakeMode)
    {
        case MOT_DRIVE_BRAKE_MODE_TORQUE: MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_User_SetICmd_Scalar, cmdValue); break;
        case MOT_DRIVE_BRAKE_MODE_VOLTAGE: MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_User_SetRegenCmd, 0); break;
        default: break;
    }
}

/* an alternate cmd for float is required */
void MotDrive_StartDriveZero(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE->Config.ZeroMode)
    {
        case MOT_DRIVE_ZERO_MODE_FLOAT: MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Release); break;
        case MOT_DRIVE_ZERO_MODE_CRUISE: MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_StartTorqueMode); break;
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
    switch (p_motDrive->P_MOT_DRIVE->Config.ZeroMode)
    {
        case MOT_DRIVE_ZERO_MODE_FLOAT: break;
        case MOT_DRIVE_ZERO_MODE_CRUISE: MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_User_SetICmd_Scalar, 0); break;
        case MOT_DRIVE_ZERO_MODE_REGEN: break;
        default: break;
    }
}

static inline bool _MotDrive_ProcOnDirection(const MotDrive_T * p_motDrive, MotDrive_Direction_T direction)
{
    // if((p_motDrive->P_MOT_DRIVE->Config.BuzzerFlagsEnable.OnReverse == true))
    // {
    //     if(p_this->DriveDirection == MOT_DRIVE_DIRECTION_REVERSE)
    //     {
    //         MotDrive_BeepPeriodicType1(p_this);
    //     }
    //     else
    //     {
    //         Blinky_Stop(&p_this->Buzzer);
    //     }
    // }
}




/******************************************************************************/
/*
    State Machine
*/
/******************************************************************************/
static const State_T STATE_PARK;
static const State_T STATE_DRIVE;
static const State_T STATE_NEUTRAL;

#define MOT_DRIVE_TRANSITION_TABLE_LENGTH (3U)

const StateMachine_Machine_T MOT_DRIVE_MACHINE =
{
    .P_STATE_INITIAL = &STATE_PARK,
    .TRANSITION_TABLE_LENGTH = MOT_DRIVE_TRANSITION_TABLE_LENGTH,
};

/******************************************************************************/
/*!
    @brief
    Motors in Stop State.
    May enter from Neutral State or Drive State
*/
/******************************************************************************/
static void Park_Entry(const MotDrive_T * p_motDrive)
{
    // assert(MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == true);

    if (MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == true) { MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Hold); }

    // MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Stop);
}

static void Park_Proc(const MotDrive_T * p_motDrive)
{
    if (MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == false) { p_motDrive->P_MOT_DRIVE->Status = MOT_DRIVE_STATUS_FAULT; }
}

// static State_T * Park_InputBlocking(const MotDrive_T * p_motDrive, state_input_value_t lockId)
// {
//     return ((MotDrive_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER) ? &STATE_LOCK : NULL;
// }

static State_T * Park_InputDirection(const MotDrive_T * p_motDrive, state_input_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((MotDrive_Direction_T)direction)
    {
        case MOT_DRIVE_DIRECTION_PARK:       p_nextState = NULL;             break;
        case MOT_DRIVE_DIRECTION_NEUTRAL:    p_nextState = &STATE_NEUTRAL;   break;
        case MOT_DRIVE_DIRECTION_FORWARD:    MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ApplyDirectionForward); p_nextState = &STATE_DRIVE; break;
        case MOT_DRIVE_DIRECTION_REVERSE:    MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ApplyDirectionReverse); p_nextState = &STATE_DRIVE; break;
            // case MOT_DRIVE_DIRECTION_FORWARD:    p_nextState = MotDrive_SetDirectionForwardAll(p_this) ? &STATE_DRIVE : NULL; break;
            // case MOT_DRIVE_DIRECTION_REVERSE:    p_nextState = MotDrive_SetDirectionReverseAll(p_this) ? &STATE_DRIVE : NULL; break;
        default: break;
    }
    return p_nextState;
}

static const State_Input_T PARK_TRANSITION_TABLE[MOT_DRIVE_TRANSITION_TABLE_LENGTH] =
{
    [MOT_DRIVE_STATE_INPUT_DIRECTION] = (State_Input_T)Park_InputDirection,

    // [MOT_DRIVE_INPUT_FAULT] = (State_Input_T)TransitionFault,
    // [MOT_DRIVE_INPUT_LOCK] = (State_Input_T)Park_InputBlocking,
    // #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
    //     [MOT_DRIVE_INPUT_SERVO]      = (State_Input_T)Park_InputServo,
    // #endif
        // [MOT_DRIVE_INPUT_USER_MODE]  = (State_Input_T)Park_InputUser,
};

static const State_T STATE_PARK =
{
    .ID = MOT_DRIVE_STATE_ID_PARK,
    .P_TRANSITION_TABLE = &PARK_TRANSITION_TABLE[0U],
    .ENTRY = (State_Action_T)Park_Entry,
    .LOOP = (State_Action_T)Park_Proc,
};


/******************************************************************************/
/*!
    @brief Drive Set Common
*/
/******************************************************************************/
static State_T * Drive_Common_InputDirection(const MotDrive_T * p_motDrive, state_input_value_t direction)
{
    State_T * p_nextState = NULL;

    switch ((MotDrive_Direction_T)direction)
    {
        case MOT_DRIVE_DIRECTION_PARK:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Stop);
            p_nextState = MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) ? &STATE_PARK : NULL;
            break;
        case MOT_DRIVE_DIRECTION_FORWARD:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ApplyDirectionForward);
            p_nextState = MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) ? &STATE_DRIVE : NULL;
            break;
        case MOT_DRIVE_DIRECTION_REVERSE:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ApplyDirectionReverse);
            p_nextState = MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionReverse) ? &STATE_DRIVE : NULL;
            break;
        case MOT_DRIVE_DIRECTION_NEUTRAL: p_nextState = &STATE_NEUTRAL; break; /* set on entry */
        case MOT_DRIVE_DIRECTION_ERROR: p_nextState = NULL; break;
        default: break;
    }

    return p_nextState;
}

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
    // MotMotors_SetCmdValue(&p_motDrive->MOTORS, 0);
    // MotDrive_StartControlAll(p_this);
    // p_this->StateFlags.IsStopped = 0U;
    p_motDrive->P_MOT_DRIVE->Input.Cmd = MOT_DRIVE_CMD_RELEASE;
}

/* periodic on values ~ 1-10ms */
static void Drive_Proc(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE->Input.Cmd)
    {
        case MOT_DRIVE_CMD_BRAKE: MotDrive_SetBrakeValue(p_motDrive, p_motDrive->P_MOT_DRIVE->Input.BrakeValue); break;
        case MOT_DRIVE_CMD_THROTTLE: MotDrive_SetThrottleValue(p_motDrive, p_motDrive->P_MOT_DRIVE->Input.ThrottleValue); break;
        case MOT_DRIVE_CMD_RELEASE: MotDrive_ProcDriveZero(p_motDrive); break;
        default: break;
    }
}

/* handle on edge */
static State_T * Drive_InputCmdStart(const MotDrive_T * p_motDrive, state_input_value_t mode)
{
    /* optionally use prevCmd or   substate to handle feedbackonly or activeOutputWith feedback */
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

static State_T * Drive_InputDirection(const MotDrive_T * p_motDrive, state_input_value_t direction)
{
    State_T * p_nextState = NULL;
    switch((MotDrive_Direction_T)direction)
    {
        case MOT_DRIVE_DIRECTION_PARK:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Stop);
            p_nextState = MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) ? &STATE_PARK : NULL;
            break;
        case MOT_DRIVE_DIRECTION_NEUTRAL:    p_nextState = &STATE_NEUTRAL;   break;
        case MOT_DRIVE_DIRECTION_FORWARD:    p_nextState = NULL;             break;
        case MOT_DRIVE_DIRECTION_REVERSE:    p_nextState = NULL;             break;
        // case MOT_DRIVE_DIRECTION_FORWARD: p_nextState = MotDrive_SetDirectionForwardAll(p_this) ? &STATE_DRIVE : NULL; break;
        // case MOT_DRIVE_DIRECTION_REVERSE: p_nextState = MotDrive_SetDirectionReverseAll(p_this) ? &STATE_DRIVE : NULL; break;
        default: break;
    }

    return p_nextState;
}

static const State_Input_T DRIVE_TRANSITION_TABLE[MOT_DRIVE_TRANSITION_TABLE_LENGTH] =
{
    [MOT_DRIVE_STATE_INPUT_DIRECTION] = (State_Input_T)Drive_InputDirection,
    [MOT_DRIVE_STATE_INPUT_CMD_START] = (State_Input_T)Drive_InputCmdStart,
    // [MOT_DRIVE_INPUT_FAULT] = (State_Input_T)TransitionFault,
    // [MOT_DRIVE_STATE_INPUT_THROTTLE] = (State_Input_T)Drive_InputThrottle,
    // [MOT_DRIVE_STATE_INPUT_BRAKE] = (State_Input_T)Drive_InputBrake,
    // [MOT_DRIVE_STATE_INPUT_CMD] = (State_Input_T)Drive_InputCmd,
    // [MOT_DRIVE_STATE_INPUT_CMD_MODE] = (State_Input_T)Drive_InputFeedbackMode,
};

static const State_T STATE_DRIVE =
{
    .ID = MOT_DRIVE_STATE_ID_DRIVE,
    .ENTRY = (State_Action_T)Drive_Entry,
    .LOOP = (State_Action_T)Drive_Proc,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief  Neutral State
    Drive extension
    Motor States: Drive, Freewheel
        Motor maybe in Drive when Braking
        MOT_DRIVE remains in Neutral state
    Accepted Inputs: Brake only, Throttle no effect.
    Motor Direction unchanged upon entering MC Neutral
*/
/******************************************************************************/
static void Neutral_Entry(const MotDrive_T * p_motDrive)
{
    if (p_motDrive->P_MOT_DRIVE->Input.Cmd != MOT_DRIVE_CMD_BRAKE)
        { MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Release); }  /* If enter neutral while braking, handle discontinuity */
}

static void Neutral_Proc(const MotDrive_T * p_motDrive)
{
    switch (p_motDrive->P_MOT_DRIVE->Input.Cmd)
    {
        case MOT_DRIVE_CMD_RELEASE:
            // MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_PASSIVE)
            break;
        case MOT_DRIVE_CMD_BRAKE: MotDrive_SetBrakeValue(p_motDrive, p_motDrive->P_MOT_DRIVE->Input.BrakeValue); break;
        case MOT_DRIVE_CMD_THROTTLE: break;
        default: break;
    }
}

static State_T * Neutral_InputDirection(const MotDrive_T * p_motDrive, state_input_value_t direction)
{
    State_T * p_nextState = NULL;

    switch((MotDrive_Direction_T)direction)
    {
        case MOT_DRIVE_DIRECTION_PARK:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Stop);
            p_nextState = MotMotors_IsEveryValue(&p_motDrive->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) ? &STATE_PARK : NULL;
            break;
        case MOT_DRIVE_DIRECTION_FORWARD:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ApplyDirectionForward);
            p_nextState = MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) ? &STATE_DRIVE : NULL;
            break;
        case MOT_DRIVE_DIRECTION_REVERSE:
            MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ApplyDirectionReverse);
            p_nextState = MotMotors_IsEvery(&p_motDrive->MOTORS, Motor_IsDirectionForward) ? &STATE_DRIVE : NULL;
            break;
        case MOT_DRIVE_DIRECTION_NEUTRAL: p_nextState = NULL; break;
        case MOT_DRIVE_DIRECTION_ERROR: p_nextState = NULL; break;
        default: break;
    }

    return p_nextState;
}

static State_T * Neutral_InputBrake(const MotDrive_T * p_motDrive, state_input_value_t mode)
{
    switch ((MotDrive_Cmd_T)mode)
    {
        case MOT_DRIVE_CMD_RELEASE: MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Release); break;
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
    // [MOT_DRIVE_INPUT_FAULT]              = (State_Input_T)TransitionFault,
    // [MOT_DRIVE_STATE_INPUT_DIRECTION]    = (State_Input_T)Neutral_InputDirection,
    // [MOT_DRIVE_STATE_INPUT_BRAKE]        = (State_Input_T)Neutral_InputBrake,
    // [MOT_DRIVE_INPUT_DRIVE]              = (State_Input_T)Neutral_InputDrive,
};

static const State_T STATE_NEUTRAL =
{
    .ID = MOT_DRIVE_STATE_ID_NEUTRAL,
    .ENTRY = (State_Action_T)Neutral_Entry,
    .LOOP = (State_Action_T)Neutral_Proc,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
};

