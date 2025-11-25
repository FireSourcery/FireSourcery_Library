
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
    @file   Vehicle_StateMachine.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Vehicle_StateMachine.h"
#include "Vehicle.h"




/******************************************************************************/
/*
    State Machine
*/
/******************************************************************************/
/* Neutral with configurable brake */
static const State_T STATE_DRIVE;
static const State_T STATE_NEUTRAL;
// static const State_T STATE_DISABLED;

#define VEHICLE_TRANSITION_TABLE_LENGTH (2U)

const StateMachine_Machine_T VEHICLE_MACHINE =
{
    .P_STATE_INITIAL = &STATE_NEUTRAL,
    .TRANSITION_TABLE_LENGTH = VEHICLE_TRANSITION_TABLE_LENGTH,
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
static void Drive_Entry(const Vehicle_T * p_vehicle)
{
    // assert(MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionForward) || MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionReverse));

    // if(!(MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionForward) || MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionReverse)))
    // {
    //     Mot
    // }

    // MotMotors_SetCmdValue(&p_vehicle->MOTORS, 0);
    // Vehicle_StartControlAll(p_this);
    // p_this->StateFlags.IsStopped = 0U;
    p_vehicle->P_VEHICLE_STATE->Input.Cmd = VEHICLE_CMD_RELEASE; // next input is edge transition

    // if (p_vehicle->P_VEHICLE_STATE->Input.Direction != _MotMotors_GetDirectionAll(p_vehicle))
    // {

    // }
}

static void Drive_Proc(const Vehicle_T * p_vehicle)
{
    switch (p_vehicle->P_VEHICLE_STATE->Input.Cmd)
    {
        case VEHICLE_CMD_BRAKE:     break;
        case VEHICLE_CMD_THROTTLE:  break;
        case VEHICLE_CMD_RELEASE:   Vehicle_ProcDriveZero(p_vehicle); break;
        default: break;
    }
    // switch (p_vehicle->P_VEHICLE_STATE->Input.Cmd)
    // {
    //     case VEHICLE_CMD_BRAKE:     Vehicle_SetBrakeValue(p_vehicle, p_vehicle->P_VEHICLE_STATE->Input.BrakeValue);         break;
    //     case VEHICLE_CMD_THROTTLE:  Vehicle_SetThrottleValue(p_vehicle, p_vehicle->P_VEHICLE_STATE->Input.ThrottleValue);   break;
    //     case VEHICLE_CMD_RELEASE:   Vehicle_ProcDriveZero(p_vehicle); break;
    //     default: break;
    // }
}

/* Externally detect cmd edge */
static State_T * Drive_InputCmdStart(const Vehicle_T * p_vehicle, state_value_t mode)
{
    switch ((Vehicle_Cmd_T)mode)
    {
        case VEHICLE_CMD_BRAKE:     Vehicle_StartBrakeMode(p_vehicle);      break;
        case VEHICLE_CMD_THROTTLE:  Vehicle_StartThrottleMode(p_vehicle);   break;
        case VEHICLE_CMD_RELEASE:   Vehicle_StartDriveZero(p_vehicle);      break;
        default: break;
    }
    return NULL;
}

static State_T * Drive_InputDirection(const Vehicle_T * p_vehicle, state_value_t direction)
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

static const State_Input_T DRIVE_TRANSITION_TABLE[VEHICLE_TRANSITION_TABLE_LENGTH] =
{
    [VEHICLE_STATE_INPUT_DIRECTION] = (State_Input_T)Drive_InputDirection,
    [VEHICLE_STATE_INPUT_DRIVE_CMD] = (State_Input_T)Drive_InputCmdStart,
};

static const State_T STATE_DRIVE =
{
    .ID         = VEHICLE_STATE_ID_DRIVE,
    // .DEPTH      = 2U,
    // .P_TOP      = &MC_STATE_MAIN,
    // .P_PARENT   = &MC_STATE_MAIN_VEHICLE,
    .ENTRY      = (State_Action_T)Drive_Entry,
    .LOOP       = (State_Action_T)Drive_Proc,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief  Neutral State
    Drive extension
    Motor States: Run (Braking only), Freewheel
        Motor maybe in Run when Braking
        VEHICLE remains in Neutral state
    Accepted Inputs: Brake only, Throttle no effect.
    Motor Direction unchanged upon entering MC Neutral
*/
/******************************************************************************/
static void Neutral_Entry(const Vehicle_T * p_vehicle)
{
    // if (p_vehicle->P_VEHICLE_STATE->Input.Cmd != VEHICLE_CMD_BRAKE)
    //     { MotMotors_ActivateControlState(&p_vehicle->MOTORS, PHASE_OUTPUT_FLOAT); }  /* If enter neutral while braking, handle discontinuity */

    MotMotors_ActivateControlState(&p_vehicle->MOTORS, PHASE_OUTPUT_FLOAT);
}

static void Neutral_Proc(const Vehicle_T * p_vehicle)
{
    // switch (p_vehicle->P_VEHICLE_STATE->Input.Cmd)
    // {
    //     case VEHICLE_CMD_RELEASE:
    //         // assert (MotMotors_IsEveryValue(&p_vehicle->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_PASSIVE)); // check for consistency
    //         break;
    //     // case VEHICLE_CMD_BRAKE: Vehicle_SetBrakeValue(p_vehicle, p_vehicle->P_VEHICLE_STATE->Input.BrakeValue); break;
    //     case VEHICLE_CMD_THROTTLE: break;
    //     default: break;
    // }
}


/*
    Motor Keeps Direction
*/
static State_T * Neutral_InputForward(const Vehicle_T * p_vehicle)
{
    if (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionForward) == true) { return &STATE_DRIVE; }
    if (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_vehicle->MOTORS, MOTOR_DIRECTION_FORWARD); return &STATE_DRIVE; }
    return NULL; // return (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionForward) == true) ? &STATE_DRIVE : NULL;
}

static State_T * Neutral_InputReverse(const Vehicle_T * p_vehicle)
{
    if (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionReverse) == true) { return &STATE_DRIVE; }
    if (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_vehicle->MOTORS, MOTOR_DIRECTION_REVERSE); return &STATE_DRIVE; }
    return NULL; // return (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionReverse) == true) ? &STATE_DRIVE : NULL;
}

// static State_T * Common_InputForward(const Vehicle_T * p_vehicle)
// {
//     MotMotors_ApplyUserDirection(&p_vehicle->MOTORS, MOTOR_DIRECTION_FORWARD);
//     return (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionForward) == true) ? &STATE_DRIVE : NULL;
//     return &STATE_DRIVE;
// }

static State_T * Neutral_InputDirection(const Vehicle_T * p_vehicle, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((Motor_Direction_T)direction)
    {
        case MOTOR_DIRECTION_FORWARD: p_nextState = Neutral_InputForward(p_vehicle); break;
        case MOTOR_DIRECTION_REVERSE: p_nextState = Neutral_InputReverse(p_vehicle); break;
        case MOTOR_DIRECTION_NONE:    p_nextState = &STATE_NEUTRAL; break;
        default: break;
    }
    return p_nextState;

//     if (MotMotors_IsEveryValue(&p_vehicle->MOTORS, Motor_IsDirection, direction) == true) { return &STATE_DRIVE; }
//     if (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_vehicle->MOTORS, direction); return &STATE_DRIVE; }
//     return NULL; // return (MotMotors_IsEvery(&p_vehicle->MOTORS, Motor_IsDirectionForward) == true) ? &STATE_DRIVE : NULL;
}

static State_T * Neutral_InputCmdStart(const Vehicle_T * p_vehicle, state_value_t mode)
{
    switch ((Vehicle_Cmd_T)mode)
    {
        case VEHICLE_CMD_RELEASE:   MotMotors_ActivateControlState(&p_vehicle->MOTORS, PHASE_OUTPUT_FLOAT); break;
        case VEHICLE_CMD_BRAKE:     Vehicle_StartBrakeMode(p_vehicle); break;
        case VEHICLE_CMD_THROTTLE:  break;
        default: break;
    }
    return NULL;
}

static const State_Input_T NEUTRAL_TRANSITION_TABLE[VEHICLE_TRANSITION_TABLE_LENGTH] =
{
    [VEHICLE_STATE_INPUT_DIRECTION]   = (State_Input_T)Neutral_InputDirection,
    [VEHICLE_STATE_INPUT_DRIVE_CMD]   = (State_Input_T)Neutral_InputCmdStart,
};

static const State_T STATE_NEUTRAL =
{
    .ID     = VEHICLE_STATE_ID_NEUTRAL,
    .ENTRY  = (State_Action_T)Neutral_Entry,
    .LOOP   = (State_Action_T)Neutral_Proc,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
};



