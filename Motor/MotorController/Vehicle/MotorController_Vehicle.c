
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
    @file   MotorController_Vehicle.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_Vehicle.h"
#include "../MotorController_StateMachine.h"


/******************************************************************************/
/*!
    @brief [Vehicle] HSM SubState

    [Throttle] + [Brake] + [Neutral State]
    override MotorController StateMachine Inputs
*/
/******************************************************************************/
static const State_T STATE_DRIVE;
static const State_T STATE_NEUTRAL;


/* From Park */
static State_T * EnterMain(const MotorController_T * p_mc, state_value_t fromPark)
{
    // sign_t direction = _Motor_Table_GetDirectionAll(&p_mc->MOTORS); // Motor keeps direction in STOP or allowed to transition to PASSIVE
    switch (p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd)
    {
        case MOTOR_DIRECTION_NULL:       return &STATE_NEUTRAL;
        case MOTOR_DIRECTION_FORWARD:    return &STATE_DRIVE;
        case MOTOR_DIRECTION_REVERSE:    return &STATE_DRIVE;
        default: break;
    }
    // if (p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd != 0) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd); }  /* Buffered direction cmd from user input, independent of motor state */
    // else if (p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd == 0) { Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z); } /* If enter drive with no direction, handle discontinuity */
}

MotorController_App_T MC_APP_VEHICLE =
{
    .PROC_ANALOG_USER = MotorController_Vehicle_ProcAnalogUser,
    // .P_INITIAL_STATE = &STATE_NEUTRAL,
    .ENTER_MAIN = (State_Input_T)EnterMain,
};

/*
    setting MOTOR_DIRECTION_NULL motor maps STOP
    remap to Neutral State VZ/V0
*/
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
static void Drive_Entry(const MotorController_T * p_mc)
{
    assert(p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd != 0); /* Direction should have been checked on transition.*/
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd = VEHICLE_CMD_RELEASE; // next input is edge transition
    if (p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd != 0) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd); }  /* Buffered direction cmd from user input, independent of motor state */
}

/* alternatively call on input */
static void Drive_Proc(const MotorController_T * p_mc)
{
    // Vehicle_ProcInputCmd(&p_mc->VEHICLE);
    switch (p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd)
    {
        case VEHICLE_CMD_BRAKE:     Vehicle_ProcBrakeValue(&p_mc->VEHICLE);      break;
        case VEHICLE_CMD_THROTTLE:  Vehicle_ProcThrottleValue(&p_mc->VEHICLE);   break;
        case VEHICLE_CMD_RELEASE:   Vehicle_ProcDriveZero(&p_mc->VEHICLE);       break;
        default: break;
    }
}

static State_T * Drive_InputDirection(const MotorController_T * p_mc, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch(direction)
    {
        case MOTOR_DIRECTION_NULL:       p_nextState = &STATE_NEUTRAL;   break;
        case MOTOR_DIRECTION_FORWARD:    p_nextState = NULL;             break;
        case MOTOR_DIRECTION_REVERSE:    p_nextState = NULL;             break;
        default: break;
    }
    return p_nextState;
}

static State_T * Drive_InputCmdStart(const MotorController_T * p_mc, state_value_t mode)
{
    Vehicle_StartCmdMode(&p_mc->VEHICLE, (Vehicle_Cmd_T)mode);
    return NULL;
}

// State_Input_T Drive_TransitionMapper(state_input_t inputId)
// {
//     State_Input_T inputHandler = NULL;
//     switch (inputId)
//     {
//         case VEHICLE_STATE_INPUT_DIRECTION: inputHandler = (State_Input_T)Drive_InputDirection; break;
//         case VEHICLE_STATE_INPUT_DRIVE_CMD: inputHandler = (State_Input_T)Drive_InputCmdStart; break;
//         default: break;
//     }
//     return inputHandler;
// }

/* App-specific commands — reads from Vehicle.Input (buffered by MotorController_Vehicle_ApplyDirection etc.) */
static State_T * Drive_InputAppUser(const MotorController_T * p_mc, state_value_t appCmd)
{
    switch (appCmd)
    {
        case VEHICLE_STATE_INPUT_DIRECTION: return Drive_InputDirection(p_mc, p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd);
        case VEHICLE_STATE_INPUT_DRIVE_CMD: return Drive_InputCmdStart(p_mc, p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd);
        default: return NULL;
    }
}

static const State_Input_T DRIVE_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_APP_USER] = (State_Input_T)Drive_InputAppUser,
    [MCSM_INPUT_STATE_CMD] = NULL, /* propagate up */
};

static const State_T STATE_DRIVE =
{
    .ID         = VEHICLE_STATE_ID_DRIVE,
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Drive_Entry,
    .LOOP       = (State_Action_T)Drive_Proc,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
    // .TRANSITION_MAPPER = (State_InputMapper_T)Drive_TransitionMapper,
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
static void Neutral_Entry(const MotorController_T * p_mc)
{
    assert(p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd == 0); /* Direction should have been checked on transition. */
    if (p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd == 0) { Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z); }
    // if (p_mc->p_mc_STATE->Input.Cmd != VEHICLE_CMD_BRAKE) { Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z); }  /* If enter neutral while braking, handle discontinuity */
}

static void Neutral_Proc(const MotorController_T * p_mc)
{
    switch (p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd)
    {
        case VEHICLE_CMD_RELEASE:
            // assert (Motor_Table_IsEveryValue(&p_mc->MOTORS, Motor_IsState, MSM_STATE_ID_PASSIVE)); // check for consistency
            break;
        case VEHICLE_CMD_BRAKE: Vehicle_ApplyBrakeValue(&p_mc->VEHICLE, p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue); break;
        case VEHICLE_CMD_THROTTLE: break;
        default: break;
    }
}

/*
    Motor Keeps Direction State
*/
static State_T * InputDriveDirection(const MotorController_T * p_mc, state_value_t direction)
{
    if (Motor_Table_IsEveryUserDirection(&p_mc->MOTORS, direction) == true) { return &STATE_DRIVE; }
    if (Motor_Table_IsEvery(&p_mc->MOTORS, Motor_IsSpeedZero) == true) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, direction); return &STATE_DRIVE; }
    //checkSpeedsign instead
    return NULL;
}

static State_T * Neutral_InputDirection(const MotorController_T * p_mc, state_value_t direction)
{
    switch ((sign_t)direction)
    {
        case 1:     return InputDriveDirection(p_mc, direction);
        case -1:    return InputDriveDirection(p_mc, direction);
        case 0:     return &STATE_NEUTRAL;
        default: break;
    }
}


static State_T * Neutral_InputCmdStart(const MotorController_T * p_mc, state_value_t mode)
{
    switch ((Vehicle_Cmd_T)mode)
    {
        case VEHICLE_CMD_RELEASE:   Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z);      break;
        case VEHICLE_CMD_BRAKE:     Vehicle_StartBrakeMode(&p_mc->VEHICLE);                     break;
        case VEHICLE_CMD_THROTTLE:  break;
        default: break;
    }
    return NULL;
}


/* App-specific commands — reads from Vehicle.Input (buffered by MotorController_Vehicle_ApplyDirection etc.) */
static State_T * Neutral_InputAppUser(const MotorController_T * p_mc, state_value_t appCmd)
{
    switch (appCmd)
    {
        case VEHICLE_STATE_INPUT_DIRECTION: return Neutral_InputDirection(p_mc, p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd);
        case VEHICLE_STATE_INPUT_DRIVE_CMD: return Neutral_InputCmdStart(p_mc, p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd);
        default: return NULL;
    }
}

static const State_Input_T NEUTRAL_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_APP_USER] = (State_Input_T)Neutral_InputAppUser,
};

static const State_T STATE_NEUTRAL =
{
    .ID         = VEHICLE_STATE_ID_NEUTRAL,
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Neutral_Entry,
    .LOOP       = (State_Action_T)Neutral_Proc,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* also returns NEUTRAL on motors mismatch  */
/* Motors may keep direction state in Neutral */
/* Alternatively use substates */
sign_t MotorController_Vehicle_GetDirection(const MotorController_T * p_mc)
{
    if (StateMachine_GetLeafState(p_mc->STATE_MACHINE.P_ACTIVE) == &STATE_DRIVE)
    {
        return _Motor_Table_GetDirectionAll(&p_mc->MOTORS);
    }
    return 0;
}

/******************************************************************************/
/*!
    @brief StateMachine Input
*/
/******************************************************************************/
void _MotorController_Vehicle_ApplyDirection(MotorController_T * p_mc)
{
    _StateMachine_Branch_CallInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MCSM_INPUT_APP_USER, VEHICLE_STATE_INPUT_DIRECTION);
}

void _MotorController_Vehicle_ApplyStartCmd(MotorController_T * p_mc)
{
    _StateMachine_Branch_CallInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MCSM_INPUT_APP_USER, VEHICLE_STATE_INPUT_DRIVE_CMD);
}

// /* Buffer direction then dispatch via MCSM_INPUT_APP_USER. Park sinks, Vehicle sub-states handle. */
// void MotorController_Vehicle_ApplyDirection(MotorController_T * p_mc, sign_t direction)
// {
//     // p_mc->VEHICLE.P_VEHICLE_STATE->Input.DirectionCmd = direction;
//     _StateMachine_Branch_CallInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MCSM_INPUT_APP_USER, VEHICLE_STATE_INPUT_DIRECTION);
// }

// /* alternatively include propagate value */
// void MotorController_Vehicle_ApplyStartCmd(MotorController_T * p_mc, Vehicle_Cmd_T cmd)
// {
//     // _StateMachine_Branch_CallInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, VEHICLE_STATE_INPUT_DRIVE_CMD, cmd); // command and values pass by buffered
//     _StateMachine_Branch_CallInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MCSM_INPUT_APP_USER, VEHICLE_STATE_INPUT_DRIVE_CMD);
// }



// /*
//     PollStartCmd for edge detection
// */
// void MotorController_Vehicle_StartThrottle(MotorController_T * p_mc) { MotorController_Vehicle_ApplyStartCmd(p_mc, VEHICLE_CMD_THROTTLE); }
// void MotorController_Vehicle_StartBrake(MotorController_T * p_mc) { MotorController_Vehicle_ApplyStartCmd(p_mc, VEHICLE_CMD_BRAKE); }
// void MotorController_Vehicle_StartRelease(MotorController_T * p_mc) { MotorController_Vehicle_ApplyStartCmd(p_mc, VEHICLE_CMD_RELEASE); }




/******************************************************************************/
/*
    User layer
*/
/******************************************************************************/
/*
    capture user cmd
    edge detect
    send to state machine
*/
/*
    Unified CmdId detection, inclusive of edge detection.
    Command Polling
    Cmd "state" == input, changes on edge, handle outside of StateMachine
    if call sm with value, update value in handler.
    propagating value needs state constraint. even though edge detect does not
    require caller to clear brake.
*/
void MotorController_Vehicle_PollStartCmd(MotorController_T * p_mc)
{
    Vehicle_Input_T * p_input = &p_mc->VEHICLE.P_VEHICLE_STATE->Input;
    if (Vehicle_Input_PollCmdEdge(p_input)) { _MotorController_Vehicle_ApplyStartCmd(p_mc); }
}

/*
    Poll start at time of input
    on each input
*/
/*
    Input ~10-50ms
    Proc State/Buffer ~1ms
    Apply state changes immediately
    on protocol input per value response, 50ms. alternatively buffer and poll.
    alternatively call value update,

    module handle edge detection
*/
void MotorController_Vehicle_PollThrottle(MotorController_T * p_mc, uint16_t userCmd)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = userCmd;
    MotorController_Vehicle_PollStartCmd(p_mc); // alternatively call state apply
    // MotorController_Vehicle_ApplyThrottleValue(p_mc); // alternatively call state apply
}

void MotorController_Vehicle_PollBrake(MotorController_T * p_mc, uint16_t userCmd)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue = userCmd;
    MotorController_Vehicle_PollStartCmd(p_mc);
}

void MotorController_Vehicle_PollRelease(MotorController_T * p_mc)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = 0U;
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue = 0U;
    MotorController_Vehicle_PollStartCmd(p_mc);
}

void MotorController_Vehicle_PollThrottleBrake(MotorController_T * p_mc, uint16_t throttleCmd, uint16_t brakeCmd)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = throttleCmd;
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue = brakeCmd;
    MotorController_Vehicle_PollStartCmd(p_mc);
}

// caller handled edge detection
void MotorController_Vehicle_ApplyDirectionCmd(MotorController_T * p_mc, sign_t direction)
{
    Vehicle_Input_PollDirectionEdge(&p_mc->VEHICLE.P_VEHICLE_STATE->Input, direction);
    _MotorController_Vehicle_ApplyDirection(p_mc);
    // set  p_input->DirectionCmd for direction during park.
}

void MotorController_Vehicle_PollDirectionCmd(MotorController_T * p_mc, sign_t direction)
{
    if (Vehicle_Input_PollDirectionEdge(&p_mc->VEHICLE.P_VEHICLE_STATE->Input, direction))
    {
        _MotorController_Vehicle_ApplyDirection(p_mc);
    }
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Can change to interface mapping later
*/
void MotorController_Vehicle_ProcAnalogUser(const MotorController_T * p_mc)
{
    switch (MotAnalogUser_GetDirectionEdge(&p_mc->ANALOG_USER))
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_Vehicle_ApplyDirectionCmd(p_mc, 1);       break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_Vehicle_ApplyDirectionCmd(p_mc, -1);      break;
        case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_Vehicle_ApplyDirectionCmd(p_mc, 0);       break;
        default: break;
    }

    MotorController_Vehicle_PollThrottleBrake(p_mc, MotAnalogUser_GetThrottle(&p_mc->ANALOG_USER), MotAnalogUser_GetBrake(&p_mc->ANALOG_USER));
}



/******************************************************************************/
/*!
    Protocol inputs as known edge trigger
*/
/******************************************************************************/
void MotorController_Vehicle_VarId_Set(MotorController_T * p_mc, Vehicle_VarId_T id, int value)
{
    switch (id)
    {
        case VEHICLE_VAR_DIRECTION:   MotorController_Vehicle_ApplyDirectionCmd(p_mc, (sign_t)value);   break;
        case VEHICLE_VAR_THROTTLE:    MotorController_Vehicle_PollThrottle(p_mc, (uint16_t)value);      break;
        case VEHICLE_VAR_BRAKE:       MotorController_Vehicle_PollBrake(p_mc, (uint16_t)value);         break;
        case VEHICLE_VAR_THROTTLE_ONLY:    MotorController_Vehicle_PollThrottle(p_mc, (uint16_t)value);      break;
        case VEHICLE_VAR_BRAKE_ONLY:       MotorController_Vehicle_PollBrake(p_mc, (uint16_t)value);         break;
    }
}

int MotorController_Vehicle_VarId_Get(MotorController_T * p_mc, Vehicle_VarId_T id)
{
    int value = 0;
    switch (id)
    {
        case VEHICLE_VAR_DIRECTION:   value = MotorController_Vehicle_GetDirection(p_mc);     break;
        case VEHICLE_VAR_THROTTLE:    value = p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue;  break;
        case VEHICLE_VAR_BRAKE:       value = p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue;     break;
        case VEHICLE_VAR_THROTTLE_ONLY:    value = p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue;  break;
        case VEHICLE_VAR_BRAKE_ONLY:       value = p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue;     break;
    }
    return value;
}




// void Vehicle_ProcAnalogUser(const MotorController_T * p_mc)
// {
//     cmd = MotAnalogUser_PollCmd(&p_mc->ANALOG_USER);
//     switch (cmd)
//     {
//         // case MOT_ANALOG_USER_CMD_SET_BRAKE:                 MotorController_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_mc->ANALOG_USER));          break;
//         // case MOT_ANALOG_USER_CMD_SET_THROTTLE:              MotorController_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottle(&p_mc->ANALOG_USER));    break;
//         //                                                     // Vehicle_ApplyThrottleValue(&p_mc->Vehicle, MotAnalogUser_GetThrottle(&p_mc->ANALOG_USER));
//         // case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:         MotorController_SetCmdBrake(p_mc, 0U);                                                 break;
//         // case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:      MotorController_SetCmdThrottle(p_mc, 0U);                                              break;
//         // case MOT_ANALOG_USER_CMD_PROC_ZERO:                 MotorController_SetCmdDriveZero(p_mc);                                                 break;
//         // case MOT_ANALOG_USER_CMD_SET_NEUTRAL:               MotorController_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_NEUTRAL);                       break;
//         case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD:     //         // MotorController_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);    //         break;
//         case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE:    //         // MotorController_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);    //         break;
//         case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:  break;
//         default: break;
//     }
// }

