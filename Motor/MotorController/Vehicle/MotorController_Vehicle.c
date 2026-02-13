
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
    contain Vehicle StateMachine can use a different input table

    [Throttle] + [Brake] + [Neutral State]

    override MotorController StateMachine Inputs and pass to nested state machine
    alternatively wrapping inner state machine
        can only set buffers on input to maintain consistency with outer state
*/
/******************************************************************************/
/* Neutral with configurable brake */
static const State_T STATE_DRIVE;
static const State_T STATE_NEUTRAL;

MotorController_App_T MC_APP_VEHICLE =
{
    .PROC_ANALOG_USER = MotorController_Vehicle_ProcAnalogUser,
    .P_INITIAL_STATE = &MC_STATE_MAIN_VEHICLE,
};

/******************************************************************************/
/*!

*/
/******************************************************************************/
static void Entry(const MotorController_T * p_mc)
{
    // optionally motors exit stop, in case of motor cmds, or set from ui
    // if (!Motor_Table_IsEveryUserDirection(&p_mc->MOTORS, 0)) { Vehicle_User_ApplyDirection(&p_mc->VEHICLE, (sign_t)p_mc->P_MC_STATE->CmdInput.Direction); }
}

/* Implementation as wrapping inner machine. specialized inputs */
/* Proc Per ms */
/* Proc the Sub-StateMachine - with different input table */
static void Proc(const MotorController_T * p_mc)
{
    // _StateMachine_ProcState(p_mc->VEHICLE.STATE_MACHINE.P_ACTIVE, (void *)&p_mc->VEHICLE);
}

static State_T * Next(const MotorController_T * p_mc)
{

}

// static State_T * InputDirection(const MotorController_T * p_mc, state_value_t direction)
// {
//     Vehicle_User_ApplyDirection(&p_mc->VEHICLE, direction);
//     return NULL;
// }

// override stop main, and park, clear values
// static State_T * InputStateCmd(const MotorController_T * p_mc, state_value_t cmd)
// {
//     switch (cmd)
//     {
//         case MOTOR_CONTROLLER_STATE_CMD_PARK:           break;
//         case MOTOR_CONTROLLER_STATE_CMD_E_STOP:         Vehicle_User_SetZero(p_mc->VEHICLE.p_mc_STATE); break;
//         case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN:      Vehicle_User_SetZero(p_mc->VEHICLE.p_mc_STATE); break;
//         case MOTOR_CONTROLLER_STATE_CMD_START_MAIN:     break;
//         default:  break;
//     }
//     return NULL;
// }

// MotorController stateMachine handler pass to inner state
// transition during park state call outer direction set, or move Vehicle_VarId_Set,
// static State_T * InputUser(const MotorController_T * p_mc, state_value_t event)
// {
//     State_T * p_nextState = NULL;
//     switch ((MotorController_UserEvent_T)event)
//     {
//         case MOTOR_CONTROLLER_USER_CMD_DIRECTION: Vehicle_User_ApplyDirection(&p_mc->VEHICLE, (sign_t)p_mc->P_MC_STATE->CmdInput.Direction); break;
//         case MOTOR_CONTROLLER_USER_CMD_PHASE: break; // ignore floating and hold cmds, derive form throttle and brake
//         case MOTOR_CONTROLLER_USER_CMD_SETPOINT: break; // optionally throttle by default. // Vehicle_User_SetThrottle(p_mc->VEHICLE.p_mc_STATE, p_mc->P_MC_STATE->CmdInput.CmdValue); break;
//         case MOTOR_CONTROLLER_USER_CMD_FEEDBACK: break; // ignore
//         default:  break;
//     }
//     return NULL;
// }

/* Inherit other inputs */
static const State_Input_T TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    // [MCSM_INPUT_USER] = (State_Input_T)InputUser,
};

const State_T MC_STATE_MAIN_VEHICLE =
{
    .ID         = MOTOR_CONTROLLER_MAIN_MODE_VEHICLE, // alternatively replace main state
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Entry,
    .LOOP       = (State_Action_T)Proc,
    .P_TRANSITION_TABLE = &TRANSITION_TABLE[0U],
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
static void Drive_Entry(const MotorController_T * p_mc)
{
    assert(Motor_Table_IsEvery(&p_mc->MOTORS, Motor_IsDirectionForward) || Motor_Table_IsEvery(&p_mc->MOTORS, Motor_IsDirectionReverse));

    p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd = VEHICLE_CMD_RELEASE; // next input is edge transition
    // Motor_Table_SetCmdValue(&p_mc->MOTORS, 0);

    // if (p_mc->VEHICLE.P_VEHICLE_STATE->Input.Direction != _Motor_Table_GetDirectionAll(p_mc))
    // {

    // }
}

/* alternatively call on input */
static void Drive_Proc(const MotorController_T * p_mc)
{
    Vehicle_ProcInputCmd(&p_mc->VEHICLE); /* optionally call on input update including non edge */
}

static State_T * Drive_InputDirection(const MotorController_T * p_mc, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch((sign_t)direction)
    {
        case 0:     p_nextState = &STATE_NEUTRAL;   break;
        case 1:     p_nextState = NULL;             break;
        case -1:    p_nextState = NULL;             break;
        default: break;
    }

    return p_nextState;
}

static State_T * Drive_InputCmdStart(const MotorController_T * p_mc, state_value_t mode)
{
    Vehicle_StartCmdMode(&p_mc->VEHICLE, (Vehicle_Cmd_T)mode);
    return NULL;
}

// static State_T * Drive_Input(const MotorController_T * p_mc, state_value_t input)
// {
//     // static const State_Input_T DRIVE_TRANSITION_TABLE[] =
//     // {
//     //     [VEHICLE_STATE_INPUT_DIRECTION] = (State_Input_T)Drive_InputDirection,
//     //     [VEHICLE_STATE_INPUT_DRIVE_CMD] = (State_Input_T)Drive_InputCmdStart,
//     // };

//     // return DRIVE_TRANSITION_TABLE[values.SubId](p_mc, values.Value);
//     // MotorController_StateInput2_T values = { .Pair = (uint32_t)input };
//     switch (input)
//     {
//         case VEHICLE_STATE_INPUT_DIRECTION: return Drive_InputDirection(p_mc, p_mc->P_MC_STATE->CmdInput.Direction); //use out value as temp buffer for now
//         case VEHICLE_STATE_INPUT_DRIVE_CMD: return Drive_InputCmdStart(p_mc, p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd);
//         default: break;
//     }
//     return NULL;
// }

// static State_T * Drive_InputValueUpdate(const MotorController_T * p_mc, state_value_t input)
// {
//     switch (input)
//     {
//         case VEHICLE_STATE_INPUT_DIRECTION: return Drive_InputDirection(p_mc, p_mc->P_MC_STATE->CmdInput.Direction);
//         case VEHICLE_STATE_INPUT_DRIVE_CMD:
//             // PollStart Vehicle_StartCmdMode(&p_mc->VEHICLE, (Vehicle_Cmd_T)mode);
//             // Vehicle_ProcCmd
//         default: break;
//     }
//     return NULL;
// }

// static const State_Input_T DRIVE_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
// {
//     [MCSM_INPUT_APP_USER] = (State_Input_T)Drive_Input,
// };
State_Input_T Drive_TransitionMapper(MotorController_T * p_context, state_input_t inputId)
{
    State_Input_T p_inputHandler = NULL;
    switch (inputId)
    {
        case VEHICLE_STATE_INPUT_DIRECTION: p_inputHandler = (State_Input_T)Drive_InputDirection; break;
        case VEHICLE_STATE_INPUT_DRIVE_CMD: p_inputHandler = (State_Input_T)Drive_InputCmdStart; break;
        default: break;
    }
    return p_inputHandler;
}
static const State_T STATE_DRIVE =
{
    .ID         = VEHICLE_STATE_ID_DRIVE,
    .DEPTH      = 2U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN_VEHICLE,
    .ENTRY      = (State_Action_T)Drive_Entry,
    .LOOP       = (State_Action_T)Drive_Proc,
    // .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
    .TRANSITION_MAPPER = Drive_TransitionMapper,
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
    Motor_Table_ActivateVOutput(&p_mc->MOTORS, PHASE_OUTPUT_FLOAT);
    // Motor_Table_SetCmdValue(&p_mc->MOTORS, 0);
    // if (p_mc->p_mc_STATE->Input.Cmd != VEHICLE_CMD_BRAKE)
    //     { Motor_Table_ActivateVOutput(&p_mc->MOTORS, PHASE_OUTPUT_FLOAT); }  /* If enter neutral while braking, handle discontinuity */
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
    return NULL;
}

static State_T * Neutral_InputDirection(const MotorController_T * p_mc, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((sign_t)direction)
    {
        case 1:     p_nextState = InputDriveDirection(p_mc, direction); break;
        case -1:    p_nextState = InputDriveDirection(p_mc, direction); break;
        case 0:     p_nextState = &STATE_NEUTRAL; break;
        default: break;
    }
    return p_nextState;
}


static State_T * Neutral_InputCmdStart(const MotorController_T * p_mc, state_value_t mode)
{
    switch ((Vehicle_Cmd_T)mode)
    {
        case VEHICLE_CMD_RELEASE:   Motor_Table_ActivateVOutput(&p_mc->MOTORS, PHASE_OUTPUT_FLOAT); break;
        case VEHICLE_CMD_BRAKE:     Vehicle_StartBrakeMode(&p_mc->VEHICLE); break;
        case VEHICLE_CMD_THROTTLE:  break;
        default: break;
    }
    return NULL;
}

// static State_T * Neutral_Input(const MotorController_T * p_mc, state_value_t input)
// {
//     // static const State_Input_T NEUTRAL_TRANSITION_TABLE[] =
//     // {
//     //     [VEHICLE_STATE_INPUT_DIRECTION]   = (State_Input_T)Neutral_InputDirection,
//     //     [VEHICLE_STATE_INPUT_DRIVE_CMD]   = (State_Input_T)Neutral_InputCmdStart,
//     // };
//     // return NEUTRAL_TRANSITION_TABLE[cmd](p_mc,  );
//     // MotorController_StateInput2_T values = { .Pair = (uint32_t)input };
//     switch (input)
//     {
//         case VEHICLE_STATE_INPUT_DIRECTION: return Neutral_InputDirection(p_mc, p_mc->P_MC_STATE->CmdInput.Direction); //use out value as temp buffer for now
//         case VEHICLE_STATE_INPUT_DRIVE_CMD: return Neutral_InputCmdStart(p_mc, p_mc->VEHICLE.P_VEHICLE_STATE->Input.Cmd);
//         default: break;
//     }
//     return NULL;
// }

// static const State_Input_T NEUTRAL_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
// {
//     [MCSM_INPUT_APP_USER] = (State_Input_T)Neutral_Input,
// };

State_Input_T Neutral_TransitionMapper(MotorController_T * p_context, state_input_t inputId)
{
    State_Input_T p_inputHandler = NULL;
    switch (inputId)
    {
        case VEHICLE_STATE_INPUT_DIRECTION: p_inputHandler = (State_Input_T)Neutral_InputDirection; break;
        case VEHICLE_STATE_INPUT_DRIVE_CMD: p_inputHandler = (State_Input_T)Neutral_InputCmdStart; break;
        default: break;
    }
    return p_inputHandler;
}

static const State_T STATE_NEUTRAL =
{
    .ID         = VEHICLE_STATE_ID_NEUTRAL,
    .DEPTH      = 2U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN_VEHICLE,
    .ENTRY      = (State_Action_T)Neutral_Entry,
    .LOOP       = (State_Action_T)Neutral_Proc,
    // .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
    .TRANSITION_MAPPER = Neutral_TransitionMapper,
};


/******************************************************************************/
/*!
    @brief StateMachine Input
*/
/******************************************************************************/
/*

*/
/* Caller Handle Edge Detection */
void MotorController_Vehicle_ApplyDirection(MotorController_T * p_mc, sign_t direction)
{
    _StateMachine_Branch_ProcInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, VEHICLE_STATE_INPUT_DIRECTION, direction);
    // MotorController_ApplyAppCmd(p_mc, VEHICLE_STATE_INPUT_DIRECTION, direction);
    // _StateMachine_ProcInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MCSM_INPUT_APP_USER, VEHICLE_STATE_INPUT_DIRECTION);
}

void MotorController_Vehicle_ApplyStartCmd(MotorController_T * p_mc, Vehicle_Cmd_T cmd)
{
    _StateMachine_Branch_ProcInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, VEHICLE_STATE_INPUT_DRIVE_CMD, cmd); // command and values pass by buffered
    // MotorController_ApplyAppCmd(p_mc, VEHICLE_STATE_INPUT_DRIVE_CMD, cmd);
    // _StateMachine_ProcInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, VEHICLE_STATE_INPUT_DRIVE_CMD, cmd);
}

// alternatively
// void MotorController_Vehicle_ApplyCmdValue(MotorController_T * p_mc, Vehicle_Cmd_T cmd)
// {
//     // _StateMachine_Branch_ProcInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MCSM_INPUT_APP_USER, VEHICLE_STATE_INPUT_DRIVE_CMD_VALUE); // command and values pass by buffered
// }

/*
    Caller Handle Edge Detection. Use PollStartCmd for Input cmd detection
*/
void MotorController_Vehicle_StartThrottle(MotorController_T * p_mc) { MotorController_Vehicle_ApplyStartCmd(p_mc, VEHICLE_CMD_THROTTLE); }
void MotorController_Vehicle_StartBrake(MotorController_T * p_mc) { MotorController_Vehicle_ApplyStartCmd(p_mc, VEHICLE_CMD_BRAKE); }
void MotorController_Vehicle_StartRelease(MotorController_T * p_mc) { MotorController_Vehicle_ApplyStartCmd(p_mc, VEHICLE_CMD_RELEASE); }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* also returns NEUTRAL on motors mismatch  */
/* Alternatively use substates */
/* Motors may keep direction state in Neutral */
sign_t MotorController_Vehicle_GetDirection(const MotorController_T * p_mc)
{
    if (StateMachine_GetLeafState(p_mc->STATE_MACHINE.P_ACTIVE) == &STATE_DRIVE)
    {
        return _Motor_Table_GetDirectionAll(&p_mc->MOTORS);
    }
    return 0;
}


/******************************************************************************/
/*
    Vehicle User layer
*/
/******************************************************************************/
/*
    Command Polling
    Cmd "state" == input, changes on edge, handle outside of StateMachine
*/
void MotorController_Vehicle_PollStartCmd(MotorController_T * p_mc)
{
    Vehicle_Input_T * p_input = &p_mc->VEHICLE.P_VEHICLE_STATE->Input;
    if (Vehicle_Input_PollCmdEdge(p_input)) { MotorController_Vehicle_ApplyStartCmd(p_mc, p_input->Cmd); }
}

/*
    Poll start at time of input
*/
/*
    Input ~10-50ms
    Proc State/Buffer ~1ms
    Apply state changes immediately
    on protocol input per value response, 50ms. alternatively buffer and poll.
    alternatively call value update,
*/
void MotorController_Vehicle_ApplyThrottle(MotorController_T * p_mc, uint16_t userCmd)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = userCmd;
    MotorController_Vehicle_PollStartCmd(p_mc); // alternatively call state apply
}

void MotorController_Vehicle_ApplyBrake(MotorController_T * p_mc, uint16_t userCmd)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue = userCmd;
    MotorController_Vehicle_PollStartCmd(p_mc);
}

void MotorController_Vehicle_ApplyZero(MotorController_T * p_mc)
{
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = 0U;
    p_mc->VEHICLE.P_VEHICLE_STATE->Input.BrakeValue = 0U;
    MotorController_Vehicle_PollStartCmd(p_mc);
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
    /* outer state machine handles park state */
    /* Sets MotorController_T StateMachine. passthrough to inner */
    /* Calls [InputDirection] */
    // switch (MotAnalogUser_GetDirectionEdge(&p_mc->ANALOG_USER))
    // {
    //     case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_SetDirection(p_mc, MOTOR_DIRECTION_CCW);   break;
    //     case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_SetDirection(p_mc, MOTOR_DIRECTION_CW);   break;
    //     case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_SetDirection(p_mc, MOTOR_DIRECTION_NULL);      break;
    //     default: break;
    // }
    switch (MotAnalogUser_GetDirectionEdge(&p_mc->ANALOG_USER))
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_Vehicle_ApplyDirection(p_mc, 1);       break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_Vehicle_ApplyDirection(p_mc, -1);      break;
        case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_Vehicle_ApplyDirection(p_mc, 0);       break;
        default: break;
    }

    /* Set Inner StateMachine Only */
    Vehicle_User_SetThrottle(p_mc->VEHICLE.P_VEHICLE_STATE, MotAnalogUser_GetThrottle(&p_mc->ANALOG_USER));
    Vehicle_User_SetBrake(p_mc->VEHICLE.P_VEHICLE_STATE, MotAnalogUser_GetBrake(&p_mc->ANALOG_USER));
    MotorController_Vehicle_PollStartCmd(p_mc);
}



/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
void MotorController_Vehicle_VarId_Set(MotorController_T * p_mc, Vehicle_VarId_T id, int value)
{
    switch (id)
    {
        case VEHICLE_VAR_DIRECTION:   MotorController_Vehicle_ApplyDirection(p_mc, (sign_t)value);       break; // call outer passthrough in most cases
        case VEHICLE_VAR_THROTTLE:    MotorController_Vehicle_ApplyThrottle(p_mc, (uint16_t)value);      break;
        case VEHICLE_VAR_BRAKE:       MotorController_Vehicle_ApplyBrake(p_mc, (uint16_t)value);         break;
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

