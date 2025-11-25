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
    @file   Vehicle_App.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "App_Vehicle.h"

/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Can change to interface mapping later
*/
void Vehicle_ProcAnalogUser(const MotorController_T * p_context)
{
    const Vehicle_T * const p_vehicle = &p_context->VEHICLE;

    /* Sets motor controller stateMachine. passthrough to inner */
    /* Direction Procs immediately */
    /* Alternatively check for park state */
    switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_User_SetDirection(p_context, MOTOR_DIRECTION_FORWARD);   break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_User_SetDirection(p_context, MOTOR_DIRECTION_REVERSE);   break;
        case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_User_SetDirection(p_context, MOTOR_DIRECTION_NONE);      break;
        default: break;
    }

    /* Set Inner StateMachine Only */
    Vehicle_User_SetThrottle(p_context->VEHICLE.P_VEHICLE_STATE, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
    Vehicle_User_SetBrake(p_context->VEHICLE.P_VEHICLE_STATE, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));
    Vehicle_User_PollStartCmd(&p_context->VEHICLE);
}

MotorController_App_T MC_APP_VEHICLE =
{
    .PROC_ANALOG_USER = Vehicle_ProcAnalogUser,
    .P_INITIAL_STATE = &MC_STATE_MAIN_VEHICLE,
};

// interface for   data common
// static inline void Vehicle_Input_FromProtocol(Vehicle_T * vehicle, Motor_User_Input_T * p_user, id, value)

// static inline void Vehicle_Input_FromAnalogUser(Vehicle_T * vehicle, Motor_User_Input_T * p_user, MotAnalogUser_T * P_analog)
// {
//     p_user->CmdValue = (int32_t)MotAnalogUser_GetThrottle(P_analog) / 2; // [0:32767]

//     switch (vehicle->P_VEHICLE_STATE->Config.ThrottleMode)
//     {
//         case VEHICLE_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case VEHICLE_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }

// interface direct call
// static void UserInput(const MotorController_T * p_context, int id, int value)
// {
//     switch (id)
//     {
//         case MOTOR_USER_INPUT_ID_DIRECTION: Vehicle_StateMachine_ApplyInputDirection(&p_context->VEHICLE, (Motor_User_Direction_T)value); break;
//         case MOTOR_USER_INPUT_ID_THROTTLE: p_context->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = (uint16_t)value;
//             break;
//         default:
//             break;
//     }
// }

// MotUserInput_VTable_T MOT_ANALOG_USER_INPUT =
// {
//     Vehicle_VarId_Set
// };


/******************************************************************************/
/*!
    @brief [Vehicle] SubState - Map under MotorController main state
    call Vehicle StateMachine can use a different input table

    [Throttle] + [Brake] + [Neutral State]
*/
/******************************************************************************/
void Vehicle_Entry(const MotorController_T * p_context)
{
    StateMachine_Reset(&p_context->VEHICLE.STATE_MACHINE); /* Reset StateMachine */
    // optionally motors exit stop, in case of motor cmds, or set from ui
}

/* Proc Vehicle Buffer */
/* Proc Per ms */
void Vehicle_Proc(const MotorController_T * p_context)
{
    Vehicle_T * const p_vehicle = &p_context->VEHICLE;
    /* Proc the Sub-StateMachine - with different input table */
    _StateMachine_ProcState(p_vehicle->STATE_MACHINE.P_ACTIVE, (void *)p_vehicle);
}

// top state can map MotorController inputs, pass to inner state
static State_T * Vehicle_InputGeneric(const MotorController_T * p_context, state_value_t inputsPtr)
{
    State_T * p_nextState = NULL;
    // Motor_User_Input_T * p_inputs = (Motor_User_Input_T *)inputsPtr;

    // MotMotors_ApplyInputs(&p_context->MOTORS, p_input);
    // Vehicle_User_SetThrottle(p_context->VEHICLE.P_VEHICLE_STATE, p_inputs->ThrottleValue);
    // switch (p_input->Direction)
    // {
    //     case MOTOR_DIRECTION_NONE:    break; // optionally apply V0 or VFLOAT
    //     case MOTOR_DIRECTION_FORWARD: Vehicle_User_ApplyDirection(&p_context->VEHICLE, MOTOR_DIRECTION_FORWARD); break;
    //     case MOTOR_DIRECTION_REVERSE: Vehicle_User_ApplyDirection(&p_context->VEHICLE, MOTOR_DIRECTION_FORWARD); break;
    //     default:  break;
    // }

    return NULL;
}

static State_T * Vehicle_InputDirection(const MotorController_T * p_context, state_value_t direction)
{
    Vehicle_User_ApplyDirection(&p_context->VEHICLE, direction);
    return NULL;
}

/* Overriding parent table */
static const State_Input_T VEHICLE_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    // [MCSM_INPUT_FAULT]           = (State_Input_T)TransitionFault,
    // [MCSM_INPUT_STATE_COMMAND]   = (State_Input_T)Vehicle_InputStateCmd,
    [MCSM_INPUT_DIRECTION]          = (State_Input_T)Vehicle_InputDirection, /* override outer and pass to nested state machine */
    // [MCSM_INPUT_GENERIC]         = (State_Input_T)Vehicle_InputGeneric,
    // [MCSM_INPUT_LOCK]            = (State_Input_T)Vehicle_InputLock,
};

const State_T MC_STATE_MAIN_VEHICLE =
{
    .ID         = MOTOR_CONTROLLER_MAIN_MODE_VEHICLE,
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Vehicle_Entry,
    .LOOP       = (State_Action_T)Vehicle_Proc,
    .P_TRANSITION_TABLE = &VEHICLE_TRANSITION_TABLE[0U],
};


// void Vehicle_ProcAnalogUser(const MotorController_T * p_context)
// {
//     cmd = MotAnalogUser_PollCmd(&p_context->ANALOG_USER);
//     switch (cmd)
//     {
//         // case MOT_ANALOG_USER_CMD_SET_BRAKE:                 MotorController_User_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));          break;
//         // case MOT_ANALOG_USER_CMD_SET_THROTTLE:              MotorController_User_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));    break;
//         //                                                     // Vehicle_SetThrottleValue(&p_mc->Vehicle, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
//         // case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:         MotorController_User_SetCmdBrake(p_mc, 0U);                                                 break;
//         // case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:      MotorController_User_SetCmdThrottle(p_mc, 0U);                                              break;
//         // case MOT_ANALOG_USER_CMD_PROC_ZERO:                 MotorController_User_SetCmdDriveZero(p_mc);                                                 break;
//         // case MOT_ANALOG_USER_CMD_SET_NEUTRAL:               MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_NEUTRAL);                       break;
//         case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD:     //         // MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);    //         break;
//         case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE:    //         // MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);    //         break;
//         case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:  break;
//         default: break;
//     }
// }