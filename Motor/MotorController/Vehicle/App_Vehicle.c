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
#include "../MotorController_AppTable.h" /* needed for MotorController_MainMode_T */

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

    /* outer state machine handles park state */
    /* Sets MotorController_T StateMachine. passthrough to inner */
    /* Calls [InputDirection] */
    // switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    // {
    //     case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_SetDirection(p_context, MOTOR_DIRECTION_CCW);   break;
    //     case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_SetDirection(p_context, MOTOR_DIRECTION_CW);   break;
    //     case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_SetDirection(p_context, MOTOR_DIRECTION_NULL);      break;
    //     default: break;
    // }
    switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  Vehicle_User_ApplyDirection(&p_context->VEHICLE, 1);     break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  Vehicle_User_ApplyDirection(&p_context->VEHICLE, -1);      break;
        case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  Vehicle_User_ApplyDirection(&p_context->VEHICLE, 0);    break;
        default: break;
    }

    /* Set Inner StateMachine Only */
    Vehicle_User_SetThrottle(p_context->VEHICLE.P_VEHICLE_STATE, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
    Vehicle_User_SetBrake(p_context->VEHICLE.P_VEHICLE_STATE, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));
    Vehicle_User_PollStartCmd(&p_context->VEHICLE);
}



/******************************************************************************/
/*!
    @brief [Vehicle] HSM SubState - Map under MotorController main state
    contain Vehicle StateMachine can use a different input table

    [Throttle] + [Brake] + [Neutral State]

    override MotorController StateMachine Inputs and pass to nested state machine
*/
/******************************************************************************/
static void Entry(const MotorController_T * p_context)
{
    StateMachine_Reset(&p_context->VEHICLE.STATE_MACHINE); /* Reset StateMachine */
    // optionally motors exit stop, in case of motor cmds, or set from ui
}

/* Implementation as wrapping inner machine. specialized inputs */
/* Proc Per ms */
/* Proc the Sub-StateMachine - with different input table */
static void Proc(const MotorController_T * p_context)
{
    _StateMachine_ProcState(p_context->VEHICLE.STATE_MACHINE.P_ACTIVE, (void *)&p_context->VEHICLE);
}

// static State_T * InputDirection(const MotorController_T * p_context, state_value_t direction)
// {
//     Vehicle_User_ApplyDirection(&p_context->VEHICLE, direction);
//     return NULL;
// }

// override stop main, and park, clear values
// static State_T * InputStateCmd(const MotorController_T * p_context, state_value_t cmd)
// {
//     switch (cmd)
//     {
//         case MOTOR_CONTROLLER_STATE_CMD_PARK:           break;
//         case MOTOR_CONTROLLER_STATE_CMD_E_STOP:         Vehicle_User_SetZero(p_context->VEHICLE.P_VEHICLE_STATE); break;
//         case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN:      Vehicle_User_SetZero(p_context->VEHICLE.P_VEHICLE_STATE); break;
//         case MOTOR_CONTROLLER_STATE_CMD_START_MAIN:     break;
//         default:  break;
//     }
//     return NULL;
// }

// MotorController stateMachine handler pass to inner state
// transition during park state call outer direction set, or move Vehicle_VarId_Set,
static State_T * InputUser(const MotorController_T * p_context, state_value_t event)
{
    State_T * p_nextState = NULL;
    switch ((MotorController_UserEvent_T)event)
    {
        case MOTOR_CONTROLLER_USER_CMD_DIRECTION: Vehicle_User_ApplyDirection(&p_context->VEHICLE, (sign_t)p_context->P_MC_STATE->CmdInput.Direction); break;
        case MOTOR_CONTROLLER_USER_CMD_PHASE: break; // ignore floating and hold cmds, derive form throttle and brake
        case MOTOR_CONTROLLER_USER_CMD_SETPOINT: break; // optionally throttle by default. // Vehicle_User_SetThrottle(p_context->VEHICLE.P_VEHICLE_STATE, p_context->P_MC_STATE->CmdInput.CmdValue); break;
        case MOTOR_CONTROLLER_USER_CMD_FEEDBACK: break; // ignore
        default:  break;
    }
    return NULL;
}

/* Inherit other inputs */
static const State_Input_T TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_USER] = (State_Input_T)InputUser,
};

const State_T MC_STATE_MAIN_VEHICLE =
{
    .ID         = MOTOR_CONTROLLER_MAIN_MODE_VEHICLE,
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Entry,
    .LOOP       = (State_Action_T)Proc,
    .P_TRANSITION_TABLE = &TRANSITION_TABLE[0U],
};


/* Static app interface */
MotorController_App_T MC_APP_VEHICLE =
{
    .PROC_ANALOG_USER = Vehicle_ProcAnalogUser,
    .P_INITIAL_STATE = &MC_STATE_MAIN_VEHICLE,
};


// void Vehicle_ProcAnalogUser(const MotorController_T * p_context)
// {
//     cmd = MotAnalogUser_PollCmd(&p_context->ANALOG_USER);
//     switch (cmd)
//     {
//         // case MOT_ANALOG_USER_CMD_SET_BRAKE:                 MotorController_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));          break;
//         // case MOT_ANALOG_USER_CMD_SET_THROTTLE:              MotorController_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));    break;
//         //                                                     // Vehicle_SetThrottleValue(&p_mc->Vehicle, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
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