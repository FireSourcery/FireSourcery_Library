#pragma once

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
    @file   Vehicle.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../MotMotors/MotMotors.h"
#include "Motor/Motor/Motor_Include.h"

#include "Transducer/Blinky/Blinky.h"



/*
    [Vehicle_Input_T]
*/
/* Cmd SubState use edge detection - DriveState */
typedef enum Vehicle_Cmd
{
    VEHICLE_CMD_RELEASE,
    VEHICLE_CMD_THROTTLE,
    VEHICLE_CMD_BRAKE,
}
Vehicle_Cmd_T;

/* alternatively convert to common interface */
typedef struct Vehicle_Input
{
    // Vehicle_Direction_T Direction;
    uint16_t ThrottleValue;
    uint16_t BrakeValue;
    Vehicle_Cmd_T Cmd;
    Vehicle_Cmd_T CmdPrev;
}
Vehicle_Input_T;

static inline Vehicle_Cmd_T Vehicle_Input_EvaluateCmd(const Vehicle_Input_T * p_user)
{
    if      (p_user->BrakeValue > 0U)       { return VEHICLE_CMD_BRAKE; } // check throttle active error
    else if (p_user->ThrottleValue > 0U)    { return VEHICLE_CMD_THROTTLE; }
    else                                    { return VEHICLE_CMD_RELEASE; }
}

/*
    For cases where Module handles edge detect
*/
static inline Vehicle_Cmd_T Vehicle_Input_PollCmd(Vehicle_Input_T * p_user)
{
    Vehicle_Cmd_T cmd = Vehicle_Input_EvaluateCmd(p_user);
    p_user->CmdPrev = p_user->Cmd; /* Save for Async Edge Check */
    p_user->Cmd = cmd;
    return cmd;
}

static inline bool Vehicle_Input_PollCmdEdge(Vehicle_Input_T * p_user) { return (p_user->Cmd != Vehicle_Input_PollCmd(p_user)); }
static inline bool Vehicle_Input_IsCmdEdge(Vehicle_Input_T * p_user) { return (p_user->Cmd != p_user->CmdPrev); }


/*
    Config States
*/
typedef enum Vehicle_BrakeMode
{
    VEHICLE_BRAKE_MODE_PASSIVE,
    VEHICLE_BRAKE_MODE_TORQUE,
    VEHICLE_BRAKE_MODE_VOLTAGE,
}
Vehicle_BrakeMode_T;

typedef enum Vehicle_ThrottleMode
{
    VEHICLE_THROTTLE_MODE_SPEED,
    VEHICLE_THROTTLE_MODE_TORQUE,
    VEHICLE_THROTTLE_MODE_VOLTAGE,
}
Vehicle_ThrottleMode_T;

/* Release Input */
typedef enum Vehicle_ZeroMode
{
    VEHICLE_ZERO_MODE_FLOAT,       /* "Coast". MOSFETS non conducting. Same as Neutral. */
    VEHICLE_ZERO_MODE_REGEN,       /* Regen Brake */
    VEHICLE_ZERO_MODE_CRUISE,      /* Voltage following, Zero current/torque */
    VEHICLE_ZERO_MODE_ZERO,        /* Setpoint Zero. No cmd overwrite */
}
Vehicle_ZeroMode_T;

typedef struct Vehicle_Config
{
    Vehicle_ThrottleMode_T ThrottleMode;
    Vehicle_BrakeMode_T BrakeMode;
    Vehicle_ZeroMode_T ZeroMode;
}
Vehicle_Config_T;

typedef struct Vehicle_State
{
    Vehicle_Config_T Config;
    Vehicle_Input_T Input;
    // Vehicle_Input_T InputPrev;
    // Vehicle_Status_T Status;
    StateMachine_Active_T StateMachine;
}
Vehicle_State_T;

/*
    [Vehicle Context]
    StateMachine with Sync Input
*/
typedef const struct Vehicle
{
    Vehicle_State_T * P_VEHICLE_STATE;
    StateMachine_T STATE_MACHINE;
    MotMotors_T MOTORS;
    // const Blinky_T * P_BUZZER;
    // const MotAnalogUser_T * P_ANALOG_USER;
    const Vehicle_Config_T * const P_NVM_CONFIG;
    /*   VarInterface; for Vehicle_VarId_Set */
}
Vehicle_T;

// #define VEHICLE_INIT()


extern void Vehicle_Init(const Vehicle_T * p_vehicle);

extern void Vehicle_StartThrottleMode(const Vehicle_T * p_vehicle);
extern void Vehicle_SetThrottleValue(const Vehicle_T * p_vehicle, uint16_t value);
extern void Vehicle_StartBrakeMode(const Vehicle_T * p_vehicle);
extern void Vehicle_SetBrakeValue(const Vehicle_T * p_vehicle, uint16_t value);
extern void Vehicle_StartDriveZero(const Vehicle_T * p_vehicle);
extern void Vehicle_ProcDriveZero(const Vehicle_T * p_vehicle);


// alternatively as input conversion,

// interface for   data common
// static inline void Vehicle_Input_FromProtocol(Vehicle_T * vehicle, Motor_Input_T * p_user, id, value)

// static inline void Vehicle_Input_FromAnalogUser(Vehicle_T * vehicle, Motor_Input_T * p_user, MotAnalogUser_T * P_analog)
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
//         case MOTOR_USER_INPUT_ID_DIRECTION: Vehicle_StateMachine_ApplyInputDirection(&p_context->VEHICLE, (Motor_UserDirection_T)value); break;
//         case MOTOR_USER_INPUT_ID_THROTTLE: p_context->VEHICLE.P_VEHICLE_STATE->Input.ThrottleValue = (uint16_t)value;
//             break;
//         default:
//             break;
//     }

    // switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    // {
    //     case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  p_input->Direction = MOTOR_USER_DIRECTION_FORWARD; p_input->PhaseState = PHASE_OUTPUT_VPWM;  break;
    //     case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  p_input->Direction = MOTOR_USER_DIRECTION_REVERSE; p_input->PhaseState = PHASE_OUTPUT_VPWM;  break;
    //     case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  p_input->PhaseState = PHASE_OUTPUT_FLOAT;         break; // p_input->Direction = MOTOR_USER_DIRECTION_NONE;// or return to top main
    //     default: break;
    // }
    // if (MotAnalogUser_IsAnyBrakeOn(&p_context->ANALOG_USER) == true)
    // {
    //     p_input->CmdValue = 0U;
    //     p_input->PhaseState = PHASE_OUTPUT_FLOAT;
    // }
    // else
    // {
    //     p_input->CmdValue = MotAnalogUser_GetThrottle(&p_context->ANALOG_USER) / 2U;
    // }
// }

// MotUserInput_VTable_T MOT_ANALOG_USER_INPUT =
// {
//     Vehicle_VarId_Set
// };



// static inline void Vehicle_Input_FromAnalogUser(Vehicle_Input_T * p_user, MotAnalogUser_T * P_analog)
// {
//     p_user->ThrottleValue = MotAnalogUser_GetAInValue(P_analog, MOT_ANALOG_USER_AIN_THROTTLE);
//     p_user->BrakeValue = MotAnalogUser_GetAInValue(P_analog, MOT_ANALOG_USER_AIN_BRAKE);
// }

// void Vehicle_Input_ToMotorInput (const Vehicle_Input_T * p_user, Motor_Input_T * P_input )
// {
    //     switch (dir)
    //     {
    //         case MOTOR_CONTROLLER_DIRECTION_PARK:
    //             P_input->Direction = MOTOR_USER_DIRECTION_NONE;
    //             P_input->PhaseState = PHASE_OUTPUT_FLOAT;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_REVERSE:
    //             P_input->Direction = MOTOR_USER_DIRECTION_REVERSE;
    //             P_input->PhaseState = PHASE_OUTPUT_VPWM;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_FORWARD:
    //             P_input->Direction = MOTOR_USER_DIRECTION_FORWARD;
    //             P_input->PhaseState = PHASE_OUTPUT_VPWM;
    //             break;
    //         case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:
    //             P_input->PhaseState = PHASE_OUTPUT_FLOAT;
    //             break;
    //         default:
    //             break;
    //     }
//
// }

// static inline void Input_FromThrottle(Motor_Input_T * p_user, Vehicle_ThrottleMode_T mode, uint16_t throttle)
// {
//     p_user->CmdValue = (int32_t)throttle / 2; // [0:32767]

//     switch (mode)
//     {
//         case VEHICLE_THROTTLE_MODE_SPEED:   p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT;      break;
//         case VEHICLE_THROTTLE_MODE_TORQUE:  p_user->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT;            break;
//         default: break;
//     }
// }
