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
    @file   MotDrive.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../MotMotors/MotMotors.h"
#include "Motor/Motor/Motor_Include.h"

#include "Transducer/Blinky/Blinky.h"

/* include part */
// #include "MotDrive_StateMachine.h"

/*
*/
// typedef enum MotDrive_Status
// {
//     MOT_DRIVE_STATUS_OK,
//     MOT_DRIVE_STATUS_WARNING,
//     MOT_DRIVE_STATUS_FAULT,
// }
// MotDrive_Status_T;

// typedef enum MotDrive_Direction
// {
//     MOTOR_DIRECTION_REVERSE = -1,
//     MOTOR_DIRECTION_NONE = 0,
//     MOTOR_DIRECTION_FORWARD = 1,
// }
// MotDrive_Direction_T;

/* Drive SubState use edge detection - DriveState */
typedef enum MotDrive_Cmd
{
    MOT_DRIVE_CMD_RELEASE,
    MOT_DRIVE_CMD_THROTTLE,
    MOT_DRIVE_CMD_BRAKE,
}
MotDrive_Cmd_T;

/* altneratively convert to common interface */
typedef struct MotDrive_Input
{
    // MotDrive_Direction_T Direction;
    uint16_t ThrottleValue;
    uint16_t BrakeValue;
    MotDrive_Cmd_T Cmd;
    // MotDrive_Cmd_T CmdPrev;
}
MotDrive_Input_T;

/* Config States */
typedef enum MotDrive_BrakeMode
{
    MOT_DRIVE_BRAKE_MODE_PASSIVE,
    MOT_DRIVE_BRAKE_MODE_TORQUE,
    MOT_DRIVE_BRAKE_MODE_VOLTAGE,
}
MotDrive_BrakeMode_T;

typedef enum MotDrive_ThrottleMode
{
    MOT_DRIVE_THROTTLE_MODE_SPEED,
    MOT_DRIVE_THROTTLE_MODE_TORQUE,
    MOT_DRIVE_THROTTLE_MODE_VOLTAGE,
}
MotDrive_ThrottleMode_T;

/* Release Input */
typedef enum MotDrive_ZeroMode
{
    MOT_DRIVE_ZERO_MODE_FLOAT,       /* "Coast". MOSFETS non conducting. Same as Neutral. */
    MOT_DRIVE_ZERO_MODE_REGEN,       /* Regen Brake */
    MOT_DRIVE_ZERO_MODE_CRUISE,      /* Voltage following, Zero current/torque */
    MOT_DRIVE_ZERO_MODE_ZERO,        /* Setpoint Zero. No cmd overwrite */
}
MotDrive_ZeroMode_T;

typedef struct MotDrive_Config
{
    MotDrive_ThrottleMode_T ThrottleMode;
    MotDrive_BrakeMode_T BrakeMode;
    MotDrive_ZeroMode_T ZeroMode;
}
MotDrive_Config_T;

typedef struct MotDrive_State
{
    MotDrive_Config_T Config;
    MotDrive_Input_T Input;
    // MotDrive_Input_T InputPrev;
    // MotDrive_Status_T Status;
    StateMachine_Active_T StateMachine;
}
MotDrive_State_T;

/*
    [MotDrive Context]
    StateMachine with Sync Input
*/
typedef const struct MotDrive
{
    MotDrive_State_T * P_MOT_DRIVE_STATE;
    StateMachine_T STATE_MACHINE;
    MotMotors_T MOTORS;
    const Blinky_T * P_BUZZER;
    // const MotAnalogUser_T * P_ANALOG_USER;
    const MotDrive_Config_T * const P_NVM_CONFIG;
    /*   VarInterface; for MotDrive_VarId_Set */
}
MotDrive_T;

// #define MOT_DRIVE_STATE_MACHINE_INIT(p_MotDriveContext, p_MotDriveActive)
// #define MOT_DRIVE_STATE_MACHINE_INIT(p_MotDriveContext, p_MotDriveActive) STATE_MACHINE_INIT((p_MotDriveContext), &MOT_DRIVE_MACHINE, &((p_MotDriveActive)->StateMachine))
// #define MOT_DRIVE_INIT()

/*

*/
static inline MotDrive_Cmd_T MotDrive_Input_PollCmd(MotDrive_Input_T * p_user)
{
    MotDrive_Cmd_T cmd;

    /* Check Brake first */
    if (p_user->BrakeValue > 0U) { cmd = MOT_DRIVE_CMD_BRAKE; } // check throttle active error
    else if (p_user->ThrottleValue > 0U) { cmd = MOT_DRIVE_CMD_THROTTLE; }
    else { cmd = MOT_DRIVE_CMD_RELEASE; }

    // p_user->CmdPrev = p_user->Cmd;
    p_user->Cmd = cmd;

    return cmd;
}

static inline bool MotDrive_Input_PollCmdEdge(MotDrive_Input_T * p_user)
{
    return (p_user->Cmd != MotDrive_Input_PollCmd(p_user));
}


// alternatively as input conversion, remove

// MotorController_Direction_T MotorController_DirectionOf(Motor_User_Direction_T dir, Phase_Output_T phase)
// {
//     MotorController_Direction_T direction;
//     switch (dir)
//     {
//         case MOTOR_DIRECTION_NONE:      direction = MOTOR_CONTROLLER_DIRECTION_PARK;            break;
//         case MOTOR_DIRECTION_REVERSE:   direction = (phase == PHASE_OUTPUT_VPWM) ? MOTOR_CONTROLLER_DIRECTION_REVERSE : MOTOR_CONTROLLER_DIRECTION_ERROR; break;
//         case MOTOR_DIRECTION_FORWARD:   direction = (phase == PHASE_OUTPUT_VPWM) ? MOTOR_CONTROLLER_DIRECTION_FORWARD : MOTOR_CONTROLLER_DIRECTION_ERROR; break;
//         default:                        direction = MOTOR_CONTROLLER_DIRECTION_ERROR;           break;
//     }
//     return direction;
// }

// void MotDrive_Input_ToMotorInput (const MotDrive_Input_T * p_user, Motor_User_Input_T * P_input )
// {
//
// }

// void MotDrive_MotorInput_Of(Motor_User_Input_T * P_input, MotorController_Direction_T dir)
// {
//     switch (dir)
//     {
//         case MOTOR_CONTROLLER_DIRECTION_PARK:
//             P_input->Direction = MOTOR_DIRECTION_NONE;
//             P_input->PhaseState = PHASE_OUTPUT_FLOAT;
//             break;
//         case MOTOR_CONTROLLER_DIRECTION_REVERSE:
//             P_input->Direction = MOTOR_DIRECTION_REVERSE;
//             P_input->PhaseState = PHASE_OUTPUT_VPWM;
//             break;
//         case MOTOR_CONTROLLER_DIRECTION_FORWARD:
//             P_input->Direction = MOTOR_DIRECTION_FORWARD;
//             P_input->PhaseState = PHASE_OUTPUT_VPWM;
//             break;
//         case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:
//             P_input->PhaseState = PHASE_OUTPUT_FLOAT;
//             break;
//         default:
//             break;
//     }
// }

extern void MotDrive_Init(const MotDrive_T * p_handle);


// extern void MotDrive_StartThrottleMode(const MotDrive_T * p_motDrive);
// extern void MotDrive_SetThrottleValue(const MotDrive_T * p_motDrive, uint16_t userCmdThrottle);
// extern void MotDrive_StartBrakeMode(const MotDrive_T * p_motDrive);
// extern void MotDrive_SetBrakeValue(const MotDrive_T * p_motDrive, uint16_t userCmdBrake);
// extern void MotDrive_StartDriveZero(const MotDrive_T * p_motDrive);
// extern void MotDrive_ProcDriveZero(const MotDrive_T * p_motDrive);

