/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotorController_StateMachine.h
    @author FireSourcery
    @brief  MotorController_StateMachine
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_STATE_MACHINE_H
#define MOTOR_CONTROLLER_STATE_MACHINE_H

#include "MotorController.h"
#include "Utility/StateMachine/StateMachine.h"

// #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
//     #define _MCSM_TRANSITION_TABLE_LENGTH_SERVO (1U)
// #else
//     #define _MCSM_TRANSITION_TABLE_LENGTH_SERVO (0U)
// #endif

// #define MCSM_TRANSITION_TABLE_LENGTH (6U + _MCSM_TRANSITION_TABLE_LENGTH_SERVO)
#define MCSM_TRANSITION_TABLE_LENGTH (7U)

typedef enum MotorController_StateMachine_Input
{
    MCSM_INPUT_FAULT,
    MCSM_INPUT_LOCK,
    MCSM_INPUT_DIRECTION,   /* Drive Direction */
    MCSM_INPUT_CMD,
    // MCSM_INPUT_DRIVE,    /* Drive Throttle/Brake */
    MCSM_INPUT_THROTTLE,
    MCSM_INPUT_BRAKE,
    // MCSM_INPUT_RELEASE,
    MCSM_INPUT_SERVO,
    // MCSM_INPUT_USER,
}
MotorController_StateMachine_Input_T;

typedef enum MotorController_StateMachine_StateId
{
    MCSM_STATE_ID_INIT,
    MCSM_STATE_ID_PARK,
    MCSM_STATE_ID_DRIVE,
    MCSM_STATE_ID_NEUTRAL,
    MCSM_STATE_ID_LOCK,
    MCSM_STATE_ID_FAULT,
    MCSM_STATE_ID_SERVO,
}
MotorController_StateMachine_StateId_T;

extern const StateMachine_Machine_T MCSM_MACHINE;

#define MOTOR_CONTROLLER_STATE_MACHINE_INIT(p_MotorController) STATE_MACHINE_INIT(&MCSM_MACHINE, p_MotorController, false)

extern bool MotorController_StateMachine_IsFault(const MotorControllerPtr_T p_mc);
extern bool MotorController_StateMachine_ClearFault(MotorControllerPtr_T p_mc);
extern void MotorController_StateMachine_SetFault(MotorControllerPtr_T p_mc);

#endif
