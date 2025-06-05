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
    @file   MotorController_StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController.h"
#include "Utility/StateMachine/StateMachine.h"

// #include "MotorController_Analog.h"
#include "System/SysTime/SysTime.h"

#include <string.h>

#define MCSM_TRANSITION_TABLE_LENGTH (4U)

typedef enum MotorController_StateMachine_Input
{
    MCSM_INPUT_FAULT,
    MCSM_INPUT_LOCK,
    MCSM_INPUT_CMD,
    MCSM_INPUT_MAIN_MODE,
}
MotorController_State_Input_T;

typedef enum MotorController_StateId
{
    MCSM_STATE_ID_INIT,
    MCSM_STATE_ID_MAIN,
    MCSM_STATE_ID_CMD,
    MCSM_STATE_ID_LOCK,
    MCSM_STATE_ID_FAULT,
    // MCSM_STATE_ID_SERVO,
    // MCSM_STATE_ID_USER,
    // MCSM_STATE_ID_EXTERNAL,
}
MotorController_StateId_T;

/*
    Must be extern for Init
*/
extern const StateMachine_Machine_T MCSM_MACHINE;

#define MOTOR_CONTROLLER_STATE_MACHINE_INIT(p_MotorControllerConst, p_MotorControllerActive) STATE_MACHINE_INIT(p_MotorControllerConst, &MCSM_MACHINE, (&(p_MotorControllerActive)->StateMachine))

extern bool MotorController_StateMachine_IsFault(const MotorController_State_T * p_active);

extern bool MotorController_StateMachine_ExitFault(const MotorController_T * p_context);
extern void MotorController_StateMachine_EnterFault(const MotorController_T * p_context);
// extern void MotorController_StateMachine_SetFault(const MotorController_T * p_context, uint16_t faultFlags);
extern bool MotorController_StateMachine_ClearFault(const MotorController_T * p_context, uint16_t faultFlags);
