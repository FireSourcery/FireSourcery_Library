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
    MCSM_INPUT_MAIN_MODE,
    // MCSM_INPUT_MOTORS_CMD,
}
MotorController_State_Input_T;

typedef enum MotorController_StateId
{
    MCSM_STATE_ID_INIT,
    MCSM_STATE_ID_MAIN,
    MCSM_STATE_ID_LOCK,
    MCSM_STATE_ID_FAULT,
    // MCSM_STATE_ID_EXTERNAL,
}
MotorController_StateId_T;

/*
    extern for Init
*/
extern const StateMachine_Machine_T MCSM_MACHINE;

extern const State_T MAIN_STATE_MOTOR_CMD;

#define MOTOR_CONTROLLER_STATE_MACHINE_INIT(p_MotorControllerConst, MotorControllerActive) STATE_MACHINE_INIT(p_MotorControllerConst, &MCSM_MACHINE, &((MotorControllerActive).StateMachine))

// is exact substate, faster check without traversal
static inline bool MotorController_StateMachine_IsLeafState(const MotorController_T * p_context, const State_T * p_state) { return StateMachine_IsActiveSubState(p_context->STATE_MACHINE.P_ACTIVE, p_state); }

static inline bool MotorController_StateMachine_IsMotorCmd(const MotorController_T * p_context) { return StateMachine_IsActiveSubState(p_context->STATE_MACHINE.P_ACTIVE, &MAIN_STATE_MOTOR_CMD); }

// static inline bool MotorController_StateMachine_IsState(const MotorController_T * p_context, ) { return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE,  ); }
static inline bool _MotorController_StateMachine_IsFault(const MotorController_State_T * p_active) { return (StateMachine_IsActiveStateId(&p_active->StateMachine, MCSM_STATE_ID_FAULT)); }
static inline bool MotorController_StateMachine_IsFault(const MotorController_T * p_context) { return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, MCSM_STATE_ID_FAULT); }

/******************************************************************************/
/*!
    Blocking SubState/Function Id
    LockCmd
*/
/******************************************************************************/
/* Lock_InputLockOp_Blocking  */
typedef enum MotorController_LockId
{
    MOTOR_CONTROLLER_LOCK_ENTER,
    MOTOR_CONTROLLER_LOCK_EXIT,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
    MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG,
    MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG, /* on Error read from Nvm to RAM */
    MOTOR_CONTROLLER_LOCK_REBOOT,
    // MOTOR_CONTROLLER_LOCK_NVM_SAVE_BOOT,
    // MOTOR_CONTROLLER_LOCK_NVM_WRITE_ONCE,
    // MOTOR_CONTROLLER_LOCK_NVM_READ_ONCE,
}
MotorController_LockId_T;

typedef enum MotorController_LockOpStatus
{
    MOTOR_CONTROLLER_LOCK_OP_STATUS_OK,
    MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR,
    MOTOR_CONTROLLER_LOCK_OP_STATUS_TIMEOUT,
}
MotorController_LockOpStatus_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
extern bool MotorController_StateMachine_ExitFault(const MotorController_T * p_context);
extern void MotorController_StateMachine_EnterFault(const MotorController_T * p_context);
extern void MotorController_StateMachine_SetFault(const MotorController_T * p_context, uint16_t faultFlags);
extern void MotorController_StateMachine_ClearFault(const MotorController_T * p_context, uint16_t faultFlags);
