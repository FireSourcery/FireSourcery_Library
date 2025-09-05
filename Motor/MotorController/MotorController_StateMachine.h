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
#include "Utility/StateMachine/_StateMachine_Tree.h"

// #include "MotorController_Analog.h"
#include "System/SysTime/SysTime.h"

#include <string.h>

#define MCSM_TRANSITION_TABLE_LENGTH (4U)

typedef enum MotorController_State_Input
{
    MCSM_INPUT_FAULT,
    MCSM_INPUT_LOCK,
    MCSM_INPUT_MAIN_MODE, // temporary update main mode
    MCSM_INPUT_STATE_COMMAND, /* System Command with per State Mapping */
    // handle common mapping. inputs proc synchronous buffer
}
MotorController_State_Input_T;

/* PARK covers standby, LOCK covers calibration, FAULT covers errors */
typedef enum MotorController_StateId
{
    MCSM_STATE_ID_INIT,
    MCSM_STATE_ID_PARK,
    MCSM_STATE_ID_MAIN,
    MCSM_STATE_ID_LOCK,
    MCSM_STATE_ID_FAULT,
    // MCSM_STATE_ID_EXTERNAL,
}
MotorController_StateId_T;

extern const State_T MC_STATE_MAIN;
extern const State_T MC_STATE_MAIN_MOTOR_CMD;
extern const State_T MC_STATE_MAIN_MOT_DRIVE;
extern const State_T MC_STATE_LOCK;

/*
    extern for Init
*/
extern const StateMachine_Machine_T MCSM_MACHINE;
#define MOTOR_CONTROLLER_STATE_MACHINE_INIT(p_MotorControllerConst, MotorControllerActive) STATE_MACHINE_INIT(p_MotorControllerConst, &MCSM_MACHINE, &((MotorControllerActive).StateMachine))

/* Top State only */
static inline bool MotorController_StateMachine_IsState(MotorController_T * p_context, MotorController_StateId_T stateId) { return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, stateId); }

// // is exact substate, faster check without traversal
// static inline bool MotorController_StateMachine_IsLeafState(MotorController_T * p_context, State_T * p_state) { return StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, p_state); }

// // check ancestor for deeper nesting
// static inline bool MotorController_StateMachine_IsActive(MotorController_T * p_context) { return StateMachine_IsActivePathState(p_context->STATE_MACHINE.P_ACTIVE, & ); }

static inline bool MotorController_StateMachine_IsFault(MotorController_T * p_context) { return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, MCSM_STATE_ID_FAULT); }

static inline bool MotorController_StateMachine_IsLock(MotorController_T * p_context) { return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, MCSM_STATE_ID_LOCK); }

static inline bool MotorController_StateMachine_IsConfig(MotorController_T * p_context) { return (MotorController_StateMachine_IsLock(p_context) || MotorController_StateMachine_IsFault(p_context)); }

/******************************************************************************/
/*!

*/
/******************************************************************************/
typedef enum MotorController_StateCmd
{
    MOTOR_CONTROLLER_STATE_CMD_PARK,
    MOTOR_CONTROLLER_STATE_CMD_E_STOP,
    MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN, /* Enter Idle */
    MOTOR_CONTROLLER_STATE_CMD_START_MAIN, /* Enter Configured  */
    // MOTOR_CONTROLLER_MAIN_MODE_DRIVE,
}
MotorController_StateCmd_T;


static inline void MotorController_StateMachine_InputStateCommand(const MotorController_T * p_context, MotorController_StateCmd_T cmd)
{
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_STATE_COMMAND, cmd);
}

static inline void MotorController_StateMachine_EnterPark(const MotorController_T * p_context)
{
    // p_context->P_MC_STATE->CmdInput.Direction = MOTOR_DIRECTION_NONE;
    // // p_context->P_MC_STATE->CmdInput.PhaseState = PHASE_OUTPUT_V0;
    // MotorController_User_ApplyMotorsCmd(p_context);
    MotorController_StateMachine_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_PARK);
}

/* Transition to idle */
static inline void MotorController_User_EnterStop(const MotorController_T * p_context) { MotorController_StateMachine_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN); }
static inline void MotorController_User_EnterMain(const MotorController_T * p_context) { MotorController_StateMachine_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_START_MAIN); }


/******************************************************************************/
/*!
    Blocking SubState/Function Id
    LockCmd
*/
/******************************************************************************/
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
    // MOTOR_CONTROLLER_LOCK_OP_STATUS_PROCESSING,
    MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR,
    MOTOR_CONTROLLER_LOCK_OP_STATUS_TIMEOUT,
}
MotorController_LockOpStatus_T;

/******************************************************************************/
/*
    Lock/Blocking
    Safe state actions, maybe blocking
*/
/******************************************************************************/
/* Transition on Call */
static inline void MotorController_User_InputLock(const MotorController_T * p_context, MotorController_LockId_T id)
{
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_LOCK, id); /* May transition to substate */
}

// static inline bool MotorController_User_EnterLockState(const MotorController_T * p_context)
// {
//     MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_ENTER);
//     return MotorController_StateMachine_IsLock(p_context);
// }

// static inline bool MotorController_User_ExitLockState(const MotorController_T * p_context)
// {
//     MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_EXIT);
//     return !MotorController_StateMachine_IsLock(p_context);
// }

static inline bool MotorController_User_IsEnterLockError(const MotorController_T * p_context, MotorController_LockId_T id)
{
    return (((MotorController_LockId_T)id != MOTOR_CONTROLLER_LOCK_EXIT) && (MotorController_StateMachine_IsLock(p_context) == false));
}

/* Save RAM to NVM */
static inline NvMemory_Status_T MotorController_User_SaveConfig_Blocking(const MotorController_T * p_context)
{
    MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);
    return p_context->P_MC_STATE->NvmStatus;
}

/* Lock State returns to ENTER */
/* Alternatively Caller check Top state. then _StateMachine_GetActiveSubStateId (potential substate id collision) */
/* IsComplete SubState => 0xFF, TopState => MC_STATE_LOCK */
/* Processing SubState => id or 0, TopState => MC_STATE_LOCK */
static inline MotorController_LockId_T MotorController_User_GetLockState(const MotorController_T * p_context) { return StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

// if all calibration function use substate
static inline bool MotorController_User_IsLockOpComplete(const MotorController_T * p_context) { return StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

/* return union status */
// return nvm on nvm
static inline int MotorController_User_GetLockOpStatus(const MotorController_T * p_context)
{
    return p_context->P_MC_STATE->LockOpStatus;
}

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline MotorController_MainMode_T MotorController_User_GetMainStateId(const MotorController_State_T * p_mcState) { return StateMachine_GetActiveSubStateId(&p_mcState->StateMachine, &MC_STATE_MAIN); }


/******************************************************************************/
/*
*/
/******************************************************************************/
// MC_STATE_MAIN_MOTOR_CMD


/******************************************************************************/
/*!
*/
/******************************************************************************/
extern bool MotorController_StateMachine_ExitFault(const MotorController_T * p_context);
extern void MotorController_StateMachine_EnterFault(const MotorController_T * p_context);
extern void MotorController_StateMachine_SetFault(const MotorController_T * p_context, uint16_t faultFlags);
extern void MotorController_StateMachine_ClearFault(const MotorController_T * p_context, uint16_t faultFlags);

extern Motor_User_Direction_T MotorController_StateMachine_GetDirection(MotorController_T * p_context);
