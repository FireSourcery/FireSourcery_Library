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

/******************************************************************************/
/*
    State Machine Definitions
*/
/******************************************************************************/
#define MCSM_TRANSITION_TABLE_LENGTH (5U)

/*
    state_input_t
    Input Id
*/
/*
    static const State_Input_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
    {
        [MCSM_INPUT_FAULT]  = NULL,
    };
*/
typedef enum MotorController_State_Input
{
    MCSM_INPUT_FAULT,           /* System fault handling */
    MCSM_INPUT_LOCK,            /* Enter locked/calibration operations */
    MCSM_INPUT_STATE_COMMAND,   /* System state commands (Park/Stop/Start) with per State Mapping */
    MCSM_INPUT_DIRECTION,       /* Simplify including common Park State and AnalogUser  */
    MCSM_INPUT_GENERIC,         /* MotorCmd/Generic Input */ // handle common mapping. inputs proc synchronous buffer
}
MotorController_State_Input_T;

typedef enum MotorController_StateId
{
    MCSM_STATE_ID_INIT,
    MCSM_STATE_ID_PARK,     /* includes standby */
    MCSM_STATE_ID_MAIN,
    MCSM_STATE_ID_LOCK,     /* includes calibration */
    MCSM_STATE_ID_FAULT,    /* includes error handling */
    // MCSM_STATE_ID_EXTERNAL,
}
MotorController_StateId_T;

extern const State_T MC_STATE_MAIN;
extern const State_T MC_STATE_LOCK;

// temp
extern const State_T MC_STATE_MAIN_VEHICLE;
extern const State_T MC_STATE_MAIN_MOTOR_CMD;

/*
    extern for Init
*/
extern const StateMachine_Machine_T MCSM_MACHINE;

#define MOTOR_CONTROLLER_STATE_MACHINE_INIT(p_MotorControllerConst, MotorControllerActive) STATE_MACHINE_INIT(p_MotorControllerConst, &MCSM_MACHINE, &((MotorControllerActive).StateMachine))

/* Top State only */
static inline bool MotorController_IsState(MotorController_T * p_context, MotorController_StateId_T stateId) { return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, stateId); }

static inline bool MotorController_IsPark(MotorController_T * p_context)   { return MotorController_IsState(p_context, MCSM_STATE_ID_PARK); }
static inline bool MotorController_IsFault(MotorController_T * p_context)  { return MotorController_IsState(p_context, MCSM_STATE_ID_FAULT); }
static inline bool MotorController_IsLock(MotorController_T * p_context)   { return MotorController_IsState(p_context, MCSM_STATE_ID_LOCK); }

static inline bool MotorController_IsConfig(MotorController_T * p_context) { return (MotorController_IsLock(p_context) || MotorController_IsFault(p_context)); }

/* todo input mode dependent, and state dependent - properties or fn map */

// is exact substate, faster check without traversal
// static inline bool MotorController_IsExact(MotorController_T * p_context, State_T * p_state) { return StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, p_state); }

// check ancestor for deeper nesting
// static inline bool MotorController_IsActive(MotorController_T * p_context, State_T * p_state) { return StateMachine_IsActivePathState(p_context->STATE_MACHINE.P_ACTIVE, & ); }

/******************************************************************************/
/*!
    Input Values passed as [state_value_t]
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Input Cmd Values
    Multi State common commands
*/
/******************************************************************************/
typedef enum MotorController_StateCmd
{
    MOTOR_CONTROLLER_STATE_CMD_PARK,
    MOTOR_CONTROLLER_STATE_CMD_E_STOP,
    MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN, /* Enter Idle */
    MOTOR_CONTROLLER_STATE_CMD_START_MAIN, /* Enter Configured */
}
MotorController_StateCmd_T;

static inline void MotorController_InputStateCommand(MotorController_T * p_context, MotorController_StateCmd_T cmd)
{
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_STATE_COMMAND, cmd);
}

/******************************************************************************/
/*!
    Input Cmd Values
*/
/******************************************************************************/
typedef enum MotorController_MotorCmd
{
    MOTOR_CONTROLLER_MOTOR_CMD_SETPOINT,
    MOTOR_CONTROLLER_MOTOR_CMD_PHASE,
    MOTOR_CONTROLLER_MOTOR_CMD_FEEDBACK,
    MOTOR_CONTROLLER_MOTOR_CMD_DIRECTION,
}
MotorController_MotorCmd_T;

static inline void MotorController_InputMotorCmd(MotorController_T * p_context, MotorController_MotorCmd_T cmd)
{
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_GENERIC, cmd);
}

/******************************************************************************/
/*!
    Lock SubState/Function Id
    Safe state actions, maybe blocking
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
    MOTOR_CONTROLLER_LOCK_MOTOR_CMD_MODE, /* Exit to Motor Default Mode. from lock only */
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

/* Transition on Call */
/* May transition to substate */
static inline void MotorController_InputLock(MotorController_T * p_context, MotorController_LockId_T id)
{
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_LOCK, id);
}

static inline MotorController_LockId_T MotorController_GetLockSubState(MotorController_T * p_context)
{
    return StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
static inline MotorController_MainMode_T MotorController_GetMainSubState(MotorController_T * p_context)
{
    return StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN);
}

/* check by var set */
static inline bool MotorController_IsMotorCmd(MotorController_T * p_context)
{
    return (MotorController_GetMainSubState(p_context) == MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD);
}


/******************************************************************************/
/*!
*/
/******************************************************************************/
extern bool MotorController_ExitFault(MotorController_T * p_context);
extern void MotorController_EnterFault(MotorController_T * p_context);
extern void MotorController_SetFault(MotorController_T * p_context, uint16_t faultFlags);
extern void MotorController_ClearFault(MotorController_T * p_context, uint16_t faultFlags);
