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
#include "Framework/StateMachine/StateMachine.h"
#include "Framework/StateMachine/_StateMachine_Tree.h"

#include "System/SysTime/SysTime.h"

#include <string.h>

/******************************************************************************/
/*
    State Machine Definitions
*/
/******************************************************************************/
/*
    state_input_t
    Input Id
*/
typedef enum MotorController_StateInput
{
    MC_STATE_INPUT_FAULT,           /* System fault handling */
    MC_STATE_INPUT_LOCK,            /* Enter locked/calibration operations */
    MC_STATE_INPUT_STATE_CMD,       /* System state commands (Park/Stop/Start) with per State Mapping */
    MC_STATE_INPUT_MOTOR_CMD,       /* User Control vars or analog */
    MC_STATE_INPUT_APP_USER,        /* specialized inputs. no top state handler, substates overload and call from within different handlers */

    /* App extends the input alphabet. keep as one list.  */
    // MC_STATE_INPUT_APP_USER_1,   /* Reserve table space, or use mapper, or buffer */
    // App wraps motor generic
    // MC_STATE_INPUT_SETPOINT,
    // MC_STATE_INPUT_PHASE,
    // MC_STATE_INPUT_FEEDBACK,
    // MC_STATE_INPUT_DIRECTION,

    MC_TRANSITION_TABLE_LENGTH,
}
MotorController_StateInput_T;

typedef enum MotorController_StateId
{
    MC_STATE_ID_INIT,
    MC_STATE_ID_PARK,         /* includes standby */
    MC_STATE_ID_MAIN,
    MC_STATE_ID_MOTOR_CMD,    /* alternatively as Substate under main, for motor control command handling. */
    MC_STATE_ID_LOCK,         /* includes calibration */
    MC_STATE_ID_FAULT,        /* includes error handling */
    _MC_STATE_ID_END,
}
MotorController_StateId_T;

extern const State_T MC_STATE_INIT;
extern const State_T MC_STATE_PARK;
extern const State_T MC_STATE_MAIN;
extern const State_T MC_STATE_MAIN_TUNING;
extern const State_T MC_STATE_MAIN_MOTOR_CMD;
extern const State_T MC_STATE_LOCK;
extern const State_T MC_STATE_FAULT;

/*
    extern for Init
*/
extern const StateMachine_Machine_T MCSM_MACHINE;

#define MOTOR_CONTROLLER_STATE_MACHINE_INIT(p_MotorControllerConst, MotorControllerActive) STATE_MACHINE_INIT(p_MotorControllerConst, &MCSM_MACHINE, &((MotorControllerActive).StateMachine))

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline MotorController_StateId_T MotorController_GetStateId(const MotorController_State_T * p_data) { return StateMachine_GetRootStateId(&p_data->StateMachine); }

/* Host side checks Root state to parse id */
/* handle with unique handler per type */
static inline state_t _MotorController_GetSubStateId(const MotorController_State_T * p_data) { return StateMachine_GetLeafStateId(&p_data->StateMachine); }
// static inline State_PathId_T MotorController_GetSubStateId(const MotorController_State_T * p_data) { return StateMachine_GetPathId(&p_data->StateMachine); }


static inline MotorController_FaultFlags_T MotorController_GetFaultFlags(const MotorController_State_T * p_data) { return p_data->FaultFlags; }

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Top State only */
// static inline bool MotorController_IsState(MotorController_T * p_dev, MotorController_StateId_T stateId) { return StateMachine_IsRootStateId(p_dev->STATE_MACHINE.P_ACTIVE, stateId); }

static inline bool MotorController_IsPark(MotorController_T * p_dev) { return StateMachine_IsRootState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_PARK); }
static inline bool MotorController_IsFault(MotorController_T * p_dev) { return StateMachine_IsRootState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_FAULT); }
static inline bool MotorController_IsLock(MotorController_T * p_dev) { return StateMachine_IsRootState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

static inline bool MotorController_IsConfig(MotorController_T * p_dev) { return (MotorController_IsLock(p_dev) || MotorController_IsFault(p_dev)); }

static inline bool MotorController_IsTuning(MotorController_T * p_dev) { return StateMachine_IsLeafState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN_TUNING); }

static inline bool MotorController_IsMotorCmdState(MotorController_T * p_dev) { return StateMachine_IsActiveBranch(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN_MOTOR_CMD); }

static inline bool MotorController_IsMotorCmdAccess(MotorController_T * p_dev) { return MotorController_IsMotorCmdState(p_dev) || MotorController_IsTuning(p_dev); }


/******************************************************************************/
/*
    User Input Interface; into StateMachine process
    _StateMachine_CallInput Same Thread as Proc, no guard used
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
    MOTOR_CONTROLLER_STATE_CMD_PARK, /* Enter Park */
    MOTOR_CONTROLLER_STATE_CMD_E_STOP,
    MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN, /* exit Main-App to Main-Idle */
    MOTOR_CONTROLLER_STATE_CMD_START_MAIN, /* exit park. Enter Configured */
}
MotorController_StateCmd_T;

static inline void MotorController_InputStateCommand(MotorController_T * p_dev, MotorController_StateCmd_T cmd)
{
    _StateMachine_Branch_CallInput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MC_STATE_INPUT_STATE_CMD, cmd);
}

static inline void MotorController_EnterPark(MotorController_T * p_dev) { MotorController_InputStateCommand(p_dev, MOTOR_CONTROLLER_STATE_CMD_PARK); }
static inline void MotorController_EnterMain(MotorController_T * p_dev) { MotorController_InputStateCommand(p_dev, MOTOR_CONTROLLER_STATE_CMD_START_MAIN); }
/* Transition to idle */
static inline void MotorController_EnterMainIdle(MotorController_T * p_dev) { MotorController_InputStateCommand(p_dev, MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN); }


/******************************************************************************/
/*
    Main / Operational States
*/
/******************************************************************************/
/* Active Main  */
/* file scope. app use subtype getter */
typedef enum MotorController_MainSubstateId
{
    MC_STATE_ID_MAIN_MOTOR_CMD,
    MC_STATE_ID_MAIN_TUNING,
    // MC_STATE_ID_MAIN_APP_START,
}
MotorController_MainSubstateId_T;

static inline MotorController_MainSubstateId_T MotorController_GetMainSubstateId(MotorController_T * p_dev)
{
    if (StateMachine_IsLeafState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN_TUNING)) { return MC_STATE_ID_MAIN_TUNING; }
    if (StateMachine_IsLeafState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN_MOTOR_CMD)) { return MC_STATE_ID_MAIN_MOTOR_CMD; }
    return (MotorController_MainSubstateId_T)STATE_ID_NULL; /* ignores app state under main */
    // return StateMachine_GetActiveSubStateId(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_MAIN);
}

/******************************************************************************/
/*!
    Motor Cmd Generalized Input
*/
/******************************************************************************/
typedef enum MotorController_MotorCmd
{
    MOTOR_CONTROLLER_USER_CMD_SETPOINT, /* user calls are less frequent than motor module */
    MOTOR_CONTROLLER_USER_CMD_PHASE,
    MOTOR_CONTROLLER_USER_CMD_FEEDBACK,
    MOTOR_CONTROLLER_USER_CMD_DIRECTION,
}
MotorController_MotorCmd_T;

static inline void MotorController_ApplyUserCmd(MotorController_T * p_dev, MotorController_MotorCmd_T cmd)
{
    _StateMachine_Branch_CallInput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MC_STATE_INPUT_MOTOR_CMD, cmd);
}

/*
    StateMachine handle push to all or primary motor
    Proc on input, optionally proc with sync buffer
    Input 10ms-50ms, Proc 1ms
*/
static inline void MotorController_SetCmdValue(MotorController_T * p_dev, int16_t userCmd) { p_dev->P_MC->CmdInput.CmdValue = userCmd; MotorController_ApplyUserCmd(p_dev, MOTOR_CONTROLLER_USER_CMD_SETPOINT); }
static inline void MotorController_SetDirection(MotorController_T * p_dev, int direction) { p_dev->P_MC->CmdInput.Direction = direction; MotorController_ApplyUserCmd(p_dev, MOTOR_CONTROLLER_USER_CMD_DIRECTION); }
static inline void MotorController_SetControlState(MotorController_T * p_dev, Phase_Output_T controlState) { p_dev->P_MC->CmdInput.PhaseOutput = controlState; MotorController_ApplyUserCmd(p_dev, MOTOR_CONTROLLER_USER_CMD_PHASE); }
static inline void MotorController_SetFeedbackMode(MotorController_T * p_dev, Motor_FeedbackMode_T feedbackMode) { p_dev->P_MC->CmdInput.FeedbackMode = feedbackMode; MotorController_ApplyUserCmd(p_dev, MOTOR_CONTROLLER_USER_CMD_FEEDBACK); }
/*
    General Direction
    App may overwrite.
*/
static int MotorController_GetDirection(MotorController_T * p_dev)
{
    return _Motor_Table_GetDirectionAll(&p_dev->MOTORS);
}


/* Combination Input */
typedef union MotorController_MotorCmdValue
{
    struct { uint16_t CmdValue; uint16_t CmdId; };
    uint32_t Value;
}
MotorController_MotorCmdValue_T;

static inline void MotorController_ApplyUserCmdValue(MotorController_T * p_dev, MotorController_MotorCmd_T cmd, int16_t value)
{
    MotorController_MotorCmdValue_T input = (MotorController_MotorCmdValue_T) { .CmdId = cmd, .CmdValue = value };
    _StateMachine_Branch_CallInput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MC_STATE_INPUT_MOTOR_CMD, input.Value);
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
    MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR, //resv
    MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
    MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG,
    MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG, /* on Error read from Nvm to RAM */
    MOTOR_CONTROLLER_LOCK_REBOOT,
    MOTOR_CONTROLLER_LOCK_MOTOR_CMD_MODE, /* Exit to Motor Cmd Mode. from lock only */
    MOTOR_CONTROLLER_LOCK_MOTOR_TUNING_MODE,  /*  */
    // MOTOR_CONTROLLER_LOCK_NVM_SAVE_BOOT,
    // MOTOR_CONTROLLER_LOCK_NVM_WRITE_ONCE,
    // MOTOR_CONTROLLER_LOCK_NVM_READ_ONCE,
}
MotorController_LockId_T;

/*!
    @retval 0xFF when TopState not in Lock State
    @retval IsComplete SubState => 0xFF, TopState => MC_STATE_LOCK
    @retval Processing SubState => id or 0, TopState => MC_STATE_LOCK
*/
/* specialized handlers per substateId. alternatively PathId Scheme. */
static inline MotorController_LockId_T MotorController_GetLockSubstateId(MotorController_T * p_dev)
{
    return (MotorController_LockId_T)StateMachine_GetActiveSubStateId(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK);
}

typedef enum MotorController_LockOpStatus
{
    MOTOR_CONTROLLER_LOCK_OP_STATUS_OK,
    // MOTOR_CONTROLLER_LOCK_OP_STATUS_PROCESSING, /* optionally check state instead */
    MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR,
    MOTOR_CONTROLLER_LOCK_OP_STATUS_TIMEOUT,
    MOTOR_CONTROLLER_LOCK_OP_STATUS_ENTER_ERROR, /* invalid enter, e.g. not in park, or motor running */
    MOTOR_CONTROLLER_LOCK_OP_STATUS_CMD_ERROR, /* invalid cmd, e.g. not support cmd or exit cmd when not in lock */
}
MotorController_LockOpStatus_T;

/* Transition on Call */
/* May transition to substate */
static inline MotorController_LockOpStatus_T MotorController_InputLock(MotorController_T * p_dev, MotorController_LockId_T id)
{
    _StateMachine_Branch_CallInput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev, MC_STATE_INPUT_LOCK, id);
    return p_dev->P_MC->LockOpStatus; /* optionally return status, or use getter after */
}

static inline MotorController_LockOpStatus_T MotorController_EnterLockState(MotorController_T * p_dev) { return MotorController_InputLock(p_dev, MOTOR_CONTROLLER_LOCK_ENTER); }
static inline MotorController_LockOpStatus_T MotorController_ExitLockState(MotorController_T * p_dev) { return MotorController_InputLock(p_dev, MOTOR_CONTROLLER_LOCK_EXIT); }

static inline bool MotorController_IsEnterLockError(MotorController_T * p_dev, MotorController_LockId_T id) { return ((id == MOTOR_CONTROLLER_LOCK_ENTER) && !MotorController_IsLock(p_dev)); }

/* Save RAM to NVM */
static inline NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_dev)
{
    MotorController_InputLock(p_dev, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);
    return p_dev->P_MC->NvmStatus;
}

// if all calibration function use substate
static inline bool MotorController_IsLockOpComplete(MotorController_T * p_dev)
{
    return StateMachine_IsLeafState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK) && Motor_Table_IsCalibrationComplete(&p_dev->MOTORS);
}

/* return union status */
// 0 as success
// if (MotorController_IsLockOpComplete(p_dev) == true)
static inline int MotorController_GetLockOpStatus(MotorController_T * p_dev) { return p_dev->P_MC->LockOpStatus; }


/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    FaultCmd - Encodes set and clear delta flags into a single state_value_t.
    Handlers apply both unconditionally: OR FaultSet, AND-NOT FaultClear.
    Eliminates read-modify-pass race on FaultFlags.Value.
*/
typedef union MotorController_FaultCmd
{
    struct { uint16_t FaultSet; uint16_t FaultClear; };
    uint32_t Value;
}
MotorController_FaultCmd_T;

// extern void MotorController_SetFault(MotorController_T * p_dev, MotorController_FaultFlags_T faultFlags);
// extern void MotorController_ClearFault(MotorController_T * p_dev, MotorController_FaultFlags_T faultFlags);
// extern bool MotorController_TryClearFaultAll(MotorController_T * p_dev);
static void MotorController_SetFault(MotorController_T * p_dev, MotorController_FaultFlags_T faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_dev->STATE_MACHINE, MC_STATE_INPUT_FAULT, (MotorController_FaultCmd_T) { .FaultSet = faultFlags.Value }.Value);
}

static void MotorController_ClearFault(MotorController_T * p_dev, MotorController_FaultFlags_T faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_dev->STATE_MACHINE, MC_STATE_INPUT_FAULT, (MotorController_FaultCmd_T) { .FaultClear = faultFlags.Value }.Value);
}

static bool MotorController_TryClearFaultAll(MotorController_T * p_dev)
{
    MotorController_ClearFault(p_dev, (MotorController_FaultFlags_T) { .Value = UINT16_MAX });
    return !MotorController_IsFault(p_dev);
}


