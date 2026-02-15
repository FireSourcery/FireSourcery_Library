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
    MCSM_INPUT_USER,            /* User Control vars or analog */
    // MCSM_INPUT_APP_USER,        /* specialized inputs */
    // MCSM_INPUT_DIRECTION,    /* or separate */
    // MCSM_INPUT_PHASE_VOUT,
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

/******************************************************************************/
/*
    Motor Controller State Variables
*/
/******************************************************************************/
static inline MotorController_StateId_T MotorController_GetStateId(const MotorController_State_T * p_data) { return StateMachine_GetActiveStateId(&p_data->StateMachine); }

/* Corresponds to known Top State */
static inline state_t _MotorController_GetSubStateId(const MotorController_State_T * p_data) { return _StateMachine_GetActiveSubStateId(&p_data->StateMachine); }
static inline State_BranchId_T MotorController_GetSubStateId(const MotorController_State_T * p_data) { return StateMachine_GetBranchId(&p_data->StateMachine); }

static inline MotorController_FaultFlags_T MotorController_GetFaultFlags(const MotorController_State_T * p_data) { return p_data->FaultFlags; }

/*
    General Direction
    App may overwrite.
    PARK => 0
    FORWARD => 1
    REVERSE => -1
    0 =>  0
    Caller handle
    0, PARK_STATE => PARK
    0, !PARK_STATE => 0
*/
// alternatively map getter to State
static int MotorController_GetDirection(MotorController_T * p_context)
{
    switch (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_ACTIVE))
    {
        case MCSM_STATE_ID_MAIN:       return _Motor_Table_GetDirectionAll(&p_context->MOTORS); /* None is error in this case */
        case MCSM_STATE_ID_PARK:       return 0;
        case MCSM_STATE_ID_LOCK:       return 0;
        case MCSM_STATE_ID_FAULT:      return 0;
        default:                       return 0;
    }
}

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Active Main  */
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
/*
    User Input Interface; into StateMachine process
    _StateMachine_ProcInput Same Thread as Proc, no guard used
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

static inline void MotorController_InputStateCommand(MotorController_T * p_context, MotorController_StateCmd_T cmd)
{
    _StateMachine_Branch_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_STATE_COMMAND, cmd);
}

static inline void MotorController_EnterPark(MotorController_T * p_context) { MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_PARK); }
static inline void MotorController_EnterMain(MotorController_T * p_context) { MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_START_MAIN); }
/* Transition to idle */
static inline void MotorController_EnterMainIdle(MotorController_T * p_context) { MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN); }

/******************************************************************************/
/*!
    Input Cmd Trigger
    Call handle value with struct
    handled within main state
    // handle common mapping. optionally inputs proc synchronous buffer?
    split from stateCmd for refined override handling
*/
/******************************************************************************/
typedef enum MotorController_UserEvent
{
    MOTOR_CONTROLLER_USER_CMD_SETPOINT, /* user calls are less frequent than motor module */
    MOTOR_CONTROLLER_USER_CMD_PHASE,
    MOTOR_CONTROLLER_USER_CMD_FEEDBACK,
    MOTOR_CONTROLLER_USER_CMD_DIRECTION,
    // MOTOR_CONTROLLER_USER_CMD_PARK,
}
MotorController_UserEvent_T;
// set to mapper

static inline void MotorController_ApplyUserCmd(MotorController_T * p_context, MotorController_UserEvent_T cmd)
{
    _StateMachine_Branch_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_USER, cmd);
}

/*
    StateMachine handle push to all or primary motor
    Proc on input, optionally proc with sync buffer
    Input 10ms-50ms, Proc 1ms
*/
static inline void MotorController_SetCmdValue(MotorController_T * p_context, int16_t userCmd) { p_context->P_MC_STATE->CmdInput.CmdValue = userCmd; MotorController_ApplyUserCmd(p_context, MOTOR_CONTROLLER_USER_CMD_SETPOINT); }
static inline void MotorController_SetDirection(MotorController_T * p_context, int direction) { p_context->P_MC_STATE->CmdInput.Direction = direction; MotorController_ApplyUserCmd(p_context, MOTOR_CONTROLLER_USER_CMD_DIRECTION); }
static inline void MotorController_SetControlState(MotorController_T * p_context, Phase_Output_T controlState) { p_context->P_MC_STATE->CmdInput.PhaseOutput = controlState; MotorController_ApplyUserCmd(p_context, MOTOR_CONTROLLER_USER_CMD_PHASE); }
static inline void MotorController_SetFeedbackMode(MotorController_T * p_context, Motor_FeedbackMode_T feedbackMode) { p_context->P_MC_STATE->CmdInput.FeedbackMode = feedbackMode; MotorController_ApplyUserCmd(p_context, MOTOR_CONTROLLER_USER_CMD_FEEDBACK); }



/* Combination Input */
// typedef union MotorController_StateInput2
// {
//     struct
//     {
//         uint16_t SubId : 16U;
//         uint16_t Value : 16U;
//     };
//     uint32_t Pair;
// }
// MotorController_StateInput2_T;

// static inline void MotorController_ApplyUserCmdValue(MotorController_T * p_context, uint16_t cmd, uint16_t value)
// {
//     _StateMachine_Branch_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_USER, (MotorController_StateInput2_T) { .SubId = cmd, .Value = value }.Pair);
// }

/*
    if a future app  also needs MCSM_INPUT_APP_USER, the sub-ID namespaces collide.
        Remote side syncs with state
        Protocol handler check state, or use vtable
        name space for state_value_t values
        additional state_input_t slots
*/
// static inline void MotorController_ApplyAppCmd(MotorController_T * p_context, uint16_t cmd, uint16_t value)
// {
//     _StateMachine_Branch_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_APP_USER, (MotorController_StateInput2_T) { .SubId = cmd, .Value = value }.Pair);
// }

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
    MOTOR_CONTROLLER_LOCK_MOTOR_TUNING_MODE,  /* Motors to Calibration->Tuning */
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
    _StateMachine_Branch_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_LOCK, id);
}

/* Lock State returns to ENTER */
/*!
    @retval 0xFF when TopState not in Lock State
    @retval IsComplete SubState => 0xFF, TopState => MC_STATE_LOCK
    @retval Processing SubState => id or 0, TopState => MC_STATE_LOCK
*/
/* may return 0xFF. most states passthrough. */
static inline MotorController_LockId_T MotorController_GetLockState(MotorController_T * p_context) { return StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

static inline void MotorController_EnterLockState(MotorController_T * p_context) { MotorController_InputLock(p_context, MOTOR_CONTROLLER_LOCK_ENTER); }
static inline void MotorController_ExitLockState(MotorController_T * p_context) { MotorController_InputLock(p_context, MOTOR_CONTROLLER_LOCK_EXIT); }

static inline bool MotorController_IsEnterLockError(MotorController_T * p_context, MotorController_LockId_T id)
{
    return (((MotorController_LockId_T)id != MOTOR_CONTROLLER_LOCK_EXIT) && (MotorController_IsLock(p_context) == false));
}

/* Save RAM to NVM */
static inline NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_context)
{
    MotorController_InputLock(p_context, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);
    return p_context->P_MC_STATE->NvmStatus;
}


// if all calibration function use substate
static inline bool MotorController_IsLockOpComplete(MotorController_T * p_context) { return StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

/* return union status */
// 0 as success
// if (MotorController_IsLockOpComplete(p_context) == true)
static inline int MotorController_GetLockOpStatus(MotorController_T * p_context) { return p_context->P_MC_STATE->LockOpStatus; }


/******************************************************************************/
/*!
*/
/******************************************************************************/
extern bool MotorController_ExitFault(MotorController_T * p_context);
extern void MotorController_EnterFault(MotorController_T * p_context);
extern void MotorController_SetFault(MotorController_T * p_context, uint16_t faultFlags);
extern void MotorController_ClearFault(MotorController_T * p_context, uint16_t faultFlags);
