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
    @file   MotorController_User.h
    @author FireSourcery
    @brief  User Interface wrapper accessor functions (chip external inputs).
                Includes error checking

*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_USER_H
#define MOTOR_CONTROLLER_USER_H

#include "MotorController_StateMachine.h"
#include "../Version.h"


/*
    System Cmds

    Non Polling Cmds, write only, subroutine
    Pass 2+ arguments. Host does not retain var value state.
    optionally return a status

    Directly as VarTypeId, MotVarId_Type_SystemCommand
*/
typedef enum MotorController_User_SystemCmd
{
    MOT_USER_SYSTEM_BEEP, /* Beep N */
    MOT_USER_SYSTEM_BEEP_STOP,
    /* alterntively as general */
    MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL,  // Force Disable control Non StateMachine checked, also handled via MOT_PACKET_STOP_ALL
    MOT_USER_SYSTEM_CLEAR_FAULT,            // Fault State / Flags
    MOT_USER_SYSTEM_LOCK_STATE_INPUT,       // MotorController_LockId_T as input: Nvm, Calibration
    // MOT_USER_SYSTEM_LOCK_STATE_EXIT,
    MOT_USER_SYSTEM_LOCK_STATE_STATUS,  // MotorController_LockId_T as status
    MOT_USER_SYSTEM_LOCK_ASYNC_STATUS,  // Async operation status. optionally pass MotorController_LockId_T for selection
    MOT_USER_SYSTEM_MAIN_MODE_INPUT,
    MOT_USER_SYSTEM_RX_WATCHDOG,        // on/off
    // MOT_USER_SYSTEM_SERVO, // servo mode
    // MOT_USER_SYSTEM_BEEP_CMD, /* disable/enable/config */
//     // MOT_VAR_TYPE_COMMAND_METER, move to other services
//     // MOT_VAR_TYPE_COMMAND_RELAY,
//     MOT_VAR_TYPE_COMMAND_NVM,
//     MOT_VAR_TYPE_COMMAND_CALIBRATION,
    _MOT_USER_SYSTEM_CMD_END = 16U,
}
MotorController_User_SystemCmd_T;

// typedef enum MotorController_User_GenericStatus
// {
//     MOT_USER_CALL_STATUS_OK = 0,
//     MOT_USER_CALL_STATUS_ERROR = 1,
// }
// MotorController_User_GenericStatus_T;


/******************************************************************************/
/*
    User Input Interface; into StateMachine process

    [StateMachine_Sync] mode
        process last set input, once per ms
        SetSyncInput will overwrite previous input
        effectively limit to 1 input per ms/packet

    _StateMachine_ProcInput Same Thread as Proc
*/
/******************************************************************************/


/******************************************************************************/
/*
    Main Mode
*/
/******************************************************************************/
static inline void MotorController_User_InputMainMode(const MotorController_T * p_context, MotorController_MainMode_T value)
{
    // _StateMachine_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_MAIN_MODE, (state_input_value_t)value);
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_MAIN_MODE, (state_input_value_t)value);
}

/******************************************************************************/
/*
    passthrough Common
    Push to all or primary motor
*/
/******************************************************************************/
/*
    Apply in MCSM_STATE_ID_MOTORS State only
*/
// id for the input that has been set
// static inline void MotorController_User_ApplyMotorsCmd(const MotorController_T * p_context, Motor_State_Input_T inputId)
static inline void MotorController_User_ApplyMotorsCmd(const MotorController_T * p_context)
{
    p_context->P_ACTIVE->CmdInput.IsUpdated = true; /* sync Input 10ms-50ms, Proc 1ms */
    // _StateMachine_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_MOTORS_CMD, inputId);
}

// static inline void _MotorController_User_SetCmdValue(const MotorController_T * p_context, int16_t userCmd) { p_context->P_ACTIVE->CmdInput.CmdValue = userCmd; }
// static inline void _MotorController_User_SetFeedbackMode(const MotorController_T * p_context, Motor_FeedbackMode_T feedbackMode) { p_context->P_ACTIVE->CmdInput.FeedbackMode = feedbackMode; }
// static inline void _MotorController_User_SetDirection(const MotorController_T * p_context, sign_t direction) { p_context->P_ACTIVE->CmdInput.Direction = direction; }
// static inline void _MotorController_User_SetControlState(const MotorController_T * p_context, Phase_Output_T controlState) { p_context->P_ACTIVE->CmdInput.ControlState = controlState; }

/*
    Pass outer context in case implementation changes
    push to motor state machine on edge

    Validate by statemachine
*/
static inline void MotorController_User_SetCmdValue(const MotorController_T * p_context, int16_t userCmd) { p_context->P_ACTIVE->CmdInput.CmdValue = userCmd; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetDirection(const MotorController_T * p_context, sign_t direction) { p_context->P_ACTIVE->CmdInput.Direction = direction; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetControlState(const MotorController_T * p_context, Phase_Output_T controlState) { p_context->P_ACTIVE->CmdInput.ControlState = controlState; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetFeedbackMode(const MotorController_T * p_context, Motor_FeedbackMode_T feedbackMode) { p_context->P_ACTIVE->CmdInput.FeedbackMode = feedbackMode; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetFeedbackMode_Cast(const MotorController_T * p_context, int feedbackMode) { MotorController_User_SetFeedbackMode(p_context, Motor_FeedbackMode_Cast(feedbackMode)); }

// static inline void MotorController_User_Release(MotorController_T * p_context) { _StateMachine_ProcAsyncInput(&p_context->StateMachine, MCSM_INPUT_MODE, MOTOR_CONTROLLER_RELEASE); }
// static inline void MotorController_User_Hold(MotorController_T * p_context) { _StateMachine_ProcAsyncInput(&p_context->StateMachine, MCSM_INPUT_MODE, MOTOR_CONTROLLER_HOLD); }

/* Non StateMachine checked disable motors. Caller ensure non field weakening state */
static inline void MotorController_User_ForceDisableControl(const MotorController_T * p_context)
{
    MotMotors_ForceDisableControl(&p_context->MOTORS);
    p_context->P_ACTIVE->CmdInput.CmdValue = 0;
    p_context->P_ACTIVE->CmdInput.ControlState = PHASE_OUTPUT_FLOAT;
    p_context->P_ACTIVE->CmdInput.Direction = SIGN_ZERO;
    p_context->P_ACTIVE->CmdInput.IsUpdated = true;

    /* if drive mode */
    MotDrive_User_SetZero(p_context->MOT_DRIVE.P_MOT_DRIVE_STATE); // set drive to zero
    MotDrive_User_SetDirection(&p_context->MOT_DRIVE, MOT_DRIVE_DIRECTION_NEUTRAL); // set drive direction to neutral

}

/******************************************************************************/
/*
    Lock/Blocking
    Safe state actions, maybe blocking
*/
/******************************************************************************/
static inline void MotorController_User_InputLock(const MotorController_T * p_context, MotorController_LockId_T id)
{
    // _StateMachine_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_LOCK, id);
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_LOCK, id); /* May transition to substate */
}

static inline bool MotorController_User_IsLockState(const const MotorController_T * p_context)
{
    return StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, MCSM_STATE_ID_LOCK);
}

static inline bool MotorController_User_IsEnterLockError(const const MotorController_T * p_context, MotorController_LockId_T id)
{
    return (((MotorController_LockId_T)id != MOTOR_CONTROLLER_LOCK_EXIT) && (MotorController_User_IsLockState(p_context) == false));
}

static inline bool MotorController_User_EnterLockState(const MotorController_T * p_context)
{
    MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_ENTER);
    return MotorController_User_IsLockState(p_context);
}

static inline bool MotorController_User_ExitLockState(const MotorController_T * p_context)
{
    MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_EXIT);
    return !MotorController_User_IsLockState(p_context); /* Park or Servo */
}


/* return union status */
static inline int MotorController_User_GetLockOpStatus(const MotorController_T * p_context)
{
    return p_context->P_ACTIVE->LockOpStatus;
    // return nvm on nvm
}

/*
*/
static inline bool MotorController_User_IsConfigState(const MotorController_T * p_context)
{
    return (MotorController_User_IsLockState(p_context) || MotorController_StateMachine_IsFault(p_context));
}

/* Save RAM to NVM */
static inline NvMemory_Status_T MotorController_User_SaveConfig_Blocking(const MotorController_T * p_context)
{
    MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);
    return p_context->P_ACTIVE->NvmStatus;
}

/* Lock State returns to ENTER */
static inline MotorController_LockId_T MotorController_User_GetLockState(const MotorController_T * p_context)
{
    // return MotorController_User_IsLockState(p_context) ? p_context->LockSubState : MOTOR_CONTROLLER_LOCK_EXIT;
    // return p_context->P_ACTIVE->LockSubState;
    // return MotorController_User_IsLockState(p_context) ? _StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE) : MOTOR_CONTROLLER_LOCK_EXIT;

    // if (MotorController_User_IsLockState(p_context) && !StateMachine_IsActiveSubState(p_context->STATE_MACHINE.P_ACTIVE, &STATE_LOCK))
    // {
        //     return (MotorController_LockId_T)_StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE);
        // }
        // else
        // {
            //     return MOTOR_CONTROLLER_LOCK_EXIT;
            // }
}

static inline bool MotorController_User_IsLockOpComplete(const MotorController_T * p_context)
{
    // return (p_context->P_ACTIVE->LockSubState == MOTOR_CONTROLLER_LOCK_ENTER);
    // StateMachine_IsActiveSubState(p_context->STATE_MACHINE.P_ACTIVE, &STATE_LOCK);
    // return MotorController_User_IsLockState(p_context)
}
/******************************************************************************/
/*
    User Setting Speed/I Limit
*/
/******************************************************************************/
static inline bool MotorController_User_SetSpeedLimitAll(const MotorController_T * p_context, uint16_t limit_fract16)
{
    MotorController_SetSpeedLimitAll(p_context, MOT_SPEED_LIMIT_USER, limit_fract16);
}

static inline bool MotorController_User_ClearSpeedLimitAll(const MotorController_T * p_context)
{
    MotorController_ClearSpeedLimitAll(p_context, MOT_SPEED_LIMIT_USER);
}

static inline bool MotorController_User_SetILimitAll(const MotorController_T * p_context, uint16_t limit_fract16)
{
    MotorController_SetILimitAll(p_context, MOT_I_LIMIT_USER, limit_fract16);
}

static inline bool MotorController_User_ClearILimitAll(const MotorController_T * p_context)
{
    MotorController_ClearILimitAll(p_context, MOT_I_LIMIT_USER);
}

/* using user channel */
static inline void MotorController_User_SetOptSpeedLimitOnOff(const MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_User_SetSpeedLimitAll(p_context, p_context->P_ACTIVE->Config.OptSpeedLimit_Fract16); }
    else { MotorController_User_ClearSpeedLimitAll(p_context); }
}

static inline void MotorController_User_SetOptILimitOnOff(const MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_User_SetILimitAll(p_context, p_context->P_ACTIVE->Config.OptILimit_Fract16); }
    else { MotorController_User_ClearILimitAll(p_context); }
}

/******************************************************************************/
/*
    Non StateMachine Checked Direct Inputs
*/
/******************************************************************************/
/* UserMain, which may use Watchdog  */
static inline Socket_T * MotorController_User_GetMainSocket(const MotorController_T * p_context)
{
    assert(p_context->USER_PROTOCOL_INDEX < p_context->PROTOCOL_COUNT);
    return MotorController_GetMainSocket(p_context);
}

static inline void MotorController_User_EnableRxWatchdog(const MotorController_T * p_context) { _Socket_EnableRxWatchdog(MotorController_User_GetMainSocket(p_context)->P_SOCKET_STATE); }
static inline void MotorController_User_DisableRxWatchdog(const MotorController_T * p_context) { _Socket_DisableRxWatchdog(MotorController_User_GetMainSocket(p_context)->P_SOCKET_STATE); }
static inline void MotorController_User_SetRxWatchdog(const MotorController_T * p_context, bool isEnable) { _Socket_SetRxWatchdogOnOff(MotorController_User_GetMainSocket(p_context)->P_SOCKET_STATE, isEnable); }


/******************************************************************************/
/*
    Motor Controller State Variables
*/
/******************************************************************************/
static inline MotorController_StateId_T MotorController_User_GetStateId(const MotorController_State_T * p_mcState) { return StateMachine_GetActiveStateId(&p_mcState->StateMachine); }
static inline MotorController_FaultFlags_T MotorController_User_GetFaultFlags(const MotorController_State_T * p_mcState) { return p_mcState->FaultFlags; }

/*
    Status Flags for User Interface

    Combined boolean outputs for protocol convenience
*/
typedef union MotorController_User_StatusFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U; // ILimit by Heat
        uint16_t VSourceLow         : 1U; // ILimit by VSourceLow
        // uint16_t SpeedLimit         : 1U;
        // uint16_t ILimit             : 1U;
        // uint16_t BuzzerEnable       : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
    };
    uint16_t Value;
}
MotorController_User_StatusFlags_T;

static inline MotorController_User_StatusFlags_T MotorController_User_GetStatusFlags(const MotorController_T * p_context)
{
    return (MotorController_User_StatusFlags_T)
    {
        // .HeatWarning    = Monitor_GetStatus(p_context->HEAT_PCB.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT ||
        //                   Monitor_GetStatus(p_context->HEAT_MOSFETS.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT,
        // .HeatWarning    = p_context->StateFlags.HeatWarning,
        // .VSourceLow     = p_context->StateFlags.VSourceLow,
        // .BuzzerEnable   = p_context->StateFlags.BuzzerEnable,
    };
}


/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
/*
    Boot Buffer
*/
static inline BootRef_T MotorController_User_GetBootReg(const MotorController_State_T * p_mcState)          { return p_mcState->BootRef; }
static inline void MotorController_User_SetBootReg(MotorController_State_T * p_mcState, BootRef_T bootReg)  { p_mcState->BootRef.Word = bootReg.Word; }
static inline void MotorController_User_SetFastBoot(MotorController_State_T * p_mcState, bool isEnable)     { p_mcState->BootRef.FastBoot = isEnable; }
static inline void MotorController_User_SetBeep(MotorController_State_T * p_mcState, bool isEnable)         { p_mcState->BootRef.Beep  = isEnable; }
static inline void MotorController_User_SetBlink(MotorController_State_T * p_mcState, bool isEnable)        { p_mcState->BootRef.Blink = isEnable; }

/*
    Controller NvM Variables Config
*/
// static inline uint16_t MotorController_User_GetVSupplyRef(const MotorController_State_T * p_mcState)                           { return p_mcState->Config.VSupplyRef; }

// static inline MotorController_InputMode_T MotorController_User_GetInputMode(const MotorController_State_T * p_mcState)         { return p_mcState->Config.InputMode; }

// static inline MotorController_MainMode_T MotorController_User_GetInitMode(const MotorController_State_T * p_mcState)           { return p_mcState->Config.InitMode; }
// static inline void MotorController_User_SetInitMode(MotorController_State_T * p_mcState, MotorController_MainMode_T mode)      { p_mcState->Config.InitMode = mode; }

// static inline void MotorController_User_SetILimitOnLowV(MotorController_State_T * p_mcState, uint16_t i_Fract16)              { p_mcState->Config.VLowILimit_Fract16 = i_Fract16; }

// static inline void MotorController_User_SetOptDinMode(MotorController_State_T * p_mcState, MotorController_OptDinMode_T mode) { p_mcState->Config.OptDinMode = mode; }
// static inline void MotorController_User_DisableOptDin(MotorController_State_T * p_mcState)                                    { p_mcState->Config.OptDinMode = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }

// static inline void MotorController_User_SetOptDinSpeedLimit(MotorController_State_T * p_mcState, uint16_t i_Fract16)          { p_mcState->Config.OptSpeedLimit_Fract16 = i_Fract16; }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/

extern void MotorController_User_SetVSupplyRef(const MotorController_T * p_context, uint16_t volts);
extern void MotorController_User_SetInputMode(const MotorController_T * p_context, MotorController_InputMode_T mode);

extern int MotorController_User_Call(const MotorController_T * p_context, MotorController_User_SystemCmd_T id, int value);

// extern NvMemory_Status_T MotorController_User_ReadManufacture_Blocking(const MotorController_T * p_context, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer);
// extern NvMemory_Status_T MotorController_User_WriteManufacture_Blocking(const MotorController_T * p_context, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size);
// extern void MotorController_User_SetOptSpeedLimitOnOff(const MotorController_T * p_context, bool isEnable);
// extern void MotorController_User_SetOptILimitOnOff(const MotorController_T * p_context, bool isEnable);
#endif

