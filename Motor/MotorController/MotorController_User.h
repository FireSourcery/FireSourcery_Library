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

    Optionally, Directly as VarType
*/
typedef enum MotorController_SystemCmd
{
    MOT_USER_SYSTEM_BEEP, /* Beep N */
    MOT_USER_SYSTEM_BEEP_STOP,
    /* alterntively as general */
    MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL,  // Force Disable control Non StateMachine checked, also handled via MOT_PACKET_STOP_ALL
    MOT_USER_SYSTEM_CLEAR_FAULT,            // Fault State / Flags
    MOT_USER_SYSTEM_RX_WATCHDOG,        // on/off
    MOT_USER_SYSTEM_LOCK_STATE_INPUT,       // MotorController_LockId_T as input: Nvm, Calibration
    // MOT_USER_SYSTEM_LOCK_STATE_EXIT,
    MOT_USER_SYSTEM_LOCK_STATE_STATUS,  // substate id, MotorController_LockId_T as status
    MOT_USER_SYSTEM_LOCK_ASYNC_STATUS,  // operation, Async operation status. optionally pass MotorController_LockId_T for selection
    MOT_USER_SYSTEM_MAIN_MODE_INPUT, // remove
    MOT_USER_SYSTEM_STATE_COMMAND,      // State Command
//     // MOT_VAR_TYPE_COMMAND_METER, move to other services
//     // MOT_VAR_TYPE_COMMAND_RELAY,
//     MOT_VAR_TYPE_COMMAND_NVM,
//     MOT_VAR_TYPE_COMMAND_CALIBRATION,
    _MOT_USER_SYSTEM_CMD_END = 16U,
}
MotorController_SystemCmd_T;

// typedef enum MotorController_GenericStatus
// {
//     MOT_USER_CALL_STATUS_OK = 0,
//     MOT_USER_CALL_STATUS_ERROR = 1,
// }
// MotorController_GenericStatus_T;

/******************************************************************************/
/*
    User Input Interface; into StateMachine process

    _StateMachine_ProcInput Same Thread as Proc
*/
/******************************************************************************/
/******************************************************************************/
/*

*/
/******************************************************************************/
/* Non StateMachine checked disable motors. Caller ensure non field weakening state */
/* Update State to prevent input overwrite */
static inline void MotorController_ForceDisableControl(MotorController_T * p_context)
{
    MotMotors_ForceDisableControl(&p_context->MOTORS);
    p_context->P_MC_STATE->CmdInput.CmdValue = 0;
    p_context->P_MC_STATE->CmdInput.PhaseState = PHASE_OUTPUT_FLOAT;
    p_context->P_MC_STATE->CmdInput.Direction = MOTOR_USER_DIRECTION_NONE;
    // p_context->P_MC_STATE->CmdInput.IsUpdated = true;

    MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_E_STOP);

    /* if drive mode */
    // Vehicle_User_SetZero(p_context->VEHICLE.P_VEHICLE_STATE); // set drive to zero
    // Vehicle_User_ApplyDirection(&p_context->VEHICLE, MOTOR_CONTROLLER_DIRECTION_NEUTRAL); // set drive direction to neutral
}

/******************************************************************************/
/*
    Common Park
*/
/******************************************************************************/
// p_context->P_MC_STATE->CmdInput.Direction = MOTOR_USER_DIRECTION_NONE;
// p_context->P_MC_STATE->CmdInput.PhaseState = PHASE_OUTPUT_V0;
// MotorController_ApplyMotorsCmd(p_context);
static inline void MotorController_EnterPark(MotorController_T * p_context) { MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_PARK); }

/******************************************************************************/
/*
    Main / Exit Park
*/
/******************************************************************************/
/* Transition to idle */
static inline void MotorController_EnterMainIdle(MotorController_T * p_context) { MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN); }
static inline void MotorController_EnterMain(MotorController_T * p_context) { MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_START_MAIN); }

/******************************************************************************/
/*
    Direction Handle Separately
*/
/******************************************************************************/
/* Simplify AnalogUser implementation */
static inline void MotorController_ApplyDirectionCmd(MotorController_T * p_context, Motor_UserDirection_T direction)
{
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_DIRECTION, direction);
}

/*
    General Direction
*/
/*
    MOTOR_CONTROLLER_DIRECTION_PARK => MOTOR_USER_DIRECTION_NONE
    MOTOR_CONTROLLER_DIRECTION_FORWARD => MOTOR_USER_DIRECTION_FORWARD
    MOTOR_CONTROLLER_DIRECTION_REVERSE => MOTOR_USER_DIRECTION_REVERSE
    MOTOR_CONTROLLER_DIRECTION_NEUTRAL => ERROR
    MOTOR_CONTROLLER_DIRECTION_ERROR => MOTOR_DIRECTION_ERROR
*/
// alternatively map getter to State
static Motor_UserDirection_T MotorController_GetDirection(MotorController_T * p_context)
{
    switch (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_ACTIVE))
    {
        case MCSM_STATE_ID_MAIN:       return _MotMotors_GetDirectionAll(&p_context->MOTORS); /* None is error in this case */
        case MCSM_STATE_ID_PARK:       return MOTOR_USER_DIRECTION_NONE;
        case MCSM_STATE_ID_LOCK:       return MOTOR_USER_DIRECTION_NONE;
        case MCSM_STATE_ID_FAULT:      return MOTOR_USER_DIRECTION_NONE;
        default:                       return MOTOR_USER_DIRECTION_NONE;
    }
}


/******************************************************************************/
/*
    Apply in [MC_STATE_MAIN_MOTOR_CMD] State
    StateMachine handle push to all or primary motor
    Input 10ms-50ms, Proc 1ms
    Proc on input
*/
/******************************************************************************/
static inline void MotorController_SetCmdValue(MotorController_T * p_context, int16_t userCmd) { p_context->P_MC_STATE->CmdInput.CmdValue = userCmd; MotorController_InputMotorCmd(p_context, MOTOR_CONTROLLER_MOTOR_CMD_SETPOINT); }
static inline void MotorController_SetDirection(MotorController_T * p_context, Motor_UserDirection_T direction) { p_context->P_MC_STATE->CmdInput.Direction = direction; MotorController_InputMotorCmd(p_context, MOTOR_CONTROLLER_MOTOR_CMD_DIRECTION); }
static inline void MotorController_SetControlState(MotorController_T * p_context, Phase_Output_T controlState) { p_context->P_MC_STATE->CmdInput.PhaseState = controlState; MotorController_InputMotorCmd(p_context, MOTOR_CONTROLLER_MOTOR_CMD_PHASE); }
static inline void MotorController_SetFeedbackMode(MotorController_T * p_context, Motor_FeedbackMode_T feedbackMode) { p_context->P_MC_STATE->CmdInput.FeedbackMode = feedbackMode; MotorController_InputMotorCmd(p_context, MOTOR_CONTROLLER_MOTOR_CMD_FEEDBACK); }


/******************************************************************************/
/*
    Lock State
*/
/******************************************************************************/
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

/* Lock State returns to ENTER */
/* Alternatively Caller check Top state. then _StateMachine_GetActiveSubStateId (potential substate id collision) */
/*!
    @retval 0xFF when TopState not in Lock State
    @retval IsComplete SubState => 0xFF, TopState => MC_STATE_LOCK
    @retval Processing SubState => id or 0, TopState => MC_STATE_LOCK
*/
/* alternative getSubstate */
static inline MotorController_LockId_T MotorController_GetLockState(MotorController_T * p_context) { return StateMachine_GetActiveSubStateId(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

// if all calibration function use substate
static inline bool MotorController_IsLockOpComplete(MotorController_T * p_context) { return StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK); }

/* return union status */
// return nvm on nvm
static inline int MotorController_GetLockOpStatus(MotorController_T * p_context) { return p_context->P_MC_STATE->LockOpStatus; }

/******************************************************************************/
/*
    Motor Controller State Variables
*/
/******************************************************************************/
static inline MotorController_StateId_T MotorController_GetStateId(const MotorController_State_T * p_mcState) { return StateMachine_GetActiveStateId(&p_mcState->StateMachine); }

/* Corresponds to known Top State */
static inline state_t _MotorController_GetSubStateId(const MotorController_State_T * p_mcState) { return _StateMachine_GetActiveSubStateId(&p_mcState->StateMachine); }

static inline MotorController_FaultFlags_T MotorController_GetFaultFlags(const MotorController_State_T * p_mcState) { return p_mcState->FaultFlags; }

/*
//move to alternate interface extension
    Status Flags for User Interface

    Combined boolean outputs for protocol convenience
*/
typedef union MotorController_StatusFlags
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
MotorController_StatusFlags_T;

static inline MotorController_StatusFlags_T MotorController_GetStatusFlags(MotorController_T * p_context)
{
    return (MotorController_StatusFlags_T)
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
    Non StateMachine Checked
*/
/******************************************************************************/
/******************************************************************************/
/*
    User Setting Speed/I Limit
*/
/******************************************************************************/
static inline bool MotorController_SetSpeedLimitAll(MotorController_T * p_context, uint16_t limit_fract16)    { _MotorController_SetSpeedLimitAll(p_context, MOT_SPEED_LIMIT_USER, limit_fract16); }
static inline bool MotorController_ClearSpeedLimitAll(MotorController_T * p_context)                          { _MotorController_ClearSpeedLimitAll(p_context, MOT_SPEED_LIMIT_USER); }
static inline bool MotorController_SetILimitAll(MotorController_T * p_context, uint16_t limit_fract16)        { _MotorController_SetILimitAll(p_context, MOT_I_LIMIT_USER, limit_fract16); }
static inline bool MotorController_ClearILimitAll(MotorController_T * p_context)                              { _MotorController_ClearILimitAll(p_context, MOT_I_LIMIT_USER); }

/* using user channel */
static inline void MotorController_SetOptSpeedLimitOnOff(MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_SetSpeedLimitAll(p_context, p_context->P_MC_STATE->Config.OptSpeedLimit_Fract16); }
    else { MotorController_ClearSpeedLimitAll(p_context); }
}

static inline void MotorController_SetOptILimitOnOff(MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_SetILimitAll(p_context, p_context->P_MC_STATE->Config.OptILimit_Fract16); }
    else { MotorController_ClearILimitAll(p_context); }
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/* UserMain, which may use Watchdog  */
static inline void MotorController_EnableRxWatchdog(MotorController_T * p_context) { _Socket_EnableRxWatchdog(MotorController_GetMainSocket(p_context)->P_SOCKET_STATE); }
static inline void MotorController_DisableRxWatchdog(MotorController_T * p_context) { _Socket_DisableRxWatchdog(MotorController_GetMainSocket(p_context)->P_SOCKET_STATE); }
static inline void MotorController_SetRxWatchdog(MotorController_T * p_context, bool isEnable) { _Socket_SetRxWatchdogOnOff(MotorController_GetMainSocket(p_context)->P_SOCKET_STATE, isEnable); }


/******************************************************************************/
/*
    Config MotorController NvM Variables
*/
/******************************************************************************/
/*
    Boot Buffer
*/
static inline BootRef_T MotorController_GetBootReg(const MotorController_State_T * p_mcState)          { return p_mcState->BootRef; }
static inline void MotorController_SetBootReg(MotorController_State_T * p_mcState, BootRef_T bootReg)  { p_mcState->BootRef.Word = bootReg.Word; }
static inline void MotorController_SetFastBoot(MotorController_State_T * p_mcState, bool isEnable)     { p_mcState->BootRef.FastBoot = isEnable; }
static inline void MotorController_SetBeep(MotorController_State_T * p_mcState, bool isEnable)         { p_mcState->BootRef.Beep = isEnable; }
static inline void MotorController_SetBlink(MotorController_State_T * p_mcState, bool isEnable)        { p_mcState->BootRef.Blink = isEnable; }

// static inline bool MotorController_IsB(const MotorController_T * p_mc) { return (p_mc->P_MC_STATE->BootRef.IsValid == p_mc->MOT_NVM.P_BOOT_REF->IsValid); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_SetVSupplyRef(MotorController_T * p_context, uint16_t volts);
extern void MotorController_SetInputMode(MotorController_T * p_context, MotorController_InputMode_T mode);

extern int MotorController_CallSystemCmd(MotorController_T * p_context, MotorController_SystemCmd_T id, int value);

// extern NvMemory_Status_T MotorController_ReadManufacture_Blocking(MotorController_T * p_context, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer);
// extern NvMemory_Status_T MotorController_WriteManufacture_Blocking(MotorController_T * p_context, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size);

// extern void MotorController_SetOptSpeedLimitOnOff(MotorController_T * p_context, bool isEnable);
// extern void MotorController_SetOptILimitOnOff(MotorController_T * p_context, bool isEnable);
#endif

/*
    Controller NvM Variables Config
    boundry check on external input if needed
*/
// static inline void MotorController_User_SetInitMode(MotorController_State_T * p_mcState, MotorController_MainMode_T mode)      { p_mcState->Config.InitMode = mode; }

// static inline void MotorController_User_SetILimitOnLowV(MotorController_State_T * p_mcState, uint16_t i_Fract16)              { p_mcState->Config.VLowILimit_Fract16 = i_Fract16; }

// static inline void MotorController_User_SetOptDinMode(MotorController_State_T * p_mcState, MotorController_OptDinMode_T mode) { p_mcState->Config.OptDinMode = mode; }
// static inline void MotorController_User_DisableOptDin(MotorController_State_T * p_mcState)                                    { p_mcState->Config.OptDinMode = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }

// static inline void MotorController_User_SetOptDinSpeedLimit(MotorController_State_T * p_mcState, uint16_t i_Fract16)          { p_mcState->Config.OptSpeedLimit_Fract16 = i_Fract16; }

