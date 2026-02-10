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
    @file   MotorController_User.h
    @author FireSourcery
    @brief  User Interface wrapper accessor functions (chip external inputs).
                Includes error checking
*/
/******************************************************************************/
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
    /* alternatively as general */
    MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL,  // Force Disable control Non StateMachine checked, also handled via MOT_PACKET_STOP_ALL
    MOT_USER_SYSTEM_CLEAR_FAULT,            // Fault State / Flags
    MOT_USER_SYSTEM_RX_WATCHDOG,        // on/off
    MOT_USER_SYSTEM_LOCK_STATE_INPUT,       // MotorController_LockId_T as input: Nvm, Calibration
    // MOT_USER_SYSTEM_LOCK_STATE_EXIT,
    MOT_USER_SYSTEM_LOCK_STATE_STATUS,  // substate id, MotorController_LockId_T as status
    MOT_USER_SYSTEM_LOCK_ASYNC_STATUS,  // operation, Async operation status. optionally pass MotorController_LockId_T for selection

    MOT_USER_SYSTEM_STATE_COMMAND,      // State Command
    MOT_USER_SYSTEM_DIRECTION_COMMAND,
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


/* Non StateMachine checked disable motors. Caller ensure non field weakening state */
/* Update State to prevent input overwrite */
static inline void MotorController_ForceDisableControl(MotorController_T * p_context)
{
    Motor_Table_ForceDisableControl(&p_context->MOTORS);
    MotorController_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_E_STOP);
    p_context->P_MC_STATE->CmdInput.CmdValue = 0;
    p_context->P_MC_STATE->CmdInput.PhaseOutput = PHASE_OUTPUT_FLOAT;
    p_context->P_MC_STATE->CmdInput.Direction = MOTOR_DIRECTION_NULL;
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

/*
    Controller NvM Variables Config
    boundry check on external input if needed
*/
// static inline void MotorController_User_SetInitMode(MotorController_State_T * p_mcState, MotorController_MainMode_T mode)      { p_mcState->Config.InitMode = mode; }
// static inline void MotorController_User_SetILimitOnLowV(MotorController_State_T * p_mcState, uint16_t i_Fract16)              { p_mcState->Config.VLowILimit_Fract16 = i_Fract16; }
// static inline void MotorController_User_SetOptDinMode(MotorController_State_T * p_mcState, MotorController_OptDinMode_T mode) { p_mcState->Config.OptDinMode = mode; }
// static inline void MotorController_User_DisableOptDin(MotorController_State_T * p_mcState)                                    { p_mcState->Config.OptDinMode = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }
// static inline void MotorController_User_SetOptDinSpeedLimit(MotorController_State_T * p_mcState, uint16_t i_Fract16)          { p_mcState->Config.OptSpeedLimit_Fract16 = i_Fract16; }

