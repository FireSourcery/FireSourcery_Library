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
    MOT_USER_SYSTEM_LOCK_STATE_INPUT,       // [MotorController_LockId_T]. includes Nvm, Calibration
    // MOT_USER_SYSTEM_LOCK_STATE_EXIT,
    MOT_USER_SYSTEM_LOCK_STATE_STATUS,  // substate id, [MotorController_LockId_T] as status
    MOT_USER_SYSTEM_LOCK_ASYNC_STATUS,  // operation, Async operation status. optionally pass [MotorController_LockId_T] for selection

    MOT_USER_SYSTEM_STATE_COMMAND,      // State Command
    MOT_USER_SYSTEM_DIRECTION_COMMAND,

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


/*
    Status Flags for User Interface
    Combined boolean outputs for protocol convenience
*/
typedef union MotorController_StatusFlags
{
    struct
    {
        // uint16_t HeatWarning : 1U; // ILimit by Heat
        // uint16_t VSourceLow : 1U; // ILimit by VSourceLow
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

static inline MotorController_StatusFlags_T MotorController_GetStatusFlags(MotorController_T * p_dev)
{
    return (MotorController_StatusFlags_T)
    {
        // .HeatWarning    = Monitor_GetStatus(p_dev->HEAT_PCB.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT ||
        //                   Monitor_GetStatus(p_dev->HEAT_MOSFETS.P_STATE) == HEAT_MONITOR_STATUS_WARNING_OVERHEAT,
        // .HeatWarning    = p_dev->StateFlags.HeatWarning,
        // .VSourceLow     = p_dev->StateFlags.VSourceLow,
        // .BuzzerEnable   = p_dev->StateFlags.BuzzerEnable,
    };
}

/******************************************************************************/
/*
    Non StateMachine Checked
*/
/******************************************************************************/
/* Non StateMachine checked disable motors. Caller ensure non field weakening state */
/* Update State to prevent input overwrite */
static inline void MotorController_ForceDisableControl(MotorController_T * p_dev)
{
    Motor_Table_ForceDisableControl(&p_dev->MOTORS);
    MotorController_InputStateCommand(p_dev, MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN);
    p_dev->P_MC->CmdInput.CmdValue = 0;
    p_dev->P_MC->CmdInput.PhaseOutput = PHASE_VOUT_Z;
    p_dev->P_MC->CmdInput.Direction = MOTOR_DIRECTION_NULL;
}

/******************************************************************************/
/*
    User Setting Speed/I Limit
    derate_fract16: Q15 ufract16 in [0, FRACT16_MAX] ≡ [0, 1] of rated.
    Caller passes a unitless ratio; rated multiplication happens at the Motor_Table_Apply* boundary.
*/
/******************************************************************************/
static inline bool MotorController_SetUserSpeedLimitAll(MotorController_T * p_dev, uint16_t derate_fract16) { return _MotorController_SetSpeedLimitAll(p_dev, MOT_SPEED_LIMIT_USER, derate_fract16); }
static inline bool MotorController_ClearUserSpeedLimitAll(MotorController_T * p_dev) { return _MotorController_ClearSpeedLimitAll(p_dev, MOT_SPEED_LIMIT_USER); }
static inline bool MotorController_SetUserILimitAll(MotorController_T * p_dev, uint16_t derate_fract16) { return _MotorController_SetILimitAll(p_dev, MOT_I_LIMIT_USER, derate_fract16); }
static inline bool MotorController_ClearUserILimitAll(MotorController_T * p_dev) { return _MotorController_ClearILimitAll(p_dev, MOT_I_LIMIT_USER); }


/******************************************************************************/
/*

*/
/******************************************************************************/
/* UserMain, which may use Watchdog  */
static inline void MotorController_EnableRxWatchdog(MotorController_T * p_dev) { _Socket_EnableRxWatchdog(MotorController_GetMainSocket(p_dev)->P_SOCKET_STATE); }
static inline void MotorController_DisableRxWatchdog(MotorController_T * p_dev) { _Socket_DisableRxWatchdog(MotorController_GetMainSocket(p_dev)->P_SOCKET_STATE); }
static inline void MotorController_SetRxWatchdog(MotorController_T * p_dev, bool isEnable) { _Socket_SetRxWatchdogOnOff(MotorController_GetMainSocket(p_dev)->P_SOCKET_STATE, isEnable); }


/******************************************************************************/
/*
    Config MotorController NvM Variables
*/
/******************************************************************************/
/*
    Boot Buffer
*/
static inline BootRef_T MotorController_GetBootReg(const MotorController_State_T * p_mcState) { return p_mcState->BootRef; }
static inline void MotorController_SetBootReg(MotorController_State_T * p_mcState, BootRef_T bootReg) { p_mcState->BootRef.Word = bootReg.Word; }
static inline void MotorController_SetFastBoot(MotorController_State_T * p_mcState, bool isEnable) { p_mcState->BootRef.FastBoot = isEnable; }
static inline void MotorController_SetBeep(MotorController_State_T * p_mcState, bool isEnable) { p_mcState->BootRef.Beep = isEnable; }
static inline void MotorController_SetBlink(MotorController_State_T * p_mcState, bool isEnable) { p_mcState->BootRef.Blink = isEnable; }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern int MotorController_CallSystemCmd(MotorController_T * p_dev, MotorController_SystemCmd_T id, int value);
extern bool MotorController_CheckDirection(MotorController_T * p_dev, sign_t direction);

