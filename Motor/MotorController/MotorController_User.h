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
typedef enum MotorController_User_SystemCmd
{
    MOT_USER_SYSTEM_BEEP, /* Beep N */
    MOT_USER_SYSTEM_BEEP_STOP,
    /* alterntively as general */
    MOT_USER_SYSTEM_FORCE_DISABLE_CONTROL,  // Force Disable control Non StateMachine checked, also handled via MOT_PACKET_STOP_ALL
    MOT_USER_SYSTEM_CLEAR_FAULT,            // Fault State / Flags
    MOT_USER_SYSTEM_RX_WATCHDOG,        // on/off
    MOT_USER_SYSTEM_LOCK_STATE_INPUT,       // MotorController_LockId_T as input: Nvm, Calibration
    // MOT_USER_SYSTEM_LOCK_STATE_EXIT,
    MOT_USER_SYSTEM_LOCK_STATE_STATUS,  // MotorController_LockId_T as status
    MOT_USER_SYSTEM_LOCK_ASYNC_STATUS,  // Async operation status. optionally pass MotorController_LockId_T for selection
    MOT_USER_SYSTEM_MAIN_MODE_INPUT,
    MOT_USER_SYSTEM_STATE_COMMAND,      // State Command
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
    _StateMachine_ProcBranchInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_MAIN_MODE, (state_value_t)value);
}

/******************************************************************************/
/*
    passthrough Common
    StateMachine handle push to all or primary motor
    Apply in MCSM_STATE_ID_MOTORS State

    MotorController_User_Set
    MotDrive_User_Set
    Motor_User_Set
*/
/******************************************************************************/
static inline void MotorController_User_ApplyMotorsCmd(const MotorController_T * p_context)
{
    p_context->P_MC_STATE->CmdInput.IsUpdated = true; /* sync Input 10ms-50ms, Proc 1ms */
    // _StateMachine_ProcInput(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, MCSM_INPUT_MOTORS_CMD, inputId);
}

/*
    sync write to motor state machine on edge
    Validate by statemachine

    set the buffer, let state machine handle depending on mode
    proc is on the same thread

    Pass outer context in case implementation changes

    as base motor cmd, or generic interface?
*/
static inline void MotorController_User_SetCmdValue(const MotorController_T * p_context, int16_t userCmd) { p_context->P_MC_STATE->CmdInput.CmdValue = userCmd; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetDirection(const MotorController_T * p_context, Motor_User_Direction_T direction) { p_context->P_MC_STATE->CmdInput.Direction = direction; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetControlState(const MotorController_T * p_context, Phase_Output_T controlState) { p_context->P_MC_STATE->CmdInput.PhaseState = controlState; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetFeedbackMode(const MotorController_T * p_context, Motor_FeedbackMode_T feedbackMode) { p_context->P_MC_STATE->CmdInput.FeedbackMode = feedbackMode; MotorController_User_ApplyMotorsCmd(p_context); }
static inline void MotorController_User_SetFeedbackMode_Cast(const MotorController_T * p_context, int feedbackMode) { MotorController_User_SetFeedbackMode(p_context, Motor_FeedbackMode_Cast(feedbackMode)); }


/******************************************************************************/
/*

*/
/******************************************************************************/
/* Non StateMachine checked disable motors. Caller ensure non field weakening state */
/* Update State to prevent input overwrite */
static inline void MotorController_User_ForceDisableControl(const MotorController_T * p_context)
{
    MotMotors_ForceDisableControl(&p_context->MOTORS);
    p_context->P_MC_STATE->CmdInput.CmdValue = 0;
    p_context->P_MC_STATE->CmdInput.PhaseState = PHASE_OUTPUT_FLOAT;
    p_context->P_MC_STATE->CmdInput.Direction = MOTOR_DIRECTION_STOP;
    p_context->P_MC_STATE->CmdInput.IsUpdated = true;

    MotorController_StateMachine_InputStateCommand(p_context, MOTOR_CONTROLLER_STATE_CMD_E_STOP);

    /* if drive mode */
    // MotDrive_User_SetZero(p_context->MOT_DRIVE.P_MOT_DRIVE_STATE); // set drive to zero
    // MotDrive_User_ApplyDirection(&p_context->MOT_DRIVE, MOTOR_CONTROLLER_DIRECTION_NEUTRAL); // set drive direction to neutral
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
    if (isEnable == true) { MotorController_User_SetSpeedLimitAll(p_context, p_context->P_MC_STATE->Config.OptSpeedLimit_Fract16); }
    else { MotorController_User_ClearSpeedLimitAll(p_context); }
}

static inline void MotorController_User_SetOptILimitOnOff(const MotorController_T * p_context, bool isEnable)
{
    if (isEnable == true) { MotorController_User_SetILimitAll(p_context, p_context->P_MC_STATE->Config.OptILimit_Fract16); }
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
/* alternatively move to state machine */
static inline MotorController_StateId_T MotorController_User_GetStateId(const MotorController_State_T * p_mcState) { return StateMachine_GetActiveStateId(&p_mcState->StateMachine); }
static inline state_t MotorController_User_GetSubStateId(const MotorController_State_T * p_mcState) { return _StateMachine_GetActiveSubStateId(&p_mcState->StateMachine); }

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

// typedef enum MotorController_Direction
// {
//     MOTOR_CONTROLLER_DIRECTION_PARK,
//     MOTOR_CONTROLLER_DIRECTION_FORWARD,
//     MOTOR_CONTROLLER_DIRECTION_NEUTRAL,
//     MOTOR_CONTROLLER_DIRECTION_REVERSE,
//     MOTOR_CONTROLLER_DIRECTION_ERROR,
// }
// MotorController_Direction_T;

// MotorController_Direction_T MotorController_User_GetDirection(MotorController_T * p_context)
// {
//     switch (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_ACTIVE))
//     {
//         case MCSM_STATE_ID_PARK:       return MOTOR_CONTROLLER_DIRECTION_PARK;
//         default:
//             switch (_MotMotors_GetDirectionAll(&p_context->MOTORS))
//             {
//                 case MOTOR_DIRECTION_FORWARD:  return MOTOR_CONTROLLER_DIRECTION_FORWARD;
//                     // case MOTOR_DIRECTION_NEUTRAL:  return MOTOR_CONTROLLER_DIRECTION_NEUTRAL;
//                 case MOTOR_DIRECTION_REVERSE:  return MOTOR_CONTROLLER_DIRECTION_REVERSE;
//                 default:                       return MOTOR_CONTROLLER_DIRECTION_ERROR;
//             }
//     }
// }

// typedef union MotorController_Direction
// {
//     struct
//     {
//         uint16_t IsPark     : 1U;
//         uint16_t Direction  : 2U; // 0 => ZERO, 1 => FORWARD, 3 => REVERSE
//     };
//     uint16_t Value;
// }
// MotorController_Direction_T;



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
static inline void MotorController_User_SetBeep(MotorController_State_T * p_mcState, bool isEnable)         { p_mcState->BootRef.Beep = isEnable; }
static inline void MotorController_User_SetBlink(MotorController_State_T * p_mcState, bool isEnable)        { p_mcState->BootRef.Blink = isEnable; }

// static inline bool MotorController_User_IsB(const MotorController_T * p_mc) { return (p_mc->P_MC_STATE->BootRef.IsValid == p_mc->MOT_NVM.P_BOOT_REF->IsValid); }


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

