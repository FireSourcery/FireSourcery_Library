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
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_USER_H
#define MOTOR_CONTROLLER_USER_H

#include "MotorController_StateMachine.h"
#include "MotorController_Analog.h"
#include "../MotVarId.h"
#include "../Version.h"

/*
    VarId Base
*/
typedef enum MotorController_User_System
{
    MOT_USER_SYSTEM_BEEP,
    MOT_USER_SYSTEM_LOCK_STATE,
    MOT_USER_SYSTEM_SERVO,
    MOT_USER_SYSTEM_CLEAR_FAULT,
    MOT_USER_SYSTEM_RX_WATCHDOG,
    //     MOT_CALL_SYSTEM_RX_WATCHDOG_ENABLE,
    //     MOT_CALL_SYSTEM_RX_WATCHDOG_DISABLE,
}
MotorController_User_System_T;

typedef enum MotorController_User_SubState
{
    MOT_USER_DRIVE_SUB_STATE,
    MOT_USER_LOCK_SUB_STATE,
    MOT_USER_NVM_STATUS,
    MOT_USER_CALIBRATION_STATUS,
}
MotorController_User_SubState_T;

typedef enum MotorController_Output_General
{
    MOT_VAR_ZERO,
    MOT_VAR_MILLIS,
    MOT_VAR_DEBUG,
    MOT_VAR_MC_STATE,
    MOT_VAR_MC_STATE_FLAGS,
    MOT_VAR_MC_FAULT_FLAGS,
    MOT_VAR_V_SOURCE,
    MOT_VAR_V_SENSOR,
    MOT_VAR_V_ACCS,
    MOT_VAR_BATTERY_CHARGE, // if CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    MOT_VAR_HEAT_PCB,
    MOT_VAR_HEAT_MOSFETS,
    // MOT_VAR_HEAT_MOSFETS_TOP,
    // MOT_VAR_HEAT_MOSFETS_BOT,
    // MOT_VAR_HEAT_MOSFETS_2,
    // MOT_VAR_HEAT_MOSFETS_3,
}
MotorController_Output_General_T;

typedef enum MotorController_Output_Debug
{
    MOT_OUTPUT_DEBUG0, MOT_OUTPUT_DEBUG1, MOT_OUTPUT_DEBUG2, MOT_OUTPUT_DEBUG3, MOT_OUTPUT_DEBUG4, MOT_OUTPUT_DEBUG5, MOT_OUTPUT_DEBUG6, MOT_OUTPUT_DEBUG7,
}
MotorController_Output_Debug_T;

typedef uint32_t mot_status_t;

/******************************************************************************/
/*!
    Instance Select
    @return may return null
*/
/******************************************************************************/
// typedef enum MotVarId_Instance_ThermistorBoard
// {
//     MOT_VAR_ID_THERMISTOR_PCB,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_0,
//     MOT_VAR_ID_THERMISTOR_MOSFETS_1,
// }
// MotVarId_Instance_ThermistorBoard_T;

// typedef enum MotVarId_Instance_VMonitor
// {
//     MOT_VAR_ID_VMONITOR_SOURCE,
//     MOT_VAR_ID_VMONITOR_SENSOR,
//     MOT_VAR_ID_VMONITOR_ACC,
// }
// MotVarId_Instance_VMonitor_T;

/*
    Index corresponds to external user interface
*/
static inline Motor_T * MotorController_User_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex)
{
    return (motorIndex < p_mc->CONST.MOTOR_COUNT) ? MotorController_GetPtrMotor(p_mc, motorIndex) : NULL;
}

static inline Thermistor_T * MotorController_User_GetPtrThermistor(const MotorController_T * p_mc, uint8_t index)
{
    const Thermistor_T * p_thermistor;
    switch (index)
    {
        case 0U: p_thermistor = &p_mc->ThermistorPcb; break;
        default:
            p_thermistor = ((index - 1U) < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT) ? &p_mc->MosfetsThermistors[index - 1U] : NULL;
            break;

    }
    return (Thermistor_T *)p_thermistor;
}

static inline VMonitor_T * MotorController_User_GetPtrVMonitor(const MotorController_T * p_mc, uint8_t index)
{
    const VMonitor_T * p_vMonitor;
    switch (index)
    {
        case 0U: p_vMonitor = &p_mc->VMonitorSource; break;
        case 1U: p_vMonitor = &p_mc->VMonitorSense;  break;
        case 2U: p_vMonitor = &p_mc->VMonitorAccs;   break;
        default: p_vMonitor = NULL; break;
    }
    return (VMonitor_T *)p_vMonitor;
}

static inline Protocol_T * MotorController_User_GetPtrProtocol(const MotorController_T * p_mc, uint8_t protocolIndex)
{
    return (protocolIndex < p_mc->CONST.PROTOCOL_COUNT) ? &p_mc->CONST.P_PROTOCOLS[protocolIndex] : NULL;
}

/* UserMain, which may use Watchdog  */
static inline Protocol_T * MotorController_User_GetMainProtocol(const MotorController_T * p_mc)
{
    &p_mc->CONST.P_PROTOCOLS[p_mc->CONST.USER_PROTOCOL_INDEX];
}


/******************************************************************************/
/*
    Real Time User Input Interface; into StateMachine process

    StateMachine in Sync mode process last set input, may overwrite, once per ms

    Inputs that passes to Motor_StateMachine AND mark Substate, must be Sync to wait for Motor
*/
/******************************************************************************/

/******************************************************************************/
/* Common */
/******************************************************************************/
/*
    All or Primary motor
*/
/*! @param[in] userCmd [-32767:32767] */
static inline void MotorController_User_SetCmdValue(MotorController_T * p_mc, int16_t userCmd)
{
    _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_CMD, (uint32_t)userCmd);
}

static inline void MotorController_User_SetFeedbackMode(MotorController_T * p_mc, uint8_t feedbackMode)
{
    p_mc->UserCmdMode.Value = feedbackMode; // or include mod in bitfield
}

// static inline int32_t MotorController_User_GetCmdValue(const MotorController_T * p_mc) { return p_mc->UserCmdValue; }

// float all and ground all, stop state then use motor
// static inline void MotorController_User_ReleaseControl(MotorController_T * p_mc) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_MODE, MOTOR_CONTROLLER_RELEASE); }
// static inline void MotorController_User_SetDriveGround(MotorController_T * p_mc) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE, MOTOR_CONTROLLER_HOLD); }

/******************************************************************************/
/* Drive Mode */
/******************************************************************************/
/******************************************************************************/
 /*!
    DriveCmd - Throttle, Brake, Zero
    @param[in] driveCmd Throttle, Brake, Zero
    @param[in] cmdValue [0:65535]
*/
/******************************************************************************/
// static inline void MotorController_User_SetDriveCmd(MotorController_T * p_mc, MotorController_DriveId_T cmdId, uint16_t cmdValue)
// {
//     _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_DRIVE, cmdId);
//     p_mc->UserCmdValue = cmdValue;
// }
/* Same as Brake/Throttle 0 */
// static inline void MotorController_User_SetCmdDriveZero(MotorController_T * p_mc)               { MotorController_User_SetDriveCmd(p_mc, MOTOR_CONTROLLER_DRIVE_RELEASE, 0U); }

static inline void MotorController_User_SetCmdDriveZero(MotorController_T * p_mc)                  { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE, 0U); }
static inline void MotorController_User_SetCmdThrottle(MotorController_T * p_mc, uint16_t userCmd) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_THROTTLE, userCmd); }
static inline void MotorController_User_SetCmdBrake(MotorController_T * p_mc, uint16_t userCmd)    { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_BRAKE, userCmd); }

// todo move Drive image to statemachine or let host maintain state
// static inline void MotorController_User_SetCmdThrottle(MotorController_T * p_mc, uint16_t userCmd)
// {
//     if (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_DRIVE)
//     {
//         MotorController_SetThrottleValue(p_mc, userCmd);
//     }
// }


/******************************************************************************/
/*
    Lock/Blocking
    Safe state actions, maybe blocking
*/
/******************************************************************************/
static inline void MotorController_User_InputLock(MotorController_T * p_mc, MotorController_LockId_T id)
{
    _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_LOCK, id);
}

/* Alternatively return union status */
static inline uint32_t MotorController_UserPollLockAsyncStatus(MotorController_T * p_mc, MotorController_LockId_T opId)
{
    uint32_t status = MOT_VAR_STATUS_OK;

    switch (opId)
    {
        case MOTOR_CONTROLLER_LOCK_ENTER:               status = MotorController_User_IsLockState(p_mc) ? MOT_VAR_STATUS_OK : MOT_VAR_STATUS_ERROR;       break;
        case MOTOR_CONTROLLER_LOCK_EXIT:                status = !MotorController_User_IsLockState(p_mc) ? MOT_VAR_STATUS_OK : MOT_VAR_STATUS_ERROR;      break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:    status = p_mc->LockOpStatus;    break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC:       status = p_mc->LockOpStatus;    break;
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:     status = p_mc->NvmStatus;       break;
        case MOTOR_CONTROLLER_LOCK_REBOOT:              status = MOT_VAR_STATUS_OK;     break;
        default: break;
    }

    return status;
}

static inline bool MotorController_User_IsLockState(MotorController_T * p_mc)
{
    return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_LOCK);
}

static inline bool MotorController_User_EnterLockState(MotorController_T * p_mc)
{
    MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_ENTER);
    return MotorController_User_IsLockState(p_mc);
}

static inline bool MotorController_User_ExitLockState(MotorController_T * p_mc)
{
    MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_EXIT);
    return !MotorController_User_IsLockState(p_mc); /* Park or Servo */
}

// Save RAM to NVM
static inline NvMemory_Status_T MotorController_User_SaveConfig_Blocking(MotorController_T * p_mc)
{
    MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);
    return p_mc->NvmStatus;
}

static inline bool MotorController_User_IsConfigState(MotorController_T * p_mc)
{
    return (MotorController_User_IsLockState(p_mc) || MotorController_StateMachine_IsFault(p_mc));
}

/* Lock State returns to ENTER */
static inline bool MotorController_User_IsLockOpComplete(const MotorController_T * p_mc) { return p_mc->LockSubState == MOTOR_CONTROLLER_LOCK_ENTER; }
static inline uint8_t MotorController_User_GetLockOpStatus(const MotorController_T * p_mc) { return p_mc->LockOpStatus; }

/******************************************************************************/
/*
    Servo
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static inline void MotorController_User_InputServoMode(MotorController_T * p_mc, MotorController_ServoMode_T servoMode)
{
    _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, servoMode);
}

static inline bool MotorController_User_IsServoState(MotorController_T * p_mc)
{
    return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO);
}

static inline void MotorController_User_EnterServoMode(MotorController_T * p_mc)   { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_MACHINE_INPUT_VALUE_NULL); }
static inline void MotorController_User_ExitServoMode(MotorController_T * p_mc)    { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_MACHINE_INPUT_VALUE_NULL); }


// static inline void MotorController_User_StartControlMode(MotorController_T * p_mc, uint8_t feedbackMode)
// {
//     if (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO)
//     {
//         MotorController_StartControlModeAll(p_mc, feedbackMode);
//     }
// }
#endif

/******************************************************************************/
/*
    Non StateMachine Checked Direct Inputs
*/
/******************************************************************************/
/* StateMachine unchecked disable motors, use with caution */
static inline void MotorController_User_ForceDisableControl(MotorController_T * p_mc)
{
    MotorController_ForceDisableAll(p_mc);
    p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_RELEASE;
}

static inline void MotorController_User_EnableRxWatchdog(MotorController_T * p_mc) { Protocol_EnableRxWatchdog(MotorController_User_GetMainProtocol(p_mc)); }
static inline void MotorController_User_DisableRxWatchdog(MotorController_T * p_mc) { Protocol_DisableRxWatchdog(MotorController_User_GetMainProtocol(p_mc)); }
static inline void MotorController_User_SetRxWatchdog(MotorController_T * p_mc, bool isEnable) { Protocol_SetRxWatchdogOnOff(MotorController_User_GetMainProtocol(p_mc), isEnable); }

static inline void MotorController_User_BeepN(MotorController_T * p_mc, uint32_t onTime, uint32_t offTime, uint8_t n)   { Blinky_BlinkN(&p_mc->Buzzer, onTime, offTime, n); }
static inline void MotorController_User_BeepStart(MotorController_T * p_mc, uint32_t onTime, uint32_t offTime)          { Blinky_StartPeriodic(&p_mc->Buzzer, onTime, offTime); }
static inline void MotorController_User_BeepStop(MotorController_T * p_mc)                                              { Blinky_Stop(&p_mc->Buzzer); }

// todo base flag
static inline void MotorController_User_DisableBuzzer(MotorController_T * p_mc)
{
    Blinky_Stop(&p_mc->Buzzer);
    p_mc->StateFlags.BuzzerEnable = 0U;
}

static inline bool MotorController_User_SetSpeedLimitAll(MotorController_T * p_mc, uint16_t limit_fract16)
{
    struct_array_for_any_set_uint16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (try_uint16_t)Motor_User_TrySpeedLimit, limit_fract16);
    // MotorController_SetSpeedLimitAll(MotorController_T * p_mc, USER,   limit_fract16)
}

static inline bool MotorController_User_ClearSpeedLimitAll(MotorController_T * p_mc)
{
    void_array_for_any(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_ClearSpeedLimit);
}

static inline bool MotorController_User_SetILimitAll(MotorController_T * p_mc, uint16_t limit_fract16)
{
    struct_array_for_any_set_uint16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (try_uint16_t)Motor_User_TryILimit, limit_fract16);
}

static inline bool MotorController_User_ClearILimitAll(MotorController_T * p_mc)
{
    void_array_for_any(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_ClearILimit);
}

static inline void MotorController_User_SetOptSpeedLimitOnOff(MotorController_T * p_mc, bool isEnable)
{
    if (isEnable == true)   { MotorController_User_SetSpeedLimitAll(p_mc, p_mc->Config.OptSpeedLimit_Fract16); }
    else                    { MotorController_User_ClearSpeedLimitAll(p_mc); }
}

static inline void MotorController_User_SetOptILimitOnOff(MotorController_T * p_mc, bool isEnable)
{
    if (isEnable == true)   { MotorController_User_SetILimitAll(p_mc, p_mc->Config.OptILimit_Fract16); }
    else                    { MotorController_User_ClearILimitAll(p_mc); }
}

/******************************************************************************/
/*
    Motor Controller Struct Variables
*/
/******************************************************************************/
/*
    Controller RAM Variables
*/
/*
    Status Flags for User Interface
*/
typedef union MotorController_User_StatusFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U; // ILimit by Heat
        uint16_t VLow               : 1U; // ILimit by VLow
        uint16_t SpeedLimit         : 1U;
        uint16_t ILimit             : 1U;
        // uint16_t IsStopped          : 1U;
        uint16_t BuzzerEnable       : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
    };
    uint16_t Word;
}
MotorController_User_StatusFlags_T;

static inline MotorController_User_StatusFlags_T MotorController_User_GetStatusFlags(const MotorController_T * p_mc)
{
    MotorController_User_StatusFlags_T statusFlags;
    statusFlags.HeatWarning     = p_mc->StateFlags.HeatWarning;
    statusFlags.VLow            = p_mc->StateFlags.VLow;
    statusFlags.BuzzerEnable    = p_mc->StateFlags.BuzzerEnable;
    return statusFlags;
}

static inline MotorController_StateMachine_StateId_T MotorController_User_GetStateId(const MotorController_T * p_mc)   { return StateMachine_GetActiveStateId(&p_mc->StateMachine); }
static inline MotorController_StateFlags_T MotorController_User_GetStateFlags(const MotorController_T * p_mc)          { return p_mc->StateFlags; }
static inline MotorController_FaultFlags_T MotorController_User_GetFaultFlags(const MotorController_T * p_mc)          { return p_mc->FaultFlags; }

static inline uint16_t MotorController_User_GetHeatPcb_Adcu(const MotorController_T * p_mc)                         { return MotorController_Analog_GetHeatPcb(p_mc); }
static inline uint16_t MotorController_User_GetHeatMosfets_Adcu(const MotorController_T * p_mc)                     { return MotorController_Analog_GetHeatMosfets(p_mc, 0); }
static inline uint16_t MotorController_User_GetHeatMosfetsIndex_Adcu(const MotorController_T * p_mc, uint8_t index) { return MotorController_Analog_GetHeatMosfets(p_mc, index); }

/*
    Boot Buffer
*/
static inline BootRef_T MotorController_User_GetBootReg(const MotorController_T * p_mc)                { return p_mc->BootRef; }
static inline void MotorController_User_SetBootReg(MotorController_T * p_mc, BootRef_T bootReg)        { p_mc->BootRef.Word = bootReg.Word; }
static inline void MotorController_User_SetFastBoot(MotorController_T * p_mc, bool isEnable)           { p_mc->BootRef.FastBoot = isEnable; }
static inline void MotorController_User_SetBeep(MotorController_T * p_mc, bool isEnable)               { p_mc->BootRef.Beep  = isEnable; }
static inline void MotorController_User_SetBlink(MotorController_T * p_mc, bool isEnable)              { p_mc->BootRef.Blink = isEnable; }

/*
    Controller NvM Variables Config
*/
static inline uint16_t MotorController_User_GetVSourceRef(const MotorController_T * p_mc) { return p_mc->Config.VSourceRef; }

// static inline Motor_FeedbackMode_T MotorController_User_GetDefaultFeedbackMode(const MotorController_T * p_mc)      { return p_mc->Config.DefaultCmdMode; }
// static inline void MotorController_User_SetDefaultFeedbackMode_Cast(MotorController_T * p_mc, uint16_t wordValue)   { p_mc->Config.DefaultCmdMode.Value = wordValue; }

static inline MotorController_MainMode_T MotorController_User_GetInitMode(const MotorController_T * p_mc)           { return p_mc->Config.InitMode; }
static inline MotorController_InputMode_T MotorController_User_GetInputMode(const MotorController_T * p_mc)         { return p_mc->Config.InputMode; }
static inline MotorController_BrakeMode_T MotorController_User_GetBrakeMode(const MotorController_T * p_mc)         { return p_mc->Config.BrakeMode; }
static inline MotorController_ThrottleMode_T MotorController_User_GetThrottleMode(const MotorController_T * p_mc)   { return p_mc->Config.ThrottleMode; }
static inline MotorController_DriveZeroMode_T MotorController_User_GetDriveZeroMode(const MotorController_T * p_mc) { return p_mc->Config.DriveZeroMode; }

static inline void MotorController_User_SetInitMode(MotorController_T * p_mc, MotorController_MainMode_T mode)             { p_mc->Config.InitMode = mode; }
static inline void MotorController_User_SetInputMode(MotorController_T * p_mc, MotorController_InputMode_T mode)           { p_mc->Config.InputMode = mode; }
static inline void MotorController_User_SetBrakeMode(MotorController_T * p_mc, MotorController_BrakeMode_T mode)           { p_mc->Config.BrakeMode = mode; }
static inline void MotorController_User_SetThrottleMode(MotorController_T * p_mc, MotorController_ThrottleMode_T mode)     { p_mc->Config.ThrottleMode = mode; }
static inline void MotorController_User_SetDriveZeroMode(MotorController_T * p_mc, MotorController_DriveZeroMode_T mode)   { p_mc->Config.DriveZeroMode = mode; }
static inline void MotorController_User_SetILimitOnLowV(MotorController_T * p_mc, uint16_t i_Fract16)                      { p_mc->Config.VLowILimit_Fract16 = i_Fract16; }

static inline void MotorController_User_SetOptDinMode(MotorController_T * p_mc, MotorController_OptDinMode_T mode) { p_mc->Config.OptDinMode = mode; }
static inline void MotorController_User_DisableOptDin(MotorController_T * p_mc)                                    { p_mc->Config.OptDinMode = MOTOR_CONTROLLER_OPT_DIN_DISABLE; }
static inline void MotorController_User_SetOptDinSpeedLimit(MotorController_T * p_mc, uint16_t i_Fract16)          { p_mc->Config.OptSpeedLimit_Fract16 = i_Fract16; }


/******************************************************************************/
/*
    Read Only
*/
/******************************************************************************/
static inline uint16_t MotorController_User_GetVMax(void) { return MOTOR_STATIC.V_MAX_VOLTS; }
static inline uint16_t MotorController_User_GetIMax(void) { return MOTOR_STATIC.I_MAX_AMPS; }
static inline uint16_t MotorController_User_GetIMaxAdcu(void) { return MOTOR_STATIC.I_PEAK_ADCU; }
// static inline char * MotorController_User_GetBoardName(void)  { return MOTOR_STATIC.BOARD_NAME; }

static inline uint32_t MotorController_User_GetLibraryVersion(void) { return MOTOR_LIBRARY_VERSION; }
static inline uint8_t MotorController_User_GetLibraryVersionIndex(uint8_t charIndex)
{
    uint8_t versionChar;
    switch (charIndex)
    {
        case 0U: versionChar = MOTOR_LIBRARY_VERSION_FIX;    break;
        case 1U: versionChar = MOTOR_LIBRARY_VERSION_MINOR;  break;
        case 2U: versionChar = MOTOR_LIBRARY_VERSION_MAJOR;  break;
        case 3U: versionChar = MOTOR_LIBRARY_VERSION_OPT;    break;
        default: versionChar = 0U; break;
    }
    return versionChar;
}

static inline uint32_t MotorController_User_GetMainVersion(const MotorController_T * p_mc) { return *((uint32_t *)(&p_mc->CONST.MAIN_VERSION[0U])); }
static inline uint8_t MotorController_User_GetMainVersionIndex(const MotorController_T * p_mc, uint8_t charIndex) { return p_mc->CONST.MAIN_VERSION[charIndex]; }

// static inline void MotorController_User_GetBoardRef(const MotorController_T * p_mc, void * p_destBuffer)
// {
//     ((uint32_t *)p_destBuffer)[0U] = p_mc->VMonitorSource.CONST.UNITS_R1;
//     ((uint32_t *)p_destBuffer)[1U] = p_mc->VMonitorSource.CONST.UNITS_R2;
//     ((uint32_t *)p_destBuffer)[2U] = p_mc->ThermistorPcb.CONST.R_SERIES;
//     ((uint32_t *)p_destBuffer)[3U] = p_mc->ThermistorPcb.CONST.R_PARALLEL;
//     ((uint32_t *)p_destBuffer)[4U] = p_mc->MosfetsThermistors[0U].CONST.R_SERIES;
//     ((uint32_t *)p_destBuffer)[5U] = p_mc->MosfetsThermistors[0U].CONST.R_PARALLEL;
// }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern MotorController_Direction_T MotorController_User_GetDirection(const MotorController_T * p_mc);
extern void MotorController_User_SetDirection(MotorController_T * p_mc, MotorController_Direction_T direction);
extern void MotorController_User_SetVSourceRef(MotorController_T * p_mc, uint16_t volts);
extern NvMemory_Status_T MotorController_User_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer);
extern NvMemory_Status_T MotorController_User_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_source, uint8_t size);

uint32_t MotorController_User_InputControl(MotorController_T * p_mc, MotVarId_Control_General_T id, int32_t value);
uint32_t MotorController_User_InputCmd(MotorController_T * p_mc, MotVarId_Cmd_General_T id, int32_t value);
int32_t MotorController_Output_Debug(MotorController_T * p_mc, MotorController_Output_Debug_T id);
uint32_t MotorController_User_InputSystem(MotorController_T * p_mc, MotorController_User_System_T id, int32_t value);

int32_t MotorController_User_GetConfigGeneral(const MotorController_T * p_mc, MotVarId_Config_General_T id);
uint32_t MotorController_User_SetConfigGeneral(MotorController_T * p_mc, MotVarId_Config_General_T id, int32_t value);
uint32_t MotorController_User_SetConfigAnalogUser(MotorController_T * p_mc, MotVarId_Config_AnalogUser_T id, int32_t value);

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
extern void MotorController_User_SetBatteryLifeDefault(MotorController_T * p_mc);
extern void MotorController_User_SetBatteryLife_MilliV(MotorController_T * p_mc, uint32_t zero_mV, uint32_t max_mV);
#endif

#endif

