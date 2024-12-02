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
    @file   MotorController.h
    @author FireSourcery
    @brief  Facade Wrapper
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"
#include "MotorControllerAnalog.h"
#include "MotAnalogUser/MotAnalogUser.h"

#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor_User.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"
#include "Transducer/VMonitor/VMonitor.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"
#include "Peripheral/Serial/Serial.h"
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
#include "Peripheral/CanBus/CanBus.h"
#endif

#include "Utility/Array/struct_array.h"
#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Protocol/Protocol.h"
#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
#include "Utility/Shell/Shell.h"
#endif
#include "Utility/BootRef/BootRef.h"

#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <string.h>

typedef enum MotorController_InitMode
{
    MOTOR_CONTROLLER_INIT_MODE_DRIVE,
    MOTOR_CONTROLLER_INIT_MODE_SERVO,
}
MotorController_InitMode_T;

typedef enum MotorController_InputMode
{
    MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
    MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
    MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum
{
    MOTOR_CONTROLLER_BRAKE_MODE_PASSIVE,
    MOTOR_CONTROLLER_BRAKE_MODE_TORQUE,
    MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE,
}
MotorController_BrakeMode_T;

typedef enum
{
    MOTOR_CONTROLLER_THROTTLE_MODE_SPEED,
    MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE,
    MOTOR_CONTROLLER_THROTTLE_MODE_VOLTAGE,
}
MotorController_ThrottleMode_T;

typedef enum MotorController_DriveZeroMode
{
    MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT,       /* "Coast". MOSFETS non conducting. Same as Neutral. */
    MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN,       /* Regen Brake */
    MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE,      /* Voltage following, Zero current/torque */
    MOTOR_CONTROLLER_DRIVE_ZERO_MODE_ZERO,        /* Setpoint Zero. No cmd overwrite */
}
MotorController_DriveZeroMode_T;

/* MultiState SubState - Drive State */
typedef enum MotorController_Direction
{
    MOTOR_CONTROLLER_DIRECTION_PARK,
    MOTOR_CONTROLLER_DIRECTION_NEUTRAL,
    MOTOR_CONTROLLER_DIRECTION_FORWARD,
    MOTOR_CONTROLLER_DIRECTION_REVERSE,
    MOTOR_CONTROLLER_DIRECTION_ERROR = 255U,
}
MotorController_Direction_T;

/* Drive SubState use edge detection - DriveMode */
typedef enum MotorController_DriveId
{
    MOTOR_CONTROLLER_DRIVE_ZERO,
    MOTOR_CONTROLLER_DRIVE_THROTTLE,
    MOTOR_CONTROLLER_DRIVE_BRAKE,
    MOTOR_CONTROLLER_DRIVE_CMD,
}
MotorController_DriveId_T;

/* Blocking SubState/Function Id */
typedef enum MotorController_LockedId
{
    MOTOR_CONTROLLER_LOCK_PARK,
    MOTOR_CONTROLLER_LOCK_ENTER,
    MOTOR_CONTROLLER_LOCK_EXIT,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
    MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG,
    MOTOR_CONTROLLER_LOCK_REBOOT,
    // MOTOR_CONTROLLER_LOCK_NVM_SAVE_BOOT,
    // MOTOR_CONTROLLER_LOCK_NVM_WRITE_ONCE,
    // MOTOR_CONTROLLER_LOCK_NVM_READ_ONCE,
}
MotorController_LockId_T;

typedef enum MotorController_OptDinMode
{
    MOTOR_CONTROLLER_OPT_DIN_DISABLE,
    MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_SERVO,
    MOTOR_CONTROLLER_OPT_DIN_USER_FUNCTION,
}
MotorController_OptDinMode_T;

// todo split status and state
typedef union MotorController_StatusFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U; // ILimit by Heat
        uint16_t LowV               : 1U; // ILimit by LowV
        // uint16_t SpeedLimit         : 1U; // use active speed limit?
        // uint16_t ILimit          : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
        // uint16_t IsStopped          : 1U;
        uint16_t BuzzerEnable       : 1U;
    };
    uint16_t Word;
}
MotorController_StatusFlags_T;

/*
    Fault SubState flags
    Faults flags with exception of RxLost retain set state until user clears
*/
typedef union MotorController_FaultFlags
{
    struct
    {
        uint16_t PcbOverheat        : 1U;
        uint16_t MosfetsOverheat    : 1U;
        uint16_t VSourceLimit       : 1U;
        uint16_t VSenseLimit        : 1U;
        uint16_t VAccsLimit         : 1U;
        uint16_t Motors             : 1U;
        // uint16_t DirectionSync      : 1U;
        uint16_t RxLost             : 1U;
        uint16_t User               : 1U;
    };
    uint16_t Word;
}
MotorController_FaultFlags_T;

/*
    Init SubState
*/
typedef union MotorController_InitFlags
{
    struct
    {
        uint16_t IsThrottleZero : 1U;
        uint16_t IsDirectionSet : 1U;
        // uint16_t IsConfigLoaded : 1U;
    };
    uint16_t Word;
}
MotorController_InitFlags_T;

/* Buzzer Control */
typedef union MotorController_BuzzerFlags
{
    struct
    {
        uint16_t IsEnableOnBoot         : 1U; /* Primary Enable */
        // uint16_t OnInit              : 1U;
        // uint16_t OnDirectionChange   : 1U;
        // uint16_t OnReverse           : 2U; /* 0: Off, 1: Short Beep, 2: Continuous */
        // uint16_t OnInitThrottle      : 1U;
        // uint16_t ThrottleOnBrakeCmd;
        // uint16_t ThrottleOnBrakeRelease;
        // uint16_t ThrottleOnNeutralRelease;
    };
    uint16_t Word;
}
MotorController_BuzzerFlags_T;

/*
    MotorController Voltages
    MOTOR_STATIC.VMAX -> controller voltage max
    MotorController_User_GetVSourceRef(), Config.VSourceRef, Motor_Static.VSourceRef_V -> user set nominal voltage
    MotorController_User_GetVSource_V() -> live voltage
*/
typedef struct MotorController_Config
{
    uint16_t VSourceRef;        /* Source/Battery Voltage. Sync with Motor_Static VSourceRef_V */
    Motor_FeedbackMode_T DefaultCmdMode;
    MotorController_InitMode_T InitMode;
    MotorController_InputMode_T InputMode;
    MotorController_BrakeMode_T BrakeMode;
    MotorController_ThrottleMode_T ThrottleMode;
    MotorController_DriveZeroMode_T DriveZeroMode;
    MotorController_OptDinMode_T OptDinMode;
    uint16_t OptSpeedLimit_Scalar16;
    uint16_t OptILimit_Scalar16;
    uint16_t VLowILimit_Scalar16;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    uint8_t CanServicesId;
    bool CanIsEnable;
#endif
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    uint16_t BatteryZero_Adcu;
    uint16_t BatteryFull_Adcu;
#endif
    // MotorController_BuzzerFlags_T BuzzerFlags; /* which options are enabled for use */
}
MotorController_Config_T;

/*
    Allocated memory outside for less CONFIG define repetition
*/
typedef const struct MotorController_Const
{
    /*  */
    const MotorController_Config_T * const P_NVM_CONFIG;
    /*
        Modules
    */
    Motor_T * const P_MOTORS; const uint8_t MOTOR_COUNT;
    // Thermistor_T const P_THERMISTORS; const uint8_t THERMISTOR_COUNT;
    // VMonitor_T const P_VMONITORS; const uint8_t VMONITOR_COUNT;

// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE) || defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_FLASH)
    Flash_T * const P_FLASH;    /* Flash controller defined outside module, ensure flash config/params are in RAM */
// #endif
#if defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_EEPROM)
    EEPROM_T * const P_EEPROM;   /* Defined outside for regularity */
#endif
    AnalogN_T * const P_ANALOG_N;
    const MotAnalog_Conversions_T ANALOG_CONVERSIONS;
    // alternatively pass count by macro, allows aliasing in struct
    /* Simultaneous active serial */
    Serial_T * const P_SERIALS; const uint8_t SERIAL_COUNT;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * const P_CAN_BUS;
#endif

    Protocol_T * const P_PROTOCOLS; const uint8_t PROTOCOL_COUNT; /* Simultaneously active protocols */

    /*
        Static Config
    */
    // const uint8_t MAIN_VERSION[4U]; // alternatively pass using macro
    const uintptr_t MANUFACTURE_ADDRESS; const uint8_t MANUFACTURE_SIZE;
#if defined(CONFIG_MOTOR_CONTROLLER_USER_CONFIG_FLASH)
    const uintptr_t CONFIG_ADDRESS; const uint16_t CONFIG_SIZE;  /* Flash params start. */
    // alternatively use p_mc->CONST..P_FLASH->P_PARTITIONS[id]
    // NvMemory_Partition_T * P_CONFIG_PARTITION;
    // NvMemory_Partition_T * P_MANUFACTURE_PARTITION; /* pointer to partition table held by P_FLASH */
#endif
    const BootRef_T * const P_BOOT_REF;
    const uint32_t ANALOG_USER_DIVIDER;  /* In Pow2 - 1 */
    const uint32_t MAIN_DIVIDER_10;
    const uint32_t MAIN_DIVIDER_1000;
    const uint32_t TIMER_DIVIDER_1000;
}
MotorController_Const_T;

/*   */
typedef struct MotorController
{
    const MotorController_Const_T CONST;
    MotorController_Config_T Config;
    BootRef_T BootRef; /* Buffer */

    volatile MotAnalog_Results_T AnalogResults; // todo split for thermistor
#if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) // NDEBUG
    MotAnalog_Results_T FaultAnalogRecord;
#endif

    MotAnalogUser_T AnalogUser;
    Blinky_T Buzzer;
    Blinky_T Meter;
    Pin_T Relay;
    Debounce_T OptDin;     /* Configurable input */

    // init   as array todo
    Thermistor_T ThermistorPcb;
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    Thermistor_T ThermistorMosfetsTop;
    Thermistor_T ThermistorMosfetsBot;
#else
    Thermistor_T ThermistorMosfets;
#endif

    VMonitor_T VMonitorSource;  /* Controller Supply */
    VMonitor_T VMonitorSense;   /* ~5V */
    VMonitor_T VMonitorAccs;    /* ~12V */

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    Linear_T BatteryLife;       /* Battery Life percentage */
#endif
    Timer_T TimerMillis;
    uint32_t MainDividerCounter;
    uint32_t TimerDividerCounter;

#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
    Shell_T Shell;
    uint16_t ShellSubstate;
#endif

    /* State and SubState */
    StateMachine_T StateMachine;
    MotorController_StatusFlags_T StatusFlags;
    MotorController_InitFlags_T InitFlags;
    MotorController_FaultFlags_T FaultFlags;

    /* SubStates - effectively previous input */
    MotorController_DriveId_T DriveSubState;
    MotorController_LockId_T LockSubState;
    // int32_t UserCmdValue; /* Not needed unless comparing greater/less then */
    /* Async return status */
    // union
    // {
    NvMemory_Status_T NvmStatus; /* Common NvmStatus, e.g. EEPROM/Flash */
    //     Calibration_Status_T CalibrationStatus;
    // } AsyncStatus;
}
MotorController_T, * MotorControllerPtr_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline Motor_T * MotorController_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) { return &(p_mc->CONST.P_MOTORS[motorIndex]); }

/******************************************************************************/
/*
    Alarm
*/
/******************************************************************************/
static inline void MotorController_BeepShort(MotorController_T * p_mc)             { Blinky_Blink(&p_mc->Buzzer, 500U); }
static inline void MotorController_BeepPeriodicType1(MotorController_T * p_mc)     { Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U); }
static inline void MotorController_BeepDouble(MotorController_T * p_mc)            { Blinky_BlinkN(&p_mc->Buzzer, 250U, 250U, 2U); }

/******************************************************************************/
/*
   MotorN Array Functions - Proc by StateMachine

   todo move call by state machine to StateMachine
*/
/******************************************************************************/
// static inline void void_array_foreach(MotorController_T * p_mc, Motor_ProcVoid_T cmdFunction)                        { MotorN_Proc(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, cmdFunction); }
// static inline void _MotorController_SetCmdAll(MotorController_T * p_mc, Motor_SetInt16_T cmdFunction, int16_t userCmd)     { MOTOR_N_SET_EACH(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, cmdFunction, userCmd); }

static inline bool MotorController_IsAnyMotorFault(const MotorController_T * p_mc) { return void_array_is_any(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_StateMachine_IsFault); }
/* returns true if atleast 1 fault is cleared */
static inline bool MotorController_IsAnyClearMotorFault(MotorController_T * p_mc)  { return void_array_for_any(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_StateMachine_ClearFault); }

static inline void MotorController_DisableAll(MotorController_T * p_mc)        { void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_ForceDisableControl); }
static inline bool MotorController_TryReleaseAll(MotorController_T * p_mc)     { void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryRelease); }
// static inline bool MotorController_ActivateAll(MotorController_T * p_mc)    { MotorN_Proc(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_ActivateControl); }
static inline bool MotorController_TryHoldAll(MotorController_T * p_mc)        { void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryHold); }

static inline bool MotorController_TryDirectionForwardAll(MotorController_T * p_mc, MotorController_Direction_T direction) { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryDirectionForward); }
static inline bool MotorController_TryDirectionReverseAll(MotorController_T * p_mc, MotorController_Direction_T direction) { return void_array_for_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_User_TryDirectionReverse); }
static inline bool MotorController_CheckForwardAll(const MotorController_T * p_mc)     { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_User_IsDirectionForward); }
static inline bool MotorController_CheckReverseAll(const MotorController_T * p_mc)     { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_User_IsDirectionReverse); }
static inline bool MotorController_CheckStopAll(const MotorController_T * p_mc)        { return void_array_is_every(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_test_t)Motor_User_IsStopState); }

static inline bool MotorController_SetSystemILimitAll(MotorController_T * p_mc, uint16_t limit_scalar16)   { return struct_array_is_any_set_uint16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (try_uint16_t)Motor_TrySystemILimit, limit_scalar16); }
/* returns true if limit of id is cleared on at least 1 motor */
static inline bool MotorController_ClearSystemILimitAll(MotorController_T * p_mc)                          { return void_array_for_any(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_poll_t)Motor_ClearSystemILimit); }

// static inline void MotorController_SetSpeedLimitAll(MotorController_T * p_mc, uint16_t limit_scalar16)  { MotorN_User_SetScalar16(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_SetSpeedLimitActive, limit_scalar16); }
// static inline void MotorController_ClearSpeedLimitAll(MotorController_T * p_mc)                         { MotorN_Proc(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_ClearSpeedLimitActive); }
// static inline void MotorController_SetILimitAll(MotorController_T * p_mc, uint16_t limit_scalar16)      { MotorN_User_SetScalar16(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_SetILimitActive, limit_scalar16); }
// static inline void MotorController_ClearILimitAll(MotorController_T * p_mc)                             { MotorN_Proc(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_ClearILimitActive); }

// static inline bool MotorController_ClearSystemILimitAll(MotorController_T * p_mc)                          { return MotorN_User_ClearLimit(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_ClearILimitEntry, MOTOR_I_LIMIT_ACTIVE_MC); }
// static inline bool MotorController_SetSystemILimitAll(MotorController_T * p_mc, uint16_t limit_scalar16)   { return MotorN_User_SetLimit(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_SetILimitEntry, limit_scalar16, MOTOR_I_LIMIT_ACTIVE_MC); }

/* Default FeedbackMode store in MC, active mode in Motor */
// static inline void MotorController_StartCmdModeDefault(MotorController_T * p_mc)                               { MotorN_User_ActivateControl(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, p_mc->Config.DefaultCmdMode); }
// static inline void MotorController_StartCmdMode(MotorController_T * p_mc, Motor_FeedbackMode_T feedbackMode)   { MotorN_User_ActivateControl(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, feedbackMode);}
// static inline void MotorController_SetCmdModeValue(MotorController_T * p_mc, int16_t userCmd)                  { _MotorController_SetCmdAll(p_mc, Motor_User_SetActiveCmdValue, userCmd); }
// static inline void MotorController_StartCmdMode(MotorController_T * p_mc, Motor_FeedbackMode_T feedbackMode)   { MOTOR_N_SET_EACH(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_ActivateControl_Cast, feedbackMode.Word); }
// static inline void MotorController_SetCmdModeValue(MotorController_T * p_mc, int16_t userCmd)                  { MOTOR_N_SET_EACH(p_mc->CONST.P_MOTORS, p_mc->CONST.MOTOR_COUNT, Motor_User_SetActiveCmdValue, userCmd); }

static inline void MotorController_StartThrottleMode(MotorController_T * p_mc)
{
    switch (p_mc->Config.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetSpeedMode);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetTorqueMode);    break;
        default: break;
    }
    // MotorController_ActivateAll(p_mc);
}

static inline void MotorController_SetThrottleValue(MotorController_T * p_mc, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;
    switch (p_mc->Config.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  struct_array_set_all_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetSpeedCmdValue, cmdValue);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: struct_array_set_all_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)Motor_User_SetTorqueCmdValue, cmdValue);    break;
        default: break;
    }
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
// todo brake hold threshold
static inline void _SetBrakeCmd(Motor_T * p_motor, int16_t brake)
{
    if (Motor_User_GetSpeed_UFrac16(p_motor) > (UINT16_MAX / 40U)) // 2.5%
    {
        Motor_User_SetTorqueCmdValue(p_motor, (int32_t)0 - brake);
    }
    else
    {
        Motor_User_TryRelease(p_motor);
        // Motor_User_TryHold(p_motor);
        // Motor_User_SetVoltageModeCmd(p_motor);
    }
}

static inline void MotorController_StartBrakeMode(MotorController_T * p_mc)
{
    switch (p_mc->Config.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_SetTorqueMode); break;
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: break;
        default: break;
    }
    // MotorController_ActivateAll(p_mc);
}

static inline void MotorController_SetBrakeValue(MotorController_T * p_mc, uint16_t userCmdBrake)
{
    int16_t cmdValue = userCmdBrake / 2; // 32767 max
    switch (p_mc->Config.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: struct_array_set_all_int16(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (set_int16_t)_SetBrakeCmd, cmdValue); break;
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: break;
        default: break;
    }
}

static inline void MotorController_StartDriveZero(MotorController_T * p_mc)
{
    switch (p_mc->Config.DriveZeroMode)
    {
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT: MotorController_TryReleaseAll(p_mc); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_SetRegenMotorAll(p_mc); */ break;
            // case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: MotorController_SetCruiseMotorAll(p_mc); break;
        default: break;
    }
}

/*
    Check Stop / Zero Throttle
    Eventually release for stop transition
*/
static inline void MotorController_ProcDriveZero(MotorController_T * p_mc)
{
    switch (p_mc->Config.DriveZeroMode)
    {
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT: break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_ProcRegenMotorAll(p_mc); */ break;
            // case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: break;
        default: break;
    }

    //check for 0 speed, motor run does nto transition
    // if(MotorController_CheckStopAll(p_mc) == true)
    // {
    //     MotorController_TryHoldAll(p_mc);
    //     // p_mc->StatusFlags.IsStopped = 1U;
    // }
}

static inline void MotorController_CalibrateAdc(MotorController_T * p_mc)
{
    void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_CalibrateAdc);
    MotAnalogUser_SetThrottleZero(&p_mc->AnalogUser, p_mc->AnalogResults.Throttle_Adcu); // todo wait filter state
    MotAnalogUser_SetBrakeZero(&p_mc->AnalogUser, p_mc->AnalogResults.Brake_Adcu);
}

static inline void MotorController_CalibrateSensorAll(MotorController_T * p_mc)
{
    void_array_foreach(p_mc->CONST.P_MOTORS, sizeof(Motor_T), p_mc->CONST.MOTOR_COUNT, (void_op_t)Motor_User_CalibrateSensor);
}

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(MotorController_T * p_mc);
extern void MotorController_LoadConfigDefault(MotorController_T * p_mc);
extern void MotorController_ResetBootDefault(MotorController_T * p_mc);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
extern void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc);
#endif
extern void MotorController_SetAdcResultsNominal(MotorController_T * p_mc);
extern void MotorController_PollAdcFaultFlags(MotorController_T * p_mc);

extern NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer);
extern NvMemory_Status_T MotorController_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size);

#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE) && defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
extern void MotorController_ServoExtern_Start(MotorController_T * p_mc);
extern void MotorController_ServoExtern_Proc(MotorController_T * p_mc);
extern void MotorController_ServoExtern_SetCmd(MotorController_T * p_mc, int32_t cmd);
#else
static inline void MotorController_Servo_Start(MotorController_T * p_mc)
{

}

static inline void MotorController_Servo_Proc(MotorController_T * p_mc)
{

}

static inline void MotorController_Servo_SetCmd(MotorController_T * p_mc, uint32_t cmd)
{

}
#endif

#endif /* MOTOR_CONTROLLER_H */

// typedef enum MotorController_SpeedLimitActiveId
// {
//     MOTOR_CONTROLLER_SPEED_LIMIT_ACTIVE_DISABLE = 0U,
//     MOTOR_CONTROLLER_SPEED_LIMIT_ACTIVE_OPT = 1U, /* From parent class */
// }
// MotorController_SpeedLimitActiveId_T;

// typedef enum MotorController_ILimitActiveId
// {
//     MOTOR_CONTROLLER_I_LIMIT_ACTIVE_DISABLE = 0U,
//     MOTOR_CONTROLLER_I_LIMIT_ACTIVE_HEAT = 1U,
//     MOTOR_CONTROLLER_I_LIMIT_ACTIVE_LOW_V = 2U,
// }
// MotorController_ILimitActiveId_T;

// static inline bool MotorController_ProcOnDirection(MotorController_T * p_mc, MotorController_Direction_T direction)
// {
//     // if((p_mc->Config.BuzzerFlagsEnable.OnReverse == true))
//     // {
//     //     if(p_mc->DriveDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE)
//     //     {
//     //         MotorController_BeepPeriodicType1(p_mc);
//     //     }
//     //     else
//     //     {
//     //         Blinky_Stop(&p_mc->Buzzer);
//     //     }
//     // }
//
// }