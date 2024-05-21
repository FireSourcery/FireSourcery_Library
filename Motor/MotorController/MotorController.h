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

#include "Motor/Motor/Motor_Params.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/MotorN_User.h"

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
    MOTOR_CONTROLLER_INPUT_MODE_SERIAL_ONLY,
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

typedef enum MotorController_OptDinFunction
{
    MOTOR_CONTROLLER_OPT_DIN_DISABLE,
    MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_SERVO,
    MOTOR_CONTROLLER_OPT_DIN_USER_FUNCTION,
}
MotorController_OptDinMode_T;

/* Drive SubState */
typedef enum MotorController_Direction
{
    MOTOR_CONTROLLER_DIRECTION_PARK,
    MOTOR_CONTROLLER_DIRECTION_NEUTRAL,
    MOTOR_CONTROLLER_DIRECTION_FORWARD,
    MOTOR_CONTROLLER_DIRECTION_REVERSE,
    MOTOR_CONTROLLER_DIRECTION_ERROR = 255U,
}
MotorController_Direction_T;

/* Drive SubState use edge detection */
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
    MOTOR_CONTROLLER_LOCKED_ENTER,
    MOTOR_CONTROLLER_LOCKED_EXIT,
    MOTOR_CONTROLLER_LOCKED_CALIBRATE_SENSOR,
    MOTOR_CONTROLLER_LOCKED_CALIBRATE_ADC,
    MOTOR_CONTROLLER_LOCKED_NVM_SAVE_PARAMS,
    MOTOR_CONTROLLER_LOCKED_REBOOT,
    // MOTOR_CONTROLLER_LOCKED_NVM_SAVE_BOOT,
    // MOTOR_CONTROLLER_LOCKED_NVM_WRITE_ONCE,
    // MOTOR_CONTROLLER_LOCKED_NVM_READ_ONCE,
}
MotorController_LockedId_T;

typedef union MotorController_StatusFlags
{
    struct
    {
        uint16_t HeatWarning        : 1U; // ILimit by Heat
        uint16_t LowV               : 1U; // ILimit by LowV
        uint16_t SpeedLimit         : 1U; // use active speed limit?
        // uint16_t ILimit          : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
        // uint16_t IsStopped          : 1U;
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
        // uint16_t IsParamsLoaded : 1U;
    };
    uint16_t Word;
}
MotorController_InitFlags_T;

// typedef union MotorController_BuzzerOptionsFlags
// {
//     struct
//     {
//         uint16_t OnInit              : 1U;
//         uint16_t OnDirectionChange   : 1U;
//         uint16_t ThrottleOnInit      : 1U;
//         uint16_t OnReverse           : 2U; /* 0: Off, 1: Short Beep, 2: Continuous */
//         // uint16_t ThrottleOnBrakeCmd;
//         // uint16_t ThrottleOnBrakeRelease;
//         // uint16_t ThrottleOnNeutralRelease;
//     };
//     uint16_t Word;
// }
// MotorController_BuzzerFlags_T;

/*
    MotorController Voltages
    GLOBAL_MOTOR.VMAX -> controller voltage max
    MotorController_User_GetVSourceRef(), Params.VSourceRef, Global_Motor.VSourceRef_V -> user set nominal voltage
    MotorController_User_GetVSource_V() -> live voltage
*/
typedef struct MotorController_Params
{
    uint16_t VSourceRef;        /* Source/Battery Voltage. Sync with Global_Motor VSourceRef_V */
    Motor_FeedbackMode_T DefaultCmdMode;
    MotorController_InitMode_T InitMode;
    MotorController_InputMode_T InputMode;
    MotorController_BrakeMode_T BrakeMode;
    MotorController_ThrottleMode_T ThrottleMode;
    MotorController_DriveZeroMode_T DriveZeroMode;
    MotorController_OptDinMode_T OptDinMode;
    uint16_t OptDinSpeedLimit_Scalar16;
    uint16_t ILimitLowV_Scalar16;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    uint8_t CanServicesId;
    bool CanIsEnable;
#endif
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    uint16_t BatteryZero_Adcu;  // todo, use Vsource warning?
    uint16_t BatteryFull_Adcu;
#endif
    // MotorController_BuzzerFlags_T BuzzerFlagsEnable; /* which options are enabled for use */
}
MotorController_Params_T;


/*
    Allocated memory outside for less CONFIG define repetition
*/
typedef const struct MotorController_Config
{
    const MotorController_Params_T * const P_PARAMS_NVM;
    // const uint8_t MAIN_VERSION[4U]; //alternatively pass using macro

// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE) || defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
    Flash_T * const P_FLASH;    /* Flash controller defined outside module, ensure flash config/params are in RAM */
// #endif
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
    EEPROM_T * const P_EEPROM;   /* Defined outside for regularity */
#endif
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
    const uintptr_t PARAMS_ADDRESS; const uint16_t PARAMS_SIZE;  /* Flash params start */
    // alternatively use p_mc->CONFIG..P_FLASH->P_PARTITIONS[id]
    // NvMemory_Partition_T * P_PARAMS_PARTITION;
    // NvMemory_Partition_T * P_MANUFACTURE_PARTITION; /* pointer to partition table held by P_FLASH */
#endif
    const uintptr_t MANUFACTURE_ADDRESS; const uint8_t MANUFACTURE_SIZE;
    const BootRef_T * const P_BOOT_REF;

    AnalogN_T * const P_ANALOG_N;
    const MotAnalog_Conversions_T ANALOG_CONVERSIONS;

    // alternatively pass count by macro, allows aliasing in struct
    MotorPtr_T const P_MOTORS; const uint8_t MOTOR_COUNT;

    // Thermistor_T const P_THERMISTORS; const uint8_t THERMISTOR_COUNT;
    // VMonitor_T const P_VMONITORS; const uint8_t VMONITOR_COUNT;

    /* Simultaneous active serial */
    Serial_T * const P_SERIALS; const uint8_t SERIAL_COUNT;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * const P_CAN_BUS;
#endif
    Protocol_T * const P_PROTOCOLS; const uint8_t PROTOCOL_COUNT; /* Simultaneously active protocols */
    const uint32_t ANALOG_USER_DIVIDER;  /* In Pow2 - 1 */
    const uint32_t MAIN_DIVIDER_10;
    const uint32_t MAIN_DIVIDER_1000;
    const uint32_t TIMER_DIVIDER_1000;
}
MotorController_Config_T;

/*   */
typedef struct MotorController
{
    const MotorController_Config_T CONFIG;
    MotorController_Params_T Parameters;
    BootRef_T BootRef; /* Buffer */
    volatile MotAnalog_Results_T AnalogResults; // todo split for thermistor

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
#if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
    MotAnalog_Results_T FaultAnalogRecord;
#endif
    /* SubStates - effectively previous input */
    MotorController_DriveId_T DriveSubState;
    MotorController_LockedId_T LockSubState;
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
static inline MotorPtr_T MotorController_GetPtrMotor(const MotorControllerPtr_T p_mc, uint8_t motorIndex) { return &(p_mc->CONFIG.P_MOTORS[motorIndex]); }

/******************************************************************************/
/*
    Alarm
*/
/******************************************************************************/
static inline void MotorController_BeepShort(MotorControllerPtr_T p_mc)             { Blinky_Blink(&p_mc->Buzzer, 500U); }
static inline void MotorController_BeepPeriodicType1(MotorControllerPtr_T p_mc)     { Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U); }
static inline void MotorController_BeepDouble(MotorControllerPtr_T p_mc)            { Blinky_BlinkN(&p_mc->Buzzer, 250U, 250U, 2U); }

/******************************************************************************/
/*
   MotorN Array Functions - Proc by StateMachine
*/
/******************************************************************************/
static inline void _MotorController_ProcAll(MotorControllerPtr_T p_mc, Motor_User_ProcVoid_T cmdFunction) { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, cmdFunction); }
static inline void _MotorController_SetCmdAll(MotorControllerPtr_T p_mc, Motor_User_SetCmd_T cmdFunction, int16_t userCmd) { MotorN_User_SetCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, cmdFunction, userCmd); }

/* returns true if no faults remain active */
static inline bool MotorController_ClearMotorsFaultAll(MotorControllerPtr_T p_mc) { return MotorN_User_ProcStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_StateMachine_ClearFault); }
static inline bool MotorController_CheckMotorsFaultAny(const MotorControllerPtr_T p_mc) { return MotorN_User_CheckStatusAny(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_StateMachine_IsFault); }

static inline void MotorController_DisableAll(MotorControllerPtr_T p_mc)    { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_DisableControl); }
static inline void MotorController_ReleaseAll(MotorControllerPtr_T p_mc)    { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ReleaseControl); }
// static inline void MotorController_ActivateAll(MotorControllerPtr_T p_mc)   { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ActivateControl); }
static inline void MotorController_TryHoldAll(MotorControllerPtr_T p_mc)    { MotorN_User_ProcStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_TryHold); }

static inline bool MotorController_TryDirectionForwardAll(MotorControllerPtr_T p_mc, MotorController_Direction_T direction) { return MotorN_User_ProcStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_TryDirectionForward); }
static inline bool MotorController_TryDirectionReverseAll(MotorControllerPtr_T p_mc, MotorController_Direction_T direction) { return MotorN_User_ProcStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_TryDirectionReverse); }
static inline bool MotorController_CheckForwardAll(const MotorControllerPtr_T p_mc)  { return MotorN_User_CheckStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_IsDirectionForward); }
static inline bool MotorController_CheckReverseAll(const MotorControllerPtr_T p_mc)  { return MotorN_User_CheckStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_IsDirectionReverse); }
static inline bool MotorController_CheckStopAll(const MotorControllerPtr_T p_mc)     { return MotorN_User_CheckStatusAll(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_IsStop); }

static inline void MotorController_SetSpeedLimitAll(MotorControllerPtr_T p_mc, uint16_t limit_scalar16)  { MotorN_User_SetScalar16(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_SetSpeedLimitActive, limit_scalar16); }
static inline void MotorController_ClearSpeedLimitAll(MotorControllerPtr_T p_mc)                         { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ClearSpeedLimitActive); }
static inline void MotorController_SetILimitAll(MotorControllerPtr_T p_mc, uint16_t limit_scalar16)      { MotorN_User_SetScalar16(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_SetILimitActive, limit_scalar16); }
static inline void MotorController_ClearILimitAll(MotorControllerPtr_T p_mc)                             { MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ClearILimitActive); }

/* returns true if limit of id is cleared on at least 1 motor */
static inline bool MotorController_ClearILimitAll_Id(MotorControllerPtr_T p_mc)                          { return MotorN_User_ClearLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_ClearILimitActive_Id, MOTOR_I_LIMIT_ACTIVE_MC); }
static inline bool MotorController_SetILimitAll_Id(MotorControllerPtr_T p_mc, uint16_t limit_scalar16)   { return MotorN_User_SetLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_SetILimitActive_Id, limit_scalar16, MOTOR_I_LIMIT_ACTIVE_MC); }

/* Default FeedbackMode store in MC, active mode in Motor */
static inline void MotorController_StartCmdModeDefault(MotorControllerPtr_T p_mc)                               { MotorN_User_ActivateControl(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, p_mc->Parameters.DefaultCmdMode); }
static inline void MotorController_StartCmdMode(MotorControllerPtr_T p_mc, Motor_FeedbackMode_T feedbackMode)   { MotorN_User_ActivateControl(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, feedbackMode);}
static inline void MotorController_SetCmdModeValue(MotorControllerPtr_T p_mc, int16_t userCmd)                  { MotorN_User_SetCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, Motor_User_SetActiveCmdValue, userCmd); }
// static inline void MotorController_SetCmdModeValue(MotorControllerPtr_T p_mc, int16_t userCmd)               { _MotorController_SetCmdAll(p_mc, Motor_User_SetActiveCmdValue, userCmd); }

static inline void MotorController_StartThrottleMode(MotorControllerPtr_T p_mc)
{
    switch(p_mc->Parameters.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  _MotorController_ProcAll(p_mc, Motor_User_SetSpeedMode);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: _MotorController_ProcAll(p_mc, Motor_User_SetTorqueMode);    break;
        default: break;
    }
    // MotorController_ActivateAll(p_mc);
}

static inline void MotorController_SetThrottleValue(MotorControllerPtr_T p_mc, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;
    switch(p_mc->Parameters.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  _MotorController_SetCmdAll(p_mc, Motor_User_SetSpeedCmdValue, cmdValue);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: _MotorController_SetCmdAll(p_mc, Motor_User_SetTorqueCmdValue, cmdValue);    break;
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
static inline void _SetBrakeCmd(MotorPtr_T p_motor, int16_t brake)
{
    if(Motor_User_GetSpeed_UFrac16(p_motor) >  (INT16_MAX * .05))
    {
        Motor_User_SetTorqueCmdValue(p_motor, (int32_t)0 - brake);
    }
    else
    {
        Motor_User_ReleaseControl(p_motor);
        // Motor_User_TryHold(p_motor);
        // Motor_User_SetVoltageModeCmd(p_motor);
    }
}

static inline void MotorController_StartBrakeMode(MotorControllerPtr_T p_mc)
{
    switch(p_mc->Parameters.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: _MotorController_ProcAll(p_mc, Motor_User_SetTorqueMode); break;
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: break;
        default: break;
    }
    // MotorController_ActivateAll(p_mc);
}

static inline void MotorController_SetBrakeValue(MotorControllerPtr_T p_mc, uint16_t userCmdBrake)
{
    int16_t cmdValue = userCmdBrake / 2;
    switch(p_mc->Parameters.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: _MotorController_SetCmdAll(p_mc, _SetBrakeCmd, cmdValue);
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE: break;
        default: break;
    }
}

static inline void MotorController_StartDriveZero(MotorControllerPtr_T p_mc)
{
    switch(p_mc->Parameters.DriveZeroMode)
    {
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_FLOAT: MotorController_ReleaseAll(p_mc); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_REGEN: /* MotorController_SetRegenMotorAll(p_mc); */ break;
        // case MOTOR_CONTROLLER_DRIVE_ZERO_MODE_CRUISE: MotorController_SetCruiseMotorAll(p_mc); break;
        default: break;
    }
}

/*
    Check Stop / Zero Throttle
    Eventually release for stop transition
*/
static inline void MotorController_ProcDriveZero(MotorControllerPtr_T p_mc)
{
    switch(p_mc->Parameters.DriveZeroMode)
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

static inline void MotorController_CalibrateAdc(MotorControllerPtr_T p_mc)
{
    _MotorController_ProcAll(p_mc, Motor_User_CalibrateAdc);
    MotAnalogUser_SetThrottleZero(&p_mc->AnalogUser, p_mc->AnalogResults.Throttle_Adcu); // todo wait filter state
    MotAnalogUser_SetBrakeZero(&p_mc->AnalogUser, p_mc->AnalogResults.Brake_Adcu);
}

// static inline void MotorController_CalibrateSensorAll(MotorControllerPtr_T p_mc)
// {
//     // _MotorController_ProcAll(p_mc, Motor_User_CalibrateSensor);
// }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(MotorControllerPtr_T p_mc);
extern void MotorController_LoadParamsDefault(MotorControllerPtr_T p_mc);
extern void MotorController_ResetBootDefault(MotorControllerPtr_T p_mc);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
extern void MotorController_ResetUnitsBatteryLife(MotorControllerPtr_T p_mc);
#endif
extern void MotorController_SetAdcResultsNominal(MotorControllerPtr_T p_mc);
extern void MotorController_PollAdcFaultFlags(MotorControllerPtr_T p_mc);

extern NvMemory_Status_T MotorController_SaveParameters_Blocking(MotorControllerPtr_T p_mc);
extern NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorControllerPtr_T p_mc);
extern NvMemory_Status_T MotorController_ReadManufacture_Blocking(MotorControllerPtr_T p_mc, uint8_t * p_destBuffer, uintptr_t onceAddress, uint8_t size);
extern NvMemory_Status_T MotorController_WriteManufacture_Blocking(MotorControllerPtr_T p_mc, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size);


#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE) && defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
extern void MotorController_ServoExtern_Start(MotorControllerPtr_T p_mc);
extern void MotorController_ServoExtern_Proc(MotorControllerPtr_T p_mc);
extern void MotorController_ServoExtern_SetCmd(MotorControllerPtr_T p_mc, int32_t cmd);
#else
static inline void MotorController_Servo_Start(MotorControllerPtr_T p_mc)
{

}

static inline void MotorController_Servo_Proc(MotorControllerPtr_T p_mc)
{

}

static inline void MotorController_Servo_SetCmd(MotorControllerPtr_T p_mc, uint32_t cmd)
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

// static inline bool MotorController_ProcOnDirection(MotorControllerPtr_T p_mc, MotorController_Direction_T direction)
// {
//     // if((p_mc->Parameters.BuzzerFlagsEnable.OnReverse == true))
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