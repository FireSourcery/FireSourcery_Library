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

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"
#include "Peripheral/Serial/Serial.h"
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
#include "Peripheral/CanBus/CanBus.h"
#endif

#include "Type/Array/struct_array.h"
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

/* Init Mode */
typedef enum MotorController_MainMode
{
    MOTOR_CONTROLLER_MAIN_MODE_DRIVE,
    MOTOR_CONTROLLER_MAIN_MODE_SERVO,
}
MotorController_MainMode_T;

typedef enum MotorController_InputMode
{
    MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
    MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
    MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum MotorController_BrakeMode
{
    MOTOR_CONTROLLER_BRAKE_MODE_PASSIVE,
    MOTOR_CONTROLLER_BRAKE_MODE_TORQUE,
    MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE,
}
MotorController_BrakeMode_T;

typedef enum MotorController_ThrottleMode
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

typedef enum MotorController_ServoMode
{
    MOTOR_CONTROLLER_SERVO_MODE_ENTER,
    MOTOR_CONTROLLER_SERVO_MODE_EXIT,
    // MOTOR_CONTROLLER_SERVO_MODE_SINGLE_SELECT,
    // POSITION_PUSE_TIME
}
MotorController_ServoMode_T;

/* Drive SubState use edge detection - DriveState */
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
    MOTOR_CONTROLLER_LOCK_PARK, /* convience for park first before lock */
    MOTOR_CONTROLLER_LOCK_ENTER,
    MOTOR_CONTROLLER_LOCK_EXIT,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
    MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG,
    // MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG, /* on Error read from Nvm to RAM */
    MOTOR_CONTROLLER_LOCK_REBOOT,
    MOTOR_CONTROLLER_LOCK_POLL_STATUS,
    // MOTOR_CONTROLLER_LOCK_ENTER_SERVO,
    // MOTOR_CONTROLLER_LOCK_ENTER_DRIVE,
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

typedef union MotorController_StatusFlags
{
    struct
    {
        uint16_t HeatWarning            : 1U; // ILimit by Heat
        uint16_t VLow                   : 1U; // ILimit by VLow
        // uint16_t SpeedLimit          : 1U; // use active speed limit?
        // uint16_t ILimit              : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
        // uint16_t IsStopped          : 1U;
        uint16_t BuzzerEnable          : 1U;
    };
    uint16_t Word;
}
MotorController_StateFlags_T;

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
        // uint16_t User               : 1U;
    };
    uint16_t Value;
}
MotorController_FaultFlags_T;

/*
    Init SubState
*/
typedef union MotorController_InitFlags
{
    struct
    {
        uint16_t ThrottleZero : 1U;
        uint16_t DirectionSet : 1U;
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
        uint16_t IsEnabled              : 1U; /* Primary Enable */
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
    uint16_t VSourceRef;    /* Source/Battery Voltage. Sync with Motor_Static VSourceRef_V */
    Motor_FeedbackMode_T DefaultCmdMode;
    MotorController_MainMode_T InitMode;
    MotorController_InputMode_T InputMode;
    MotorController_BrakeMode_T BrakeMode;
    MotorController_ThrottleMode_T ThrottleMode;
    MotorController_DriveZeroMode_T DriveZeroMode;
    MotorController_OptDinMode_T OptDinMode;
    uint16_t OptSpeedLimit_Fract16;
    uint16_t OptILimit_Fract16;
    uint16_t VLowILimit_Fract16;
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
    const uint8_t MAIN_VERSION[4U];

    /*  */
    const MotorController_Config_T * const P_NVM_CONFIG;
    /*
        Modules
    */
    Motor_T * const P_MOTORS; const uint8_t MOTOR_COUNT;
    // Thermistor_T const P_THERMISTORS; const uint8_t THERMISTOR_COUNT;
    // VMonitor_T const P_VMONITORS; const uint8_t VMONITOR_COUNT;

// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE) || defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
    Flash_T * const P_FLASH;    /* Flash controller defined outside module, ensure flash config/params are in RAM */
// #endif
#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_EEPROM)
    EEPROM_T * const P_EEPROM;   /* Defined outside for regularity */
#endif


    Analog_T * const P_ANALOG; /* pointer since it is shared */

    const Analog_Conversion_T CONVERSION_VSOURCE;
    const Analog_Conversion_T CONVERSION_VSENSE;
    const Analog_Conversion_T CONVERSION_VACCS;
    const Analog_Conversion_T CONVERSION_HEAT_PCB;
    /* COUNT is defined by macro. It is also needed to determine global channel index  */
    const Analog_Conversion_T HEAT_MOSFETS_CONVERSIONS[MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT];
    const Analog_Conversion_T CONVERSION_THROTTLE;
    const Analog_Conversion_T CONVERSION_BRAKE;

    // alternatively pass count by macro, allows aliasing in struct
    /* Simultaneous active serial */
    Serial_T * const P_SERIALS; const uint8_t SERIAL_COUNT;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * const P_CAN_BUS;
#endif

    Protocol_T * const P_PROTOCOLS; const uint8_t PROTOCOL_COUNT; /* Simultaneously active protocols */
    uint8_t USER_PROTOCOL_INDEX; /* The corresponding Xcvr will not be changed for now */

    /*
        Static Config
    */
    // const uint8_t MAIN_VERSION[4U]; // alternatively pass using macro
    const uintptr_t MANUFACTURE_ADDRESS; const uint8_t MANUFACTURE_SIZE;
#if defined(CONFIG_MOTOR_CONTROLLER_USER_NVM_FLASH)
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

//     volatile MotAnalog_Results_T AnalogResults;
// #if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) // NDEBUG
//     MotAnalog_Results_T FaultAnalogRecord;
// #endif

    MotAnalogUser_T AnalogUser;
    Blinky_T Buzzer;
    Blinky_T Meter;
    Pin_T Relay;
    Debounce_T OptDin;     /* Configurable input */

    Thermistor_T ThermistorPcb;
    Thermistor_T MosfetsThermistors[MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT];

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
    MotorController_StateFlags_T StateFlags;
    MotorController_FaultFlags_T FaultFlags; /* Fault SubState */
    MotorController_InitFlags_T InitFlags;

    /* SubStates - effectively previous input */
    MotorController_DriveId_T DriveSubState;
    MotorController_LockId_T LockSubState;
    uint8_t CmdMotorId; /* for VarId, Value input mode only */
    // bool isAsyncStatusAvailable; /* Async return status */
    // uint8_t CalibrationStatus;
    /* Async return status */
    // union
    // {
    NvMemory_Status_T NvmStatus; /* Common NvmStatus, e.g. EEPROM/Flash */
    uint8_t CalibrationStatus;
        // Calibration_Status_T CalibrationStatus;
    // } AsyncStatus;
    // int32_t UserCmdValue; /* Not needed unless comparing greater/less then */
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
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(MotorController_T * p_mc);
extern void MotorController_LoadConfigDefault(MotorController_T * p_mc);
extern void MotorController_ResetBootDefault(MotorController_T * p_mc);
extern void MotorController_ResetVSourceActiveRef(MotorController_T * p_mc);
extern void MotorController_ResetVSourceMonitorDefaults(MotorController_T * p_mc);
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
extern void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc);
#endif

extern NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_ReadManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, uint8_t size, uint8_t * p_destBuffer);
extern NvMemory_Status_T MotorController_WriteManufacture_Blocking(MotorController_T * p_mc, uintptr_t onceAddress, const uint8_t * p_sourceBuffer, uint8_t size);

extern void MotorController_CalibrateAdc(MotorController_T * p_mc);
extern void MotorController_CalibrateSensorAll(MotorController_T * p_mc);

#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE) && defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
extern void MotorController_ServoExtern_Start(MotorController_T * p_mc);
extern void MotorController_ServoExtern_Proc(MotorController_T * p_mc);
extern void MotorController_ServoExtern_SetCmd(MotorController_T * p_mc, int32_t cmd);
#endif

// #if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE) && !defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
//  void MotorController_Servo_Start(MotorController_T * p_mc);
//  void MotorController_Servo_Proc(MotorController_T * p_mc);
//  void MotorController_Servo_SetCmd(MotorController_T * p_mc, uint32_t cmd);
// #endif
extern bool MotorController_IsEveryMotorForward(const MotorController_T * p_mc);
extern bool MotorController_IsEveryMotorReverse(const MotorController_T * p_mc);
extern bool MotorController_IsEveryMotorStopState(const MotorController_T * p_mc);
extern bool MotorController_IsEveryMotorRunState(const MotorController_T * p_mc);

extern bool MotorController_IsAnyMotorFault(const MotorController_T * p_mc);
extern bool MotorController_ForEveryMotorExitFault(MotorController_T * p_mc);
extern void MotorController_ForceDisableAll(MotorController_T * p_mc);
extern bool MotorController_TryReleaseAll(MotorController_T * p_mc);
extern bool MotorController_TryHoldAll(MotorController_T * p_mc);
extern bool MotorController_TryDirectionForwardAll(MotorController_T * p_mc, MotorController_Direction_T direction);
extern bool MotorController_TryDirectionReverseAll(MotorController_T * p_mc, MotorController_Direction_T direction);
extern void MotorController_SetSpeedLimitAll(MotorController_T * p_mc, Motor_SpeedLimitId_T id, uint16_t limit_fract16);
extern void MotorController_ClearSpeedLimitAll(MotorController_T * p_mc, Motor_SpeedLimitId_T id);
extern void MotorController_SetSpeedLimitAll_Scalar(MotorController_T * p_mc, Motor_SpeedLimitId_T id, uint16_t scalar_fract16);
extern void MotorController_SetILimitAll(MotorController_T * p_mc, Motor_ILimitId_T id, uint16_t limit_fract16);
extern void MotorController_SetILimitAll_Scalar(MotorController_T * p_mc, Motor_ILimitId_T id, uint16_t scalar_fract16);
extern void MotorController_ClearILimitAll(MotorController_T * p_mc, Motor_ILimitId_T id);

extern void MotorController_StartControlModeAll(MotorController_T * p_mc, Motor_FeedbackMode_T feedbackMode);
extern void MotorController_SetFeedbackModeAll_Cast(MotorController_T * p_mc, uint8_t feedbackMode);
extern void MotorController_SetCmdValueAll(MotorController_T * p_mc, int16_t userCmd);

extern void MotorController_StartThrottleMode(MotorController_T * p_mc);
extern void MotorController_SetThrottleValue(MotorController_T * p_mc, uint16_t userCmdThrottle);
extern void MotorController_StartBrakeMode(MotorController_T * p_mc);
extern void MotorController_SetBrakeValue(MotorController_T * p_mc, uint16_t userCmdBrake);
extern void MotorController_StartDriveZero(MotorController_T * p_mc);
extern void MotorController_ProcDriveZero(MotorController_T * p_mc);

#endif /* MOTOR_CONTROLLER_H */


