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

#include "MotNvm/MotNvm.h"
#include "MotAnalogUser/MotAnalogUser.h"
#include "MotLimits/MotLimits.h"
#include "MotDrive/MotDrive.h"

#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_StateMachine.h"
#include "Motor/Motor/Motor_Array.h"
#include "Motor/Motor/MotorTimeRef.h"
// #include "Motor/Motor/Motor_Include.h"

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

#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Protocol/Protocol.h"
#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
#include "Utility/Shell/Shell.h"
#endif
#include "Utility/BootRef/BootRef.h"
#include "Type/Array/struct_array.h"

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
    MOTOR_CONTROLLER_DIRECTION_ERROR,
}
MotorController_Direction_T;

typedef enum MotorController_ServoMode
{
    MOTOR_CONTROLLER_SERVO_MODE_ENTER,
    MOTOR_CONTROLLER_SERVO_MODE_EXIT,
    // POSITION_PUSE_TIME
}
MotorController_ServoMode_T;

/* Drive SubState use edge detection - DriveState */
typedef enum MotorController_DriveId
{
    MOTOR_CONTROLLER_DRIVE_RELEASE,
    MOTOR_CONTROLLER_DRIVE_THROTTLE,
    MOTOR_CONTROLLER_DRIVE_BRAKE,
    MOTOR_CONTROLLER_DRIVE_CMD,
}
MotorController_DriveId_T;

/* Blocking SubState/Function Id */
typedef enum MotorController_LockId
{
    MOTOR_CONTROLLER_LOCK_ENTER,
    MOTOR_CONTROLLER_LOCK_EXIT,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR,
    // MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR_REF,
    MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
    MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG,
    // MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG, /* on Error read from Nvm to RAM */
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

// typedef union MotorController_State_Common
typedef union MotorController_StateFlags
{
    struct
    {
        // these can move to sub module edge polling
        uint16_t HeatWarning            : 1U; // ILimit by Heat
        uint16_t VSourceLow             : 1U; // ILimit by VSourceLow
        // uint16_t SpeedLimit          : 1U;
        // uint16_t ILimit              : 1U;
        // derive from thermistor functions
        // uint16_t ILimitHeatMosfets  : 1U;
        // uint16_t ILimitHeatPcb      : 1U;
        // uint16_t ILimitHeatMotors   : 1U;
        // uint16_t IsStopped          : 1U;
        uint16_t BuzzerEnable          : 1U;
    };
    uint16_t Value;
}
MotorController_StateFlags_T;

/*
    Fault SubState flags
    Faults flags with exception of RxLost retain set state until user clears
*/
// typedef union MotorController_State_Fault
typedef union MotorController_FaultFlags
{
    struct
    {
        uint16_t PcbOverheat        : 1U;
        uint16_t MosfetsOverheat    : 1U;
        uint16_t VSourceLimit       : 1U; /* VSourceMonitor for over/under  */
        uint16_t VSenseLimit        : 1U;
        uint16_t VAccsLimit         : 1U;
        // uint16_t ILimit             : 1U;
        uint16_t Motors             : 1U; /* Sensor/StartUp */
        uint16_t RxLost             : 1U;
        // uint16_t StateError         : 1U;
        uint16_t InitCheck          : 1U;
        // uint16_t DirectionSync      : 1U;
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
    uint16_t Value;
}
MotorController_InitFlags_T;

/* Buzzer Config  options enabled for use */
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
    uint16_t Value;
}
MotorController_BuzzerFlags_T;



/*
    MotorController Voltages
    MOTOR_ANALOG_REFERENCE.V_MAX -> ADC Saturation
    MOTOR_ANALOG_REFERENCE.V_RATED -> controller voltage max
    Config.VSourceRef -> user set nominal voltage
    MotorAnalogReference.VSourceRef_V-> live voltage
*/
typedef struct MotorController_Config
{
    uint16_t VSourceRef;    /* VMonitor.Nominal Source/Battery Voltage. Sync with MotorAnalogReference VSourceRef_V */
    // Motor_FeedbackMode_T DefaultCmdMode;
    MotorController_MainMode_T InitMode;
    MotorController_InputMode_T InputMode;
    MotorController_BrakeMode_T BrakeMode;
    MotorController_ThrottleMode_T ThrottleMode;
    MotorController_DriveZeroMode_T DriveZeroMode;
    MotorController_OptDinMode_T OptDinMode;
// MotorController_BuzzerFlags_T BuzzerFlags;
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
}
MotorController_Config_T;


/*
    Allocated memory outside
*/
typedef const struct MotorController_Const
{
    const uint8_t MAIN_VERSION[4U];
    /*  */
    const MotorController_Config_T * const P_CONFIG;

    /*
        Modules
    */
    const Motor_Array_T MOTORS;
    const MotNvm_T MOT_NVM; /* Non-volatile Memory controller */

    Analog_T * const P_ANALOG;

    /* Conversions */
    // const MotAnalogHeat_T MOT_ANALOG_HEAT; /* Heat monitoring */
    // const MotAnalogVMonitor_T MOT_ANALOG_VMONITOR;
    // const MotAnalogUser_T MOT_ANALOG_USER;

    const Analog_Conversion_T CONVERSION_VSOURCE;
    const Analog_Conversion_T CONVERSION_VSENSE;
    const Analog_Conversion_T CONVERSION_VACCS;
    const Analog_Conversion_T CONVERSION_HEAT_PCB;
    /* COUNT is defined by macro. It is also needed to determine global channel index  */
    //change to pointer on move
    const Analog_Conversion_T HEAT_MOSFETS_CONVERSIONS[MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT];
    const Analog_Conversion_T CONVERSION_THROTTLE;
    const Analog_Conversion_T CONVERSION_BRAKE;

    /* Simultaneous active serial */
    Serial_T * const P_SERIALS; const uint8_t SERIAL_COUNT;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * const P_CAN_BUS;
#endif

    Protocol_T * const P_PROTOCOLS; const uint8_t PROTOCOL_COUNT; /* Simultaneously active protocols */
    uint8_t USER_PROTOCOL_INDEX; /* The corresponding Xcvr will not be changed for now */

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

    // Analog_T Analog;
    Thermistor_T ThermistorPcb;
    Thermistor_T MosfetsThermistors[MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT];

    VMonitor_T VMonitorSource;  /* Controller Supply */
    VMonitor_T VMonitorSense;   /* V Analog Sensors ~5V */
    VMonitor_T VMonitorAccs;    /* ~12V */

    MotAnalogUser_T AnalogUser; /* drive input */
    Debounce_T OptDin;     /* Configurable input */
    Blinky_T Buzzer;
    Blinky_T Meter;
    Pin_T Relay;

    MotLimits_T MotLimits;

    Timer_T TimerMillis;
    uint32_t MainDividerCounter;
    uint32_t TimerDividerCounter;
    uint32_t StateCounter;
    uint32_t ControlCounter;

    /* State and SubState */
    StateMachine_T StateMachine;
    MotorController_StateFlags_T StateFlags;
    MotorController_FaultFlags_T FaultFlags; /* Fault SubState */
    MotorController_InitFlags_T InitFlags;
    /* SubStates - effectively previous input */
    MotorController_DriveId_T DriveSubState; // change for full values
    MotorController_LockId_T LockSubState;

    /* Input Image / Polling State */
    // int32_t UserCmdValue;
    // Motor_FeedbackMode_T UserCmdMode;

    // uint8_t CmdMotorId; /* for VarId, Value input mode only */
    /* Async return status */
    // union
    // {
    NvMemory_Status_T NvmStatus; /* Common NvmStatus, e.g. EEPROM/Flash */
    uint8_t LockOpStatus; /* async status */
    // Calibration_Status_T CalibrationStatus;
    // } AsyncStatus;

    Filter_T AvgBuffer0;
    Filter_T AvgBuffer1;

#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
    Shell_T Shell;
    uint16_t ShellSubState;
#endif
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
    Linear_T BatteryLife;       /* Battery Life percentage */
#endif
}
MotorController_T, * MotorControllerPtr_T;



/******************************************************************************/
/*

*/
/******************************************************************************/
static inline Motor_T * MotorController_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) { return &(p_mc->CONST.MOTORS.P_ARRAY[motorIndex]); }


/******************************************************************************/
/*
    Buzzer
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
extern void MotorController_CaptureVSourceActiveRef(MotorController_T * p_mc);
extern void MotorController_ResetVSourceMonitorDefaults(MotorController_T * p_mc);

#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
extern void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc);
#endif

extern void MotorController_StartThrottleMode(MotorController_T * p_mc);
extern void MotorController_SetThrottleValue(MotorController_T * p_mc, uint16_t userCmdThrottle);
extern void MotorController_StartBrakeMode(MotorController_T * p_mc);
extern void MotorController_SetBrakeValue(MotorController_T * p_mc, uint16_t userCmdBrake);
extern void MotorController_StartDriveZero(MotorController_T * p_mc);
extern void MotorController_ProcDriveZero(MotorController_T * p_mc);

extern NvMemory_Status_T MotorController_SaveConfig_Blocking(MotorController_T * p_mc);

#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE) && defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
extern void MotorController_ServoExtern_Start(MotorController_T * p_mc);
extern void MotorController_ServoExtern_Proc(MotorController_T * p_mc);
extern void MotorController_ServoExtern_SetCmd(MotorController_T * p_mc, int32_t cmd);
#endif

#endif /* MOTOR_CONTROLLER_H */

