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

#include "System/MemMapBoot/MemMapBoot.h"

#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <string.h>

typedef enum MotorController_Direction_Tag
{
    // MOTOR_CONTROLLER_DIRECTION_DISABLED,
    MOTOR_CONTROLLER_DIRECTION_NEUTRAL,
    MOTOR_CONTROLLER_DIRECTION_REVERSE,
    MOTOR_CONTROLLER_DIRECTION_FORWARD,
}
MotorController_Direction_T;

typedef enum MotorController_InputMode_Tag
{
    MOTOR_CONTROLLER_INPUT_MODE_DISABLE,
    MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
    MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL,
    MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum MotorController_ZeroCmdMode_Tag
{
    MOTOR_CONTROLLER_ZERO_CMD_MODE_FLOAT,       /* "Coast". MOSFETS non conducting. Same as neutral. */
    MOTOR_CONTROLLER_ZERO_CMD_MODE_REGEN,       /* Regen Brake */
    MOTOR_CONTROLLER_ZERO_CMD_MODE_CRUISE,      /* Voltage following, Zero currrent/torque */
    MOTOR_CONTROLLER_ZERO_CMD_MODE_ZERO,        /* Setpoint Zero. No overwrite */
}
MotorController_ZeroCmdMode_T;

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
}
MotorController_ThrottleMode_T;

/* For edge detection */
typedef enum
{
    MOTOR_CONTROLLER_INPUT_BRAKE,
    MOTOR_CONTROLLER_INPUT_THROTTLE,
    MOTOR_CONTROLLER_INPUT_ZERO,
}
MotorController_InputState_T;

/* Blocking Functions Ids */
typedef enum MotorController_OperationId_Tag
{
    MOTOR_CONTROLLER_ENTER_CRITICAL,
    MOTOR_CONTROLLER_EXIT_CRITICAL,
    MOTOR_CONTROLLER_OP_NVM_SAVE_PARAMS,
    MOTOR_CONTROLLER_OP_NVM_BOOT,
    MOTOR_CONTROLLER_OP_NVM_WRITE_ONCE,
    MOTOR_CONTROLLER_OP_NVM_READ_ONCE,
    MOTOR_CONTROLLER_OP_CALIBRATE_SENSOR,
}
MotorController_OperationId_T;

// typedef enum MotorController_SpeedLimitActiveId_Tag
// {
//     MOTOR_CONTROLLER_SPEED_LIMIT_ACTIVE_DISABLE = 0U,
//     MOTOR_CONTROLLER_SPEED_LIMIT_ACTIVE_OPT = 1U, /* From parent class */
// }
// MotorController_SpeedLimitActiveId_T;

typedef enum MotorController_ILimitActiveId_Tag
{
    MOTOR_CONTROLLER_I_LIMIT_ACTIVE_DISABLE = 0U,
    MOTOR_CONTROLLER_I_LIMIT_ACTIVE_HEAT = 1U,
    MOTOR_CONTROLLER_I_LIMIT_ACTIVE_LOW_V = 2U,
}
MotorController_ILimitActiveId_T;

typedef enum MotorController_OptDinFunction_Tag
{
    MOTOR_CONTROLLER_OPT_DIN_DISABLE,
    MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_SERVO,
    MOTOR_CONTROLLER_OPT_DIN_USER_FUNCTION,
}
MotorController_OptDinFunction_T;

/*
    Fault substate flags
    Faults flags with exception of RxLost retain set state until user clears
*/
typedef union MotorController_FaultFlags_Tag
{
    struct
    {
        uint16_t PcbOverHeat : 1U;
    #if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
        uint16_t MosfetsTopOverHeat : 1U;
        uint16_t MosfetsBotOverHeat : 1U;
    #else
        uint16_t MosfetsOverHeat : 1U;
    #endif
        uint16_t VSourceLimit : 1U;
        uint16_t VSenseLimit : 1U;
        uint16_t VAccLimit : 1U;
        uint16_t Motors : 1U;
        uint16_t DirectionSync : 1U;
        uint16_t RxLost : 1U;
        uint16_t User : 1U;
    };
    uint16_t Word;
}
MotorController_FaultFlags_T;

/*
    flags retain prev state for edge detection
*/
typedef union MotorController_StatusFlags_Tag
{
    struct
    {
        uint16_t Heat : 1U;
        uint16_t LowV : 1U;
        // uint16_t SetParams : 1U;
    };
    uint16_t Word;
}
MotorController_StatusFlags_T;

typedef union MotorController_BuzzerFlags_Tag
{
    struct
    {
        uint16_t ThrottleOnInit : 1U;
        uint16_t OnReverse      : 2U; /* 0: Off, 1: Short Beep, 2: Continuous */
        // uint16_t ThrottleOnBrakeCmd;
        // uint16_t ThrottleOnBrakeRelease;
        // uint16_t ThrottleOnNeutralRelease;
    };
    uint16_t Word;
}
MotorController_BuzzerFlags_T;

/*
    MotorController Voltages
    GLOBAL_MOTOR.VMAX -> controller voltage max
    MotorControllerMain.Params.VSourceRef, Global_Motor.VSourceRef_V -> user set nominal voltage
    MotorController_User_GetVSourceRef() -> user set nominal voltage
    MotorController_User_GetVSource() -> live voltage
*/
typedef struct __attribute__((aligned(2U))) MotorController_Params_Tag
{
    uint16_t VSourceRef;        /* Nominal Battery Voltage. Sync with Global_Motor VSourceRef_V */
    uint16_t BatteryZero_Adcu;
    uint16_t BatteryFull_Adcu;
    uint16_t ILimitLowV_Scalar16;
    MotorController_InputMode_T UserInputMode;
    MotorController_BrakeMode_T BrakeMode;
    MotorController_ThrottleMode_T ThrottleMode;
    MotorController_ZeroCmdMode_T ZeroCmdMode;
    Motor_FeedbackMode_T DefaultCmdMode;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    uint8_t CanServicesId;
    bool CanIsEnable;
#endif
    // MotorController_BuzzerFlags_T BuzzerFlagsEnable; /* which options are enabled for use */
    MotorController_OptDinFunction_T OptDinFunction;
    uint16_t OptDinSpeedLimit_Scalar16;
}
MotorController_Params_T;

typedef struct __attribute__((aligned(FLASH_UNIT_WRITE_ONCE_SIZE))) MotorController_Manufacture_Tag
{
    char NAME[8U];
    union { uint8_t SERIAL_NUMBER[4U]; uint32_t SERIAL_NUMBER_REG; };
    union
    {
        uint8_t MANUFACTURE_NUMBER[4U];
        uint32_t MANUFACTURE_NUMBER_REG;
        struct { uint8_t MANUFACTURE_DAY; uint8_t MANUFACTURE_MONTH; uint8_t MANUFACTURE_YEAR; uint8_t MANUFACTURE_RESV; };
    };
    union { uint8_t HARDWARE_VERSION[4U]; uint32_t HARDWARE_VERSION_REG; };
    uint8_t ID_EXT[4U];
    uint8_t RESERVED[8U];
}
MotorController_Manufacture_T;

/*
    allocated memory outside for less CONFIG define redundancy
*/
typedef const struct MotorController_Config_Tag
{
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_FLASH)
    const void * const P_PARAMS_START; /* All params start */
    const uint16_t PARAMS_SIZE;
#endif
    const MotorController_Params_T * const P_PARAMS_NVM;
    const MotorController_Manufacture_T * const P_MANUFACTURE; /* cannot read directly if FlashOnce is selected */
    const MemMapBoot_T * const P_MEM_MAP_BOOT;
//RAM_START, RAM_END for Read Address
    Motor_T * const     P_MOTORS;
    const uint8_t       MOTOR_COUNT;
    Serial_T * const    P_SERIALS;     /* Simultaneous active serial */
    const uint8_t       SERIAL_COUNT;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * const    P_CAN_BUS;
#endif
// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
    Flash_T * const     P_FLASH;        /* Flash defined outside module, ensure flash config/params are in RAM */
// #endif
#if defined(CONFIG_MOTOR_CONTROLLER_PARAMETERS_EEPROM)
    EEPROM_T * const    P_EEPROM;      /* Defined outside for regularity */
#endif
    AnalogN_T * const P_ANALOG_N;
    const MotAnalog_Conversions_T ANALOG_CONVERSIONS;
    Protocol_T * const P_PROTOCOLS; /* Simultaneously active protocols */
    const uint8_t PROTOCOL_COUNT;
    const uint32_t ANALOG_USER_DIVIDER;  /* In Pow2 */
    const uint32_t MAIN_DIVIDER_10;
    const uint32_t MAIN_DIVIDER_1000;
    const uint32_t TIMER_DIVIDER_1000;
    const uint8_t SOFTWARE_VERSION[4U];
}
MotorController_Config_T;

/*   */
typedef struct MotorController_Tag
{
    const MotorController_Config_T CONFIG;
    MotorController_Params_T Parameters;         /* ram copy */
    MemMapBoot_T MemMapBoot;
    volatile MotAnalog_Results_T AnalogResults;

    MotAnalogUser_T AnalogUser;
    Blinky_T Buzzer;
    Blinky_T Meter;
    Pin_T Relay;
    Debounce_T OptDin;     /* Configurable input */

    Thermistor_T ThermistorPcb;
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    Thermistor_T ThermistorMosfetsTop;
    Thermistor_T ThermistorMosfetsBot;
#else
    Thermistor_T ThermistorMosfets;
#endif
    VMonitor_T VMonitorSource;  // Controller Supply
    VMonitor_T VMonitorSense;   // ~5V
    VMonitor_T VMonitorAcc;     // ~12V
    Linear_T BatteryLife;       // Battery Life percentage

    Timer_T TimerMillis;
    uint32_t MainDividerCounter;
    uint32_t TimerDividerCounter;

#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
    Shell_T Shell;
    uint16_t ShellSubstate;
#endif

    /* State and SubState */
    StateMachine_T StateMachine;
    MotorController_FaultFlags_T FaultFlags; /* Fault SubState */
    MotorController_StatusFlags_T StatusFlags;
#if    defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
    MotAnalog_Results_T FaultAnalogRecord;
#endif
    // UserInputState
    /* Set by StateMachine only */
    MotorController_Direction_T ActiveDirection;
    // Motor_FeedbackMode_T UserCmdMode;
    int32_t  UserCmd; /* For Edge Detect, using 0 *///move to Avalue?

    MotorController_InputState_T InputState;

    /* CriticalOp SubState */
    /* Async return status */
    NvMemory_Status_T NvmStatus;
    // calibration status

    // MotorController_OperationId_T SubState;
    // uint16_t ParamId;
    // int32_t ParamValue;

    // MotorController_SpeedLimitActiveId_T SpeedLimitActiveId;
    // MotorController_ILimitActiveId_T ILimitActiveId;
}
MotorController_T; // MotorCtrlr_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline Motor_T * MotorController_GetPtrMotor(const MotorController_T * p_mc, uint8_t motorIndex) { return &(p_mc->CONFIG.P_MOTORS[motorIndex]); }

/******************************************************************************/
/*
    Alarm
*/
/******************************************************************************/
static inline void MotorController_BeepShort(MotorController_T * p_mc) { Blinky_Blink(&p_mc->Buzzer, 500U); }
static inline void MotorController_BeepPeriodicType1(MotorController_T * p_mc) { Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U); }
// static inline void MotorController_BeepType1(MotorController_T * p_mc) { Blinky_Blink(&p_mc->Buzzer, Type1OnTime, Type1OffTime); }

/******************************************************************************/
/*
    Proc by StateMachine - propagate to motors
*/
/******************************************************************************/
static inline void MotorController_ProcAll(MotorController_T * p_mc, Motor_User_ProcVoid_T cmdFunction)
{
    MotorN_User_ProcFunction(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, cmdFunction);
}

static inline void MotorController_SetCmdAll(MotorController_T * p_mc, Motor_User_SetCmd_T cmdFunction, int16_t userCmd)
{
    MotorN_User_SetCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, cmdFunction, userCmd);
}

static inline void MotorController_SetThrottleMode(MotorController_T * p_mc)
{
    switch(p_mc->Parameters.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  MotorController_ProcAll(p_mc, Motor_User_SetSpeedMode);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: MotorController_ProcAll(p_mc, Motor_User_SetTorqueMode);    break;
        default: break;
    }
}

/*
    brake [0:32767]
*/
static inline void MotorController_SetThrottleValue(MotorController_T * p_mc, int16_t userCmdThrottle)
{
    int16_t cmdValue = userCmdThrottle / 2;
    switch(p_mc->Parameters.ThrottleMode)
    {
        case MOTOR_CONTROLLER_THROTTLE_MODE_SPEED:  MotorController_SetCmdAll(p_mc, Motor_User_SetSpeedCmdValue, cmdValue);     break;
        case MOTOR_CONTROLLER_THROTTLE_MODE_TORQUE: MotorController_SetCmdAll(p_mc, Motor_User_SetTorqueCmdValue, cmdValue);    break;
        default: break;
    }
}

static inline void MotorController_SetBrakeMode(MotorController_T * p_mc)
{
    switch(p_mc->Parameters.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE: MotorController_ProcAll(p_mc, Motor_User_SetTorqueMode); break;
        default: break;
    }
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:65535]
*/
void Motor_User_SetBrakeCmd(Motor_T * p_motor, uint16_t brake)
{
    // if(p_motor->FeedbackMode.Hold == 0U)
    // {
    if(Motor_User_GetSpeed_UFrac16(p_motor) > INT16_MAX / 100U)
    {
        // Motor_User_SetTorqueModeCmd(p_motor, (int32_t)0 - (brake / 2U));
        Motor_User_SetTorqueCmdValue(p_motor, (int32_t)0 - (brake / 2U));
    }
    else
    {
        // p_motor->FeedbackMode.Hold = 1U;  //clears on throttle
        Motor_User_ReleaseControl(p_motor);
        // Motor_User_ReleaseHold(p_motor);
        // Motor_User_SetVoltageModeCmd(p_motor);
    }
    // }
}

static inline void MotorController_SetBrakeValue(MotorController_T * p_mc, uint16_t userCmdBrake)
{
    int16_t cmdValue = userCmdBrake / 2;
    switch(p_mc->Parameters.BrakeMode)
    {
        case MOTOR_CONTROLLER_BRAKE_MODE_TORQUE:
            MotorController_SetCmdAll(p_mc, Motor_User_SetBrakeCmd, cmdValue);
        case MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE:

            break;
        default: break;
    }
}

/*
    Full direction proc, during stop only
    Assume edge type input.
*/
static inline bool MotorController_ProcUserDirection(MotorController_T * p_mc, MotorController_Direction_T inputDirection)
{
    bool isSuccess;

    switch(inputDirection)
    {
        // case MOTOR_CONTROLLER_DIRECTION_PARK:         isSuccess = true; break;
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:     isSuccess = true; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD:     isSuccess = MotorN_User_SetDirectionForward(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE:     isSuccess = MotorN_User_SetDirectionReverse(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); break;
        default: isSuccess = false; break;
    }

    // if((p_mc->Parameters.BuzzerFlagsEnable.OnReverse == true))
    // {
    //     if(p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE)
    //     {
    //         MotorController_BeepPeriodicType1(p_mc);
    //     }
    //     else
    //     {
    //         Blinky_Stop(&p_mc->Buzzer);
    //     }
    // }

    return isSuccess;
}

static inline void MotorController_DisableAll(MotorController_T * p_mc)     { MotorController_ProcAll(p_mc, Motor_User_DisableControl); }
static inline void MotorController_ReleaseAll(MotorController_T * p_mc)     { MotorController_ProcAll(p_mc, Motor_User_ReleaseControl); }
static inline void MotorController_ActivateAll(MotorController_T * p_mc)    { MotorController_ProcAll(p_mc, Motor_User_ActivateControl); }
static inline void MotorController_HoldAll(MotorController_T * p_mc)        { MotorController_ProcAll(p_mc, Motor_User_Hold); }

// static inline void MotorController_SetTorqueAll(MotorController_T * p_mc, int16_t userCmd) { MotorN_User_SetTorqueCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, userCmd); }
// static inline void MotorController_SetTorqueValue(MotorController_T * p_mc, uint8_t motorId, int16_t userCmd) { Motor_User_SetTorqueCmdValue(&p_mc->CONFIG.P_MOTORS[motorId], userCmd); }
// static inline void MotorController_SetTorqueMode(MotorController_T * p_mc, uint8_t motorId) { Motor_User_SetTorqueMode(&p_mc->CONFIG.P_MOTORS[motorId]); }
// static inline void MotorController_SetCruiseAll(MotorController_T * p_mc)  { MotorN_User_SetCruise(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }

/* Checks 0 speed. alternatively check stop state. */
static inline bool MotorController_CheckStopMotorAll(MotorController_T * p_mc)      { return MotorN_User_CheckStop(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }
static inline bool MotorController_CheckFaultMotorAll(MotorController_T * p_mc)     { return MotorN_User_CheckFault(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }

/* returns true if no faults remain active */
static inline bool MotorController_ClearFaultAll(MotorController_T * p_mc)          { return MotorN_User_ClearFault(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }

static inline void MotorController_SetSpeedLimitAll(MotorController_T * p_mc, uint16_t limit_frac16)    { MotorN_User_SetSpeedLimitActive(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, limit_frac16); }
static inline void MotorController_ClearSpeedLimitAll(MotorController_T * p_mc)                         { MotorN_User_ClearSpeedLimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT); }

static inline bool MotorController_SetILimitAll(MotorController_T * p_mc, uint16_t limit_frac16, MotorController_ILimitActiveId_T id)
    { return MotorN_User_SetILimitActive(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, limit_frac16, (id + MOTOR_I_LIMIT_ACTIVE_SYSTEM)); }

/* returns true if limit of id is cleared */
static inline bool MotorController_ClearILimitAll(MotorController_T * p_mc, MotorController_ILimitActiveId_T id)
    { return MotorN_User_ClearILimit(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT, (id + MOTOR_I_LIMIT_ACTIVE_SYSTEM)); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(MotorController_T * p_mc);
// extern void MotorController_SetVSourceRef(MotorController_T * p_mc, uint16_t volts);
extern void MotorController_ResetUnitsBatteryLife(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_SaveParameters_Blocking(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_SaveBootReg_Blocking(MotorController_T * p_mc);
extern NvMemory_Status_T MotorController_ReadOnce_Blocking(MotorController_T * p_mc, uint8_t * p_sourceBuffer);
extern NvMemory_Status_T MotorController_SaveOnce_Blocking(MotorController_T * p_mc, const uint8_t * p_destBuffer);
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE)
    #if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    extern void MotorController_ServoExtern_Start(MotorController_T * p_mc);
    extern void MotorController_ServoExtern_Proc(MotorController_T * p_mc);
    extern void MotorController_ServoExtern_SetCmd(MotorController_T * p_mc, int32_t cmd);
    #else
    static inline MotorController_Servo_Start(MotorController_T * p_mc)
    {

    }

    static inline MotorController_Servo_Proc(MotorController_T * p_mc)
    {

    }

    static inline MotorController_Servo_SetCmd(MotorController_T * p_mc, uint32_t cmd)
    {

    }
    #endif
#endif

extern void MotorController_LoadParamsDefault(MotorController_T * p_mc);

#endif
