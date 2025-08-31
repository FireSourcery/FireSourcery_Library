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
    @file   MotorController.h
    @author FireSourcery
    @brief  Facade Wrapper
*/
/******************************************************************************/
#include "Config.h"
// #include "MotorControllerAnalog.h"

#include "MotNvm/MotNvm.h"
#include "MotAnalogUser/MotAnalogUser.h"
#include "MotAnalogUser/MotAnalogUser_Conversion.h"

#include "MotMotors/MotMotors.h"
#include "MotDrive/MotDrive.h"
#include "MotDrive/MotDrive_User.h"
#include "MotLimits/MotLimits.h"

#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_StateMachine.h"
// #include "Motor/Motor/Motor_Include.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Analog/Analog_ADC.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"
#include "Peripheral/Serial/Serial.h"
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
#include "Peripheral/CanBus/CanBus.h"
#endif

#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Protocol/Protocol.h"
#include "Utility/Protocol/Socket.h"
#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
#include "Utility/Shell/Shell.h"
#endif
#include "Utility/BootRef/BootRef.h"
#include "Type/Word/Version.h"

#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <string.h>


/******************************************************************************/
/*!
*/
/******************************************************************************/
/* Operation Mode */
typedef enum MotorController_MainMode
{
    MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD,
    MOTOR_CONTROLLER_MAIN_MODE_DRIVE,
    MOTOR_CONTROLLER_MAIN_MODE_SERVO,
}
MotorController_MainMode_T;

/*
    InputMux
*/
typedef enum MotorController_InputMode
{
    MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
    MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
    MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

typedef enum MotorController_OptDinMode
{
    MOTOR_CONTROLLER_OPT_DIN_DISABLE,
    MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT,
    MOTOR_CONTROLLER_OPT_DIN_SERVO,
    MOTOR_CONTROLLER_OPT_DIN_USER_FUNCTION,
}
MotorController_OptDinMode_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    Fault SubState flags
    Faults flags retain set state until user clears
        to do buzzer while active
*/
typedef union MotorController_FaultFlags
{
    struct
    {
        uint16_t MosfetsOverheat    : 1U;
        uint16_t PcbOverheat        : 1U;
        uint16_t VSourceLimit       : 1U; /* VSourceMonitor for over/under  */
        uint16_t VAccsLimit         : 1U;
        uint16_t VAnalogLimit       : 1U;
        // uint16_t ILimit          : 1U;
        uint16_t Motors             : 1U; /* Sensor/StartUp */
        uint16_t RxLost             : 1U;
        uint16_t InitCheck          : 1U;
        // uint16_t StateError         : 1U;
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


typedef struct MotorController_Config
{
    /*
        MotorController Voltages
        PHASE_CALIBRATION.V_MAX -> ADC Saturation
        PHASE_CALIBRATION.V_RATED -> controller voltage max
        Config.VSupplyRef -> user set nominal voltage
        MotorAnalogReference.VSource_V-> live voltage
    */
    // alternatively split motor static instance
    uint16_t VSupplyRef;            /* VMonitor.Nominal Source/Battery Voltage. Sync with MotorAnalogReference VSource_V */
    uint16_t VLowILimit_Fract16;

    MotorController_MainMode_T InitMode; /* alternatively state_t stand in */
    MotorController_InputMode_T InputMode;
    // MotorController_BuzzerFlags_T BuzzerEnable;
    // MotorController_InitFlags_T InitChecksEnabled;

    /* OptDin */
    MotorController_OptDinMode_T OptDinMode;
    uint16_t OptSpeedLimit_Fract16;
    uint16_t OptILimit_Fract16;

#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    uint8_t CanServicesId;
    bool CanIsEnable;
#endif
}
MotorController_Config_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
/* Internal runtime state. these can be moved to submodules eventually. */
typedef struct MotorController_State
{
    uint32_t StateCounter; /* calibration */
    uint32_t ControlCounter; /* pwm */

    Motor_User_Input_T CmdInput; /* Buffered Input for StateMachine */
    // Motor_User_Input_T CmdInputPrev; /* Previous buffered Input for StateMachine */

    /* State and SubState */
    StateMachine_Active_T StateMachine; /* Data */
    MotorController_FaultFlags_T FaultFlags; /* Fault SubState */
    MotorController_InitFlags_T InitFlags;
    MotDrive_State_T MotDrive; /* Optionally contain on init */

    /* Generic async return status */
    uint8_t LockOpStatus; /* async status */
    NvMemory_Status_T NvmStatus; /* Common NvmStatus, e.g. EEPROM/Flash */
    // Calibration_Status_T CalibrationStatus;

    MotorController_Config_T Config;
    BootRef_T BootRef; /* Buffer */

    // there can move to calibration as static
    Filter_T AvgBuffer0;
    Filter_T AvgBuffer1;

    uint32_t MicrosRef;
    uint32_t ControlLoopProfile;

#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
    Shell_T Shell;
    uint16_t ShellSubState;
#endif
}
MotorController_State_T;

/*
    Allocated memory context
*/
typedef const struct MotorController
{
    /*
        Peripheral Init
    */
    Analog_ADC_T * P_ANALOG_ADCS;
    uint8_t ADC_COUNT; /* Analog ADCs */

    Serial_T * P_SERIALS;
    uint8_t SERIAL_COUNT;
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * P_CAN_BUS;
#endif

    /*
        Peripheral Services Context
    */
    /* Transducer */
    MotAnalogUser_T ANALOG_USER;
    MotAnalogUser_Conversion_T ANALOG_USER_CONVERSIONS; /* AnalogUser Conversions */
    UserDIn_T OPT_DIN; /* Configurable input */
    Blinky_T BUZZER;
    Blinky_T METER;
    Pin_T RELAY_PIN;

    Socket_T * P_PROTOCOLS;  /* Sockets */
    uint8_t PROTOCOL_COUNT;
    uint8_t USER_PROTOCOL_INDEX; /* The corresponding Xcvr will not be changed for now */

    MotNvm_T MOT_NVM; /* Non-volatile Memory controller */

    /* Motor Services Context */
    MotMotors_T MOTORS;
    LimitArray_T MOT_SPEED_LIMITS;
    LimitArray_T MOT_I_LIMITS;

    /* Monitor - Detection + response with full context */
    HeatMonitor_Context_T HEAT_PCB;
    HeatMonitor_GroupContext_T HEAT_MOSFETS;
    VMonitor_Context_T V_SOURCE;        /* Controller Supply */
    VMonitor_Context_T V_ACCESSORIES;   /* ~12V */
    VMonitor_Context_T V_ANALOG;        /* V Analog Sensors ~5V */
    // Analog_Conversion_T V_ASENSE_CONVERSION;

    /* State */
    TimerT_T MILLIS_TIMER; /* Timer Context */
    StateMachine_T STATE_MACHINE;
    MotDrive_T MOT_DRIVE; /* Drive */
    MotorController_State_T * P_MC_STATE; /* Pointer to the Runtime buffer */

    Version_T MAIN_VERSION;
    const MotorController_Config_T * P_NVM_CONFIG;
}
MotorController_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
/* Set Motor Ref using read Value */
static inline void MotorController_CaptureVSource(const MotorController_T * p_context) { Phase_VBus_CaptureAdcu(Analog_Conversion_GetResult(&p_context->V_SOURCE.ANALOG_CONVERSION)); }

static inline Socket_T * MotorController_GetMainSocket(const MotorController_T * p_context) { return &(p_context->P_PROTOCOLS[p_context->USER_PROTOCOL_INDEX]); }

/* check all applicable */
static inline bool MotorController_PollRxLost(const MotorController_T * p_context)
{
    assert(p_context->USER_PROTOCOL_INDEX < p_context->PROTOCOL_COUNT);
    p_context->P_MC_STATE->FaultFlags.RxLost = Socket_IsRxLost(MotorController_GetMainSocket(p_context));
    return p_context->P_MC_STATE->FaultFlags.RxLost;
}


/******************************************************************************/
/*
    split MotBuzzer
*/
/******************************************************************************/
static inline void MotorController_BeepShort(const MotorController_T * p_context)            { Blinky_Blink(&p_context->BUZZER, 500U); }
static inline void MotorController_BeepPeriodicType1(const MotorController_T * p_context)    { Blinky_StartPeriodic(&p_context->BUZZER, 500U, 500U); }
static inline void MotorController_BeepPeriodic(const MotorController_T * p_context)         { Blinky_StartPeriodic(&p_context->BUZZER, 500U, 500U); }
static inline void MotorController_BeepDouble(const MotorController_T * p_context)           { Blinky_BlinkN(&p_context->BUZZER, 250U, 250U, 2U); }
static inline void MotorController_BeepMonitorTrigger(const MotorController_T * p_context)   { Blinky_BlinkN(&p_context->BUZZER, 250U, 250U, 1U); }
static inline void MotorController_BeepStop(const MotorController_T * p_context)             { Blinky_Stop(&p_context->BUZZER); }
static inline void MotorController_DisableBuzzer(const MotorController_T * p_context)        { Blinky_Disable(&p_context->BUZZER); }

// buzzer state
// on Init poll
// if(p_mc->InitFlags.Word != 0U) { wait = true; }
//     if((p_mc->Config.BuzzerFlagsEnable.ThrottleOnInit == true) && (p_mc->BuzzerFlagsActive.ThrottleOnInit == 0U))
//     {
//         p_mc->BuzzerFlagsActive.ThrottleOnInit = 1U;
        // MotorController_BeepShort(p_mc);
//     }


/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(const MotorController_T * p_context);

extern void MotorController_LoadConfigDefault(const MotorController_T * p_context);
extern void MotorController_ResetVSourceMonitorDefaults(const MotorController_T * p_context);
extern void MotorController_ResetBootDefault(MotorController_State_T * p_mc);

extern bool MotorController_SetSpeedLimitAll(const MotorController_T * p_context, MotSpeedLimit_Id_T id, limit_t limit_fract16);
extern bool MotorController_ClearSpeedLimitAll(const MotorController_T * p_context, MotSpeedLimit_Id_T id);
extern bool MotorController_SetILimitAll(const MotorController_T * p_context, MotILimit_Id_T id, limit_t limit_fract16);
extern bool MotorController_ClearILimitAll(const MotorController_T * p_context, MotILimit_Id_T id);

// extern NvMemory_Status_T MotorController_SaveConfig_Blocking(const MotorController_T * p_context);


