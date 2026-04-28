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
#include "MotAnalogUser/MotAnalogUser.h"
#include "MotAnalogUser/MotAnalogUser_Conversion.h"
#include "MotNvm/MotNvm.h"
#include "MotLimits/MotLimits.h"

#include "Motor/Motor/Motor_Table.h"
#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_StateMachine.h"
// #include "Motor/Motor/Motor_Include.h"

#include "Motor/Motor/VBus/VBus.h"
#include "Motor/Motor/VBus/VBus_Monitor.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Analog/Analog_ADC.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"
#include "Peripheral/Serial/Serial.h"
#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
#include "Peripheral/CanBus/CanBus.h"
#include "Peripheral/CanBus/CanBus_Service.h"
#endif

#include "Framework/Timer/Timer.h"
#include "Framework/StateMachine/StateMachine.h"
#include "Framework/Protocol/Protocol.h"
#include "Framework/Protocol/Socket.h"
#if defined(MOTOR_CONTROLLER_SHELL_ENABLE)
#include "Framework/Shell/Shell.h"
#endif
#include "Framework/BootRef/BootRef.h"
#include "Type/Word/Version.h"

#include "Math/Linear/Linear.h"
#include "Math/Accumulator/Accumulator.h"

#include <stdint.h>
#include <string.h>

/* Part  */
#include "MotorController_App.h"



/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    User InputMux
*/
// DriveUserSource
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
    MOTOR_CONTROLLER_OPT_DIN_PARK,
    // MOTOR_CONTROLLER_OPT_DIN_USER_FUNCTION,
}
MotorController_OptDinMode_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    Fault SubState flags
    Faults flags latch state until user clears
*/
typedef union MotorController_FaultFlags
{
    struct
    {
        uint16_t MosfetsOverheat    : 1U;
        uint16_t PcbOverheat        : 1U;
        uint16_t VBusLimit          : 1U; /* VBus monitor over/under */
        uint16_t VAccsLimit         : 1U;
        uint16_t VAnalogLimit       : 1U;
        uint16_t Motors             : 1U; /* Sensor/StartUp */
        uint16_t RxLost             : 1U;
        uint16_t InitCheck          : 1U;
        // uint16_t ILimit          : 1U;
        // uint16_t StateError         : 1U;
        // uint16_t DirectionSync      : 1U;
        // uint16_t User               : 1U;
    };
    uint16_t Value;
}
MotorController_FaultFlags_T;

static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_MOSFETS_OVERHEAT = { .MosfetsOverheat = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_PCB_OVERHEAT     = { .PcbOverheat     = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_VBUS_LIMIT       = { .VBusLimit       = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_VACCS_LIMIT      = { .VAccsLimit      = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_VANALOG_LIMIT    = { .VAnalogLimit    = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_MOTORS           = { .Motors          = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_RX_LOST          = { .RxLost          = 1U };
static const MotorController_FaultFlags_T MOTOR_CONTROLLER_FAULT_INIT_CHECK       = { .InitCheck       = 1U };


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
    // MotorController_MainMode_T InitMode;
    int InitMode; /* enum stand in */
    MotorController_InputMode_T InputMode;
    bool IsParkStateEnabled;

    // MotorController_BuzzerFlags_T BuzzerEnable;
    // MotorController_InitFlags_T InitChecksEnabled;

    /* OptDin */
    MotorController_OptDinMode_T OptDinMode;
    uint16_t OptSpeedLimit_Fract16;
    uint16_t OptILimit_Fract16;

#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
//     uint8_t CanServicesId;
    bool IsCanEnable;
#endif
}
MotorController_Config_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef struct MotorController_State
{
    /* State and SubState */
    StateMachine_Active_T StateMachine; /* Data */
    MotorController_FaultFlags_T FaultFlags; /* Fault SubState */
    MotorController_InitFlags_T InitFlags;
    uint32_t StateCounter; /* Calibration */
    uint32_t ControlCounter; /* PWM */

    // MotLimits_T Limits; /* mot to contigous alloc */

    // MotorController_InputMode_T ActiveInput;
    Motor_Input_T CmdInput; /* Buffered Input for StateMachine */

    /* Generic async return status, alternatively as union */
    uint8_t LockOpStatus; /* async status */
    NvMemory_Status_T NvmStatus; /* Common NvmStatus, e.g. EEPROM/Flash */

    MotorController_Config_T Config;
    BootRef_T BootRef; /* Buffer */

    // there can move to calibration as static
    Accumulator_T AvgBuffer0;
    Accumulator_T AvgBuffer1;

    uint32_t MicrosRef;
    uint32_t ControlLoopProfile;

#if defined(MOTOR_CONTROLLER_SHELL_ENABLE)
    Shell_T Shell;
    uint16_t ShellSubState;
#endif

#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // CanBus_Service_T * p_CanBus_Service; /* optional runtime disable */
#endif
}
MotorController_State_T;

/*
    Allocated memory context
*/
typedef const struct MotorController
{
    MotorController_State_T * P_MC; /* Pointer to the Runtime buffer */

    /*
        Peripheral Init
    */
    Analog_ADC_T * P_ANALOG_ADCS;
    uint8_t ADC_COUNT; /* Analog ADCs */

    Serial_T * P_SERIALS;
    uint8_t SERIAL_COUNT;
#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * P_CAN_BUS;
    // CanBus_Service_T CAN_BUS_SERVICE;
    CanBus_Broadcast_T CAN_BUS_BROADCAST_20; /* Broadcast function callback, e.g. for periodic Tx */
    CanBus_Broadcast_T CAN_BUS_BROADCAST_1000;
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
// alternatively wrap inner
// typedef const struct MotorDrive
// {
//     Motor_Table_T MOTORS;
//     LimitArray_T SPEED_LIMIT_SOURCES;
//     LimitArray_T I_LIMIT_SOURCES;
// }
// MotorDrive_T;
    Motor_Table_T MOTORS; /* Motor Array Context */
    LimitArray_T SPEED_LIMIT_SOURCES;
    LimitArray_T I_LIMIT_SOURCES;
    LimitArray_T I_GEN_LIMIT_SOURCES;

    /* Monitor - Detection + response with full context */
    HeatMonitor_T HEAT_PCB;
    Analog_Conversion_T HEAT_PCB_CONVERSION;

    HeatMonitor_Group_T HEAT_MOSFETS;
    Analog_Conversion_T * P_HEAT_MOSFET_CONVERSIONS;

    VBus_T * P_VBUS;                    /* DC bus — owns live fract16, derate config, monitor */
    VBus_Config_T * P_VBUS_NVM_CONFIG;    /* hold vs config */
    Analog_Conversion_T VBUS_CONVERSION;

    VMonitor_T V_ACCESSORIES;   /* ~12V */
    Analog_Conversion_T V_ACCESSORIES_CONVERSION;

    VMonitor_T V_ANALOG;        /* V Analog Sensors ~5V */
    Analog_Conversion_T V_ANALOG_CONVERSION;

    /* State */
    TimerT_T MILLIS_TIMER; /* Timer Context */
    StateMachine_T STATE_MACHINE;

    MotorController_App_T * P_APP; /* Single compile time selection for now */
    const void * P_APP_CONTEXT;
    // directly map components. this would be most similar to implementing within MotorController_T, minus the type coupling.
    // void * P_APP_RUNTIME;
    // const void * P_APP_NVM_CONFIG;

    const MotorController_Config_T * P_NVM_CONFIG;
    Version_T MAIN_VERSION;
}
MotorController_T;

/******************************************************************************/
/*!
    App Dispatch - delegates to embedded APP vtable in MotorController_T
*/
/******************************************************************************/
/* in case implementation changes */
static inline MotorController_App_T * MotorController_App(MotorController_T * p_mc) { return p_mc->P_APP; }
static inline State_T * MotorController_App_EnterMain(MotorController_T * p_mc) { return p_mc->P_APP->ENTER_MAIN((void *)p_mc, 0); }
static inline void MotorController_App_ProcAnalogUser(MotorController_T * p_mc) { p_mc->P_APP->PROC_ANALOG_USER(p_mc); }
static inline void MotorController_App_Init(MotorController_T * p_mc) { if (p_mc->P_APP->INIT != NULL) { p_mc->P_APP->INIT(p_mc); } }

/******************************************************************************/
/*

*/
/******************************************************************************/
static inline Socket_T * MotorController_GetMainSocket(const MotorController_T * p_dev) { return &(p_dev->P_PROTOCOLS[p_dev->USER_PROTOCOL_INDEX]); }

/* check all applicable */
static inline bool MotorController_PollRxLost(const MotorController_T * p_dev)
{
    assert(p_dev->USER_PROTOCOL_INDEX < p_dev->PROTOCOL_COUNT);
    p_dev->P_MC->FaultFlags.RxLost = Socket_IsRxLost(MotorController_GetMainSocket(p_dev));
    return p_dev->P_MC->FaultFlags.RxLost;
}



/* Common Buffered Input */
static inline Motor_Input_T * MotorController_GetMotorInput(const MotorController_T * p_dev) { return &p_dev->P_MC->CmdInput; }


/******************************************************************************/
/*
    split MotBuzzer
*/
/******************************************************************************/
static inline Blinky_T * MotorController_Buzzer(const MotorController_T * p_dev) { return &p_dev->BUZZER; }

static inline void MotorController_BeepShort(const MotorController_T * p_dev)            { Blinky_Blink(&p_dev->BUZZER, 500U); }
static inline void MotorController_BeepPeriodicType1(const MotorController_T * p_dev)    { Blinky_StartPeriodic(&p_dev->BUZZER, 500U, 500U); }
static inline void MotorController_BeepPeriodic(const MotorController_T * p_dev)         { Blinky_StartPeriodic(&p_dev->BUZZER, 500U, 500U); }
static inline void MotorController_BeepDouble(const MotorController_T * p_dev)           { Blinky_BlinkN(&p_dev->BUZZER, 250U, 250U, 2U); }
static inline void MotorController_BeepMonitorTrigger(const MotorController_T * p_dev)   { Blinky_BlinkN(&p_dev->BUZZER, 250U, 250U, 1U); }
static inline void MotorController_BeepStop(const MotorController_T * p_dev)             { Blinky_Stop(&p_dev->BUZZER); }
static inline void MotorController_DisableBuzzer(const MotorController_T * p_dev)        { Blinky_Disable(&p_dev->BUZZER); }



/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(const MotorController_T * p_dev);

extern void MotorController_ResetBootDefault(MotorController_State_T * p_mc);
extern void _MotorController_SetVSupply_V(const MotorController_T * p_dev, uint16_t volts);


extern bool _MotorController_SetSpeedLimitAll(const MotorController_T * p_dev, MotSpeedLimitId_T id, limit_t limit_fract16);
extern bool _MotorController_ClearSpeedLimitAll(const MotorController_T * p_dev, MotSpeedLimitId_T id);
extern bool _MotorController_SetILimitAll(const MotorController_T * p_dev, MotILimitId_T id, limit_t limit_fract16);
extern bool _MotorController_ClearILimitAll(const MotorController_T * p_dev, MotILimitId_T id);
extern bool _MotorController_SetIGenLimitAll(const MotorController_T * p_dev, MotIGenLimitId_T id, limit_t limit_fract16);
extern bool _MotorController_ClearIGenLimitAll(const MotorController_T * p_dev, MotIGenLimitId_T id);

// extern NvMemory_Status_T MotorController_SaveConfig_Blocking(const MotorController_T * p_dev);

