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
#include "MotAnalogUser/Shifter/Shifter.h"
#include "MotAnalogUser/OptPin/OptDin.h"
#include "MotNvm/MotNvm.h"
#include "MotLimits/MotLimits.h"
#include "MotBuzzer/MotBuzzer.h"

#include "Motor/Motor/Motor_Table.h"
#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/StateMachine/Motor_StateMachine.h"
// #include "Motor/Motor/Motor_Include.h"
#include "Motor/Motor/VBus/VBus.h"
#include "Motor/Motor/VBus/VBus_Monitor.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"
#include "Transducer/UserIn/UserDIn_Cmd.h"

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
#include "Math/Filter/MovAvg.h"
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



/******************************************************************************/
/*!
*/
/******************************************************************************/
/*
    User Input Mux
*/
typedef enum MotorController_InputMode
{
    MOTOR_CONTROLLER_INPUT_MODE_SERIAL,
    MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
    MOTOR_CONTROLLER_INPUT_MODE_CAN,
}
MotorController_InputMode_T;

/*
    Standby exit policy — how the safe stationary state is left.
    Resolved configuration to decouple input-layer from the state machine.
*/
typedef enum MotorController_StandbyExitMode
{
    MOTOR_CONTROLLER_STANDBY_EXIT_MANUAL,      /* Await an explicit START_MAIN — serial/CAN master, or a DIN edge */
    MOTOR_CONTROLLER_STANDBY_EXIT_AUTO,        /* Auto-advance to Main once fault-free and stationary */
    // MOTOR_CONTROLLER_STANDBY_EXIT_ON_THROTTLE, /* Headless analog — throttle is the enable interface; auto-advance gated on neutral */
}
MotorController_StandbyExitMode_T;


/*
    Init SubState
    Configurable checks on init
*/
// typedef union MotorController_InitFlags
// {
//     struct
//     {
//         uint16_t ThrottleZero : 1U;
//         uint16_t DirectionSet : 1U;
//         // uint16_t IsConfigLoaded : 1U;
//     };
//     uint16_t Value;
// }
// MotorController_InitFlags_T;


typedef struct MotorController_Config
{
    // MotorController_MainMode_T InitMode;
    int InitMode; /* Sub app on init. enum stand in. Def in AppTable */
    MotorController_InputMode_T InputMode;

    // MotorController_BuzzerFlags_T BuzzerEnable;
    // MotorController_InitFlags_T InitChecksEnabled;

    OptDin_Config_T OptDinConfig;
    Shifter_Config_T ShifterConfig;
    // optionally move to Din P_VM
    UserDIn_Config_T DInConfigs[MOT_USER_DIN_COUNT]; /* stores cmd id */
    UserAIn_Config_T AInConfigs[MOT_USER_AIN_COUNT];
}
MotorController_Config_T;

/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef struct MotorController_Context
{
    /* State and SubState */
    StateMachine_Active_T StateMachine; /* Data */
    MotorController_FaultFlags_T FaultFlags; /* Fault SubState */
    // MotorController_InitFlags_T InitFlags;
    MotorController_StandbyExitMode_T StandbyExitMode; /* Runtime policy — resolved at init from config, overridable at runtime, not persisted */
    uint32_t ControlCounter; /* PWM */

    MotLimits_T Limits;     /* Q15 unitless derate ratios — contiguous augments + values for I and Speed system arbitration. */

    Motor_Input_T CmdInput; /* Buffered Input for StateMachine. Unused for now */
    // MotorController_InputMode_T ActiveInput;

    /* AIN state — parallel to MotorController_T.AINS[] / .AIN_CONVERSIONS[] */
    /* alternatively ain wraper handle */
    UserAIn_State_T AInStates[MOT_USER_AIN_COUNT];
    UserDIn_State_T AInGateStates[MOT_USER_AIN_COUNT];

    OptDin_State_T OptDinState;

    /* Generic async return status, alternatively as union */
    uint8_t LockOpStatus; /* async status */
    NvMemory_Status_T NvmStatus; /* Common NvmStatus, e.g. EEPROM/Flash */

    MotorController_Config_T Config;
    BootRef_T BootRef; /* Buffer */

    // uint8_t CalibrationBuffer[32];
    uint32_t StateCounter;
    MovAvg_T AvgBuffer0;
    MovAvg_T AvgBuffer1;

    uint32_t MicrosRef;
    uint32_t ControlLoopProfile;

#if defined(MOTOR_CONTROLLER_SHELL_ENABLE)
    Shell_T Shell;
    uint16_t ShellSubState;
#endif

}
MotorController_Context_T;

/*
    Allocated memory context
*/
typedef const struct MotorController
{
    MotorController_Context_T * P_MC; /* Pointer to the Runtime buffer */

    /*
        Peripheral Init
    */
    Analog_ADC_T * P_ANALOG_ADCS;
    uint8_t ADC_COUNT; /* Analog ADCs */

    Serial_T * P_SERIALS;
    uint8_t SERIAL_COUNT;
#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    CanBus_T * P_CAN_SOCKETS;
    uint8_t CAN_SOCKET_COUNT;
#endif

    /*
        Peripheral Services Context
    */
    /* Transducer */
    Shifter_T SHIFTER;              /* Direction shifter pins */
    UserDIn_T DINS[MOT_USER_DIN_COUNT];
    struct { UserAIn_T PIN; Analog_Conversion_T CONVERSION; } AINS[MOT_USER_AIN_COUNT];

    Blinky_T BUZZER;
    Blinky_T METER;
    Pin_T RELAY_PIN;

    Socket_T * P_PROTOCOLS;  /* Sockets */
    uint8_t PROTOCOL_COUNT;
    uint8_t USER_PROTOCOL_INDEX; /* The corresponding Xcvr will not be changed for now */

    MotNvm_T MOT_NVM; /* Non-volatile Memory controller */

    /* Motor Services Context */
    Motor_Table_T MOTORS; /* Motor Array Context */
    uint8_t * P_MOTOR_ADAPTER_BUFFER; /* parallel for now */

    /* Monitor - Detection + response with full context */
    HeatMonitor_T HEAT_PCB;
    Analog_Conversion_T HEAT_PCB_CONVERSION;

    HeatMonitor_Group_T HEAT_MOSFETS;
    Analog_Conversion_T * P_HEAT_MOSFET_CONVERSIONS;

    VBus_T * P_VBUS;                    /* DC bus — owns live fract16, derate config, monitor */
    const VBus_Config_T * P_VBUS_NVM_CONFIG;    /* hold vbus config */
    Analog_Conversion_T VBUS_CONVERSION;

    VMonitor_T V_ACCESSORIES;   /* ~12V */
    Analog_Conversion_T V_ACCESSORIES_CONVERSION;

    // reference for AINS
    VMonitor_T V_ANALOG;        /* V Analog Sensors ~5V */
    Analog_Conversion_T V_ANALOG_CONVERSION;

    /* State */
    TimerT_T MILLIS_TIMER; /* Timer Context */
    StateMachine_T STATE_MACHINE;

    MotorController_App_T * P_APP; /* Single compile time selection for now */
    /* directly map sub app components. same as implementing within MotorController_T, minus the type coupling. */
    void * P_APP_STATE;             /* Adapter state — cast to app's concrete type (Traction_T, Servo_T, ...) */
    const void * P_APP_NVM_CONFIG;  /* Adapter NVM config — cast to app's concrete config type */

    const MotorController_Config_T * P_NVM_CONFIG;
    Version_T MAIN_VERSION;
}
MotorController_T;


/******************************************************************************/
/*
    Expose components interface
*/
/******************************************************************************/
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

/*  */
static inline Socket_T * MotorController_GetMainSocket(MotorController_T * p_dev) { assert(p_dev->USER_PROTOCOL_INDEX < p_dev->PROTOCOL_COUNT); return &(p_dev->P_PROTOCOLS[p_dev->USER_PROTOCOL_INDEX]); }

/* check all applicable */
static inline bool MotorController_PollRxLost(MotorController_T * p_dev)
{
    p_dev->P_MC->FaultFlags.RxLost = Socket_IsRxLost(MotorController_GetMainSocket(p_dev));
    return p_dev->P_MC->FaultFlags.RxLost;
}

static inline MotBuzzer_T * MotorController_Buzzer(MotorController_T * p_dev) { return &p_dev->BUZZER; }
// static inline VBus_T * MotorController_VBus(MotorController_T * p_dev) { return p_dev->P_VBUS; }
// static inline VMonitor_T * MotorController_VMonitorAcces (MotorController_T * p_dev) { return &p_dev->V_ACCESSORIES; }

/* Common Buffered Input */
// static inline Motor_Input_T * MotorController_GetMotorInput(MotorController_T * p_dev) { return &p_dev->P_MC->CmdInput; }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotorController_Init(MotorController_T * p_dev);

extern void MotorController_ResetBootDefault(MotorController_Context_T * p_mc);

extern MotorController_StandbyExitMode_T MotorController_ResolveStandbyExitMode(MotorController_T * p_dev);

extern bool _MotorController_SetSpeedLimitAll(MotorController_T * p_dev, MotSpeedLimitId_T id, limit_t limit_fract16);
extern bool _MotorController_ClearSpeedLimitAll(MotorController_T * p_dev, MotSpeedLimitId_T id);
extern bool _MotorController_SetILimitAll(MotorController_T * p_dev, MotILimitId_T id, limit_t limit_fract16);
extern bool _MotorController_ClearILimitAll(MotorController_T * p_dev, MotILimitId_T id);
