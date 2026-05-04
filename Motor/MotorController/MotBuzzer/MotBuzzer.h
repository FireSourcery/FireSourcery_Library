#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   MotBuzzer.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Transducer/Blinky/Blinky.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*

*/
/******************************************************************************/
typedef union MotBuzzer_OptionFlags
{
    struct
    {
        uint16_t OnInit                 : 1U; /* startup chirp */
        uint16_t OnDirectionChange      : 1U; /* F/R/N transition */
        uint16_t OnReverse              : 1U; /* continuous while reversing */
        uint16_t OnThrottleOnInit       : 1U; /* throttle active at power-up */
        uint16_t OnThrottleOnBrake      : 1U; /* throttle + brake simultaneously */
        uint16_t OnFault                : 1U; /* fault latched */
        uint16_t OnMonitorTrigger       : 1U; /* range-monitor edge */
        uint16_t OnLockError            : 1U; /* lock-state rejection */
    };
    uint16_t Value;
}
MotBuzzer_OptionFlags_T;

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
typedef struct MotBuzzer_Config
{
    MotBuzzer_OptionFlags_T EventFlags; /* which events trigger a beep */
    bool IsEnabled;
}
MotBuzzer_Config_T;

typedef Blinky_T MotBuzzer_T;

static inline void MotBuzzer_Short(MotBuzzer_T * p_dev) { Blinky_Blink(p_dev, 500U); }
static inline void MotBuzzer_PeriodicType1(MotBuzzer_T * p_dev) { Blinky_StartPeriodic(p_dev, 500U, 500U); }
static inline void MotBuzzer_Periodic(MotBuzzer_T * p_dev) { Blinky_StartPeriodic(p_dev, 500U, 500U); }
static inline void MotBuzzer_Double(MotBuzzer_T * p_dev) { Blinky_BlinkN(p_dev, 250U, 250U, 2U); }

/* Domain mapped */
static inline void MotBuzzer_MonitorTrigger(MotBuzzer_T * p_dev) { Blinky_BlinkN(p_dev, 250U, 250U, 1U); }

static inline void MotBuzzer_Stop(MotBuzzer_T * p_dev) { Blinky_Stop(p_dev); }
static inline void MotBuzzer_Disable(MotBuzzer_T * p_dev) { Blinky_Disable(p_dev); }


// buzzer state
// on Init poll
// if(p_mc->InitFlags.Word != 0U) { wait = true; }
//     if((p_mc->Config.BuzzerFlagsEnable.ThrottleOnInit == true) && (p_mc->BuzzerFlagsActive.ThrottleOnInit == 0U))
//     {
//         p_mc->BuzzerFlagsActive.ThrottleOnInit = 1U;
        // MotorController_BeepShort(p_mc);
//     }


// void MotBuzzer_Init(MotBuzzer_T * p_buzzer)
// {
//     if (p_buzzer->P_NVM_CONFIG != NULL) { p_buzzer->P_STATE->Config = *p_buzzer->P_NVM_CONFIG; }
//     Blinky_Init(&p_buzzer->BLINKY);
//     if (p_buzzer->P_STATE->Config.IsEnabled == 0U) { Blinky_Disable(&p_buzzer->BLINKY); }
// }

// int32_t MotBuzzer_VarId_Get(MotBuzzer_T * p_buzzer, MotBuzzer_VarId_T id)
// {
//     int32_t value = 0;
//     switch (id)
//     {
//         case MOT_BUZZER_VAR_IS_ENABLED: value = MotBuzzer_IsEnabled(p_buzzer); break;
//         default: break;
//     }
//     return value;
// }

// int32_t MotBuzzer_ConfigId_Get(const MotBuzzer_Config_T * p_buzzer, MotBuzzer_ConfigId_T id)
// {
//     int32_t value = 0;
//     switch (id)
//     {
//         case MOT_BUZZER_CONFIG_IS_ENABLED:      value = p_buzzer->P_STATE->Config.IsEnabled;            break;
//         case MOT_BUZZER_CONFIG_EVENT_FLAGS:     value = p_buzzer->P_STATE->Config.EventFlags.Value;     break;
//         default: break;
//     }
//     return value;
// }

// void MotBuzzer_ConfigId_Set(MotBuzzer_Config_T * p_buzzer, MotBuzzer_ConfigId_T id, int32_t value)
// {
//     switch (id)
//     {
//         case MOT_BUZZER_CONFIG_IS_ENABLED:      MotBuzzer_SetEnabled(p_buzzer, (value != 0));                       break;
//         case MOT_BUZZER_CONFIG_EVENT_FLAGS:     p_buzzer->P_STATE->Config.EventFlags.Value = (uint16_t)value;       break;
//         default: break;
//     }
// }
