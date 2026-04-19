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
    @brief  Audible indicator wrapping Blinky_T. Owns the beep pin and the
            runtime enable / beep-on-event configuration. Aggregate modules
            (MotorController, Traction) drive it via the Beep* API; MotBuzzer
            itself has no policy knowledge.
*/
/******************************************************************************/
#include "Transducer/Blinky/Blinky.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Event flags — which conditions the aggregate should beep for.
    MotBuzzer stores these; callers read the flag they need before invoking a
    Beep* helper. Keep the struct 16-bit aligned so it fits a single NVM word.
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
    uint8_t IsEnabled : 1U;             /* master enable; when 0 the pin stays silent */
}
MotBuzzer_Config_T;

/******************************************************************************/
/*
    Runtime state
*/
/******************************************************************************/
typedef struct MotBuzzer_State
{
    Blinky_State_T BlinkyState;
    MotBuzzer_Config_T Config;
}
MotBuzzer_State_T;

/******************************************************************************/
/*
    Compile-time context
*/
/******************************************************************************/
typedef const struct MotBuzzer
{
    MotBuzzer_State_T * P_STATE;
    Blinky_T BLINKY;
    const MotBuzzer_Config_T * P_NVM_CONFIG;
}
MotBuzzer_T;

#define MOT_BUZZER_STATE_ALLOC() (&(MotBuzzer_State_T){0})

#define MOT_BUZZER_INIT(p_State, Pin, Timer, p_Config) (MotBuzzer_T) \
{                                                                    \
    .P_STATE = (p_State),                                            \
    .BLINKY = BLINKY_INIT((Pin), &(p_State)->BlinkyState, (Timer)),  \
    .P_NVM_CONFIG = (p_Config),                                      \
}

#define MOT_BUZZER_INIT_FROM(p_State, p_PinHal, PinId, p_TimerBase, TimerBaseFreq, p_Config) (MotBuzzer_T) \
{                                                                                                           \
    .P_STATE = (p_State),                                                                                   \
    .BLINKY = BLINKY_INIT_FROM((p_PinHal), (PinId), &(p_State)->BlinkyState, (p_TimerBase), (TimerBaseFreq)), \
    .P_NVM_CONFIG = (p_Config),                                                                             \
}

/******************************************************************************/
/*
    Queries
*/
/******************************************************************************/
static inline bool MotBuzzer_IsEnabled(const MotBuzzer_T * p_buzzer) { return p_buzzer->P_STATE->Config.IsEnabled; }

static inline MotBuzzer_OptionFlags_T MotBuzzer_GetEventFlags(const MotBuzzer_T * p_buzzer)
{
    return p_buzzer->P_STATE->Config.EventFlags;
}

/******************************************************************************/
/*
    Drive API — callers invoke these; all honor IsEnabled master gate.
    Timing constants match the prior MotorController inline helpers.
*/
/******************************************************************************/
static inline void MotBuzzer_Proc(const MotBuzzer_T * p_buzzer)
{
    Blinky_Proc(&p_buzzer->BLINKY);
}

static inline void MotBuzzer_Stop(const MotBuzzer_T * p_buzzer)
{
    Blinky_Stop(&p_buzzer->BLINKY);
}

static inline void MotBuzzer_BeepShort(const MotBuzzer_T * p_buzzer)
{
    if (MotBuzzer_IsEnabled(p_buzzer) == true) { Blinky_Blink(&p_buzzer->BLINKY, 500U); }
}

static inline void MotBuzzer_BeepDouble(const MotBuzzer_T * p_buzzer)
{
    if (MotBuzzer_IsEnabled(p_buzzer) == true) { Blinky_BlinkN(&p_buzzer->BLINKY, 250U, 250U, 2U); }
}

static inline void MotBuzzer_BeepMonitorTrigger(const MotBuzzer_T * p_buzzer)
{
    if (MotBuzzer_IsEnabled(p_buzzer) == true) { Blinky_BlinkN(&p_buzzer->BLINKY, 250U, 250U, 1U); }
}

static inline void MotBuzzer_BeepN(const MotBuzzer_T * p_buzzer, uint32_t onTime, uint32_t offTime, uint8_t n)
{
    if (MotBuzzer_IsEnabled(p_buzzer) == true) { Blinky_BlinkN(&p_buzzer->BLINKY, onTime, offTime, n); }
}

static inline void MotBuzzer_BeepPeriodic(const MotBuzzer_T * p_buzzer)
{
    if (MotBuzzer_IsEnabled(p_buzzer) == true) { Blinky_StartPeriodic(&p_buzzer->BLINKY, 500U, 500U); }
}

static inline void MotBuzzer_BeepPeriodicCustom(const MotBuzzer_T * p_buzzer, uint32_t onTime, uint32_t offTime)
{
    if (MotBuzzer_IsEnabled(p_buzzer) == true) { Blinky_StartPeriodic(&p_buzzer->BLINKY, onTime, offTime); }
}

static inline void MotBuzzer_Disable(const MotBuzzer_T * p_buzzer) { Blinky_Disable(&p_buzzer->BLINKY); }
static inline void MotBuzzer_Enable(const MotBuzzer_T * p_buzzer)  { Blinky_Enable(&p_buzzer->BLINKY); }

/******************************************************************************/
/*
    Extern
*/
/******************************************************************************/
extern void MotBuzzer_Init(const MotBuzzer_T * p_buzzer);
extern void MotBuzzer_SetEnabled(const MotBuzzer_T * p_buzzer, bool isEnabled);
extern void MotBuzzer_SetEventFlags(const MotBuzzer_T * p_buzzer, MotBuzzer_OptionFlags_T flags);

/******************************************************************************/
/*
    VarId / ConfigId
*/
/******************************************************************************/
typedef enum MotBuzzer_VarId
{
    MOT_BUZZER_VAR_IS_ENABLED,
}
MotBuzzer_VarId_T;

typedef enum MotBuzzer_ConfigId
{
    MOT_BUZZER_CONFIG_IS_ENABLED,
    MOT_BUZZER_CONFIG_EVENT_FLAGS,  /* MotBuzzer_OptionFlags_T.Value */
}
MotBuzzer_ConfigId_T;

extern int32_t MotBuzzer_VarId_Get(const MotBuzzer_T * p_buzzer, MotBuzzer_VarId_T id);
extern int32_t MotBuzzer_ConfigId_Get(const MotBuzzer_T * p_buzzer, MotBuzzer_ConfigId_T id);
extern void MotBuzzer_ConfigId_Set(const MotBuzzer_T * p_buzzer, MotBuzzer_ConfigId_T id, int32_t value);
