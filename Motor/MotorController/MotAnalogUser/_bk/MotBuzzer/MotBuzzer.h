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



// buzzer state
// on Init poll
// if(p_mc->InitFlags.Word != 0U) { wait = true; }
//     if((p_mc->Config.BuzzerFlagsEnable.ThrottleOnInit == true) && (p_mc->BuzzerFlagsActive.ThrottleOnInit == 0U))
//     {
//         p_mc->BuzzerFlagsActive.ThrottleOnInit = 1U;
        // MotorController_BeepShort(p_mc);
//     }
