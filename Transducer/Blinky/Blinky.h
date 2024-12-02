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
    @file    Blinky.h
    @author FireSourcery
    @brief     Pin Indicator
    @version V0
*/
/******************************************************************************/
#ifndef BLINKY_H
#define BLINKY_H

#include "Peripheral/Pin/Pin.h"
#include "Utility/Timer/Timer.h"

#include <stdint.h>
#include <stdbool.h>

// typedef struct
// {
//     void (*Pattern)(void * p_context);
//     void * p_Context;
// }
// Blinky_Pattern_T;

typedef struct Blinky
{
    Pin_T Pin;
    Timer_T Timer;
    bool IsOn;
    uint32_t Index;
    uint32_t Max;
    uint32_t OnTime;
    uint32_t OffTime;
    // uint32_t OffTimeDefault; unchanged between activation types
    void (*PatternFunction)(struct Blinky * p_this);
    // uint8_t ActiveSourceId;
}
Blinky_T;

#define BLINKY_INIT(p_PinHal, PinId, p_TimerBase, TimerBaseFreq)    \
{                                                                   \
    .Pin    = PIN_INIT(p_PinHal, PinId),                            \
    .Timer  = TIMER_INIT(p_TimerBase, TimerBaseFreq)                \
}

static inline void _Blinky_Toggle(Blinky_T * p_blinky) { Pin_Output_Toggle(&p_blinky->Pin); }

extern void Blinky_Init(Blinky_T * p_blinky);
extern void Blinky_Proc(Blinky_T * p_blinky);
extern void Blinky_On(Blinky_T * p_blinky);
extern void Blinky_Off(Blinky_T * p_blinky);
extern void _Blinky_Toggle(Blinky_T * p_blinky);
extern void Blinky_Stop(Blinky_T * p_blinky);
extern void Blinky_Blink_OnOff(Blinky_T * p_blinky, uint32_t duration);
extern void Blinky_Blink_Toggle(Blinky_T * p_blinky, uint32_t duration);
extern void Blinky_Blink(Blinky_T * p_blinky, uint32_t onTime);
extern void Blinky_BlinkN(Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime, uint8_t nRepeat);
extern void Blinky_StartPeriodic(Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime);

#endif
