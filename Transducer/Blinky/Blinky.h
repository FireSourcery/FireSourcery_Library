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

*/
/******************************************************************************/
#ifndef BLINKY_H
#define BLINKY_H

#include "Peripheral/Pin/Pin.h"
#include "Utility/Timer/Timer.h"

#include <stdint.h>
#include <stdbool.h>

struct Blinky;

typedef struct Blinky_State
{
    // Timer_T Timer;
    // Blinky_Mode_T Mode;
    bool IsOn;
    uint32_t Index;
    uint32_t End;
    uint32_t OnTime;
    uint32_t OffTime;
    // uint32_t OffTimeDefault; unchanged between activation types
    void (*PatternFunction)(const struct Blinky * p_this);
    // uint8_t ActiveSourceId;
}
Blinky_State_T;

typedef const struct Blinky
{
    Pin_T PIN;
    Blinky_State_T * P_STATE; /* Pointer to runtime state */
    TimerT_T TIMER;
    // const volatile uint32_t * P_TIMER;
}
Blinky_T;

#define BLINKY_ALLOC(p_PinHal, PinId, p_TimerBase, TimerBaseFreq)   \
{                                                                   \
    .PIN = PIN_INIT(p_PinHal, PinId),                               \
    .TIMER = TIMER_T_ALLOC(p_TimerBase, TimerBaseFreq),             \
    .P_STATE = &(Blinky_State_T){ 0 },                               \
}


// static inline void Blinky_Disable(const Blinky_T * p_blinky) { p_blinky->P_STATE->Mode = BLINKY_STATE_DISABLED; }
// static inline void Blinky_Enable(const Blinky_T * p_blinky) { p_blinky->P_STATE->Mode = BLINKY_STATE_ENABLED; }
static inline void Blinky_Disable(const Blinky_T * p_blinky) { p_blinky->TIMER.P_STATE->Mode = TIMER_MODE_DISABLED; }
static inline void Blinky_Enable(const Blinky_T * p_blinky) { p_blinky->TIMER.P_STATE->Mode = TIMER_MODE_STOPPED; }

extern void Blinky_Init(const Blinky_T * p_blinky);
extern void Blinky_Proc(const Blinky_T * p_blinky);

extern void Blinky_On(const Blinky_T * p_blinky);
extern void Blinky_Off(const Blinky_T * p_blinky);
extern void Blinky_Toggle(const Blinky_T * p_blinky);
extern void Blinky_Stop(const Blinky_T * p_blinky);
extern void Blinky_Blink_OnOff(const Blinky_T * p_blinky, uint32_t duration);
extern void Blinky_Blink_Toggle(const Blinky_T * p_blinky, uint32_t duration);
extern void Blinky_Blink(const Blinky_T * p_blinky, uint32_t onTime);
extern void Blinky_BlinkN(const Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime, uint8_t nRepeat);
extern void Blinky_StartPeriodic(const Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime);

#endif
