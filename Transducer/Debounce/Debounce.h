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
    @file    Debounce.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef PIN_DEBOUNCE_H
#define PIN_DEBOUNCE_H

#include "Peripheral/Pin/Pin.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum Debounce_Edge_Tag
{
    DEBOUNCE_EDGE_NULL,
    DEBOUNCE_EDGE_FALLING,
    DEBOUNCE_EDGE_RISING,
}
Debounce_Edge_T;

typedef const struct Debounce_Config_Tag
{
    const volatile uint32_t * const P_TIMER;
}
Debounce_Config_T;

typedef struct Debounce_Tag
{
    Debounce_Config_T CONFIG;
    Pin_T Pin;
    uint16_t DebounceTime;    /* Outer module set single param */
    uint16_t TimePrev;
    bool DebouncedState;
    bool DebouncedStatePrev; /* for edge read */
    bool RawStatePrev;
}
Debounce_T;

#define DEBOUNCE_INIT(p_PinHal, PinId, PinIsInvert, p_Timer)     \
{                                                                \
    .CONFIG =  { .P_TIMER = p_Timer, },                            \
    .Pin = PIN_INIT_INVERT(p_PinHal, PinId, PinIsInvert),        \
}

static inline bool Debounce_GetState(const Debounce_T * p_debounce)     { return p_debounce->DebouncedState;}
static inline bool Debounce_GetRawState(const Debounce_T * p_debounce)     { return p_debounce->RawStatePrev; }

/* During same synchronous cycle only */
static inline bool Debounce_GetIsFallingEdge(Debounce_T * p_debounce)   { return ((p_debounce->DebouncedState == false) && (p_debounce->DebouncedStatePrev == true)); }
static inline bool Debounce_GetIsRisingEdge(Debounce_T * p_debounce)    { return ((p_debounce->DebouncedState == true) && (p_debounce->DebouncedStatePrev == false)); }
static inline bool Debounce_GetIsDualEdge(Debounce_T * p_debounce)      { return ((p_debounce->DebouncedState ^ p_debounce->DebouncedStatePrev) == true); }
static inline Debounce_Edge_T Debounce_GetDualEdge(Debounce_T * p_debounce)
{
    return ((Debounce_GetIsDualEdge(p_debounce) == true) ? ((p_debounce->DebouncedState = true) ? DEBOUNCE_EDGE_RISING : DEBOUNCE_EDGE_FALLING) : DEBOUNCE_EDGE_NULL);
}

static inline void Debounce_SetTime(Debounce_T * p_debounce, uint16_t millis) { p_debounce->DebounceTime = millis; }
// static inline void Debounce_EnableInvert(Debounce_T * p_debounce) { Pin_EnableInvert(&p_debounce->Pin); }
// static inline void Debounce_DisableInvert(Debounce_T * p_debounce) { Pin_DisableInvert(&p_debounce->Pin); }

extern void Debounce_Init(Debounce_T * p_debounce, uint16_t debounceTime);
extern bool Debounce_CaptureState(Debounce_T * p_debounce);
extern bool Debounce_PollFallingEdge(Debounce_T * p_debounce);
extern bool Debounce_PollRisingEdge(Debounce_T * p_debounce);
extern Debounce_Edge_T Debounce_PollDualEdge(Debounce_T * p_debounce);

//static inline uint16_t Debounce_GetStateTime()   { }
//static inline bool Debounce_GetLongPress(Debounce_T * p_debounce){}
//todo capturetime for long and short press
//static inline bool Debounce_GetShortPress(Debounce_T * p_debounce){}

#endif /* DEBOUNCE_H */
