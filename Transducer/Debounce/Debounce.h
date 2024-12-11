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
    @file Debounce.h
    @author FireSourcery
    @brief Din
    @version V0
*/
/******************************************************************************/
#ifndef PIN_DEBOUNCE_H
#define PIN_DEBOUNCE_H

#include "Peripheral/Pin/Pin.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum Debounce_Edge
{
    DEBOUNCE_EDGE_NULL = 0,
    DEBOUNCE_EDGE_RISING = 1,
    DEBOUNCE_EDGE_FALLING = -1,
}
Debounce_Edge_T;

typedef const struct Debounce_Const
{
    const volatile uint32_t * const P_TIMER;
}
Debounce_Const_T;

typedef struct Debounce
{
    Debounce_Const_T CONST;
    Pin_T Pin;
    uint16_t DebounceTime;    /* Outer module set single config param */
    uint16_t TimeStart;
    bool PinState;
    bool DebouncedState;
    bool EdgeState; /* for edge read */
}
Debounce_T;

#define DEBOUNCE_INIT(p_PinHal, PinId, PinIsInvert, p_Timer)   \
{                                                              \
    .CONST =  { .P_TIMER = p_Timer, },                         \
    .Pin = PIN_INIT_INVERT(p_PinHal, PinId, PinIsInvert),      \
}

#define DEBOUNCE_INIT_STRUCT(PinInit, DebounceConst)    \
{                                                       \
    .CONST = DebounceConst,                             \
    .Pin = PinInit,                                     \
}

static inline bool is_falling_edge(bool prevState, bool newState)   { return ((prevState == true) && (newState == false)); }
static inline bool is_rising_edge(bool prevState, bool newState)    { return ((prevState == false) && (newState == true)); }
static inline bool is_edge(bool prevState, bool newState)           { return (prevState != newState); }
static inline int8_t edge_value(bool prevState, bool newState)      { return ((int8_t)newState - (int8_t)prevState); } /* -1: falling edge, 0: no edge, 1: rising edge */
static inline uint32_t bits_edges(uint32_t prevState, uint32_t newState)  { return (prevState ^ newState); }

// static inline bool _Debounce_IsFallingEdge(Debounce_T * p_debounce, bool pinState)   { return ((pinState == false) && (p_debounce->EdgeState == true)); }

static inline bool Debounce_GetState(const Debounce_T * p_debounce) { return p_debounce->DebouncedState; }
// static inline bool Debounce_GetRawState(const Debounce_T * p_debounce)  { return p_debounce->PinState; }
/* During same synchronous cycle only */
static inline bool Debounce_IsEdge(Debounce_T * p_debounce)         { return (p_debounce->DebouncedState != p_debounce->EdgeState); }
static inline bool Debounce_IsRisingEdge(Debounce_T * p_debounce)   { return ((p_debounce->DebouncedState == true) && (p_debounce->EdgeState == false)); }
static inline bool Debounce_IsFallingEdge(Debounce_T * p_debounce)  { return ((p_debounce->DebouncedState == false) && (p_debounce->EdgeState == true)); }
static inline Debounce_Edge_T Debounce_GetEdge(Debounce_T * p_debounce) { return (Debounce_Edge_T)(p_debounce->DebouncedState - p_debounce->EdgeState); }
// { return ((Debounce_IsEdge(p_debounce) == true) ? ((p_debounce->DebouncedState = true) ? DEBOUNCE_EDGE_RISING : DEBOUNCE_EDGE_FALLING) : DEBOUNCE_EDGE_NULL); }

static inline void Debounce_SetTime(Debounce_T * p_debounce, uint16_t millis) { p_debounce->DebounceTime = millis; }
// static inline void Debounce_EnableInvert(Debounce_T * p_debounce) { Pin_EnableInvert(&p_debounce->Pin); }
// static inline void Debounce_DisableInvert(Debounce_T * p_debounce) { Pin_DisableInvert(&p_debounce->Pin); }

extern void Debounce_Init(Debounce_T * p_debounce, uint16_t debounceTime);
extern bool Debounce_CaptureState(Debounce_T * p_debounce);
extern bool Debounce_PollFallingEdge(Debounce_T * p_debounce);
extern bool Debounce_PollRisingEdge(Debounce_T * p_debounce);
extern Debounce_Edge_T Debounce_PollEdge(Debounce_T * p_debounce);

//todo capturetime for long and short press
//static inline uint16_t Debounce_GetStateTime()   { }
//static inline bool Debounce_GetLongPress(Debounce_T * p_debounce){}
//static inline bool Debounce_GetShortPress(Debounce_T * p_debounce){}

#endif /* DEBOUNCE_H */
