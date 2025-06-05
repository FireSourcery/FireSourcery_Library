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
    @file   Debounce.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Transducer/Math/math_edge.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum Debounce_Edge
{
    DEBOUNCE_EDGE_FALLING = -1,
    DEBOUNCE_EDGE_NULL = 0,
    DEBOUNCE_EDGE_RISING = 1,
}
Debounce_Edge_T;

/******************************************************************************/
/*
    Runtime State Only - No Pin or Timer References
*/
/******************************************************************************/
typedef struct Debounce
{
    uint16_t DebounceTime;      /* Configurable debounce time */
    uint32_t TimeStart;         /* Start time of current state */
    bool PinState;              /* Current raw pin state */
    bool DebouncedState;        /* Debounced output state */
    bool DebouncedStatePrev;    /* Previous debounced state for edge detection */
}
Debounce_T;


/******************************************************************************/
/*

*/
/******************************************************************************/
/*!
    @return the debounced state
*/
static inline bool Debounce_Filter(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    if (pinState != p_debounce->PinState) /* Check if state is the same for specified duration */
    {
        p_debounce->TimeStart = currentTime;
        p_debounce->PinState = pinState;
        return p_debounce->DebouncedState;
    }
    else
    {
        return (currentTime - p_debounce->TimeStart > p_debounce->DebounceTime) ? pinState : p_debounce->DebouncedState;
    }
}

/*!
    @return true if state changed
*/
static inline bool Debounce_PollEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    if (Debounce_Filter(p_debounce, currentTime, pinState) != p_debounce->DebouncedState)
    {
        p_debounce->DebouncedStatePrev = p_debounce->DebouncedState;
        p_debounce->DebouncedState = pinState; /* pinState == filtered state past conditional */
        return true;  /* State changed */
    }
    else
    {
        return false; /* No change */
    }
}

/* Optional convenience functions */
static inline bool Debounce_PollRisingEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    return Debounce_PollEdge(p_debounce, currentTime, pinState) && (p_debounce->DebouncedState == true);
}

static inline bool Debounce_PollFallingEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    return Debounce_PollEdge(p_debounce, currentTime, pinState) && (p_debounce->DebouncedState == false);
}

static inline Debounce_Edge_T Debounce_PollEdgeValue(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    return Debounce_PollEdge(p_debounce, currentTime, pinState) ? (Debounce_Edge_T)(p_debounce->DebouncedState - p_debounce->DebouncedStatePrev) : DEBOUNCE_EDGE_NULL;
}


/******************************************************************************/
/*
    Inline Accessors
*/
/******************************************************************************/
static inline bool Debounce_GetState(const Debounce_T * p_debounce) { return p_debounce->DebouncedState; }
static inline bool Debounce_IsEdge(const Debounce_T * p_debounce) { return (p_debounce->DebouncedState != p_debounce->DebouncedStatePrev); }
static inline bool Debounce_IsRisingEdge(const Debounce_T * p_debounce) { return ((p_debounce->DebouncedState == true) && (p_debounce->DebouncedStatePrev == false)); }
static inline bool Debounce_IsFallingEdge(const Debounce_T * p_debounce) { return ((p_debounce->DebouncedState == false) && (p_debounce->DebouncedStatePrev == true)); }
static inline Debounce_Edge_T Debounce_GetEdge(const Debounce_T * p_debounce) { return (Debounce_Edge_T)(p_debounce->DebouncedState - p_debounce->DebouncedStatePrev); }

static inline void Debounce_SetTime(Debounce_T * p_debounce, uint16_t millis) { p_debounce->DebounceTime = millis; }
static inline uint16_t Debounce_GetTime(const Debounce_T * p_debounce) { return p_debounce->DebounceTime; }

/******************************************************************************/
/*
    Public Functions - External pin state and timer inputs
*/
/******************************************************************************/
extern void Debounce_Init(Debounce_T * p_debounce, uint32_t debounceTime);

