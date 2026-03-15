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
#include <stdint.h>
#include <stdbool.h>


/******************************************************************************/
/*
    Runtime State
*/
/******************************************************************************/
typedef struct Debounce
{
    uint16_t DebounceTime;      /* Configurable debounce time */
    uint32_t Time0;             /* Start time of current state */
    bool State0;                /* Bounce/input state */ /* An extra state for a hot path without the timer check */
    bool Output;                /* Debounced output state */
    bool OutputPrev;            /* Previous debounced state for edge detection */
}
Debounce_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
/* recurrent form */
/* Time divergent */
static inline bool debounce(uint32_t stability_time, bool output, bool input_t0, uint32_t elapsed_time)
{
    if (input_t0 == output) return output;
    if (elapsed_time < stability_time) return output;
    return input_t0;
}


/*!
    @return the debounced state
*/
static inline bool Debounce_Poll(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    if (pinState != p_debounce->State0)
    {
        p_debounce->Time0 = currentTime;
        p_debounce->State0 = pinState;
    }

    p_debounce->Output = debounce(p_debounce->DebounceTime, p_debounce->Output, p_debounce->State0, (currentTime - p_debounce->Time0));
    return p_debounce->Output;
}

/*!
    @return true if state changed
*/
static inline bool Debounce_PollEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    p_debounce->OutputPrev = p_debounce->Output;
    Debounce_Poll(p_debounce, currentTime, pinState);
    return (p_debounce->Output != p_debounce->OutputPrev);
    /* alternatively call handle async edge detect */
    // return (p_debounce->Output != Debounce_Poll(p_debounce, currentTime, pinState));
}


/******************************************************************************/
/*
    Inline Accessors
*/
/******************************************************************************/
static inline bool Debounce_GetState(const Debounce_T * p_debounce) { return p_debounce->Output; }

/*
    Configuration
*/
static inline void Debounce_SetTime(Debounce_T * p_debounce, uint16_t millis) { p_debounce->DebounceTime = millis; }
static inline uint16_t Debounce_GetTime(const Debounce_T * p_debounce) { return p_debounce->DebounceTime; }

static void Debounce_Init(Debounce_T * p_debounce, uint16_t debounceTime)
{
    p_debounce->DebounceTime = debounceTime;
    p_debounce->Time0 = 0UL;
    p_debounce->State0 = false;
    p_debounce->Output = false;
    p_debounce->OutputPrev = false;
}


/*
    handle by din
*/
typedef enum Debounce_Edge
{
    DEBOUNCE_EDGE_FALLING = -1,
    DEBOUNCE_EDGE_NULL = 0,
    DEBOUNCE_EDGE_RISING = 1,
}
Debounce_Edge_T;



/* Optional convenience functions */
static inline bool Debounce_PollRisingEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    return Debounce_PollEdge(p_debounce, currentTime, pinState) && (p_debounce->Output == true);
}

static inline bool Debounce_PollFallingEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    return Debounce_PollEdge(p_debounce, currentTime, pinState) && (p_debounce->Output == false);
}

static inline Debounce_Edge_T Debounce_PollEdgeValue(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    return Debounce_PollEdge(p_debounce, currentTime, pinState) ? (Debounce_Edge_T)(p_debounce->Output - p_debounce->OutputPrev) : DEBOUNCE_EDGE_NULL;
}


static inline bool Debounce_IsEdge(const Debounce_T * p_debounce) { return (p_debounce->Output != p_debounce->OutputPrev); }
static inline bool Debounce_IsRisingEdge(const Debounce_T * p_debounce) { return ((p_debounce->Output == true) && (p_debounce->OutputPrev == false)); }
static inline bool Debounce_IsFallingEdge(const Debounce_T * p_debounce) { return ((p_debounce->Output == false) && (p_debounce->OutputPrev == true)); }
static inline Debounce_Edge_T Debounce_GetEdge(const Debounce_T * p_debounce) { return (Debounce_Edge_T)(p_debounce->Output - p_debounce->OutputPrev); }



