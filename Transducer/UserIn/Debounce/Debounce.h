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
// #include "Transducer/Math/math_edge.h"

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
    /* An extra state is used for a hot path without the timer check */
    bool PinState;              /* Bounce/input state */
    bool Output;                /* Debounced output state */
    bool OutputPrev;            /* Previous debounced state for edge detection */
}
Debounce_T;

/******************************************************************************/
/*

*/
/******************************************************************************/
/*!
    @return the debounced state
*/
static inline bool Debounce_Filter(const Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    if (pinState == p_debounce->Output) { return p_debounce->Output; }  /* Skip the timer check */
    return ((currentTime - p_debounce->TimeStart > p_debounce->DebounceTime) ? pinState : p_debounce->Output); /* No wrap if milliseconds, ~49 days */
}

/*!
    @return the debounced state
*/
static inline bool Debounce_Poll(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    if (pinState != p_debounce->PinState)
    {
        p_debounce->TimeStart = currentTime;
        p_debounce->PinState = pinState;
    }
    else /* Check if state is the same for specified duration */
    {
        p_debounce->Output = Debounce_Filter(p_debounce, currentTime, pinState);
    }
    return p_debounce->Output;
}

/*
    Edge detection with debounce
    alternatively move to UserDin
*/
/*!
    @return true if state changed
*/
static inline bool Debounce_PollEdge(Debounce_T * p_debounce, uint32_t currentTime, bool pinState)
{
    p_debounce->OutputPrev = p_debounce->Output;
    Debounce_Poll(p_debounce, currentTime, pinState);
    return (p_debounce->Output != p_debounce->OutputPrev);
}

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


/******************************************************************************/
/*
    Inline Accessors
*/
/******************************************************************************/
static inline bool Debounce_GetState(const Debounce_T * p_debounce) { return p_debounce->Output; }
/* mixin math_edge.h */
static inline bool Debounce_IsEdge(const Debounce_T * p_debounce) { return (p_debounce->Output != p_debounce->OutputPrev); }
static inline bool Debounce_IsRisingEdge(const Debounce_T * p_debounce) { return ((p_debounce->Output == true) && (p_debounce->OutputPrev == false)); }
static inline bool Debounce_IsFallingEdge(const Debounce_T * p_debounce) { return ((p_debounce->Output == false) && (p_debounce->OutputPrev == true)); }
static inline Debounce_Edge_T Debounce_GetEdge(const Debounce_T * p_debounce) { return (Debounce_Edge_T)(p_debounce->Output - p_debounce->OutputPrev); }

static inline void Debounce_SetTime(Debounce_T * p_debounce, uint16_t millis) { p_debounce->DebounceTime = millis; }
static inline uint16_t Debounce_GetTime(const Debounce_T * p_debounce) { return p_debounce->DebounceTime; }


static void Debounce_Init(Debounce_T * p_debounce, uint32_t debounceTime)
{
    p_debounce->DebounceTime = debounceTime;
    p_debounce->TimeStart = 0UL;
    p_debounce->Output = false;
    p_debounce->OutputPrev = false;
    p_debounce->PinState = false;
}



/******************************************************************************/
/* Time-based debounce */
static inline bool debounce_is_stable(uint32_t stability_time, uint32_t elapsed_time, bool prev_state, bool input_state)
{
    // if (input_state != prev_state) return false;  /* State changed, not stable */
    // return (elapsed_time >= stability_time);
    return ((input_state == prev_state) && (elapsed_time >= stability_time));
}

static inline bool debounce(uint32_t stability_time, uint32_t elapsed_time, bool prev_state, bool input_state)
{
    // if (input_state == prev_state) return prev_state;  /* No change in state, return previous state */
    // return (elapsed_time >= stability_time ? input_state : prev_state); /* Return stable state if elapsed time exceeds stability time */
    return (debounce_is_stable(stability_time, elapsed_time, prev_state, input_state)) ? input_state : prev_state;
}

// /* Counter-based debounce */
// static inline bool debounce_tick( uint16_t threshold, uint16_t  counter, bool target_state, bool input_state)
// {

// }

// static inline bool debounce_tick(uint16_t * p_counter, uint16_t threshold, bool target_state, bool input_state)
// {
//     if (input_state == target_state)
//     {
//         if (*p_counter < threshold) { (*p_counter)++; }
//     }
//     else
//     {
//         *p_counter = 0;
//     }
//     return (*p_counter >= threshold);
// }
