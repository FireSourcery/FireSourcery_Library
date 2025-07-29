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
    @file   math_hysteresis.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Hysteresis Functions - Boolean State
    For digital state management with noise immunity
    Following ISA-5.1 and IEC 61131-3 conventions
*/
/******************************************************************************/
// static inline bool _is_in_hysteresis(int32_t threshold_off, bool state, int32_t input) { return ((state == true) && (input >= threshold_off)); }
// static inline bool _is_hysteresis_on(int32_t threshold_on, int32_t threshold_off, bool state, int32_t input) { return (input >= threshold_on) || _is_in_hysteresis( ); }

/*!
    @brief  Standard hysteresis function for rising edge trigger
            maintain state on falling

    Note: setpoint > resetpoint for normal operation
*/
static inline bool hysteresis_output_state(int32_t setpoint, int32_t resetpoint, bool prev_state, int32_t process_value)
{
    if (process_value <= resetpoint) { return false; }      /* Below reset - deactivate */
    if (process_value >= setpoint) { return true; }         /* Above setpoint - activate */
    return prev_state; /* Maintain state in hysteresis band */
}

/*!
    @brief  Inverted hysteresis for falling edge trigger

    Note: setpoint < resetpoint for inverted operation (low alarms)
*/
static inline bool hysteresis_output_state_inverted(int32_t setpoint, int32_t resetpoint, bool prev_state, int32_t process_value)
{
    if (process_value >= resetpoint) { return false; }      /* Above reset - deactivate */
    if (process_value <= setpoint) { return true; }         /* Below setpoint - activate */
    return prev_state; /* Maintain state in hysteresis band */
}

/* 3 state */
// static inline int hysteresis_output_region(int32_t setpoint, int32_t resetpoint, int32_t process_value)
// {
//     if (process_value <= resetpoint) { return 0; }
//     if (process_value >= setpoint) { return 1; }
//     return -1;
// }

// static inline int hysteresis_output_sign(int32_t setpoint, int32_t resetpoint, int32_t process_value)
// {
//     if (process_value <= resetpoint) { return -1; }
//     if (process_value >= setpoint) { return 1; }
//     return 0;
// }

/******************************************************************************/
/*
    Aysemmetric Hysteresis Functions - Value Output
*/
/******************************************************************************/
/*!
    @brief  Sticky threshold filter - "sticks" to threshold when triggered

    can subsitute hysteresis_output_state, offering direct output scaling
    hysteresis deadband effective in reset direction only
    maintain state on falling/reset
*/
static inline int32_t hysteresis_on_falling(int32_t setpoint, int32_t resetpoint, int32_t prev_output, int32_t input)
{
    if (input <= resetpoint || input >= setpoint) { return input; }
    return (prev_output >= setpoint) ? setpoint : input;    /* In hysteresis band - check if previously triggered */
}

static inline int32_t hysteresis_on_rising(int32_t setpoint, int32_t resetpoint, int32_t prev_output, int32_t input)
{
    if (input >= resetpoint || input <= setpoint) { return input; }
    return (prev_output <= setpoint) ? setpoint : input;    /* In hysteresis band - check if previously triggered */
}

/******************************************************************************/
/*
    Hysteresis Functions - Value Output
    For signal conditioning and noise filtering
*/
/******************************************************************************/
/*!
    @brief  Deadband filter with hysteresis - maintains previous output in dead zone
            Pass-through hysteresis - preserves input value outside hysteresis band
*/
static inline int32_t hysteresis_deadband_filter(int32_t threshold_upper, int32_t threshold_lower, int32_t prev_output, int32_t input)
{
    if (input <= threshold_lower || input >= threshold_upper) { return input; }
    return prev_output;
}

/*!
    @brief  Clamping filter with hysteresis - clamps output to threshold values
*/
static inline int32_t hysteresis_clamp_filter(int32_t threshold_upper, int32_t threshold_lower, int32_t prev_output, int32_t input)
{
    if (input <= threshold_lower) { return threshold_lower; }
    if (input >= threshold_upper) { return threshold_upper; }
    return prev_output;
}

/*!
    @brief  Delta-based noise filter - only updates when change exceeds threshold
*/
static int32_t hysteresis_delta_filter(uint32_t hysteresis, int32_t prev_output, int32_t input)
{
    return (abs(input - prev_output) > hysteresis) ? input : prev_output;
}


/******************************************************************************/
/*
*/
/******************************************************************************/


/* Edge detection with hysteresis */
// static inline sign_t hysteresis_crossing_sign(int32_t threshold_upper, int32_t threshold_lower, bool prev_state, int32_t input)
// {
//     return edge_sign(prev_state, is_hysteresis_on(threshold_upper, threshold_lower, prev_state, input));
// }

/* 2 threshold setpoints determined by  */
/* Hysteresis window checking */
// static inline bool is_hystersis_in_window(int32_t min_value, int32_t max_value, int32_t hysteresis, bool prev_in_range, int32_t value)
// {
//     if (prev_in_range)
//     {
//         /* Previously in range - use expanded window to exit */
//         return math_is_in_range(value, min_value - hysteresis, max_value + hysteresis);
//     }
//     else
//     {
//         /* Previously out of range - use normal window to enter */
//         return math_is_in_range(value, min_value, max_value);
//     }
// }