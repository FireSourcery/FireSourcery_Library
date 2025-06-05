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
    @file   math_edge.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/math_general.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    DIn - Digital Input State Functions
    Boolean state transitions and edge detection
*/
/******************************************************************************/
static inline bool is_rising_edge(bool prev_state, bool state) { return ((prev_state == false) && (state == true)); }
static inline bool is_falling_edge(bool prev_state, bool state) { return ((prev_state == true) && (state == false)); }
static inline bool is_edge(bool prev_state, bool state) { return (prev_state != state); }

/* 1: Rising Edge, 0: No Change, -1: Falling Edge */
static inline sign_t edge_sign(bool prev_state, bool state) { return ((int8_t)state - (int8_t)prev_state); }

/* Bit-wise edge detection for multiple digital inputs */
static inline uint32_t bits_edges(uint32_t prev_states, uint32_t states) { return (prev_states ^ states); }
static inline uint32_t bits_rising_edges(uint32_t prev_states, uint32_t states) { return (~prev_states) & states; }
static inline uint32_t bits_falling_edges(uint32_t prev_states, uint32_t states) { return prev_states & (~states); }


/******************************************************************************/
/*
    AIn - Analog Input State Functions
    Value-based state transitions, to/from zero
*/
/******************************************************************************/
/* Edge detection based on zero crossing */
// static inline bool is_value_rising_edge(int32_t prev, int32_t new) { return is_rising_edge((prev != 0), (new != 0)); }
// static inline bool is_value_falling_edge(int32_t prev, int32_t new) { return is_falling_edge((prev != 0), (new != 0)); }
// static inline bool is_value_edge(int32_t prev, int32_t new) { return is_edge((prev != 0), (new != 0)); }
// static inline sign_t value_edge_sign(int32_t prev, int32_t new) { return edge_sign((prev != 0), (new != 0)); }
/* is_rising_zero_crossing */
static inline bool is_value_rising_edge(int32_t prev, int32_t new) { return is_rising_edge((prev > 0), (new > 0)); }
static inline bool is_value_falling_edge(int32_t prev, int32_t new) { return is_falling_edge((prev > 0), (new > 0)); }
static inline bool is_value_edge(int32_t prev, int32_t new) { return is_edge((prev > 0), (new > 0)); }
static inline sign_t value_edge_sign(int32_t prev, int32_t new) { return edge_sign((prev > 0), (new > 0)); }


/******************************************************************************/
/*
    Threshold Crossing Detection
    For fault monitoring, limit detection, etc.
*/
/******************************************************************************/
/* is_crossing_rising_edge */
static inline bool is_rising_crossing(int32_t threshold_on, int32_t prev, int32_t value) { return is_rising_edge((prev >= threshold_on), (value >= threshold_on)); }
static inline bool is_falling_crossing(int32_t threshold_off, int32_t prev, int32_t value) { return is_falling_edge((prev >= threshold_off), (value >= threshold_off)); }
static inline bool is_crossing(int32_t threshold, int32_t prev, int32_t value) { return is_edge((prev >= threshold), (value >= threshold)); }
static inline sign_t crossing_sign(int32_t threshold, int32_t prev, int32_t value) { return edge_sign((prev >= threshold), (value >= threshold)); }


/* rising crossing with state */
static inline bool is_rising_schmitt_trigger(int32_t threshold_on, bool prev_state, int32_t input) { return is_rising_edge(prev_state, (input > threshold_on)); }
static inline bool is_falling_schmitt_trigger(int32_t threshold_off, bool prev_state, int32_t input) { return is_falling_edge(prev_state, (input < threshold_off)); }

