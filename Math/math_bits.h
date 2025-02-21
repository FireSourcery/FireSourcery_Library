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
    @file   math_general.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MATH_BITS_H
#define MATH_BITS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* pow2 only */
static inline uintptr_t align_down(uintptr_t bits, size_t align) { return (bits & (-align)); }
static inline uintptr_t align_up(uintptr_t bits, size_t align) { return (-((-bits) & (-align))); }
static inline bool is_aligned(uintptr_t bits, size_t align) { return ((bits & (align - 1U)) == (uintptr_t)0U); }

/* is_aligned_mask */
static inline bool is_masked(uint32_t value, uint32_t mask) { return ((value & mask) == (uint32_t)0U); }

static inline uint32_t bits_edge(uint32_t prevState, uint32_t newState) { return (prevState ^ newState); }
/* 1: Rising Edge, 0: No Change, -1: Falling Edge */
static inline int8_t edge_value(bool prevState, bool newState) { return ((int8_t)newState - (int8_t)prevState); }

static inline bool is_falling_edge(bool prevState, bool newState)   { return ((prevState == true) && (newState == false)); }
static inline bool is_rising_edge(bool prevState, bool newState)    { return ((prevState == false) && (newState == true)); }
static inline bool is_edge(bool prevState, bool newState)           { return (prevState != newState); }
static inline int8_t edge_value(bool prevState, bool newState)      { return ((int8_t)newState - (int8_t)prevState); } /* -1: falling edge, 0: no edge, 1: rising edge */
static inline uint32_t bits_edges(uint32_t prevState, uint32_t newState)  { return (prevState ^ newState); }

#endif


