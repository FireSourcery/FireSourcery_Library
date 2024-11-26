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
    @brief  Re-implementation of C++ library not found in C
    @version V0
*/
/******************************************************************************/
#ifndef MATH_GENERAL_H
#define MATH_GENERAL_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

static inline int32_t math_max(int32_t value1, int32_t value2) { return ((value1 > value2) ? value1 : value2); }
static inline int32_t math_min(int32_t value1, int32_t value2) { return ((value1 < value2) ? value1 : value2); }
static inline int32_t math_clamp(int32_t value, int32_t low, int32_t high) { return math_min(math_max(value, low), high); }
static inline uint32_t math_abs(int32_t value) { return abs(value); } // todo check - INT_MIN
static inline int32_t math_sign(int32_t value) { return (value > 0) - (value < 0); } /* +1, 0, -1 */

static inline bool math_is_bounded(int32_t value, int32_t low, int32_t high) { return (value == math_clamp(value, low, high)); }

static inline int32_t math_add_sat(int32_t a, int32_t b)
{
    // __builtin_sadd(a, b); //todo
    int32_t result = a + b;
    if      ((a > 0) && (b > 0) && (result < 0)) { result = INT32_MAX; } /* (a > 0 && b > INT_MAX - a) */
    else if ((a < 0) && (b < 0) && (result > 0)) { result = (0 - INT32_MAX); } /* (a < 0 && b < INT_MIN - a) */
    return result;
}

static inline uintptr_t align_down(uintptr_t address, size_t align) { return (address & (-align)); }
static inline uintptr_t align_up(uintptr_t address, size_t align) { return (-(-address & (-align))); }
static inline bool is_aligned(uintptr_t address, size_t align) { return ((address & (align - 1U)) == (uintptr_t)0U); }

static inline bool is_aligned_mask(uint32_t value, uint32_t mask) { return ((value & mask) == (uint32_t)0U); }

extern uint32_t math_muldiv64_unsigned(uint32_t value, uint32_t factor, uint32_t divisor);

#endif


