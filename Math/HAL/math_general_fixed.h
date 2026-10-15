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
    @file   fixed_general.h
    @author FireSourcery
    @brief  Re-implementation of C++ library not available in C
*/
/******************************************************************************/
#include "../math_general.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))



/******************************************************************************/
/*
    HAL select,
    or generic select, concurrent with float
*/
/******************************************************************************/
static inline uint32_t fixed_abs(int32_t value) { return abs(value); } /* INT32_MIN returns INT32_MAX + 1 */

static inline int32_t fixed_max(int32_t value1, int32_t value2) { return ((value1 > value2) ? value1 : value2); }
static inline int32_t fixed_min(int32_t value1, int32_t value2) { return ((value1 < value2) ? value1 : value2); }
static inline int32_t fixed_clamp(int32_t value, int32_t lower, int32_t upper) { return fixed_min(fixed_max(value, lower), upper); }

static inline int32_t fixed_clamp_abs(int32_t x, int32_t limit)
{
    int32_t abs = fixed_abs(limit);
    return fixed_clamp(x, -abs, abs);
}

static inline bool fixed_is_in_range(int32_t value, int32_t lower, int32_t upper) { return (value >= lower) && (value <= upper); }
static inline bool fixed_is_out_of_range(int32_t value, int32_t lower, int32_t upper) { return (value < lower) || (value > upper); }

static inline bool fixed_in_wrap_window(int32_t x, int32_t lo, int32_t hi) { return (uint32_t)(x - lo) <= (uint32_t)(hi - lo); }

/* positive as lshift */
static inline int32_t fixed_shift_signed(int32_t value, int8_t shift) { return (shift > 0) ? (value << shift) : (value >> (-shift)); }


/*
    non domain specific
*/

/* +1, 0, -1 */
static inline sign_t fixed_sign(int32_t value) { return (value > 0) - (value < 0); }
static inline bool fixed_is_sign_diff(int32_t value1, int32_t value2) { return ((value1 ^ value2) < 0); }

/* +1,+1 -> 1, +1,-1 -> 0, -1,-1 -> -1 */
// static inline sign_t sign_sum(sign_t sign1, sign_t sign2) { return ((sign1 + sign2) / 2); }

static inline bool fixed_sign_bit(sign_t value) { return (value < 0); } /* reduce sign to bool */
static inline int32_t fixed_sign_mask(int32_t value) { return (value >> 31); } /* 0xFFFFFFFF for negative, 0x00000000 for positive */


