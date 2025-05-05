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
    @brief  Re-implementation of C++ library not available in C
    @version V0
*/
/******************************************************************************/
#ifndef MATH_GENERAL_H
#define MATH_GENERAL_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// typedef int32_t sign_t;
// typedef enum sign { NEGATIVE = -1, ZERO = 0, POSITIVE = 1 } sign_t;

/* simplify with return by value */
// typedef struct pair16 { int16_t x; int16_t y; } pair16_t; /* point, vector, limits */
// typedef struct triplet16 { int16_t x; int16_t y; int16_t z; } triplet16_t;

static inline int32_t math_max(int32_t value1, int32_t value2) { return ((value1 > value2) ? value1 : value2); }
static inline int32_t math_min(int32_t value1, int32_t value2) { return ((value1 < value2) ? value1 : value2); }
static inline int32_t math_limit_upper(int32_t value, int32_t upper) { return math_min(value, upper); }
static inline int32_t math_limit_lower(int32_t value, int32_t lower) { return math_max(value, lower); }

static inline int32_t math_clamp(int32_t value, int32_t lower, int32_t upper) { return math_min(math_max(value, lower), upper); }
static inline bool math_is_clamped(int32_t value, int32_t lower, int32_t upper) { return (value == math_clamp(value, lower, upper)); }



static inline uint32_t math_abs(int32_t value) { return abs(value); } /* INT32_MIN returns INT32_MAX + 1 */

static inline int32_t math_sign(int32_t value) { return (value > 0) - (value < 0); } /* +1, 0, -1 */
static inline bool math_is_sign_diff(int32_t value1, int32_t value2) { return ((value1 ^ value2) < 0); }
static inline int32_t math_sign_mask(int32_t value) { return (value >> 31); } /* 0xFFFFFFFF for negative, 0x00000000 for positive */

static inline int32_t math_shift(int32_t value, int8_t shift) { return (shift > 0) ? (value << shift) : (value >> (-shift)); }

static inline int32_t math_add_sat(int32_t a, int32_t b)
{
#if defined(__GNUC__)
    int32_t result;
    if (__builtin_add_overflow(a, b, &result))
    {
        if      (a > 0 && b > 0) { result = INT32_MAX; }
        else if (a < 0 && b < 0) { result = (0 - INT32_MAX); }
    }
    return result;
#else
    int32_t result = a + b;
    if      ((a > 0) && (b > 0) && (result < 0)) { result = INT32_MAX; }
    else if ((a < 0) && (b < 0) && (result > 0)) { result = (0 - INT32_MAX); }
    return result;
#endif
}


#endif


