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
    @file   math_general.h
    @author FireSourcery
    @brief  Re-implementation of C++ library not available in C
*/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))


static inline uint32_t math_abs(int32_t value) { return abs(value); } /* INT32_MIN returns INT32_MAX + 1 */

static inline int32_t math_max(int32_t value1, int32_t value2) { return ((value1 > value2) ? value1 : value2); }
static inline int32_t math_min(int32_t value1, int32_t value2) { return ((value1 < value2) ? value1 : value2); }
static inline int32_t math_clamp(int32_t value, int32_t lower, int32_t upper) { return math_min(math_max(value, lower), upper); }

static inline int32_t math_clamp_abs(int32_t x, int32_t limit)
{
    int32_t abs = math_abs(limit);
    return math_clamp(x, -abs, abs);
}

static inline bool math_is_in_range(int32_t value, int32_t lower, int32_t upper) { return (value >= lower) && (value <= upper); }
static inline bool math_is_out_of_range(int32_t value, int32_t lower, int32_t upper) { return (value < lower) || (value > upper); }

static inline bool math_in_wrap_window(int32_t x, int32_t lo, int32_t hi) { return (uint32_t)(x - lo) <= (uint32_t)(hi - lo); }

/* positive as lshift */
static inline int32_t math_shift_signed(int32_t value, int8_t shift) { return (shift > 0) ? (value << shift) : (value >> (-shift)); }


/* alias */
static inline int32_t math_limit_upper(int32_t value, int32_t upper) { return math_min(value, upper); }
static inline int32_t math_limit_lower(int32_t value, int32_t lower) { return math_max(value, lower); }


/*
    non domain specific
*/
/* sign */
typedef enum sign { SIGN_NEGATIVE = -1, SIGN_ZERO = 0, SIGN_POSITIVE = 1 } sign_t;

/* +1, 0, -1 */
static inline sign_t math_sign(int32_t value) { return (value > 0) - (value < 0); }
static inline bool math_is_sign_diff(int32_t value1, int32_t value2) { return ((value1 ^ value2) < 0); }

/* +1,+1 -> 1, +1,-1 -> 0, -1,-1 -> -1 */
// static inline sign_t sign_sum(sign_t sign1, sign_t sign2) { return ((sign1 + sign2) / 2); }

static inline bool math_sign_bit(sign_t value) { return (value < 0); } /* reduce sign to bool */
static inline int32_t math_sign_mask(int32_t value) { return (value >> 31); } /* 0xFFFFFFFF for negative, 0x00000000 for positive */

/*
    interval
*/
typedef struct { int32_t low; int32_t high; } interval_t;


/* smallest interval containing 0 and `value` */
static inline interval_t interval_of(int32_t value) { return (interval_t) { .low = math_min(0, value), .high = math_max(0, value) }; }

/* topological pair: magnitudes laid out as [-lowMag, +highMag] */
static inline interval_t interval_of_pair(int32_t lowMag, int32_t highMag) { return (interval_t) { .low = -lowMag, .high = +highMag }; }

/* sign-keyed half-plane: magnitude on the matching side, zero on the other */
static inline interval_t interval_of_sign(sign_t sign, uint32_t magnitude)
{
    switch (sign)
    {
        case SIGN_POSITIVE: return (interval_t) { .low = 0, .high = +magnitude };
        case SIGN_NEGATIVE: return (interval_t) { .low = -magnitude, .high = 0 };
        default:            return (interval_t) { .low = 0, .high = 0 };
    }
}

/* sign-keyed asymmetric pair: aligned magnitude goes with the sign, opposed goes against */
static inline interval_t interval_of_sign_pair(sign_t sign, int32_t alignedMag, int32_t opposedMag)
{
    switch (sign)
    {
        case SIGN_POSITIVE: return (interval_t) { .low = -opposedMag, .high = +alignedMag };
        case SIGN_NEGATIVE: return (interval_t) { .low = -alignedMag, .high = +opposedMag };
        default:            return (interval_t) { .low = 0, .high = 0 };
    }
}


static inline int32_t interval_clamp(int32_t v, interval_t b) { return math_clamp(v, b.low, b.high); }
static inline bool interval_contains(int32_t v, interval_t b) { return v >= b.low && v <= b.high; }
static inline interval_t interval_intersect(interval_t a, interval_t b) { return (interval_t) { .low = math_max(a.low, b.low), .high = math_min(a.high, b.high) }; }
// static inline interval_t interval_scale(interval_t b, fract16_t k) { return (interval_t) { .low = fract16_mul(b.low, k), .high = fract16_mul(b.high, k) }; }



/*
    define around 2 register return
*/
typedef struct { int32_t a; int32_t b; } pair_t;


/*

*/
static inline int32_t math_add_sat(int32_t a, int32_t b)
{
    /* GCC builtin for overflow detection */
    /* https://gcc.gnu.org/onlinedocs/gcc/Integer-Overflow-Builtins.html */
    /* https://godbolt.org/z/3f8c6f9bM */
    /* https://godbolt.org/z/4a7jvPz9d */
#if defined(__GNUC__)
    int32_t result;
    if (__builtin_add_overflow(a, b, &result) == true)
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