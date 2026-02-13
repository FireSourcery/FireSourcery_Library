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

typedef enum sign { SIGN_NEGATIVE = -1, SIGN_ZERO = 0, SIGN_POSITIVE = 1 } sign_t;
/* +1, 0, -1 */
static inline sign_t math_sign(int32_t value) { return (value > 0) - (value < 0); }
static inline bool math_is_sign_diff(int32_t value1, int32_t value2) { return ((value1 ^ value2) < 0); }

static inline int32_t math_sign_mask(int32_t value) { return (value >> 31); } /* 0xFFFFFFFF for negative, 0x00000000 for positive */

static inline uint32_t math_abs(int32_t value) { return abs(value); } /* INT32_MIN returns INT32_MAX + 1 */

static inline int32_t math_max(int32_t value1, int32_t value2) { return ((value1 > value2) ? value1 : value2); }
static inline int32_t math_min(int32_t value1, int32_t value2) { return ((value1 < value2) ? value1 : value2); }
static inline int32_t math_clamp(int32_t value, int32_t lower, int32_t upper) { return math_min(math_max(value, lower), upper); }

// static inline int32_t math_clamp_negative(int32_t value) { return (math_sign(value) + 1) / 2 * value; }
// static inline int32_t math_clamp_0+(int32_t value, int32_t upper) { return (math_sign(value) - 1) / 2 , upper; }
// static inline int32_t math_clamp_-0(int32_t value, int32_t lower) { return math_max(value, lower); }

/*  */
static inline int32_t math_clamp_symmetric(int32_t value, uint32_t bound) { return math_clamp(value, (int32_t)0 - bound, bound); }
/* Clamp between two bounds without specifying order */
static inline int32_t math_clamp_between(int32_t value, int32_t bound1, int32_t bound2) { return math_clamp(value, math_min(bound1, bound2), math_max(bound1, bound2)); }
/* Branchless version */
static inline int32_t math_clamp_between_fast(int32_t value, int32_t bound1, int32_t bound2)
{
    int32_t mask = math_sign_mask(bound2 - bound1);
    int32_t lower = bound1 ^ ((bound1 ^ bound2) & mask);
    int32_t upper = bound2 ^ ((bound1 ^ bound2) & mask);
    return math_clamp(value, lower, upper);
}

static inline bool math_is_in_range(int32_t value, int32_t lower, int32_t upper) { return (value >= lower) && (value <= upper); }
static inline bool math_is_out_of_range(int32_t value, int32_t lower, int32_t upper) { return (value < lower) || (value > upper); }

/* positive as lshift */
static inline int32_t math_shift_signed(int32_t value, int8_t shift) { return (shift > 0) ? (value << shift) : (value >> (-shift)); }

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



/* alias */
static inline int32_t math_limit_upper(int32_t value, int32_t upper) { return math_min(value, upper); }
static inline int32_t math_limit_lower(int32_t value, int32_t lower) { return math_max(value, lower); }
// static inline int32_t limit_upper(int32_t value, int32_t upper) { return math_min(value, upper); }
// static inline int32_t limit_lower(int32_t value, int32_t lower) { return math_max(value, lower); }
// static inline int32_t limit_clamp(int32_t value, int32_t lower, int32_t upper) { return math_min(math_max(value, lower), upper); }

// static inline int16_t fract16_limit_scalar(uint16_t value, uint16_t max)
// {
//     return (value > max) ? fract16_div(max, value) : FRACT16_MAX;
// }

/* proportional on input */
// static inline int16_t  feedback_scalar(uint16_t feedback, uint16_t limit, int16_t input)
// static inline int16_t  feedback_scalar(int16_t input, uint16_t limit, uint16_t feedback)
// {
//     return (feedback > limit) ? (int32_t)input * limit / feedback : input;
// }

// // static inline int16_t limit_feedback_signed(int16_t input, int16_t lower, int16_t upper, int16_t feedback)
// // {
// //     int16_t result;
// //     if      (feedback < lower) { result = (int32_t)input * lower / feedback; }
// //     else if (feedback > upper) { result = (int32_t)input * upper / feedback; }
// //     else                       { result = input; }
// //     return result;
// // }



