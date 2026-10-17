#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   interval_fixed.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../math_general.h" /* typedef struct interval  interval_t; changes to opaque type */

#include <stdint.h>

/******************************************************************************/
/*
    interval
    typedef around 2 register return
*/
/******************************************************************************/
/* signed aligned interval, defined as [low:high] */
// typedef struct { int32_t low; int32_t high; } interval_t;

// /* [0:value] or [value:0] keyed by value sign */
// static inline interval_t interval_of(int32_t value) { return (interval_t) { .low = fixed_min(0, value), .high = fixed_max(0, value) }; }

// static inline interval_t interval_symmetric(int32_t center, uint32_t magnitude) { return (interval_t) { .low = center - magnitude, .high = center + magnitude }; }

// /* two bounds in either order */
// static inline interval_t interval_of_bounds(int32_t a, int32_t b) { return (interval_t) { .low = fixed_min(a, b), .high = fixed_max(a, b) }; }


// /* sign-keyed half-plane: magnitude on the matching side, zero on the other */
// static inline interval_t interval_half_plane(sign_t sign, int32_t alignedMag)
// {
//     switch (sign)
//     {
//         case SIGN_POSITIVE: return (interval_t) { .low = 0, .high = +alignedMag };
//         case SIGN_NEGATIVE: return (interval_t) { .low = -alignedMag, .high = 0 };
//         default:            return (interval_t) { .low = 0, .high = 0 };
//     }
// }

// /* sign-keyed asymmetric pair: aligned magnitude goes with the sign, opposed goes against */
// static inline interval_t interval_of_sign(sign_t sign, int32_t alignedMag, int32_t opposedMag)
// {
//     switch (sign)
//     {
//         case SIGN_POSITIVE: return (interval_t) { .low = -opposedMag, .high = +alignedMag };
//         case SIGN_NEGATIVE: return (interval_t) { .low = -alignedMag, .high = +opposedMag };
//         default:            return (interval_t) { .low = 0, .high = 0 };
//     }
// }


// /*
//     convenience wrappers around multi parameter interval operations
// */
// static inline int32_t interval_clamp(interval_t b, int32_t v) { return fixed_clamp(v, b.low, b.high); }
// static inline bool interval_contains(interval_t b, int32_t v) { return fixed_is_in_range(v, b.low, b.high); }

// /*
//     Inverse of interval_of_sign — recover the unsigned magnitudes from a resolved interval.
//     interval_of_sign(sign, interval_aligned(i, sign), interval_opposed(i, sign)) == i
// */
// static inline int32_t interval_aligned(interval_t interval, sign_t sign) { return (sign == SIGN_NEGATIVE) ? -interval.low : interval.high; }
// static inline int32_t interval_opposed(interval_t interval, sign_t sign) { return (sign == SIGN_NEGATIVE) ? interval.high : -interval.low; }

// /*  */
// static inline interval_t interval_intersect(interval_t a, interval_t b) { return (interval_t) { .low = fixed_max(a.low, b.low), .high = fixed_min(a.high, b.high) }; }

// /* smallest interval holding both */
// static inline interval_t interval_hull(interval_t a, interval_t b) { return (interval_t) { .low = fixed_min(a.low, b.low), .high = fixed_max(a.high, b.high) }; }

// static inline interval_t interval_offset(interval_t i, int32_t k) { return (interval_t) { .low = i.low + k, .high = i.high + k }; }

