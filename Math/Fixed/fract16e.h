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
    @file   fract16e.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "fract16.h"

#include <stdint.h>
#include <assert.h>

/******************************************************************************/
/*
    alternative to accum32_t, accum32_t multiply without 64-bit intermediate
*/
/******************************************************************************/
/* value = mantissa × 2^exp */
typedef struct { int16_t mantissa; int8_t exp; } _fract16e_t;
// static inline accum32_t fract16e_mul(fract16e_t a, accum32_t b) { return ((int32_t)a.mantissa * b >> (FRACT16_N_BITS - a.exp)); }

/* frexp16_t */
/* coeff_t,  fract_e_t,  16 bit by default. following interval_t */ /* 32767e1 = 32767 × 2^1 = 2.0 */


/* value = mantissa × 2^-shift */
typedef struct { int16_t mantissa; uint8_t shift; } fract16e_t;
static inline int8_t fract16e_exponent(fract16e_t a) { return FRACT16_N_BITS - a.shift; }

/*
    constructor [mantissa × 2^exp] 32767e1 = 32767 × 2^1 = 2.0 × 32767
    storage     [mantissa × 2^-shift]

    [fract16e_t] × b = [fract16e_t].mantissa * b >> (FRACT16_N_BITS - [fract16e_t].exp);
*/
static inline fract16e_t fract16e(int32_t value, uint8_t exp)
{
    uint8_t m = math_max(fixed_bit_width_signed(value) + exp - FRACT16_N_BITS, 0); /* within int16 keep as is */
    return (fract16e_t) { .mantissa = (int16_t)(value >> m), .shift = FRACT16_N_BITS - m };
}



/* From fract16 : result = input * frac >> 15 (already in fract16e_t form) */
static inline fract16e_t fract16e_of_fract16(fract16_t frac) { return (fract16e_t){ .mantissa = frac, .shift = FRACT16_N_BITS }; }

/* accum in [-2^30, 2^30) */

static inline fract16e_t fract16e_of_accum32(accum32_t accum)
{
    uint8_t m = math_max(fixed_bit_width_signed(accum) - FRACT16_N_BITS, 0); /* accum < 1.0 keep as is */
    assert(m <= FRACT16_N_BITS); /* accum in [-2^30, 2^30) */
    return (fract16e_t) { .mantissa = (int16_t)(accum >> m), .shift = FRACT16_N_BITS - m };
}


/* value × 2^-shift */
static inline fract16e_t _fract16e(int32_t value, uint8_t shift)
{
    uint8_t m = math_max(fixed_bit_width_signed(value) - FRACT16_N_BITS, 0); /* within int16 keep as is */
    assert(m <= shift);
    return (fract16e_t) { .mantissa = (int16_t)(value >> m), .shift = shift - m };
}

// static inline fract16e_t fract16e_of_accum32(accum32_t accum) { return _fract16e(accum, FRACT16_N_BITS); }


/******************************************************************************/
/*!
    Operations on fract16e_t
*/
/******************************************************************************/
static inline accum32_t fract16e_mul(fract16e_t a, accum32_t b)
{
    assert(math_abs(b) < FRACT16_SCALE * 2); /* |mantissa * b| < 2^31 */
    return ((int32_t)a.mantissa * b >> a.shift);
}


/******************************************************************************/
/*!
    Additional constructors
*/
/******************************************************************************/
/*
    num / den, |num / den| < 2^15
    s places |(num << s) / den| in [2^14, 2^16] before dividing: full mantissa at any magnitude
*/
static inline fract16e_t fract16e_of_ratio(int32_t num, uint16_t den)
{
    int8_t s = FRACT16_N_BITS + fixed_bit_width(den) - fixed_bit_width_signed(num);
    assert(den != 0U);
    assert(s >= 0);
    return _fract16e((num << s) / den, s);
}

/* From factor/divisor ratio (general slope) */
// static inline fract16e_t fract16e_of_ratio(int32_t factor, int32_t divisor, int32_t inputMax)
// {
//     uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * factor / divisor);
//     return (fract16e_t) { .mantissa = (factor << shift) / divisor, .shift = shift };
// }

/* From millis duration: step per tick to traverse range in duration_Ms
   e.g. ramp slope: output += (range * Factor) >> Shift each tick */
static inline fract16e_t fract16e_of_millis(uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
{
    uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
    uint8_t shift = fixed_lshift_max_signed((int32_t)range);
    return (fract16e_t) { .mantissa = ((int32_t)range << shift) / ticks, .shift = shift };
}

/* From per-second rate: result += rate_PerS / updateFreq per tick */
static inline fract16e_t fract16e_of_rate_per_second(uint32_t updateFreq_Hz, uint16_t ratePerS)
{
    uint8_t shift = fixed_lshift_max_signed((int32_t)ratePerS);
    return (fract16e_t) { .mantissa = ((int32_t)ratePerS << shift) / updateFreq_Hz, .shift = shift };
}

/* From fixed-point fractional bits (identity scaled to Q format)
   e.g. map [0:xRange] => [0:(1 << nBits)] */
static inline fract16e_t fract16e_of_fixed(uint8_t nFractionalBits, int32_t xRange)
{
    uint8_t shift = fixed_lshift_max_signed(xRange);
    return (fract16e_t) { .mantissa = ((int32_t)1 << (nFractionalBits + shift)) / xRange, .shift = shift };
}

/* From percentage: value * percent / 100 */
static inline fract16e_t fract16e_of_percent(uint8_t percent)
{
    /* shift 7: 128 * 100 = 12800 < INT16_MAX */
    return (fract16e_t) { .mantissa = ((int32_t)percent << 7) / 100, .shift = 7 };
}

/* From two-point map: maps [0:inputMax] => [0:outputMax] */
static inline fract16e_t fract16e_of_map(int32_t inputMax, int32_t outputMax)
{
    uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * outputMax / inputMax);
    return (fract16e_t) { .mantissa = (outputMax << shift) / inputMax, .shift = shift };
}

/* From a Linear_T (extract the forward coefficient) */
// static inline fract16e_t fract16e_of_Linear(const Linear_T * p_linear)
// {
//     return (fract16e_t) { .mantissa = p_linear->Slope, .shift = p_linear->SlopeShift };
// }