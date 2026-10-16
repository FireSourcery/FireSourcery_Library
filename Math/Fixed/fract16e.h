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
    value = mantissa · 2^-shift
          = [fract16_t] mantissa · 2^exponent,   exponent = 15 - shift

    shift in [0, 31], the right shift of an int32 product

    alternative to accum32_t, accum32_t multiply without 64-bit intermediate

    typedef struct { int16_t mantissa; int8_t exp; } fract_e_t;
    fract16e_mul(fract_e_t a, accum32_t b) { return ((int32_t)a.mantissa * b >> (FRACT16_N_BITS - a.exp)); }
*/
/******************************************************************************/
typedef struct { int16_t mantissa; uint8_t shift; } fract16e_t;

static inline int8_t fract16e_exponent(fract16e_t a) { return FRACT16_N_BITS - a.shift; }
// static inline fract16e_t fract16e_k(fract16_t mantissa, int8_t m) { return (fract16e_t) { .mantissa = mantissa, .shift = FRACT16_N_BITS - m }; }

static inline fract16e_t fract16e_k(accum32_t value, int8_t m) { return (fract16e_t){ .mantissa = (int16_t)(value >> m), .shift = FRACT16_N_BITS - m }; }

/******************************************************************************/
/*!
    Constructors
*/
/******************************************************************************/
/* value · 2^-shift. Exact within int16, otherwise floored to 15 significant bits. */
/* caller-chosen format */
static inline fract16e_t _fract16e(int32_t value, int8_t shift)
{
    uint8_t excess = math_max(fixed_bit_width_signed(value) - FRACT16_N_BITS, 0);
    assert(math_is_in_range(shift - excess, 0, 31));
    return (fract16e_t) { .mantissa = (int16_t)(value >> excess), .shift = shift - excess };
}


/*
    constructor [mantissa · 2^exp],   32767e1 = 32767 · 2^1 ≈ 2.0
    storage     [mantissa · 2^-shift]

    m integer bits of value, fract16e_exponent(result) = exp + m

    [fract16e_t] · b = mantissa * b >> (FRACT16_N_BITS - exp);
*/
static inline fract16e_t fract16e(accum32_t value, int8_t exp)
{
    uint8_t m = fixed_bit_width_signed(value >> FRACT16_N_BITS);
    assert(math_is_in_range(exp + m, -16, FRACT16_N_BITS)); /* shift in [0, 31] */
    return (fract16e_t){ .mantissa = (int16_t)(value >> m), .shift = FRACT16_N_BITS - (exp + m) };
}

/* accum in [-2^30, 2^30) */
static inline fract16e_t fract16e_of(accum32_t accum) { return fract16e(accum, 0); }


/*
    num / den, |num / den| < 2^15
    s places |(num << s) / den| in [2^14, 2^16] before dividing: full mantissa at any magnitude
*/
static inline fract16e_t fract16e_of_ratio(int32_t num, uint16_t den)
{
    int8_t shift = FRACT16_N_BITS + fixed_bit_width(den) - fixed_bit_width_signed(num);
    assert(den != 0U);
    assert(shift >= 0);
    return _fract16e((num << shift) / den, shift);
}

static inline fract16e_t fract16e_of_factors(int32_t num, int32_t factor)
{
    return fract16e_of((uint64_t)num * factor >> FRACT16_N_BITS);
}

static inline fract16e_t fract16e_merge(fract16e_t a, fract16e_t b)
{
    return fract16e((int32_t)a.mantissa * b.mantissa, fract16e_exponent(a) + fract16e_exponent(b) - FRACT16_N_BITS);
}

// static inline fract16e_t fract16e_of_ratio(int32_t num, uint16_t den)
// {
//     int8_t exp = fixed_bit_width_signed(num) - fixed_bit_width(den);
//     assert(den != 0U);
//     assert(exp <= FRACT16_N_BITS); /* checked before the shift below runs */
//     return fract16e((num << (FRACT16_N_BITS - exp)) / den, exp);
// }


// static inline fract16e_t fract16e_pi(uint32_t factor) { return fract16e(FRACT16_PI, fixed_bit_width(factor >> FRACT16_N_BITS)); }

/******************************************************************************/
/*!
    Operations
*/
/******************************************************************************/
static inline accum32_t fract16e_mul(fract16e_t a, accum32_t b)
{
    assert(math_abs(b) < FRACT16_SCALE * 2); /* |mantissa · b| < 2^31 */
    return ((int32_t)a.mantissa * b >> a.shift);
}


// #define fract16_mul_wide(a, b) _Generic((a), \
//     accum32_t: accum32_mul, \
//     fract16e_t: fract16e_mul \
// )(a, b)


/*
    Thumb-1 (ARMv6-M, ARMv8-M Baseline) has no SMULL: an int64 product is a __aeabi_lmul call.
    Define on other targets to run the 32-bit form, e.g. host tests.
*/
#if !defined(FIXED_LONG_MULTIPLY_SOFT) && defined(__ARM_ARCH_ISA_THUMB) && (__ARM_ARCH_ISA_THUMB == 1)
typedef fract16e_t fract16k_t;
// static inline fract16k_t fract16k_of(accum32_t a) { return fract16e_of(a); }
static inline fract16k_t fract16k(accum32_t value, uint8_t m) { return fract16e_k(value, m); }
static inline accum32_t fract16k_mul(fract16k_t k, accum32_t b) { return fract16e_mul(k, b); }
static inline accum32_t accum32_of_fract16k(fract16k_t k) { return fract16e_mul(k, FRACT16_SCALE); }
#else
typedef accum32_t fract16k_t;
// static inline fract16k_t fract16k_of(accum32_t a) { return a; }
static inline fract16k_t fract16k(accum32_t value, uint8_t m) { (void)m; return value; }
static inline accum32_t fract16k_mul(fract16k_t k, accum32_t b) { return accum32_mul(k, b); }
static inline accum32_t accum32_of_fract16k(fract16k_t k) { return k; }
#endif



/******************************************************************************/
/*!
*/
/******************************************************************************/
// typedef struct { int32_t mantissa; int8_t exp; } fract_e_t;
// static inline fract_e_t _fract16e(int32_t value, int8_t shift)
// {
//     uint8_t excess = math_max(fixed_bit_width_signed(value) - FRACT16_N_BITS, 0);
//     assert(math_is_in_range(shift - excess, 0, 31));
//     return (fract_e_t) { .mantissa = (int16_t)(value >> excess), .shift = shift - excess };
// }

/* From factor/divisor ratio (general slope) */
// static inline fract_e_t fract16e_of_ratio(int32_t factor, int32_t divisor, int32_t inputMax)
// {
//     uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * factor / divisor);
//     return (fract_e_t) { .mantissa = (factor << shift) / divisor, .shift = shift };
// }

/* From millis duration: step per tick to traverse range in duration_Ms
   e.g. ramp slope: output += (range * Factor) >> Shift each tick */
// static inline fract_e_t fract16e_of_millis(uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
// {
//     uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
//     uint8_t shift = fixed_lshift_max_signed((int32_t)range);
//     return (fract_e_t) { .mantissa = ((int32_t)range << shift) / ticks, .shift = shift };
// }

// /* From per-second rate: result += rate_PerS / updateFreq per tick */
// static inline fract_e_t fract16e_of_rate_per_second(uint32_t updateFreq_Hz, uint16_t ratePerS)
// {
//     uint8_t shift = fixed_lshift_max_signed((int32_t)ratePerS);
//     return (fract_e_t) { .mantissa = ((int32_t)ratePerS << shift) / updateFreq_Hz, .shift = shift };
// }

// /* From fixed-point fractional bits (identity scaled to Q format)
//    e.g. map [0:xRange] => [0:(1 << nBits)] */
// static inline fract_e_t fract16e_of_fixed(uint8_t nFractionalBits, int32_t xRange)
// {
//     uint8_t shift = fixed_lshift_max_signed(xRange);
//     return (fract_e_t) { .mantissa = ((int32_t)1 << (nFractionalBits + shift)) / xRange, .shift = shift };
// }

// /* From two-point map: maps [0:inputMax] => [0:outputMax] */
// static inline fract_e_t fract16e_of_map(int32_t inputMax, int32_t outputMax)
// {
//     uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * outputMax / inputMax);
//     return (fract_e_t) { .mantissa = (outputMax << shift) / inputMax, .shift = shift };
// }

