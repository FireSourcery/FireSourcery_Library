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
    @file   Q.h
    @author FireSourcery
    @brief  fixed point / int math operations

*/
/******************************************************************************/
#ifndef FIXED_MATH_H
#define FIXED_MATH_H

#include <stdint.h>

extern uint16_t fixed_sqrt(uint32_t x);
extern uint8_t fixed_log2(uint32_t x);
extern uint8_t fixed_log2_ceiling(uint32_t x);
extern uint8_t fixed_log2_round(uint32_t x);
extern uint32_t fixed_pow2_round(uint32_t x);

extern uint8_t fixed_bit_width(uint32_t x);
extern uint8_t fixed_bit_width_signed(int32_t x);

extern uint8_t fixed_lshift_max_unsigned(uint32_t x);
extern uint8_t fixed_lshift_max_signed(int32_t x);


typedef struct linear { int32_t Factor; uint8_t Shift; } linear_t;

// or move to fixed

/* From factor/divisor ratio (general slope) */
static inline linear_t linear_of_ratio(int32_t factor, int32_t divisor, int32_t inputMax)
{
    uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * factor / divisor);
    return (linear_t) { .Factor = (factor << shift) / divisor, .Shift = shift };
}

/* From a Linear_T (extract the forward coefficient) */
// static inline linear_t linear_of_Linear(const Linear_T * p_linear)
// {
//     return (linear_t) { .Factor = p_linear->Slope, .Shift = p_linear->SlopeShift };
// }

/* From millis duration: step per tick to traverse range in duration_Ms
   e.g. ramp slope: output += (range * Factor) >> Shift each tick */
static inline linear_t linear_of_millis(uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
{
    uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
    uint8_t shift = fixed_lshift_max_signed((int32_t)range);
    return (linear_t) { .Factor = ((int32_t)range << shift) / ticks, .Shift = shift };
}

/* From per-second rate: result += rate_PerS / updateFreq per tick */
static inline linear_t linear_of_rate_per_second(uint32_t updateFreq_Hz, uint16_t ratePerS)
{
    uint8_t shift = fixed_lshift_max_signed((int32_t)ratePerS);
    return (linear_t) { .Factor = ((int32_t)ratePerS << shift) / updateFreq_Hz, .Shift = shift };
}

/* From fixed-point fractional bits (identity scaled to Q format)
   e.g. map [0:xRange] => [0:(1 << nBits)] */
static inline linear_t linear_of_fixed(uint8_t nFractionalBits, int32_t xRange)
{
    uint8_t shift = fixed_lshift_max_signed(xRange);
    return (linear_t) { .Factor = ((int32_t)1 << (nFractionalBits + shift)) / xRange, .Shift = shift };
}

/* From percentage: value * percent / 100 */
static inline linear_t linear_of_percent(uint8_t percent)
{
    /* shift 7: 128 * 100 = 12800 < INT16_MAX */
    return (linear_t) { .Factor = ((int32_t)percent << 7) / 100, .Shift = 7 };
}

/* From fract16 scalar: result = input * frac >> 15 (already in linear_t form) */
// static inline linear_t linear_of_fract16(fract16_t frac)
// {
//     return (linear_t) { .Factor = (int32_t)frac, .Shift = 15U };
// }

/* From two-point map: maps [0:inputMax] => [0:outputMax] */
static inline linear_t linear_of_map(int32_t inputMax, int32_t outputMax)
{
    uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * outputMax / inputMax);
    return (linear_t) { .Factor = (outputMax << shift) / inputMax, .Shift = shift };
}

#endif

