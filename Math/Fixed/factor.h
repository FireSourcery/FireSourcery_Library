#pragma once
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
    @file   Scalar.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "fixed.h"

#include <stdint.h>
// #include <stdfix.h>




// /* From factor/divisor ratio (general slope) */
// static inline norm32_t norm_of_ratio(int32_t factor, int32_t divisor, int32_t inputMax)
// {
//     uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * factor / divisor);
//     return (norm32_t) { .Factor = (factor << shift) / divisor, .Shift = shift };
// }

// /* From a Linear_T (extract the forward coefficient) */
// // static inline norm32_t norm_of_Linear(const Linear_T * p_linear)
// // {
// //     return (norm32_t) { .Factor = p_linear->Slope, .Shift = p_linear->SlopeShift };
// // }

// /* From millis duration: step per tick to traverse range in duration_Ms
//    e.g. ramp slope: output += (range * Factor) >> Shift each tick */
// static inline norm32_t norm_of_millis(uint32_t updateFreq_Hz, uint16_t duration_Ms, uint16_t range)
// {
//     uint32_t ticks = (duration_Ms != 0U) ? (updateFreq_Hz * duration_Ms / 1000U) : 1U;
//     uint8_t shift = fixed_lshift_max_signed((int32_t)range);
//     return (norm32_t) { .Factor = ((int32_t)range << shift) / ticks, .Shift = shift };
// }

// /* From per-second rate: result += rate_PerS / updateFreq per tick */
// static inline norm32_t norm_of_rate_per_second(uint32_t updateFreq_Hz, uint16_t ratePerS)
// {
//     uint8_t shift = fixed_lshift_max_signed((int32_t)ratePerS);
//     return (norm32_t) { .Factor = ((int32_t)ratePerS << shift) / updateFreq_Hz, .Shift = shift };
// }

// /* From fixed-point fractional bits (identity scaled to Q format)
//    e.g. map [0:xRange] => [0:(1 << nBits)] */
// static inline norm32_t norm_of_fixed(uint8_t nFractionalBits, int32_t xRange)
// {
//     uint8_t shift = fixed_lshift_max_signed(xRange);
//     return (norm32_t) { .Factor = ((int32_t)1 << (nFractionalBits + shift)) / xRange, .Shift = shift };
// }

// /* From percentage: value * percent / 100 */
// static inline norm32_t norm_of_percent(uint8_t percent)
// {
//     /* shift 7: 128 * 100 = 12800 < INT16_MAX */
//     return (norm32_t) { .Factor = ((int32_t)percent << 7) / 100, .Shift = 7 };
// }

// /* From fract16 norm32: result = input * frac >> 15 (already in norm32_t form) */
// // static inline norm32_t norm_of_fract16(fract16_t frac)
// // {
// //     return (norm32_t) { .Factor = (int32_t)frac, .Shift = 15U };
// // }

// /* From two-point map: maps [0:inputMax] => [0:outputMax] */
// static inline norm32_t norm_of_map(int32_t inputMax, int32_t outputMax)
// {
//     uint8_t shift = fixed_lshift_max_signed((inputMax * 2 - 1) * outputMax / inputMax);
//     return (norm32_t) { .Factor = (outputMax << shift) / inputMax, .Shift = shift };
// }
