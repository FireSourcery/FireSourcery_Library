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
    @file
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef MATH_FILTER_H
#define MATH_FILTER_H

#include "Math/Fixed/fract16.h"
#include <stdint.h>

/*
    y[n] = (1-lambda)*y[n-1] + lambda*x

    lambda = 1/N

    e.g.
    0.015625 -> 64 count array
    0.03125  -> 32 count array
*/
static inline int16_t filter_mov_avg(fract16_t lambda, int32_t y_prev, int32_t x)
{
    return ((y_prev * (FRACT16_1_OVERSAT - lambda)) + (x * lambda)) >> FRACT16_N_BITS;
}

/*
    y[k] = (1-lambda)*y[k-1] + lambda*x

    lambda = 1/N
*/
static inline int32_t filter_mov_avgn(uint16_t n, int32_t y_prev, int32_t x)
{
    return (y_prev * (n - 1) + x) / n;
}



/*

*/
// static inline int32_t filter_threshold(int32_t threshold_on, int32_t threshold_off, int32_t yPrev, int32_t x)
// {
//     if (x >= threshold_on || x <= threshold_off) return x;
//     return yPrev;
// }

// static inline int32_t filter_hysteresis_lower(int32_t threshold_on, int32_t threshold_off, int32_t yPrev, int32_t x)
// {
//     if (x <= threshold_off)  return x;
//     if (yPrev >= threshold_on)
//     {
//         return (x >= threshold_on) ? x : yPrev;
//     }
// }
// static inline int32_t RateLimit_Update(int32_t input, int32_t prev_output, int32_t max_increase, int32_t max_decrease)
// {
//     int32_t delta = input - prev_output;

//     if (delta > max_increase)
//         return prev_output + max_increase;
//     else if (delta < -max_decrease)
//         return prev_output - max_decrease;
//     else
//         return input;
// }

// static inline int32_t RateLimit_Symmetric(int32_t input, int32_t prev_output, int32_t max_rate)
// {
//     return RateLimit_Update(input, prev_output, max_rate, max_rate);
// }

// /* Percentage-based rate limiting */
// static inline int32_t RateLimit_Percent(int32_t input, int32_t prev_output, uint8_t max_change_percent)
// {
//     int32_t max_delta = (int32_t)((int64_t)ABS(prev_output) * max_change_percent / 100);
//     if (max_delta == 0) max_delta = 1;  /* Minimum change */

//     return RateLimit_Symmetric(input, prev_output, max_delta);
// }




#endif
