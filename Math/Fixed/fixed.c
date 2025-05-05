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
    @file   Q.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "fixed.h"
#include "Math/math_general.h"


/*!
    @brief Calculate the square root of a fixed-point number
    @param x Fixed-point number
    @return Square root of x
*/
uint16_t fixed_sqrt(uint32_t x)
{
    uint32_t result = 0U;
    uint32_t bit = 1U << 30U; // The second-to-top bit is set

    // "bit" starts at the highest power of four <= the argument.
    while (bit > x) { bit >>= 2; }

    while (bit != 0)
    {
        if (x >= result + bit)
        {
            x -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }

    return result;
}

/*!
    @brief Calculates square root

    Babylonian method
    y[n] = ( y[n-1] + (x / y[n-1]) ) / 2

    <https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method>
*/
uint16_t _fixed_sqrt(int32_t x)
{
    uint32_t yPrev;
    uint32_t y;

    if(x > 0)
    {
        /*
            Set y initial to value such that 0 < x <= UINT32_MAX is solved in 6 iterations or less

            8192*8192 == (1 << 26), solve 0x7FFFFFFF in 6 iterations
            128*128 == (1 << 14), solve < 1048576
            1048576U == (1 << 20)
        */
        yPrev = ((uint32_t)x > 1048576U) ? 8192U : 128U;
        for(uint8_t iteration = 0U; iteration < 6U; iteration++)
        {
            y = (yPrev + (x / yPrev)) / 2U;
            if(y == yPrev) { break; }
            yPrev = y;
        }
    }
    else
    {
        y = 0U;
    }

    return (uint16_t)y;
}


// uint8_t _leading_zeros(uint32_t x)
// {
// #if defined(__GNUC__)
//      return (x == 0U) ? 32U :  __builtin_clz(x);
// #elif(__STDC_VERSION__ >= 202311L)
//     return stdc_leading_zeros(x);
// #endif
// }

uint8_t fixed_bit_width(uint32_t x)
{
#if (__STDC_VERSION__ >= 202311L)
    return stdc_bit_width(x);
#elif defined(__GNUC__)
    return (x == 0U) ? 0U : (32U - __builtin_clz(x));
#else
    uint8_t shift = 0U;
    while ((x >> shift) > 0U) { shift++; }
    return shift;
#endif
}

uint8_t fixed_bit_width_signed(int32_t x)
{
    fixed_bit_width(math_abs(x));
}

/*
    65535 -> 15
    65536 -> 16
    65537 -> 16
*/
uint8_t fixed_log2(uint32_t x)
{
    return fixed_bit_width(x) - 1U;
}

/*
    65535 -> 16
    65536 -> 16
    65537 -> 17
*/
uint8_t fixed_log2_ceiling(uint32_t x)
{
    fixed_log2(x - 1U) + 1U;
}


void fixed_log2_bound(uint32_t * p_lower, uint32_t * p_upper,  uint32_t x)
{
    *p_lower = fixed_log2(x);
    *p_upper = *p_lower + 1U;
}

void fixed_pow2_bound(uint32_t * p_lower, uint32_t * p_upper,  uint32_t x)
{
    uint32_t log2lower;
    uint32_t log2upper;
    fixed_log2_bound(&log2lower, &log2upper, x);
    *p_lower = 1U << log2lower;
    *p_upper = 1U << log2upper;
}

uint8_t fixed_log2_round(uint32_t x)
{
    uint32_t lower = fixed_log2(x);
    uint32_t upper = lower + 1U;
    uint32_t pow2Lower = 1U << lower;
    uint32_t pow2Upper = 1U << upper;

    return (pow2Upper - x) < (x - pow2Lower) ? upper : lower;
}

/* nearest pow2 */
uint32_t fixed_pow2_round(uint32_t x)
{
    uint32_t lower = fixed_log2(x);
    uint32_t upper = lower + 1U;
    uint32_t pow2Lower = 1U << lower;
    uint32_t pow2Upper = 1U << upper;

    return (pow2Upper - x) < (x - pow2Lower) ? pow2Upper : pow2Lower;
}

/* leading zeros */
// uint8_t fixed_lshift_max_unsigned(uint32_t x)
// {
// #if defined(__GNUC__)
//     return __builtin_clz(x);
// #else
//     return 31U - fixed_log2(x);
// #endif
// }

/* leading sign/zero bits - 1 */
/* 31 - bit width */
// uint8_t fixed_unit_scalar_shift(int32_t x)
uint8_t fixed_lshift_max_signed(int32_t x)
{
    31U - fixed_bit_width_signed(x);
}


// int32_t fixed32_unit_scalar(int32_t x)
// {
//     return INT32_MAX >> fixed_lshift_max_signed(x);
// }