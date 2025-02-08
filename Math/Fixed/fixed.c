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
    @brief Calculates square root

    Babylonian method
    y[n] = ( y[n-1] + (x / y[n-1]) ) / 2

    <https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method>
*/
// uint16_t fixed_sqrt(int32_t x)
// {
//     uint32_t yPrev;
//     uint32_t y;

//     if(x > 0)
//     {
//         /*
//             Set y initial to value such that 0 < x <= UINT32_MAX is solved in 6 iterations or less

//             8192*8192 == (1 << 26), solve 0x7FFFFFFF in 6 iterations
//             128*128 == (1 << 14), solve < 1048576
//             1048576U == (1 << 20)
//         */
//         yPrev = ((uint32_t)x > 1048576U) ? 8192U : 128U;
//         for(uint8_t iteration = 0U; iteration < 6U; iteration++)
//         {
//             y = (yPrev + (x / yPrev)) / 2U;
//             if(y == yPrev) { break; }
//             yPrev = y;
//         }
//     }
//     else
//     {
//         y = 0U;
//     }

//     return (uint16_t)y;
// }

/*!
    @brief Calculate the square root of a fixed-point number
    @param x Fixed-point number
    @return Square root of x
*/
uint16_t fixed_sqrt(uint32_t x)
{
    // if (x < 0)
    // {
    //     // Return 0 for negative input (undefined behavior for square root of negative number)
    //     return 0;
    // }

    uint32_t result = 0U;
    uint32_t bit = 1U << 30U; // The second-to-top bit is set

    // "bit" starts at the highest power of four <= the argument.
    while (bit > (uint32_t)x)
    {
        bit >>= 2;
    }

    while (bit != 0)
    {
        if ((uint32_t)x >= result + bit)
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


uint8_t _leading_zeros(uint32_t x)
{
#if (__STDC_VERSION__ >= 202311L)
    return stdc_leading_zeros(x);
#elif defined(__GNUC__)
    return (x == 0U) ? 0U : __builtin_clz(x);
    // return  __builtin_stdc_leading_zeros (x);
#endif
}

/*
    32767 -> 14
    65535 -> 15
    65536 -> 16
    65537 -> 16
*/
uint8_t fixed_log2(uint32_t x)
{

#if (__STDC_VERSION__ >= 202311L)
    return stdc_bit_width(x) - 1U;
    // return  __builtin_stdc_bit_width(x) - 1U;
#elif defined(__GNUC__)
    return (x == 0U) ? 0U : (31U - __builtin_clz(x));
#else
    /* Iterative log2 */
    uint8_t shift = 0U;
    while((x >> shift) > 1U) { shift++; }
    return shift;
#endif
}

/*
    32768 -> 15
    32769 -> 16
    65536 -> 16
    65537 -> 17
*/
uint8_t fixed_log2_ceiling(uint32_t x)
{
    fixed_log2(x - 1U) + 1U;
}

void fixed_pow2_bound(uint32_t * p_lower, uint32_t * p_upper,  uint32_t x)
{
    uint32_t lower = fixed_log2(x);
    uint32_t upper = lower + 1U;
    *p_lower = 1U << lower;
    *p_upper = 1U << upper;
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

/* leading zeros of abs(x) - 1 */
/* log2(INT32_MAX) - log2(abs(x))) */
uint8_t fixed_lshift_max_signed(int32_t x)
{
#if defined(__GNUC__)
    return __builtin_clz(math_abs(x)) - 1U;
#else
    return 30U - fixed_log2(math_abs(x));
#endif
}

/* leading zeros */
uint8_t fixed_lshift_max_unsigned(uint32_t x)
{

#if defined(__GNUC__)
    return __builtin_clz(x);
#else
    return 31U - fixed_log2(x);
#endif
}

int32_t fixed_scalar_max_signed(int32_t x)
{
    return INT32_MAX >> fixed_lshift_max_signed(x);
}