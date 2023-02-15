/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     Q.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Q.h"
#include "Math/math_general.h"

/*!
    @brief Calculates square root

    Babylonian method
    y[n] = ( y[n-1] + (x / y[n-1]) ) / 2

    <https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Babylonian_method>
*/
uint16_t q_sqrt(int32_t x)
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

/* Iterative log2 */
/*
    0 -> 0
    1 -> 0
    2 -> 1
*/
uint8_t q_log2(uint32_t num)
{
    uint8_t shift = 0U;
    while((num >> shift) > 1U) { shift++; }
    return shift;
}

/*
    0 -> 0
    1 -> 1
    2 -> 2
*/
uint8_t q_log2_ceiling(uint32_t num)
{
    uint8_t shift = 0U;
    while((num >> shift) > 0U) { shift++; }
    return shift;
}

uint32_t q_pow2bound(uint32_t * p_lower, uint32_t * p_upper,  uint32_t num)
{
    uint32_t lower = q_log2(num);
    *p_lower = 1U << lower;
    *p_upper = 1U << (lower + 1U);
}

uint8_t q_log2_round(uint32_t num)
{
    uint32_t lower = q_log2(num);
    uint32_t upper = lower + 1U;
    uint32_t pow2Lower = 1U << lower;
    uint32_t pow2Upper = 1U << upper;

    return (pow2Upper - num) < (num - pow2Lower) ? upper : lower;
}

/* nearest pow2 */
uint32_t q_pow2_round(uint32_t num)
{
    uint32_t lower = q_log2(num);
    uint32_t upper = lower + 1U;
    uint32_t pow2Lower = 1U << lower;
    uint32_t pow2Upper = 1U << upper;

    return (pow2Upper - num) < (num - pow2Lower) ? pow2Upper : pow2Lower;
}

/* num != INT32_MIN */
// uint8_t q_lshiftlimit_signed(int32_t num)
uint8_t q_maxshift_signed(int32_t num)
{
    // uint32_t positiveMax = (num >= 0) ? INT32_MAX / num : INT32_MIN / num;
    // return  q_log2(positiveMax);
    uint32_t positiveNum = math_abs(num);
    return 30U - q_log2(positiveNum); /* q_log2(INT32_MAX) - q_log2(positiveNum); */
}

uint8_t q_maxshift_unsigned(uint32_t num)
{
    return 31U - q_log2(num);
}

int32_t q_maxfactor_signed(uint32_t num)
{

}