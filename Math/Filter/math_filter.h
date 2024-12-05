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
    @version V0
*/
/******************************************************************************/
#ifndef MATH_FILTER_H
#define MATH_FILTER_H

#include "Math/Q/QFrac16.h"
#include <stdint.h>

/*
    y[n] = (1-lambda)*y[n-1] + lambda*x

    lambda = 1/N

    e.g.
    0.015625 -> 64 size array
    0.03125  -> 32 size array
*/
static inline int16_t filter_mov_avg(qfrac16_t lambda, int32_t yPrev, int32_t x)
{
    return ((yPrev * (QFRAC16_1_OVERSAT - lambda)) + (x * lambda)) >> QFRAC16_N_BITS;
}

/*
    y[k] = (1-lambda)*y[k-1] + lambda*x

    lambda = 1/N
*/
static inline int32_t filter_mov_avgn(uint16_t n, int32_t yPrev, int32_t x)
{
    return (yPrev * (n - 1) + x) / n;
}

#endif
