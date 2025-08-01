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
#ifndef FILTER_MOV_AVG_H
#define FILTER_MOV_AVG_H

#include "math_mov_avg.h"
#include <stdint.h>

typedef struct MovAvg
{
    uint16_t Index;
    int32_t Coefficient;
    int32_t Accumulator;
    // Accumulator_T Accumulator; //todo store as shifted
}
MovAvg_T;

static inline void MovAvg_Init(MovAvg_T * p_filter, fract16_t lambda, int32_t y0)
{
    p_filter->Accumulator = y0;
    p_filter->Coefficient = lambda;
}

static inline int32_t MovAvg(MovAvg_T * p_filter, int32_t in)
{
    p_filter->Accumulator = filter_mov_avg(p_filter->Coefficient, p_filter->Accumulator, in);
    return p_filter->Accumulator;
}

static inline void MovAvgN_Init(MovAvg_T * p_filter, uint16_t n, int32_t y0)
{
    p_filter->Accumulator = y0;
    p_filter->Coefficient = n;
}

static inline int32_t MovAvgN(MovAvg_T * p_filter, int32_t in)
{
    p_filter->Accumulator = filter_mov_avgn(p_filter->Coefficient, p_filter->Accumulator, in);
    return p_filter->Accumulator;
}

#endif
