/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
#ifndef FILTER_MOV_AVG_H
#define FILTER_MOV_AVG_H

#include "Filter.h"
#include "math_filter.h"
#include <stdint.h>

/*
    Sample value must be significantly greater than N for sample to impact average

    e.g. (2000 +/- 10)/100 => 20 +/- 0, (2000 +/- 10)/10 => 200 +/- 1
*/

static inline void Filter_MovAvg_Init(Filter_T * p_filter, int32_t y0, qfrac16_t lambda)
{
    p_filter->Accumulator = y0;
    p_filter->Coefficient = lambda;
}

static inline int32_t Filter_MovAvg(Filter_T * p_filter, int32_t in)
{
    p_filter->Accumulator = filter_movavg(p_filter->Accumulator, in, p_filter->Coefficient);
    return p_filter->Accumulator;
}

static inline void Filter_MovAvg_InitN(Filter_T * p_filter, int32_t y0, uint16_t n)
{
    p_filter->Accumulator = y0;
    p_filter->Coefficient = n;
}

static inline int32_t Filter_MovAvg_N(Filter_T * p_filter, int32_t in)
{
    p_filter->Accumulator = filter_movavgn(p_filter->Accumulator, in, p_filter->Coefficient);
    return p_filter->Accumulator;
}

#endif
