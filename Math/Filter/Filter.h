#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Filter.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Math/Accumulator/Accumulator.h"
#include <stdint.h>


/*
    Optionally a generic buffer interface for selection
*/
typedef struct Filter
{
    Accumulator_T Accumulator; /* Accumulator for the filter */
    uint16_t Index;
    // int32_t Coefficient;
    // int32_t Accumulator;
}
Filter_T;

static inline void Filter_Init(Filter_T * p_filter)
{
    p_filter->Accumulator.Accumulator = 0;
    p_filter->Index = 0U;
}

static inline int32_t Filter_Avg(Filter_T * p_filter, int32_t in)
{
    p_filter->Accumulator.Accumulator += in;
    p_filter->Index++;
    return p_filter->Accumulator.Accumulator / p_filter->Index;
}

