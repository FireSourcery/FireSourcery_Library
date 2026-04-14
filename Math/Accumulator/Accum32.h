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
    @file   Accumulator.h
    @author FireSourcery
    @brief  [Brief description of the file]
    @note   Stores internal state as shifted for precision, provides unshifted interface
*/
/******************************************************************************/
#include "../math_general.h"
#include <stdint.h>
#include <stdbool.h>

#include "Accumulator.h"


/*
    Fixed version. Ignore Shift field.
*/
#define ACCUM32_SHIFT 15

static void Accumulator32_Init(Accumulator_T * p_accum, int16_t coefficient) { p_accum->Coefficient = (coefficient << ACCUM32_SHIFT); }

static inline int32_t Accumulator32_Output(Accumulator_T * p_accum) { return (p_accum->Accumulator >> ACCUM32_SHIFT); }

static inline int32_t Accumulator32_Step(Accumulator_T * p_accum, int16_t input)
{
    p_accum->Accumulator = accumulator(p_accum->Coefficient, p_accum->LimitLower, p_accum->LimitUpper, p_accum->Accumulator, input);
    return Accumulator32_Output(p_accum);
}