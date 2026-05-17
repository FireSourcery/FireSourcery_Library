
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
    @file   Accumulator.c
    @author FireSourcery
*/
/******************************************************************************/
#include "Accumulator.h"

void Accumulator_Init(Accumulator_T * p_accum)
{
    p_accum->Accumulator = 0;
    p_accum->Coefficient = 1;
    p_accum->LimitLower = _ACCUM_TO_RAW(INT16_MIN);
    p_accum->LimitUpper = _ACCUM_TO_RAW(INT16_MAX);
}

void Accumulator_InitSat(Accumulator_T * p_accum, accumulator_t coefficient, accumulator_t lower, accumulator_t upper)
{
    p_accum->Coefficient = _ACCUM_TO_RAW(coefficient);
    p_accum->LimitLower = _ACCUM_TO_RAW(lower);
    p_accum->LimitUpper = _ACCUM_TO_RAW(upper);
    p_accum->Accumulator = math_clamp(_ACCUM_TO_RAW(0), p_accum->LimitLower, p_accum->LimitUpper);
}
