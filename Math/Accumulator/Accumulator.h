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
    @file   Accumulator.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H

#include <stdint.h>

typedef struct Accumulator
{
    int32_t Coefficient;
    int8_t Shift;
    // uint16_t Index;
    int32_t State; /* Output */
    int32_t Target; /* Input */
    int32_t LimitUpper;
    int32_t LimitLower;
}
Accumulator_T;


static inline int32_t Accumulator_GetTarget(const Accumulator_T * p_accum) { return (p_accum->Target >> p_accum->Shift); }
static inline int32_t Accumulator_GetOutput(const Accumulator_T * p_accum) { return (p_accum->State >> p_accum->Shift); }

static inline void Accumulator_SetOutput(Accumulator_T * p_accum, int32_t match) { p_accum->State = (match << p_accum->Shift); }

static inline void Accumulator_SetTarget(Accumulator_T * p_accum, int32_t target) { p_accum->Target = (target << p_accum->Shift); }

static inline void Accumulator_SetOutputState(Accumulator_T * p_accum, int32_t match)
{
    p_accum->Target = (match << p_accum->Shift);
    p_accum->State = p_accum->Target;
}

static inline void Accumulator_ZeroOutputState(Accumulator_T * p_accum)
{
    p_accum->Target = 0;
    p_accum->State = 0;
}


#endif