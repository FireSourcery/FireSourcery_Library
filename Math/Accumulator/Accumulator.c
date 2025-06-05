
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
    @file   Accumulator.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "Accumulator.h"

// void Accumulator_InitFrom(Accumulator_T * p_accum, const Accumulator_Config_T * p_config)
// {
//     if (p_config != NULL) { memcpy(&p_accum->Config, p_config, sizeof(Accumulator_Config_T)); }
//     ResetGains(p_accum);
//     Accumulator_SetOutputLimits(p_accum, INT16_MIN, INT16_MAX);
//     Accumulator_Reset(p_accum);
// }

void Accumulator_InitFrom(Accumulator_T * p_accum)
{
    p_accum->LimitLower = INT16_MIN;
    p_accum->LimitUpper = INT16_MAX;

}


/* Initialization */
// static inline void Accumulator_Init(Accumulator_T * p_accum, int32_t coefficient, int8_t shift, int16_t limit_lower, int16_t limit_upper, int32_t initial_value)
// {
//     p_accum->Coefficient = coefficient;
//     p_accum->Shift = shift;
//     p_accum->LimitLower = limit_lower;
//     p_accum->LimitUpper = limit_upper;
//     p_accum->State = initial_value << shift;
// }
