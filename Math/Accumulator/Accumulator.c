
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



/******************************************************************************/
/*
    Init
*/
/******************************************************************************/
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
// void Accumulator_Init(Accumulator_T * p_accum, int32_t coefficient, int8_t shift, int16_t limit_lower, int16_t limit_upper, int32_t initial_value)
// {
//     p_accum->Coefficient = coefficient;
//     p_accum->Shift = shift;
//     p_accum->LimitLower = limit_lower;
//     p_accum->LimitUpper = limit_upper;
//     p_accum->State = initial_value << shift;
// }


/******************************************************************************/
/*

*/
/******************************************************************************/
int32_t Accumulator_Feedback(Accumulator_T * p_accum, int16_t prev)
{
    int32_t error_shifted = (prev << p_accum->Shift) - p_accum->Accumulator;

    // if (error_shifted != 0) { p_accum->State += (error_shifted * p_accum->Coefficient); }
    if (error_shifted != 0) { Accumulator_Add(p_accum, (error_shifted * p_accum->Coefficient)); }

    return (p_accum->Accumulator >> p_accum->Shift);
}


// int32_t Accumulator_Ramp(Accumulator_T * p_accum, int16_t target)
// {

// }

/*  */
int32_t _Accumulator_SatShifted(Accumulator_T * p_accum, int32_t output_shifted)
{
    int32_t output_unshifted = output_shifted >> p_accum->Shift;

    if (output_unshifted > p_accum->LimitUpper)
    {
        p_accum->Accumulator = (int32_t)p_accum->LimitUpper << p_accum->Shift;
        return p_accum->LimitUpper;
    }
    else if (output_unshifted < p_accum->LimitLower)
    {
        p_accum->Accumulator = (int32_t)p_accum->LimitLower << p_accum->Shift;
        return p_accum->LimitLower;
    }
    else
    {
        p_accum->Accumulator = output_shifted;
        return output_unshifted;
    }
}

int32_t Accumulator_Add_Limited(Accumulator_T * p_accum, int16_t input)
{
    return _Accumulator_SatShifted(p_accum, _Accumulator_Add(p_accum, input));
}