
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
void Accumulator_Init(Accumulator_T * p_accum)
{
    p_accum->LimitLower = INT16_MIN;
    p_accum->LimitUpper = INT16_MAX;
    p_accum->Accumulator = 0;
    p_accum->Index = 0;
    p_accum->Shift = 0;
    p_accum->Coefficient = 1;

}

void Accumulator_Init_AsFract16(Accumulator_T * p_accum, int16_t coefficient_param)
{
    p_accum->Shift = 15U;
    p_accum->Coefficient = (coefficient_param << p_accum->Shift);
}

// void Accumulator_Init_AsFract16Output(Accumulator_T * p_accum, int32_t coefficient_param)
// {
//     p_accum->Shift = math_max(15 - fixed_bit_width(coefficient_param), 0); /* left shift only */
//     p_accum->Coefficient = (coefficient_param << p_accum->Shift);
// }

/******************************************************************************/
/*

*/
/******************************************************************************/

// // int32_t Accumulator_Ramp(Accumulator_T * p_accum, int16_t target)
// // {

// // }

// /*  */
// int32_t _Accumulator_SatShifted(Accumulator_T * p_accum, int32_t output_shifted)
// {
//     int32_t output_unshifted = output_shifted >> p_accum->Shift;

//     if (output_unshifted > p_accum->LimitUpper)
//     {
//         p_accum->Accumulator = (int32_t)p_accum->LimitUpper << p_accum->Shift;
//         return p_accum->LimitUpper;
//     }
//     else if (output_unshifted < p_accum->LimitLower)
//     {
//         p_accum->Accumulator = (int32_t)p_accum->LimitLower << p_accum->Shift;
//         return p_accum->LimitLower;
//     }
//     else
//     {
//         p_accum->Accumulator = output_shifted;
//         return output_unshifted;
//     }
// }

// int32_t Accumulator_Add_Limited(Accumulator_T * p_accum, int16_t input)
// {
//     return _Accumulator_SatShifted(p_accum, _Accumulator_Add(p_accum, input));
// }