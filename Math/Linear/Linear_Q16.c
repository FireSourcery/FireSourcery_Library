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
    @file   Linear_Q16.c
    @author FireSourcery
    @brief  Linear

*/
/******************************************************************************/
#include "Linear_Q16.h"



/******************************************************************************/
/*!
    Q0.16 and Q1.15
    signed/unsigned 16-bit precision, using 32-bits for intermediate calculations

    [x0:xRef] => [0:65536] => [y0_Units:yRef_Units]
        direct shift back is Q16.16
    f(x0) = y0_Fixed == 0
    f(xRef) = 65535
*/
/******************************************************************************/
/*!
    Map [x0, xRef] to [0, 65535]
*/
void Linear_Q16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef)
{
    Linear_Fixed_Init(p_linear, 16U, x0, xRef);
}

// void Linear_Fixed_InitAsFract16(Linear_T * p_linear, int32_t x0, int32_t xRef)
// {
//     Linear_Fixed_Init(p_linear, 15U, x0, xRef);
// }

// void Linear_Fixed_InitAsPercent16(Linear_T * p_linear, int32_t x0, int32_t xRef)
// {
//     Linear_Fixed_Init(p_linear, 16U, x0, xRef);
// }



