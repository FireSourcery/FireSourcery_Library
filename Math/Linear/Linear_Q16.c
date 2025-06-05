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

/*
    Allow 2x input interval (XRef-X0), over saturation before overflow
        f([X0-2*XDelta:X0+2*XDelta]) == [Y0-2*YDelta:Y0+2*YDelta]
*/
#define LINEAR_INT32_MAX_SHIFT (30)  // INT32_MAX/2, accum32 << 15

/******************************************************************************/
/*!
    Linear Fixed
    f(x) = q16
    f([x0:xRef]) = [0:65536]

    Equalvalent to Linear_Init_Map(p_linear, x0, xRef, 0, (1 << nFractionalBits));
*/
/******************************************************************************/
void Linear_Fixed_Init(Linear_T * p_linear, uint8_t nFractionalBits, int32_t x0, int32_t xRef)
{
    p_linear->Slope = ((int32_t)1 << LINEAR_INT32_MAX_SHIFT) / (xRef - x0);  // (1 << 30) ensures the result is within the [INT32_MAX/2] range
    p_linear->SlopeShift = (LINEAR_INT32_MAX_SHIFT - nFractionalBits); // Adjust the shift to match the fractional bits
    p_linear->InvSlope = (xRef - x0);
    p_linear->InvSlopeShift = nFractionalBits;
    p_linear->X0 = x0;
    // unused
    p_linear->XReference = xRef;
    p_linear->XDelta = xRef - x0;
}

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



