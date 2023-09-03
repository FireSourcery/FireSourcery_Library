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
    @file   Linear_Frac16.c
    @author FireSourcery
    @brief  Linear
    @version V0
*/
/******************************************************************************/
#include "Linear_Frac16.h"

/*
    Frac16
    Shift 14 to allow over saturation f([X0-2*(XRef-X0):X0+2*(XRef-X0)]) == [Y0-2*(YRef-Y0):Y0+2*(YRef-Y0)] before overflow
        i.e 2x input interval (XRef-X0), before overflow, while retaining sign bit.
*/
#define LINEAR_FRAC16_SHIFT 14U

/******************************************************************************/
/*!
    Linear Frac16
    Sets frac16 conversion to return without division
    [x0:xRef] => [0:65536] => [y0_Units:yRef_Units]
    y0_Frac16 always 0

    f16(x) = y_Frac16,              Shift
    f(x) = y_Units,                 Shift
    invf16(y_Frac16) = x,           Shift
    invf(y_Units) = x,              Division
    units(y_Frac16) = y_Units,      Shift
    invunits(y_Units) = y_Frac16,   Division

    f16(x0) = y0_Frac16 = 0
    f16(xRef) = 65535
    f(x0) = y0_Units
    f(xRef) = yRef_Units
*/
/******************************************************************************/
/*!
    Map [x0, xRef] to [0, 65535]
    Init using (x0, y0), (xRef, yRef). Derive slope

    @param[in] y0_Units   user units at x0
    @param[in] yRef_Units user units 100%
*/
void Linear_Frac16_Init(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0_Units, int32_t yRef_Units)
{
    p_linear->Slope = (65536 << LINEAR_FRAC16_SHIFT) / (xRef - x0);
    p_linear->SlopeShift = LINEAR_FRAC16_SHIFT;
    p_linear->InvSlope = ((xRef - x0) << LINEAR_FRAC16_SHIFT) / 65536;
    p_linear->InvSlopeShift = LINEAR_FRAC16_SHIFT;
    p_linear->XOffset = x0;
    p_linear->YOffset = y0_Units;
    p_linear->XReference = xRef;
    p_linear->YReference = yRef_Units;
    p_linear->DeltaX = xRef - x0;                 /* Retain for Units conversion */
    p_linear->DeltaY = yRef_Units - y0_Units;     /* Retain for Units conversion */
}

/*
    Init using slope, y. Derive XRef
    Scales factor to 65536
    Scales divisor to xref

    max input = (yRef_Units - 0)*divisor/factor * 2
*/
// void Linear_Frac16_Init_Slope(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t x0, int32_t xRef, int32_t y0_Units, int32_t yRef_Units)
// {
//     p_linear->YReference = yRef_Units;
//     p_linear->XReference = linear_invf(factor, divisor, 0, yRef_Units); /* (yRef_Units - 0)*divisor/factor */
//     p_linear->Slope = (65536 << LINEAR_FRAC16_SHIFT) / p_linear->XReference; /* x0 == 0 */
//     p_linear->SlopeShift = LINEAR_FRAC16_SHIFT;
//     p_linear->InvSlope = (p_linear->XReference << LINEAR_FRAC16_SHIFT) / 65536; //todo maxleftshift, if factor > divisor, invslope can be > 14
//     p_linear->InvSlopeShift = LINEAR_FRAC16_SHIFT;
//     p_linear->XOffset = 0;
//     p_linear->YOffset = y0_Units;
// }
