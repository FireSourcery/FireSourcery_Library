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
    @file   Linear.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Linear.h"


/******************************************************************************/
/*!
    Linear
    f(x) = (factor * x / divisor) + y0
    f(xRef) = yRef, xRef = (yRef - y0) * divisor / factor

    Config by factor/divisor slope

    @param[in] y0 - y-intercept, f(0) = y0
    @param[in] yRef - y output as 100 percent
*/
/******************************************************************************/
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
    Linear_Map_Init(p_linear, 0, linear_invf(factor, divisor, y0, yRef), y0, yRef);
}

/******************************************************************************/
/*
    Map [x0:xRef] to [y0:yRef]. Interpolate from (x0, y0) to (xRef, yRef).
    xRef - x0 must be > INT32_MIN
    Derive slope

    Stored at the zero crossing, X0 = invf(0), x0 when y0 = 0.
    Shift sized by the larger |y|, both points evaluate with 2x headroom.
*/
/******************************************************************************/
void Linear_Map_Init(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef)
{
    assert(xRef != x0);  /* Must have non-zero x interval */
    assert(yRef != y0);  /* Must have non-zero slope, f(X0) = 0 */

    p_linear->SlopeShift        = fixed_lshift_max_signed(2 * math_max(math_abs(y0), math_abs(yRef)));
    p_linear->Slope             = ((yRef - y0) << p_linear->SlopeShift) / (xRef - x0);
    p_linear->X0                = linear_invf_full(p_linear->Slope, (int32_t)1 << p_linear->SlopeShift, x0, y0, 0);
    p_linear->XDelta            = xRef - p_linear->X0;

}

/*
    Caller bounds x, for non linear
*/
// void _Linear_Map_Init(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef)
// {
//     p_linear->X0                = x0;
//     p_linear->XDelta            = xRef - x0;
//     p_linear->SlopeShift        = fixed_lshift_max_signed(2 * (yRef - y0));
//     p_linear->Slope             = ((yRef - y0) << p_linear->SlopeShift) / (xRef - x0);
// }

/*
    Allow 2x input interval (XRef-X0), over saturation before overflow
        f([X0-2*XDelta:X0+2*XDelta]) == [-2*YRef:2*YRef]
*/
#define LINEAR_INT32_MAX_SHIFT (30)  // INT32_MAX/2, accum32 << 15

/******************************************************************************/
/*!
    Linear Fixed
    f(x) = q16
    f([x0:xRef]) = [0:65536)

    Equalvalent to Linear_Map_Init(p_linear, x0, xRef, 0, (1 << nFractionalBits));
*/
/******************************************************************************/
void Linear_Fixed_Init(Linear_T * p_linear, uint8_t nFractionalBits, int32_t x0, int32_t xRef)
{
    assert(nFractionalBits <= 30);  /* Prevent overflow in shifts */
    assert(nFractionalBits > 0);    /* Must have at least 1 fractional bit */
    assert(xRef != x0);             /* Must have non-zero interval */

    /* 1.0 exclusive. |Slope * XDelta| < 2^30, f([x0 - XDelta:x0 + XDelta]) => [-2^n:2^n - 1], a clamp on input alone bounds the output */
    p_linear->Slope = (((int32_t)1L << LINEAR_INT32_MAX_SHIFT) - 1) / (xRef - x0);
    p_linear->SlopeShift = (LINEAR_INT32_MAX_SHIFT - nFractionalBits); // Adjust the shift to match the fractional bits
    p_linear->X0 = x0;
    p_linear->XDelta = xRef - x0;
}

