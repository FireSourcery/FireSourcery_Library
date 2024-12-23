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
    @file   Linear.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Linear.h"


/*!
    Shift <= log2(INT32_MAX / ((xRef - x0) * Slope)) - 1
    @param[in] inputInterval - max input delta without overflow
*/
uint8_t _Linear_SlopeShift(int32_t factor, int32_t divisor, int32_t inputInterval)
{
    return fixed_lshift_max_signed((inputInterval * 2 - 1) * factor / divisor); /* divide first rounds up log2 */

}

/******************************************************************************/
/*!
    Linear
    f(x) = (factor * (x - x0) / divisor) + y0
    f(xRef) = yRef

    By known slope
    Derive XRef => x input max [X0 + 2*(XRef - X0)]

    Overflow: factor, divisor > 131,071.
        (divisor * (yRef - y0)) > INT32_MAX
        (factor * (xRef - x0)) > INT32_MAX

    if slope < 1, (factor < divisor), shift 14, => max input [65536+65535]
    if slope > 1, (factor > divisor), bound with yRef.

    @param[in] y0 - y-intercept, x0 = 0
    @param[in] yRef - y output as 100 percent
*/
/******************************************************************************/
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
    p_linear->X0                = 0;
    p_linear->XReference        = linear_invf(factor, divisor, y0, yRef); /* (yRef - y0)*divisor/factor + 0 */
    p_linear->XDeltaRef         = p_linear->XReference - p_linear->X0;
    p_linear->SlopeShift        = _Linear_SlopeShift(factor, divisor, p_linear->XDeltaRef); // log2(INT32_MAX / ((xRef - x0) * Slope)) - 1
    p_linear->Slope             = (factor << p_linear->SlopeShift) / divisor;
    p_linear->Y0                = y0;
    p_linear->YReference        = yRef;
    p_linear->YDeltaRef         = yRef - y0;
    p_linear->InvSlopeShift     = _Linear_SlopeShift(divisor, factor, p_linear->YDeltaRef);
    p_linear->InvSlope          = (divisor << p_linear->InvSlopeShift) / factor;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    p_linear->SlopeFactor = factor;
    p_linear->SlopeDivisor = divisor;
    p_linear->Y0 = y0;
    p_linear->YReference = yRef;
#endif
}

/*
    Map [x0:xRef] to [y0:yRef]. Interpolate from (x0, y0) to (xRef, yRef).
    xRef - x0 must be > INT32_MIN
    Derive slope
*/
void Linear_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
    p_linear->X0                = x0;
    p_linear->Y0                = y0;
    p_linear->XReference        = xRef;
    p_linear->YReference        = yRef;
    p_linear->XDeltaRef         = xRef - x0;
    p_linear->YDeltaRef         = yRef - y0;
    p_linear->SlopeShift        = _Linear_SlopeShift(p_linear->YDeltaRef, p_linear->XDeltaRef, p_linear->XDeltaRef);
    p_linear->Slope             = (p_linear->YDeltaRef << p_linear->SlopeShift) / p_linear->XDeltaRef;
    p_linear->InvSlopeShift     = _Linear_SlopeShift(p_linear->YDeltaRef, p_linear->XDeltaRef, p_linear->YDeltaRef);
    p_linear->InvSlope          = (p_linear->XDeltaRef << p_linear->InvSlopeShift) / p_linear->YDeltaRef;
#endif
}


/******************************************************************************/
/*!
    Scalar
*/
/******************************************************************************/
/* scalar may be compile time constant, can compiler unroll loop to optimize? */
int32_t Linear_Of_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar)
{
    int32_t factor = x * p_linear->Slope;
    int32_t result = 0;

    /*
        Loop N = Log_[DivisorN](scalar)
        scalar 1000
            [DivisorN == 10] => 4
            [DivisorN == 2]  => 10
    */
    for (uint16_t iDivisor = 1U; scalar >= iDivisor; iDivisor *= 4U) /* scalar / iDivisor > 0U */
    {
        if (factor < INT32_MAX / scalar * iDivisor) /* (factor * (scalar / iDivisor) < INT32_MAX) */
        {
            result = Linear_Of(p_linear, x * scalar / iDivisor) * iDivisor;
            break;
        }
    }

    if(result == 0) { result = Linear_Of(p_linear, x) * scalar; }

    return result;
}

int32_t Linear_InvOf_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar)
{
    (void)p_linear;
    (void)y;
    (void)scalar;
    return 0; //todo
}
